/*
 * app_param.c —— database 参数门卫层实现（见 app_param.h）
 *
 * 纪律：协议层不得直接改应用变量；所有 set 走这里，经权限/状态/范围检查后
 * 调用 servo_cmd.c 的受控入口（E/D/X 联锁）或 Servo_* API。set 后 get 回读真实值。
 */
#include "app_param.h"
#include "bsp_servo.h"
#include "servo_cmd.h"
#include "pos_loop.h"
#include "param_store.h"
#include "cog_ff.h"
#include <string.h>

/* 权限总闸：默认 0 = 上位机接管（与本板调试场景一致；参考工程默认本地，
 * 但本板无面板/内部控制器，若默认非 0 则上位机一上电就写不进去）。 */
static uint8_t  s_comm_auth      = 0U;
static uint8_t  s_internal_auth  = 0U;
static uint16_t s_control_word   = 0U;
static int8_t   s_op_mode        = APP_OP_MODE_POS;

/* 4 模式给定缓存（工程单位）。与 MAVLink 给定消息解耦：set 先写缓存再装配，
 * 这样上位机即使只发速度给定、位置给定仍保持上次值，切模式时不会跳到 0。 */
static float s_iq_target   = 0.0f;   /* A */
static float s_w_target    = 0.0f;   /* rad/s */
static float s_p_target    = 0.0f;   /* rad（编码器坐标系） */
static float s_mit_p       = 0.0f;
static float s_mit_v       = 0.0f;
static float s_mit_kp      = 30.0f;  /* 与 mit_run.py 优化默认值一致 */
static float s_mit_kd      = 0.05f;
static float s_mit_tau     = 0.0f;

/* 给定就绪标志：控制字 bit0 由 0→1 时，按 s_op_mode 选最近一次显式下发过的
 * 给定装配环路；没下发过就退回 do_enable() 的位置保持（安全默认）。
 * 位置/速度 0 是合法给定，所以单独用标志区分"显式发过 0"和"从未发过"。 */
static uint8_t s_iq_valid;
static uint8_t s_w_valid;
static uint8_t s_p_valid;
static uint8_t s_mit_valid;

/* 向上位机回报的控制字 bit 定义（CiA402 简化） */
#define CW_SWITCH_ON   (1U << 0)

void AppParam_Init(void)
{
    s_comm_auth     = 0U;
    s_internal_auth = 0U;
    s_control_word  = 0U;
    s_op_mode       = APP_OP_MODE_POS;
    s_iq_target = 0.0f;
    s_w_target  = 0.0f;
    s_p_target  = 0.0f;
    s_mit_p = 0.0f;
    s_mit_v = 0.0f;
    s_mit_kp = 30.0f;
    s_mit_kd = 0.05f;
    s_mit_tau = 0.0f;
    s_iq_valid = 0U;
    s_w_valid  = 0U;
    s_p_valid  = 0U;
    s_mit_valid = 0U;
}

/* 权限检查：comm_auth==0 才允许上位机写 */
static uint8_t has_host_auth(void)
{
    return (s_comm_auth == 0U) ? 1U : 0U;
}

AppParamStatus_t AppParam_SetAuthority(uint8_t comm_auth, uint8_t internal_auth)
{
    if (internal_auth != 0U) {
        return APP_PARAM_OUT_OF_RANGE;        /* 本板无内部控制器，非 0 无意义 */
    }
    s_comm_auth     = comm_auth;
    s_internal_auth = internal_auth;
    return APP_PARAM_SUCCESS;
}

uint8_t AppParam_GetCommAuthority(void)     { return s_comm_auth; }
uint8_t AppParam_GetInternalAuthority(void) { return s_internal_auth; }

/* 控制字使能：当前模式已有显式给定时装配该环，否则退回 do_enable() 的位置保持。
 * 已 armed 时只做 deadman 保活（ServoCmd_Arm 幂等），不换环、不重开 MOE。 */
static uint8_t arm_with_mode(void)
{
    uint8_t ok;

    if (ServoCmd_IsArmed() != 0U) {
        return ServoCmd_Arm();
    }

    /* 使能 = 上电待机，不等于"跑到上次的目标"（2026-11-21 修）
     *
     * 原来这里在 s_*_valid 为真时直接装配**缓存的目标**。于是只要历史上成功
     * 下发过一次给定，之后任何一次 cw 使能（disarm 后重使能、清故障后重使能、
     * 上位机重连后点"启动"）都会让电机立刻朝那个旧目标冲：
     *   速度模式 —— "一使能就跑到上次的转速"；
     *   力矩模式 —— 更糟，"一使能就按上次的 iq 出力"。
     *
     * 现在一律以安全初值起环：速度 0、力矩 0、位置 = 当前角（保持）、
     * MIT 用当前位置 + 零速度 + 零前馈（只留位置刚度）。要运动必须再发一帧给定。
     *
     * 这不会破坏"先给给定、再使能"的用法：任何给定在 armed=0 时都会自己完成
     * 装配并起转（do_speed / do_pos / do_current / mit_engage 的完整分支），
     * 那条路径根本不经过这里；等 cw 到达时 armed 已经是 1，走上面的幂等返回。
     *
     * s_iq_valid / s_w_valid / s_p_valid / s_mit_valid 保留，但从此只作
     * "曾经收到过给定"的历史记录，不再参与使能决策。 */
    switch (s_op_mode) {
    case APP_OP_MODE_TORQUE:
        ok = ServoCmd_CtrlTorque(0.0f);
        break;
    case APP_OP_MODE_VEL:
        ok = ServoCmd_CtrlSpeed(0.0f);
        break;
    case APP_OP_MODE_POS:
        ok = ServoCmd_Arm();                 /* do_enable：位置环锁在当前角 */
        break;
    case APP_OP_MODE_MIT: {
        float kp = 0.0f, kd = 0.0f;

        AppParam_GetMit(0, 0, &kp, &kd, 0);
        ok = ServoCmd_CtrlMit(BspServo_GetThetaMulti(), 0.0f, kp, kd, 0.0f);
        break;
    }
    default:
        ok = ServoCmd_Arm();
        break;
    }
    return ok;
}

AppParamStatus_t AppParam_SetControlWord(uint16_t cw, uint8_t halt)
{
    if (has_host_auth() == 0U) { return APP_PARAM_NO_AUTHORITY; }

    /* Halt 优先：等效 D，无条件封锁（D 无拒绝路径） */
    if (halt != 0U) {
        (void)ServoCmd_Disarm();
        s_control_word = 0U;
        return APP_PARAM_SUCCESS;
    }

    if ((cw & CW_SWITCH_ON) != 0U) {
        /* 被拒时返回 WRITE_STATE_ERROR，且 s_control_word 不更新（回读仍是 0）。 */
        if (arm_with_mode() == 0U) { return APP_PARAM_WRITE_STATE_ERROR; }
    } else {
        (void)ServoCmd_Disarm();
    }
    s_control_word = cw;
    return APP_PARAM_SUCCESS;
}

uint16_t AppParam_GetControlWord(void)
{
    /* 回读真实使能状态，而不是缓存上位机写入值：
     * deadman 超时自动封锁后，上位机回读应看到 0。 */
    return (ServoCmd_IsArmed() != 0U) ? (uint16_t)(s_control_word | CW_SWITCH_ON)
                                      : (uint16_t)(s_control_word & (uint16_t)~CW_SWITCH_ON);
}

AppParamStatus_t AppParam_SetOpMode(int8_t mode)
{
    if (has_host_auth() == 0U) { return APP_PARAM_NO_AUTHORITY; }
    if ((mode != APP_OP_MODE_POS) && (mode != APP_OP_MODE_VEL) &&
        (mode != APP_OP_MODE_TORQUE) && (mode != APP_OP_MODE_MIT)) {
        return APP_PARAM_OUT_OF_RANGE;
    }
    /* 模式登记 + 缓存。真正的装配由随后的给定 set 完成（见下方 SetXxxTarget）：
     * 给定消息到来时才按 s_op_mode 调对应受控入口，实现"自动先封锁再切"。
     * 单独切模式不动作环，避免给定还是 0 时就使能导致误动作。 */
    s_op_mode = mode;
    return APP_PARAM_SUCCESS;
}

int8_t AppParam_GetOpMode(void) { return s_op_mode; }

int8_t AppParam_GetActiveMode(void) { return ServoCmd_ActiveMode(); }

/* ---- 4 控制模式给定（set 先写缓存，再按 s_op_mode 装配对应环）----
 * 每个 set 都用 s_op_mode 决定装配哪条环，而不是根据消息类型 —— 这样上位机
 * 先发 30056 选模式、再发对应给定消息即可；发错消息类型时值会被缓存但环不乱切。
 * NaN/超范围一律拒绝，不落缓存、不动环。 */
static uint8_t finite_f(float v) { return (v == v) ? 1U : 0U; }  /* NaN -> 0 */

/* ---- 速度给定协议门卫上限（rad/s）----
 * 原为 ±100 rad/s（955 rpm），是首期保守值。2026-11-19 温漂长测需要 1000 rpm
 * （=104.72 rad/s）却被门卫直接拒成 OUT_OF_RANGE，故抬到 ±220 rad/s（≈2101 rpm）。
 * 依据：
 *   1) 内部 Servo_SetSpeedMode() 有 SERVO_SPD_WREF_MAX=400 rad/s（≈3820 rpm）
 *      硬限（servo_isr.c:587），220 仍留 45% 余量；
 *   2) 覆盖本板历史实测过的 2000 rpm（209.4 rad/s）与本次 1000 rpm；
 *   3) 超出仍是明确 OUT_OF_RANGE，不静默截断（静默夹曾被误读成电机极限）。
 * 注意：这是协议门卫，不是电机能力边界；真正到不了应由母线电压/超速保护暴露。 */
#define APP_SPD_W_MAX_RADS   (220.0f)

AppParamStatus_t AppParam_SetTorqueIq(float iq_a)
{
    if (has_host_auth() == 0U) { return APP_PARAM_NO_AUTHORITY; }
    if (finite_f(iq_a) == 0U || iq_a < -5.0f || iq_a > 5.0f) { return APP_PARAM_OUT_OF_RANGE; }
    s_iq_target = iq_a;
    s_iq_valid  = 1U;
    if (s_op_mode == APP_OP_MODE_TORQUE) {
        if (ServoCmd_CtrlTorque(iq_a) == 0U) { return APP_PARAM_WRITE_STATE_ERROR; }
    }
    return APP_PARAM_SUCCESS;
}

AppParamStatus_t AppParam_SetSpeedW(float w_rad_s)
{
    if (has_host_auth() == 0U) { return APP_PARAM_NO_AUTHORITY; }
    if (finite_f(w_rad_s) == 0U || w_rad_s < -APP_SPD_W_MAX_RADS || w_rad_s > APP_SPD_W_MAX_RADS) {
        return APP_PARAM_OUT_OF_RANGE;
    }
    s_w_target = w_rad_s;
    s_w_valid  = 1U;
    if (s_op_mode == APP_OP_MODE_VEL) {
        if (ServoCmd_CtrlSpeed(w_rad_s) == 0U) { return APP_PARAM_WRITE_STATE_ERROR; }
    }
    return APP_PARAM_SUCCESS;
}

AppParamStatus_t AppParam_SetPosP(float p_rad)
{
    if (has_host_auth() == 0U) { return APP_PARAM_NO_AUTHORITY; }
    /* 多圈机械角，±200 圈（±1256 rad）足够覆盖绝大多数应用，又挡住野值 */
    if (finite_f(p_rad) == 0U || p_rad < -1256.0f || p_rad > 1256.0f) {
        return APP_PARAM_OUT_OF_RANGE;
    }
    s_p_target = p_rad;
    s_p_valid  = 1U;
    if (s_op_mode == APP_OP_MODE_POS) {
        if (ServoCmd_CtrlPos(p_rad) == 0U) { return APP_PARAM_WRITE_STATE_ERROR; }
    }
    return APP_PARAM_SUCCESS;
}

AppParamStatus_t AppParam_SetPosSpeed(float p_rad, float w_rad_s)
{
    if (has_host_auth() == 0U) { return APP_PARAM_NO_AUTHORITY; }
    if (finite_f(p_rad) == 0U || finite_f(w_rad_s) == 0U) { return APP_PARAM_OUT_OF_RANGE; }
    if (p_rad < -1256.0f || p_rad > 1256.0f) { return APP_PARAM_OUT_OF_RANGE; }
    if (w_rad_s < -APP_SPD_W_MAX_RADS || w_rad_s > APP_SPD_W_MAX_RADS) { return APP_PARAM_OUT_OF_RANGE; }

    s_p_target = p_rad;
    s_w_target = w_rad_s;
    /* 旁带字段为 0 时保留上次有效目标：30019 常只填当前模式那一路，
     * 另一路是 0；若无条件置 valid，切模式后控制字使能会把位置直接打到 0。
     * 速度 0 例外（当前模式就是 VEL 时是显式停转，安全）。 */
    if (p_rad != 0.0f) { s_p_valid = 1U; }
    if (w_rad_s != 0.0f || s_op_mode == APP_OP_MODE_VEL) { s_w_valid = 1U; }

    /* 只装配当前模式那一路，另一路纯缓存 —— 与 SetOpMode 的"登记不动作环"
     * 形成闭环：切模式后控制字使能即可直接进目标环。 */
    if (s_op_mode == APP_OP_MODE_POS) {
        if (ServoCmd_CtrlPos(p_rad) == 0U) { return APP_PARAM_WRITE_STATE_ERROR; }
    } else if (s_op_mode == APP_OP_MODE_VEL) {
        if (ServoCmd_CtrlSpeed(w_rad_s) == 0U) { return APP_PARAM_WRITE_STATE_ERROR; }
    }
    return APP_PARAM_SUCCESS;
}

AppParamStatus_t AppParam_SetMitCmd(float p, float v, float kp, float kd, float tau)
{
    if (has_host_auth() == 0U) { return APP_PARAM_NO_AUTHORITY; }
    if (finite_f(p) == 0U || finite_f(v) == 0U || finite_f(kp) == 0U ||
        finite_f(kd) == 0U || finite_f(tau) == 0U) {
        return APP_PARAM_OUT_OF_RANGE;
    }
    s_mit_p = p; s_mit_v = v; s_mit_kp = kp; s_mit_kd = kd; s_mit_tau = tau;
    s_mit_valid = 1U;
    if (s_op_mode == APP_OP_MODE_MIT) {
        if (ServoCmd_CtrlMit(p, v, kp, kd, tau) == 0U) { return APP_PARAM_WRITE_STATE_ERROR; }
    }
    return APP_PARAM_SUCCESS;
}

AppParamStatus_t AppParam_SetMitConfig(float kp, float kd, float kt)
{
    if (has_host_auth() == 0U) { return APP_PARAM_NO_AUTHORITY; }
    /* kt 在 30066 里是协议字段，本板 Kt 固定 0.091 N·m/A（不接受覆盖，只校验有效性） */
    if (finite_f(kp) == 0U || finite_f(kd) == 0U || finite_f(kt) == 0U) {
        return APP_PARAM_OUT_OF_RANGE;
    }
    if (kp < 0.0f || kp > 5000.0f || kd < 0.0f || kd > 100.0f) {
        return APP_PARAM_OUT_OF_RANGE;
    }
    s_mit_kp = kp;
    s_mit_kd = kd;
    /* 配置帧只改缓存，不重发给定（2026-11-22 修）。
     *
     * 原来这里在"已在 MIT 环"时会用**缓存的 s_mit_p** 立刻刷新一次给定，
     * 理由是"避免等下一帧"。实测这是一条能把电机打飞的路：
     *
     *   07.513 起环帧 p=0 + kp=0（盲起环的标准做法 —— 此时 p_now 还没读到，
     *          只能用 0 占位，靠 kp=0 保证不出力）
     *   07.513 紧接着 30066 上刚度 kp=30
     *          → 刷新用 s_mit_p=0 当目标，而实际角是 3062.88 rad
     *          → 误差 -3062.88 被 EP_MAX 限到 -1 rad
     *          → τ = 30×(-1) = -30 N·m，iq 顶 -5 A 满反转
     *   07.557 撞 0x0020 超速（3.6 ms 就能到 30 rad/s）
     *
     * 配置帧本就不该改给定：kp/kd 是乘在**下一帧 30065 的目标**上的系数。
     * 删掉刷新后，上刚度最多晚一个帧周期生效（200 Hz → 5 ms），
     * 而 30065 每帧都会把最新 kp/kd 带进 Servo_SetMitCmd，不需要这里代劳。
     *
     * 顺带修掉一个 status 误导：原来刷新失败会返回 WRITE_STATE_ERROR(3)，
     * 上位机看到的是"配置非法"，实际是环路状态问题，排查方向完全错。 */
    return APP_PARAM_SUCCESS;
}

/* ---- 速度模式轨迹参数（30019 off32 加速度 / 30089 轨迹配置）---- */
AppParamStatus_t AppParam_SetSpeedAccel(float a_rad_s2)
{
    if (has_host_auth() == 0U) { return APP_PARAM_NO_AUTHORITY; }
    /* 与 Servo_SetSpeedAccel 内部限幅一致：5~20000 rad/s²，NaN 拒绝 */
    if (finite_f(a_rad_s2) == 0U || a_rad_s2 < 5.0f || a_rad_s2 > 20000.0f) {
        return APP_PARAM_OUT_OF_RANGE;
    }
    Servo_SetSpeedAccel(a_rad_s2);
    return APP_PARAM_SUCCESS;
}

AppParamStatus_t AppParam_SetSpeedProfile(uint8_t mode, float j_max)
{
    if (has_host_auth() == 0U) { return APP_PARAM_NO_AUTHORITY; }
    if (mode > 1U) { return APP_PARAM_OUT_OF_RANGE; }
    /* j_max=0 表示"不改"，合法；上限 200000 rad/s³（近似无 jerk 限制） */
    if (finite_f(j_max) == 0U || j_max < 0.0f || j_max > 200000.0f) {
        return APP_PARAM_OUT_OF_RANGE;
    }
    Servo_SetSpeedProfile(mode, j_max);
    return APP_PARAM_SUCCESS;
}

float AppParam_GetSpeedAccel(void) { return Servo_GetSpeedAccel(); }
void  AppParam_GetSpeedProfile(uint8_t *mode, float *j_max)
{
    Servo_GetSpeedProfile(mode, j_max);
}

float AppParam_GetTorqueIq(void) { return s_iq_target; }
float AppParam_GetSpeedW(void)  { return s_w_target; }
float AppParam_GetPosP(void)    { return s_p_target; }
void AppParam_GetMit(float *p, float *v, float *kp, float *kd, float *tau)
{
    if (p)   { *p   = s_mit_p; }
    if (v)   { *v   = s_mit_v; }
    if (kp)  { *kp  = s_mit_kp; }
    if (kd)  { *kd  = s_mit_kd; }
    if (tau) { *tau = s_mit_tau; }
}

AppParamStatus_t AppParam_ExecSysCmd(uint8_t cmd)
{
    if (has_host_auth() == 0U) { return APP_PARAM_NO_AUTHORITY; }

    switch (cmd) {
    case APP_SYS_CMD_NONE:
        return APP_PARAM_SUCCESS;
    case APP_SYS_CMD_SAVE_PARAM:
        /* 真写 Flash：Servo_ParamSaveNow 内部已做 MOE=0 / C 已标定 / Z 已对齐
         * 三道联锁，并从当前 Servo_* 状态打包完整 ParamBlob（不只是确认已有
         * blob）。齿槽表单独一页，随同落盘；任一前置不满足都返回状态错误。 */
        if (ServoCmd_IsArmed() != 0U || BspServo_IsPwmEnabled() != 0U) {
            return APP_PARAM_WRITE_STATE_ERROR;
        }
        if (Servo_ParamSaveNow() == 0U) { return APP_PARAM_WRITE_STATE_ERROR; }
        (void)CogFf_SaveToFlash();        /* 无有效齿槽表时内部自行跳过 */
        return APP_PARAM_SUCCESS;
    case APP_SYS_CMD_CLEAR_FAULT:
        if (ServoCmd_ClearFault() == 0U) { return APP_PARAM_WRITE_STATE_ERROR; }
        return APP_PARAM_SUCCESS;
    case APP_SYS_CMD_RESET:
        /* 软复位：由协议层在回发应答后执行（这里不直接复位，否则应答发不出去）。
         * 返回 SUCCESS 表示命令有效，实际复位由 MavHost 延迟触发。 */
        return APP_PARAM_SUCCESS;
    default:
        /* 2026-11-22：CAN 自检（20）是独立动作，不在 4..14 的辨识区间内，
         * 原来只判 4..14 会把它当非法动作号回 OUT_OF_RANGE。 */
        if (((cmd >= SC_ACT_CAL_CURRENT) && (cmd <= SC_ACT_ABORT_IDENT)) ||
            ((cmd >= SC_ACT_CAN_SELFTEST) && (cmd <= SC_ACT_CAN_TEST_STOP))) {
            uint8_t r = ServoCmd_ExecAction(cmd);
            if (r == SC_EXEC_ACCEPT)      { return APP_PARAM_SUCCESS; }
            if (r == SC_EXEC_NEED_DISARM) { return APP_PARAM_WRITE_STATE_ERROR; }
            return APP_PARAM_OUT_OF_RANGE;
        }
        return APP_PARAM_OUT_OF_RANGE;
    }
}

uint32_t AppParam_GetStatusWord(void)
{
    uint32_t w = 0U;
    if (ServoCmd_IsArmed() != 0U)          { w |= (1UL << 0); }  /* bit0 使能就绪 */
    if (BspServo_GetFault() != 0U)         { w |= (1UL << 1); }  /* bit1 故障 */
    if (PosLoop_IsEnabled() != 0U && PosLoop_IsSettled() != 0U) {
        w |= (1UL << 2);                                          /* bit2 目标到达 */
    }
    /* bit[5:3] = 当前实际在环模式（0=未使能 1=位置 3=速度 4=力矩 12=MIT），
     * 取 ISR 真实状态而非缓存，切环被联锁拒绝时上位机能立刻看出来。 */
    w |= ((uint32_t)ServoCmd_ActiveMode() & 0x0FUL) << 3;
    return w;
}

uint32_t AppParam_GetErrorWord(void)
{
    return (uint32_t)(BspServo_GetFault() & 0xFFFFUL);
}
