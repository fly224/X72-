/*
 * servo_isr.c —— 20kHz 电流环接线：ADC1/2 注入完成 -> 三环控制律 -> TIM1 占空比
 *
 * 中断链路（NVIC 优先级 0，高于 TIM1_UP_TIM16_IRQn 的 5）：
 *   TIM1 中心对齐 ARR=4250，OC4REF(Pulse=1) 产生 TRGO
 *     -> ADC1/ADC2 注入转换（各 1 通道，IN3）
 *     -> ADC1_2_IRQHandler -> HAL_ADC_IRQHandler(&hadc1/2)
 *     -> HAL_ADCEx_InjectedConvCpltCallback（本文件，弱函数覆盖）
 *     -> 两路都完成后执行 Servo_ControlTick()
 *
 * 不在 TIM1 更新中断里跑控制律：中心对齐下 UIF 上溢/下溢各一次，且需等注入转换完成才有新鲜电流。
 * 本文件不改 stm32g4xx_it.c，CubeMX 重新生成不会丢失接线。
 */
#include "bsp_servo.h"
#include "actuator.h"
#include "telemetry_tx.h"
#include "uart_link.h"
#include "three_loop_deploy.h"
#include "motor_ident.h"
#include "encoder_cal.h"
#include "fric_cal.h"
#include "mech_ident.h"
#include "motor_flux_ident.h"
#include "cog_ff.h"
#include "motor_rl_ident.h"
#include "can_proto.h"
#include "vel_profile.h"
#include "pos_loop.h"
#include "servo_cmd.h"
#include "mavlink_host.h"
#include "app_param.h"
#include "deadband_comp.h"
#include "cur_step.h"
#include "param_store.h"
#include "enc_comp.h"
#include "tim.h"
#include <math.h>
#include <string.h>

static volatile uint8_t  s_inj_flags = 0U;      /* bit0=ADC1, bit1=ADC2 */
static volatile uint32_t s_tick_count = 0U;
static volatile uint32_t s_inj_miss = 0U;
static volatile uint32_t s_max_cycles = 0U;
static volatile float    s_pos_ref = 0.0f;      /* 机械角 rad，**多圈**（与 theta_pos 同坐标系） */
static volatile float    s_pos_w_ff = 0.0f;     /* 位置环速度前馈 rad/s（速度轨迹模式下 = v_ref） */

/* ---- 手写位置环状态（2026-11-17）----
 * 声明必须放在 Servo_SetCurrentMode 之前：关电流环时要连带关位置环，
 * 否则下次使能会带着旧目标直接输出速度给定（等于绕过联锁上电）。
 */
/* ================= 控制状态机：给定源 =================
 * 原来用 5 个布尔（s_cur_en / s_spd_en / s_pos_en / s_mit_en / s_ol_en）表示
 * "当前在跑什么"，但它们其实是**分层**关系、不是互斥的平铺模式：
 *     位置环 -> 速度环 -> 电流环（下层必须开），MIT 是电流环的子模式。
 * 用布尔表达时，"该开哪几个"全靠 setter 的调用顺序和注释维持，已经出现过：
 *   · 速度环与位置环同时为 1 -> 位置环使能了但完全不生效（ISR 里速度优先）
 *   · en=1 而 moe=0 -> 含糊态，要从现象反推
 *   · MIT 装配失败仍报 active=12
 *
 * 现在改为**单一真值** s_src（给定源），5 个标志全部由它派生：
 *     TORQUE / MIT / SPEED / POS -> 电流环必须开
 *     SPEED / POS                -> 速度环必须开
 *     POS                        -> 位置环必须开
 * 于是"该开哪几个"成为恒等式，不再依赖调用顺序；两个给定源也不可能同时生效。
 * 派生是只读的（宏），任何地方都不能再单独置某个 *_en。 */
/* ServoSrc 枚举定义已移到 bsp_servo.h（对外可查询） */

static volatile ServoSrc s_src = SERVO_SRC_NONE;
static volatile uint8_t  s_src_seq;      /* 每次切换 +1，便于确认状态真的变了 */

static void src_enter(ServoSrc v)
{
    uint32_t primask;

    /* 临界区保护（2026-11-20）。
     *
     * s_src 会在两个上下文里被改写：
     *   · 主循环 —— 命令层看 SetXxxMode()
     *   · 1 ms 任务 —— MIT 超时、deadman、故障封锁
     * 32 位写在 M4 上是原子的，但"读 s_src 比较 → 写 → seq++"这三步不是：
     * 若 1 ms 任务在比较之后、写入之前插进来切到另一个值，主循环这次写入
     * 会把它覆盖掉（后写者赢），而且两次切换只留下一个 seq。
     * 状态机是"单一真值"，一次丢失的切换意味着实际在跑的环与 src 不符 ——
     * 那正是最难查的一类故障（S 行显示 SPEED，实际电流环在跟位置环的给定）。
     *
     * 只关中断十来个周期，且切换本身是低频事件（人手操作 / 超时），
     * 不会影响 20 kHz 电流环的节拍预算。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_src != v) {
        s_src = v;
        s_src_seq++;
    }
    __set_PRIMASK(primask);
}

#define s_ol_en   (s_src == SERVO_SRC_OPENLOOP)
#define s_cur_en  ((s_src == SERVO_SRC_TORQUE) || (s_src == SERVO_SRC_SPEED) || \
                   (s_src == SERVO_SRC_POS)    || (s_src == SERVO_SRC_MIT))
#define s_spd_en  ((s_src == SERVO_SRC_SPEED) || (s_src == SERVO_SRC_POS))
#define s_pos_en  (s_src == SERVO_SRC_POS)
#define s_mit_en  (s_src == SERVO_SRC_MIT)

ServoSrc    Servo_GetSrc(void)    { return s_src; }
uint8_t     Servo_GetSrcSeq(void) { return s_src_seq; }

const char *Servo_SrcStr(void)
{
    switch (s_src) {
    case SERVO_SRC_TORQUE:   return "TORQ";
    case SERVO_SRC_SPEED:    return "SPD";
    case SERVO_SRC_POS:      return "POS";
    case SERVO_SRC_MIT:      return "MIT";
    case SERVO_SRC_OPENLOOP: return "OL";
    default:                 return "NONE";
    }
}

/* 已由 s_src 派生，见文件下方状态机定义 */
static float             s_pos_vcmd;     /* 最近一拍速度给定（模型坐标系 rad/s） */
static uint8_t           s_pos_prev;

static volatile uint8_t  s_enable = 0U;
static uint32_t s_last_tick = 0U;

/*
 * 控制节拍时序探针（只在诊断行输出，不参与控制）。
 * 存在的意义：TIM1 中心对齐 ARR=4250 应产生 20 kHz 触发，
 * 但"设计值"不等于"实测值"——超时丢拍会让实际控制周期变成 2 倍，
 * 而模型里所有积分项都按 20 kHz 离散化，一旦变成 10 kHz 就等于
 * 全部时间常数翻倍。所以必须把真实周期量出来，不能靠推算。
 */
static volatile uint32_t s_adc1_irq = 0U;
static volatile uint32_t s_adc2_irq = 0U;
static uint32_t s_cyc_prev = 0U;
static uint32_t s_dt_min   = 0xFFFFFFFFU;
static uint64_t s_dt_sum   = 0U;
static uint32_t s_dt_cnt   = 0U;

/* 控制节拍分段峰值（cycles）：定位 20 kHz 预算的去向 */
static uint32_t s_max_theta = 0U, s_max_readinj = 0U, s_max_step = 0U;
static uint32_t s_max_duty  = 0U, s_max_enc     = 0U;

#define UPD_MAX(v, x)   do { uint32_t _x = (x); if (_x > (v)) { (v) = _x; } } while (0)

/* 速度模式：斜坡 -> 位置给定的 1 kHz 状态机 */
static vel_profile_t     s_vel;
static volatile uint8_t  s_mode = SERVO_MODE_POSITION;

/* 供 100 Hz 遥测取用的最近一拍快照（Cortex-M4 上 32 位对齐读写是原子的） */
static volatile float    s_iu = 0.0f, s_iv = 0.0f, s_iw = 0.0f;
static volatile float    s_iq_ref = 0.0f;
static volatile float    s_duty[3] = { 0.5f, 0.5f, 0.5f };

void Servo_SetPosRef(float rad)  { s_pos_ref = rad; }

void Servo_SetEnable(uint8_t en)
{
    s_enable = en ? 1U : 0U;

    /* 封锁时同步把给定源归 NONE（2026-11-20）。
     *
     * 原来这里只写 s_enable，s_src 交给各 SetXxxMode(0) 去清。问题是
     * 只要有一条路径只调了 SetEnable(0) 而没调对应的 SetXxxMode(0)，
     * 就会留下"en=0 但 src=SPD"的含糊态：S 行看着速度环还在、输出却是封锁的，
     * 现场判断不出是"没使能"还是"使能了但环没开"。
     * 11-18 那次"en=1 而 moe=0 要从现象反推"就是同一类问题。
     *
     * 不动 OPENLOOP：辨识（+ / @ / < 等）用开环跑，它们自己开关 PWM，
     * 与 s_enable 无关；清掉会把辨识中途打断。 */
    if ((s_enable == 0U) && (s_src != SERVO_SRC_OPENLOOP)) {
        src_enter(SERVO_SRC_NONE);
    }
}

/* ---------------- 开环 dq 电流闭环（对齐用） ---------------- */
#define SERVO_OL_IDREF_MAX   (5.0f)         /* id 给定上限，A */
#define SERVO_OL_KP          (0.5341f)  /* 跟随正式电流环 Kp（2026-11-17 由 0.713 按实测 L 重算） */
#define SERVO_OL_KI_TS       (0.04f)        /* 模型是 0.119（Ki*Ts）：对齐时放慢 3 倍，
                                             * 否则 2 A 误差 2 ms 就把电压顶到限幅 */
#define SERVO_OL_VLIM_FRAC   (0.06f)        /* 电压限幅 0.06*vbus（约 1.85 V @30.8 V）
                                             *
                                             * 2026-11-20 从 0.20 降下来。原来留 7 倍余量
                                             * 的后果是：Z 对齐时虚拟角与转子磁极差 90°，
                                             * 1.5 A 的 d 轴电流全落到 q 轴，转子被加速到
                                             * 几千 rpm，BEMF 叠上来后电流环只有 6 V 可顶，
                                             * 6 V / 0.4 Ω = 15 A —— 实测峰值 −16.5 A，
                                             * 超过 12 A 阈值直接 IDENT FAIL fault_run。
                                             *
                                             * 1.85 V 对应 4.6 A 上限，既留 3 倍于 1.5 A 稳态
                                             * 所需（0.6 V）的余量，又把失控时的电流关在
                                             * 过流阈值以下。对齐不会再因为"顶到电压限幅"
                                             * 而跳闸，真正的短路仍然由 BSP 的 12 A 兜住。 */
#define SERVO_OL_TS          0.00005f

/* 已由 s_src 派生，见文件下方状态机定义 */
static volatile float   s_ol_theta;
static volatile float   s_ol_idref;

/* ---- 运行时调参量（2026-11-20 从编译期常量改过来）----
 *
 * 起因：Z 对齐反复失败，改一次注入电流就要改常量、重编译、烧录，
 * 而每次烧录又会擦掉参数页（cal/z/psi 全归零），形成死循环。
 * 这类"现场要试几个值才知道合不合适"的量必须能在运行时改。
 *
 * 通过 MAVLink 30082 AppDebugParam 的 Debug_float[0..15] 读写，
 * 索引表见 mavlink_host.c 的 s_dbg_param[]。越界一律拒绝，不静默截断。 */
#define SERVO_ALIGN_ID_MIN   (0.2f)
#define SERVO_ALIGN_ID_MAX   (4.0f)
#define SERVO_OL_VLIM_MIN    (0.02f)
#define SERVO_OL_VLIM_MAX    (0.60f)

static volatile float   s_align_id  = 2.0f;   /* Z 对齐注入的 d 轴电流 A */
static volatile float   s_ol_vlim   = 0.06f;  /* 开环电压限幅系数（×vbus） */

uint8_t Servo_SetAlignId(float a)
{
    if (a != a) { return 0U; }
    if ((a < SERVO_ALIGN_ID_MIN) || (a > SERVO_ALIGN_ID_MAX)) { return 0U; }
    s_align_id = a;
    return 1U;
}
float Servo_GetAlignId(void) { return s_align_id; }

uint8_t Servo_SetOLVlim(float f)
{
    if (f != f) { return 0U; }
    if ((f < SERVO_OL_VLIM_MIN) || (f > SERVO_OL_VLIM_MAX)) { return 0U; }
    s_ol_vlim = f;
    return 1U;
}
float Servo_GetOLVlim(void) { return s_ol_vlim; }
/* 开环匀速拖动（编码器自校准用）：在 20 kHz ISR 里推进虚拟角。
 * 为什么必须在 ISR 推进而不是 1 ms 任务：600 rpm 对应 140 Hz 电频率，
 * 1 ms 更新一次等于每拍跳 50° 电角度，磁场是"跳"的不是"转"的，
 * 芯片无法在这种激励下完成自校准。20 kHz 下每拍只走 2.5°，足够平滑。 */
static volatile uint8_t s_ol_spin;
static volatile float   s_ol_w_e;        /* 电角速度 rad/s */
static uint8_t          s_ol_prev;
static float            s_ol_vd_int;
static float            s_ol_vq_int;
static volatile float   s_ol_vd;       /* 最近一拍限幅后的 dq 电压（R 辨识读） */
static volatile float   s_ol_vq;

/* 单频注入状态 */
static volatile uint8_t s_olv_en;
static float            s_olv_amp = 1.5f;
static float            s_olv_ph;
static float            s_olv_dph;

void Servo_SetOpenLoop(uint8_t enable, float theta_e, float id_ref_a)
{
    float r = id_ref_a;

    if (r < 0.0f) { r = 0.0f; }
    if (r > SERVO_OL_IDREF_MAX) { r = SERVO_OL_IDREF_MAX; }
    s_ol_theta = theta_e;
    s_ol_idref = r;

    /* 只在 0->1 的跳变时清积分。
     * 之前写成"每次调用都清"，结果做电流斜坡时（每 1 ms 调一次）积分永远建不起来。 */
    if ((enable != 0U) && (s_ol_prev == 0U)) {
        s_ol_vd_int = 0.0f;
        s_ol_vq_int = 0.0f;
    }
    s_ol_prev = (enable != 0U) ? 1U : 0U;
    if (enable != 0U) {
        src_enter(SERVO_SRC_OPENLOOP);
    } else if (s_src == SERVO_SRC_OPENLOOP) {
        src_enter(SERVO_SRC_NONE);
    }
    if (enable == 0U) { s_ol_spin = 0U; }   /* 退出开环时一并停拖动 */
}

uint8_t Servo_IsOpenLoop(void) { return s_ol_en; }

/* 当前虚拟电角度（已 wrap 到 [0, 2π)）。
 * 连续拖动式辨识要用它算"命令电角 − 实测机械角×极对数"的相位差；
 * 自己按 ωe·t 积分会和 ISR 内部的累加产生缓慢漂移（20 kHz 与 1 ms 的
 * 舍入不同），直接读真值就没有这个问题。 */
float Servo_GetOLTheta(void) { return s_ol_theta; }

/* 开环匀速拖动开关。en=0 只停"自转"，开环本身仍由 Servo_SetOpenLoop 控制。
 * 注意：开启后**不要**再从 1 ms 任务调 Servo_SetOpenLoop()，它会覆盖 s_ol_theta
 * 把角度重置回起点；角度由 20 kHz ISR 自行累加。 */
void Servo_SetOLSpin(uint8_t en, float w_elec)
{
    if (w_elec != w_elec) { return; }
    if (w_elec >  2000.0f) { w_elec =  2000.0f; }
    if (w_elec < -2000.0f) { w_elec = -2000.0f; }
    s_ol_w_e  = w_elec;
    s_ol_spin = (en != 0U) ? 1U : 0U;
}

uint8_t Servo_IsOLSpinning(void) { return s_ol_spin; }

/* ---------------- 单频电压注入（测 L 用） ----------------
 * 为什么需要它：X72 的电流时间常数 L/R ≈ 300 us，在 20 kHz 采样下整个阶跃
 * 上升只有 4~6 拍，逐点差分算 di/dt 会被噪声完全淹没（实测 spread 71%）。
 * VESC 用 HFI（多矢量高频注入 + FFT）就是为了绕开这个限制：在高频下阻抗是
 * ωL 而不是 R，量级相当，且不依赖时域微分的分辨率。
 *
 * 这里做最简等效：单频 ω = 2π·1 kHz（ωL ≈ 0.71 Ω，与 R ≈ 0.40 Ω 同量级），
 * 在虚拟角 0 的 d 轴注入正弦电压，采样 3 个整周期（60 拍），
 * 用单点 DFT（正交解调）取电流基波幅度：
 *     I_amp = 2/N · sqrt((Σ i·cos)² + (Σ i·sin)²)
 *     |Z|   = V_amp / I_amp
 *     L     = sqrt(|Z|² − R²) / ω
 * 相位固定 0、q 轴电压 0，所以 id 注入不产生转矩，转子不会动。
 */
void Servo_SetVoltInject(uint8_t enable, float amp_v, float freq_hz)
{
    float a = amp_v;

    if (a < 0.2f)  { a = 0.2f; }
    if (a > 3.0f)  { a = 3.0f; }
    if (freq_hz < 200.0f)   { freq_hz = 200.0f; }
    if (freq_hz > 2000.0f)  { freq_hz = 2000.0f; }

    s_olv_amp = a;
    /* 相位增量：每拍前进 2π·f·Ts，Ts = 50 us */
    s_olv_dph = BSP_TWO_PI * freq_hz * BSP_TS_CURRENT;
    if (enable != 0U) {
        s_olv_ph = 0.0f;
    }
    s_olv_en = (enable != 0U) ? 1U : 0U;
}

uint8_t Servo_IsVoltInject(void) { return s_olv_en; }

void Servo_GetOpenLoopVdq(float *vd, float *vq)
{
    if (vd) { *vd = s_ol_vd; }
    if (vq) { *vq = s_ol_vq; }
}

/* ---------------- 编码器方向符号（辨识结果） ----------------
 * 只影响"送进模型"的 theta_m / omega_m，不碰 BSP 内部的原始角与多圈累积：
 * 辨识过程本身必须读原始坐标（否则自己的符号会把自己绕进去）。
 */
static int8_t s_dir_sign = 1;

void   Servo_SetDirSign(int8_t dir) { s_dir_sign = (dir < 0) ? -1 : 1; }
int8_t Servo_GetDirSign(void)       { return s_dir_sign; }

/* ---------------- 编码器电角度下的 dq 电流闭环（力矩模式） ----------------
 *
 * 与上面的"虚拟角"版本共用同一套 PI，区别只有两点：
 *   1. θ 来自编码器（必须先做过 Z 辨识），定义与 three_loop_deploy.c:51 完全一致：
 *        th_e = 14*(dir*theta_m) + π/2 + 1.005e-3*(dir*omega_m)
 *      其中 π/2 是模型自带的库角偏移，1.005e-3 = 14*1.5*Ts（延时补偿，Ts=50us）。
 *   2. 增加了交叉耦合解耦与 BEMF 前馈（和模型 par_c[9]=3 时一样），
 *      否则转速升高后 BEMF 会全部压给积分器，带宽被吃掉。
 *
 * 参数取自模型（20 kHz），2026-11-17 按**实测**参数重算：
 *   Ld = Lq = 85 µH   （`@` 在线辨识 80.1–80.7 µH @1.55 A，ψ 辨识 89.07 µH；
 *                        原值 113.5 µH 是设计值，比实测大 25%，导致实际带宽
 *                        跑到 1335 Hz 而不是设计的 1000 Hz）
 *   ψ  = 4.333 mWb    （`+` 联合辨识，dev=1.85%；原值 7.593 是实测的 1.75 倍，
 *                        BEMF 前馈在 1000 rpm 时给 11.13 V 而实际只需 6.35 V）
 *   Kp = 0.534 [V/A]  = ωc·Ld，ωc=2π·1000
 *   Ki·Ts = 0.12566   = (R/L)·Kp·Ts = (0.4/85e-6)·0.534·50e-6，保证零极点对消
 *                       （原 0.1189 是按 113.5 µH 算的，换成实测 L 后零点比极点低 29%）
 *   电压圆限幅 = 0.57735×0.95×vbus×0.98 ≈ 0.5375·vbus
 */
#define SERVO_CUR_KP         (0.5341f)
#define SERVO_CUR_KI_TS      (0.12566f)
#define SERVO_CUR_VLIM_FRAC  (0.5375f)      /* 0.57735*0.95*0.98：与模型 par_c[6..8] 一致 */

/* 比例式退饱和系数。0.3 对应跟踪时间常数 Ts/Kb ≈ 167 µs，
 * 比电流环自身的响应（~200 µs）略快，超限后能及时把积分拉回来又不会振。
 * 与速度环的 s_spd_kb=0.30 保持一致。 */
#define SERVO_CUR_KB         (0.30f)
#define SERVO_CUR_IQ_MAX     (5.0f)

/* 力矩常数 Kt = 1.5·p·ψ，用**实测** ψ=4.333 mWb（`+` 联合辨识，dev=1.85%）
 * 算得 0.0910 N·m/A。与 mech_ident.c 的 MI2_KT 必须一致，否则同一台机器
 * 在"力矩模式输出多少 N·m"和"惯量辨识假设多少 Kt"两处会自相矛盾。 */
#define SERVO_KT_NM_PER_A    (0.0910f)

/* ---------------- Kt 温度补偿（2026-11-19 实测 ψ 温漂） ----------------
 *
 * 实测依据（`+` FluxIdent，升温室 15 点 44.8→50.3 °C + 冷却段 10 点 50.2→41.0 °C）：
 *   α_ψ = -0.1197 %/K   （升温室 -0.1190 / 冷却段 -0.1203，两段独立测得差 1.1%）
 *   ψ20 = 4.390 mWb，与钕铁硼永磁 -0.11~-0.13 %/K 吻合
 * Kt = 1.5·p·ψ 与 ψ 同比例，故 Kt(T) = Kt20·(1 + α·(T−20))。
 *
 * 只改"力矩换算"三处：
 *   ① MIT 的 τ→iq（否则同样 τ 在 50 °C 只出 96.4% 力矩）
 *   ② MIT 的 τ 回读（与实际一致，不能只改一边）
 *   ③ Servo_GetTorqueEst()（力矩显示）
 *
 * 2026-11-20 追加第四处（原实现漏掉，属于模型不自洽）：
 *   ④ 电流环 BEMF 前馈 dec_vq 的 ωe·ψ 项 —— 它与 Kt 用的是同一个 ψ，
 *      只补 Kt 不补前馈，等于"力矩算对了、电压算错了"。
 *      84 °C 时两条通道差 7.7%（ktc≈0.923），2000 rpm 下前馈多给
 *      ωe·Δψ ≈ 0.98 V，由 vq 积分反向顶回来，积分因此长期带偏置。
 *      见 dec_vq 处注释与《参数温漂对环路的影响与补偿_20261120.md》。
 *
 * **不改**：
 *   · 速度/位置环 —— 它们用 iq 闭环，Kt 只决定"多少 iq 出多少力矩"，不影响稳态转速
 *     （2026-11-19 一小时温漂实测：17 K 温升下速度误差 +0.020 rpm，R²=0.011）
 *   · 加速度/摩擦前馈系数 —— 编译期 A 单位常量，3.6% 误差由速度环 PI 吸收，
 *     动态改它们反而会引入新的整定风险
 *
 * 基准仍用 0.0910（即 ψ=4.333 mWb）。本次实测 ψ20=4.390 高 1.3%，
 * 在 25 K 外推的不确定度内，暂不改基准以免影响已标定的 J 与前馈。
 *
 * 安全：NTC 开路/短路时驱动层返回 NaN → 保持上次值不动；限幅 ±10% 防野值。 */
#define SERVO_KT_ALPHA       (-0.001197f)   /* 1/K，实测 α_ψ */
#define SERVO_KT_T_REF       (20.0f)        /* °C，基准温度 */
#define SERVO_KT_COMP_MIN    (0.90f)
#define SERVO_KT_COMP_MAX    (1.10f)

static volatile float   s_kt_comp    = 1.0f;
static volatile uint8_t s_kt_comp_en = 1U;      /* 默认开启 */
/* ψ 辨识时的电机温度（补偿基准）。s_psi_t_ok=0 表示未知，退回 20 °C 基准。
 * 由 Servo_SetIdentPsiEx() 写入，掉电保存见 param_store 的 PS_FLAG_PSI_T。 */
static volatile float   s_psi_t_ref  = SERVO_KT_T_REF;
static volatile uint8_t s_psi_t_ok;

/* 死区占空比：DTG=50 × tDTS(5.88 ns) = 294 ns，占 50 µs 周期的 0.59% */
#define SERVO_CUR_DEAD_FRAC  (0.0059f)
#define SERVO_CUR_WMAX       (31.415927f)   /* 电机侧 300 rpm：力矩模式没有速度环，空载会一直加速。
                                             * 按 J=5e-4/Tf=0.03 估算，1 A 空载约 120 ms 到此值，
                                             * 足够 100 Hz 遥测抓到 10 帧以上；触发后置 f=0x20 并封 MOE。 */
/* 开环拖动的超速门限必须单独给：编码器自校准要拖到 600 rpm（62.8 rad/s），
 * 直接用 SERVO_CUR_WMAX(300 rpm) 会在起拖瞬间就判超速。
 * 开环磁场是强制给定的，不存在"速度环正反馈飞车"，门限放宽到 ~1900 rpm
 * 只用于兜住"转子完全失步、电流失控"这种极端情况。 */
#define SERVO_OL_WLIM        (200.0f)
#define SERVO_CUR_POLE       (BSP_POLE_PAIRS)   /* 定义在 bsp_servo.h，与模型 14.0F 一致 */
#define SERVO_CUR_ELEC_OFF   (1.57079637f)  /* π/2，模型 theta_off */
#define SERVO_CUR_COMP       (1.05e-3f)     /* 14*1.5*Ts = 14*1.5*50us = 1.05e-3，模型 gain_comp */
#define SERVO_CUR_LD         (85.0e-6f)     /* 实测（原 113.5e-6 为设计值，大 25%） */
#define SERVO_CUR_LQ         (85.0e-6f)
#define SERVO_CUR_PSI        (4.333e-3f)    /* 实测 ψ（原 7.593e-3 大 1.75 倍）
                                             * —— 现仅作**上电默认值**：
                                             * 辨识成功后由 Servo_SetIdentPsi() 覆盖 */

/* ---- 辨识参数接管控制律（P0-1）----
 * 之前这四个量是纯编译期常量，导致 @ / + 测出来的 R/L/ψ 只能回显、不影响行为，
 * 调参必须改代码重烧。现在改为运行时变量：
 *   ψ  -> Kt 与 BEMF 前馈（dec_vq 的主导项）
 *   L  -> dq 解耦
 *   R  -> 仅保存与回显（是否用它重算电流环 Ki 由 Servo_SetIdentKiEn 单独控制）
 * 默认值等于原常量，所以上电行为与改动前逐位一致；只有辨识成功后才改变。
 * 越界值一律拒绝（野值保护），保证不会因为一次坏辨识把环路带偏。 */
#define SERVO_ID_L_MIN       (10.0e-6f)
#define SERVO_ID_L_MAX       (300.0e-6f)
#define SERVO_ID_PSI_MIN     (2.0e-3f)
#define SERVO_ID_PSI_MAX     (8.0e-3f)
#define SERVO_ID_R_MIN       (0.05f)
#define SERVO_ID_R_MAX       (2.0f)

static volatile float   s_cur_ld  = SERVO_CUR_LD;
static volatile float   s_cur_lq  = SERVO_CUR_LQ;
static volatile float   s_cur_psi = SERVO_CUR_PSI;
static volatile float   s_kt_base = SERVO_KT_NM_PER_A;
static volatile float   s_id_r    = 0.0f;
static volatile uint8_t s_id_psi_ok;
static volatile uint8_t s_id_rl_ok;

/* 已由 s_src 派生，见文件下方状态机定义 */
static volatile float   s_cur_iq_ref;       /* 目标给定（命令层写） */

/* ---- 电角度补偿（rad，叠加在电流环 Park 变换的 th_e 上）----
 * 它**不是**零点补偿：零点由 Z 写进 BspServo_SetEncoderZero()。
 * 这里只做现场排查用的偏置——扫这个值找"正反向都出力"的位置：
 *   若某个偏置能让正反向都正常出力 => 只是零点错，重做 Z 即可；
 *   若扫遍整圈都出不了力         => 编码器角度随位置失真，须重标/更换。
 *
 * 2026-11-19 背景（当时判定的反复过程，勿再重复踩坑）：
 *   · 一度用 "30040 的 elec_angle − 14×enc_sum 机械角" 算偏差，得到 −86.6°。
 *     该算法是错的：30040 的 +8 是 14*theta_m（theta_m 已含 s_enc_zero），
 *     而 enc_sum 是原始计数，两者本就相差 14*zero，不能相减当误差。
 *   · Z 收敛后 off≈92.9°（目标 π/2=90°），即 θv ≡ 14θm + π/2 在平均意义上成立；
 *     ratio 也回到 14.0~14.2，所以"斜率"没错。
 *   · 但 1 A 电流下正反向都不出转矩（idq 跟踪准确、转速恒 0），
 *     +iq 还会因角度跳变触发 f=0x20。=> 位置相关的角度误差。
 *
 * 当前值保持 0：非 0 只用于 `-<deg>` 命令的临时扫描，掉电即失效。 */
#define SERVO_ELEC_OFF_COMP  (0.0f)
#define SERVO_ELEC_OFF_LIM   (3.15f)     /* ±π，够扫一整圈电角度 */
static volatile float   s_elec_off = SERVO_ELEC_OFF_COMP;

void Servo_SetElecOff(float rad)
{
    if (rad != rad) { return; }                    /* NaN：忽略 */
    if (rad >  SERVO_ELEC_OFF_LIM) { rad =  SERVO_ELEC_OFF_LIM; }
    if (rad < -SERVO_ELEC_OFF_LIM) { rad = -SERVO_ELEC_OFF_LIM; }
    s_elec_off = rad;
}

float Servo_GetElecOff(void) { return s_elec_off; }
static volatile float   s_cur_id_ref;       /* id 给定：正常为 0，整定时由阶跃模块接管 */
static volatile float   s_cur_iq_max   = SERVO_CUR_IQ_MAX;   /* 力矩模式 iq 限幅，可运行时改 */
/* 温度降额系数（1.0 = 不限流）。由 1 ms 任务从 BspServo_GetDerate() 搬过来，
 * 只在本文件内读，避免 20 kHz 里跨模块调用。 */
static volatile float   s_derate      = 1.0f;
static float            s_cur_iq_ramp;      /* 斜坡后的 iq 给定（20 kHz 更新） */
static volatile float   s_cur_iq_tau   = 0.020f;             /* 一阶低通时间常数 s，0 = 直通 */
static volatile float   s_cur_dead_gain;    /* 死区补偿增益，0 = 关闭，1 = 理论值 */
static int8_t           s_cur_dead_sign;    /* 死区补偿的电流方向（带迟滞） */
static volatile float   s_cur_kp     = SERVO_CUR_KP;      /* 可运行时改：调 PI 用 */
static volatile float   s_cur_ki_ts  = SERVO_CUR_KI_TS;
static uint8_t          s_cur_prev;
static float            s_cur_vd_int;
static float            s_cur_vq_int;
static volatile float   s_cur_id;
static volatile float   s_cur_iq;

/* ================= MIT 模式（阻抗控制） =================
 *   τ = kp·(p_des − p) + kd·(v_des − v) + τ_ff   →   iq = τ / Kt
 *
 * 为什么单独做一个模式，而不是拼速度环 + 力矩前馈：
 *   MIT 的语义是"给定刚度/阻尼，让外部负载决定位置"，位置误差本身不是故障
 *   （被推走是正常的）。层叠的位置环会把这种偏差当成误差去积分，反而顶回去。
 *
 * 打包格式沿用 MIT Mini Cheetah（8 字节：位置 16 bit，速度/kp/kd/力矩各 12 bit），
 * 解包在 servo_cmd.c，将来接 CAN 时直接复用同一份 unpack。
 *
 * 位置用**多圈角**：跨圈不会 2π 突跳。误差再做 ±EP_MAX 限幅 ——
 * kp 可能给到几千，p_des 打错一位就是满力矩冲出去。
 */
/* 已由 s_src 派生，见文件下方状态机定义 */
static volatile float   s_mit_p;          /* 目标位置 rad（用户坐标系，多圈） */
static volatile float   s_mit_v;          /* 目标速度 rad/s（用户坐标系） */
static volatile float   s_mit_kp;         /* N·m/rad */
static volatile float   s_mit_kd;         /* N·m·s/rad */
static volatile float   s_mit_tau;        /* 前馈力矩 N·m */
static volatile float   s_mit_tau_out;    /* 上拍实际输出力矩（诊断） */
static volatile float   s_mit_ep;         /* 上拍位置误差（限幅后，诊断） */
static volatile float   s_mit_p_now;      /* 上拍实测位置（多圈，诊断/上位机算相对目标） */
static volatile float   s_mit_p_shape;    /* 整形后的目标（限速斜坡，见 SHAPE_VMAX） */
static volatile float   s_mit_v_shape;    /* 整形目标的速度 rad/s（带加速度限幅） */
static volatile uint16_t s_mit_tmo;       /* 距上一帧的毫秒数，超时冻结 v_des/τ_ff */
/* MIT 专用测速：2 拍差分。
 * 为什么不用现成的两个：
 *   om（8 拍 + 95 Hz 低通）群延迟 1.9 ms → 244 Hz 处 167°，D 项变负阻尼，
 *      实测纯阻尼 kd=0.5 就起 5 A 极限环；
 *   GetOmegaFast（单拍差分）被编码器 SPI 的异步更新放大成 0/2v 隔拍跳动，
 *      实测极限环跑到 742 Hz、4 A。
 * 2 拍差分正好跨过"隔拍"的一个完整周期（两拍总和恒为 2v），
 * 窗口 100 us、群延迟 50 us —— 254 Hz 处 9°、742 Hz 处 27°。 */
static float s_mit_th1;
static float s_mit_th2;
static volatile uint8_t s_mit_vinit;
/* 给定外推用的时间戳：20 kHz 自增计数。用 ms 计数不够 —— 1 kHz 帧间隔本身
 * 就是 1 ms，1 ms 分辨率根本分辨不出帧龄。uint32 在 20 kHz 下 59.6 h 回绕，
 * 无符号减法在回绕点仍然正确。 */
static volatile uint32_t s_mit_tick;
static volatile uint32_t s_mit_stamp;     /* 最近一帧到达时的 tick */
static volatile float    s_mit_dt;        /* 上拍实际用到的帧龄（s），诊断 */

#define SERVO_MIT_EP_MAX     (1.0f)       /* 位置误差限幅 rad */
/* 命令超时：MIT 的 p_des/v_des 只在帧里给。上位机一旦停发（哪怕只是丢了几帧），
 * 最后一帧的 v_des 会一直被 kd 项执行 —— 实测收尾时 v_des=3.14 卡住，电机被
 * 持续推着走。deadman 是 2 s，对"按速度给定运行"这种模式太慢。
 * 这里 20 ms 内没有新帧就把 v_des 与前馈清零，位置给定保持 —— 等效于
 * "松手后停在原地"，这正是 MIT 该有的行为。 */
#define SERVO_MIT_CMD_TMO_MS (20U)
#define SERVO_MIT_TAU_MAX    (1.5f)       /* 力矩限幅 N·m（≈16 A，实际先被 iq 限幅兜住） */
#define SERVO_MIT_KP_MAX     (2000.0f)
/* MIT 的超速门限独立：不能在 v_des 之外只加 10 rad/s。
 *
 * MIT 的力矩是 kp·e，位置阶跃时 e 一下子就是几十毫弧度，即使 kp 只有 3，
 * 对应加速度也是 J 决定的几千 rad/s²，峰值速度轻松超过 10 rad/s。
 * 实测：保持时改目标 0.083 rad、kp=25，0.1 s 内就撞上 10 rad/s 门限，
 * f=0x20 封锁 —— 看上去像"MIT 一改目标就跳闸"。
 *
 * 30 rad/s ≈ 286 rpm 是关节类应用的合理上限：正常 MIT 运动够用，
 * 真失控（编码器断线、dir 判错）仍会撞线。
 */
#define SERVO_MIT_WMAX       (30.0f)
#define SERVO_MIT_KD_MAX     (50.0f)

/* ---- 目标整形：把阶跃变成限速斜坡（2026-11-22）----
 *
 * 为什么需要：MIT 没有轨迹规划器，p_des 一帧跳多少，kp 就出多少力矩。
 * 实测事故（10:59:18）：保持 p=3.1867 rad，上位机给 p_des=6.2800 rad，
 * 误差 3.09 rad → 被 EP_MAX 限到 1 rad → τ=30 N·m → iq 顶 5 A →
 * 空载 3.6 ms 就到 30 rad/s → 0x0020 超速。位置模式同一个目标会走梯形规划，
 * MIT 不会 —— 这是语义差异，不是参数问题。
 *
 * 做法：控制律不再直接用 s_mit_p，而是用一个按 SERVO_MIT_SHAPE_VMAX 限速
 * 逼近它的整形值 s_mit_p_shape：
 *   · 匀速段 p_des 以恒定速度移动，稳态误差只有 τ_fric/kp ≈ 1.7 mrad，
 *     力矩很小，不会顶 iq 限幅；
 *   · 正常轨迹（每帧 p 连续变化）远低于限速，完全不受影响；
 *   · 若 v_des 本身就很大，限速自动放宽到 1.2·|v_des|，不拖慢用户明确要的高速；
 *   · kp=0 阶段（盲起环）整形器直接跟随实际位置，不预跑 —— 否则上刚度那一拍
 *     目标已经跑到别处，等于人为制造阶跃。
 *
 * 置 0 可关闭，回到"p_des 直接执行"（复现老行为用）。
 */
#ifndef SERVO_MIT_SHAPE_EN
#define SERVO_MIT_SHAPE_EN (1)            /* 0 = 关闭整形，回到 p_des 直接执行 */
#endif
#ifndef SERVO_MIT_SHAPE_VMAX
#define SERVO_MIT_SHAPE_VMAX (15.0f)      /* rad/s ≈ 143 rpm，整形目标的最大速度 */
#endif
/* 加速度上限 = VMAX × 这个比例。只限速度不限加速度时，起步那一拍 v 从 0 跳到
 * 15 rad/s，加速度是无穷大 —— 实测 iq 峰 4.47 A（接近 5 A 限幅）、rpm 峰 149.9。
 * 加上 amax 后起步变成 83 ms 的斜坡，iq 峰回落到 1 A 以内。 */
#ifndef SERVO_MIT_SHAPE_A_RATIO
#define SERVO_MIT_SHAPE_A_RATIO (12.0f)   /* amax = 12×VMAX ≈ 180 rad/s² */
#endif
/* 位置差 → 期望速度的比例（1/s）。12 对应 83 ms 时间常数，配合上面的 amax
 * 是一条"加速—匀速—减速"的准梯形曲线，不会像纯限速那样冲到位再刹。 */
#ifndef SERVO_MIT_SHAPE_KP
#define SERVO_MIT_SHAPE_KP (12.0f)
#endif

/* ---- 给定外推（一阶保持，2026-11-18）----
 *
 * 问题：MIT 帧率约 1 kHz，而控制在 2 kHz / 20 kHz 跑。帧之间 p_des 是常数
 * （零阶保持），等于目标"每 1 ms 跳一步"。轨迹跟踪时这个台阶就是**纯滞后**：
 * 实测 2 Hz、峰值速度 10 rad/s 的轨迹，误差 54 mrad，其中 50 mrad 正好是
 * 5 ms（帧间隔 + 串口 + 遥测）乘 10 rad/s。
 *
 * 做法：帧里已经有 v_des，直接用它在帧间隔内线性外推目标
 *     p_des(t) = p_frame + v_des·(t − t_frame)
 * 这一项不需要额外传感器，也不放大噪声 —— v_des 本来就是上位机给的期望速度。
 *
 * 两个必须处理的边界：
 *   ① 帧龄从 20 kHz 计数取，不用 ms 计数（1 ms 分辨率对 1 kHz 帧太粗）；
 *   ② 超时时**把已经外推出去的位移固化进 s_mit_p**，否则 v_des 清零会让
 *      目标"回弹"，力矩反向冲击一下。
 */
#define SERVO_MIT_EXTRAP_MAX_S (0.1f)     /* 外推最多 100 ms，防帧停后目标飞走 */

/* A/B 用编译期开关：默认开。置 0 重烧即可对比"零阶保持 vs 一阶外推"，
 * 接口上不占字符（可打印字符已全部占满），也不需要上位机配合。 */
#ifndef SERVO_MIT_EXTRAP_EN
#define SERVO_MIT_EXTRAP_EN  (1)
#endif

void Servo_SetMitEn(uint8_t en)
{
    if (en == 0U) {
        if (s_src == SERVO_SRC_MIT) { src_enter(SERVO_SRC_NONE); }
        s_mit_tau_out = 0.0f;
        s_mit_ep      = 0.0f;
        return;
    }
    /* 进入时清诊断量；不给积分器（MIT 没有积分项，残留的 t 由上层每帧给） */
    s_mit_tau_out = 0.0f;
    s_mit_ep      = 0.0f;
    s_mit_vinit   = 0U;          /* 进入时重建测速历史，避免用上次的位置 */
    /* 整形器必须从**当前实际位置**起步：盲起环第一帧常给 p_des=0，
     * 若整形器也从 0 起，上刚度那一拍就成了"从 0 追到实际角"的满力矩阶跃。 */
    s_mit_p_shape = BspServo_GetThetaMulti();
    s_mit_v_shape = 0.0f;
    src_enter(SERVO_SRC_MIT);    /* 电流环由 src 派生，不需要显式开 */
}

void Servo_SetMitCmd(float p, float v, float kp, float kd, float tau)
{
    if (!(p == p)) { return; }              /* NaN 帧直接丢弃，别污染给定 */
    if (!(v == v)) { v = 0.0f; }
    if (!(kp == kp)) { kp = 0.0f; }
    if (!(kd == kd)) { kd = 0.0f; }
    if (!(tau == tau)) { tau = 0.0f; }

    if (kp < 0.0f) { kp = 0.0f; }
    if (kp > SERVO_MIT_KP_MAX) { kp = SERVO_MIT_KP_MAX; }
    if (kd < 0.0f) { kd = 0.0f; }
    if (kd > SERVO_MIT_KD_MAX) { kd = SERVO_MIT_KD_MAX; }
    if (tau >  SERVO_MIT_TAU_MAX) { tau =  SERVO_MIT_TAU_MAX; }
    if (tau < -SERVO_MIT_TAU_MAX) { tau = -SERVO_MIT_TAU_MAX; }

    s_mit_p   = p;
    s_mit_v   = v;
    s_mit_kp  = kp;
    s_mit_kd  = kd;
    s_mit_tau = tau;
    s_mit_stamp = s_mit_tick;       /* 外推起点：这一帧的到达时刻 */
    s_mit_tmo = 0U;                 /* 收到帧就重置命令超时 */
}

/* 1 ms 调一次：帧停了就冻住速度给定，只保留位置刚度 */
void Servo_MitTask1ms(void)
{
    if (s_mit_en == 0U) { s_mit_tmo = 0U; return; }
    if (s_mit_tmo < 0xFFFFU) { s_mit_tmo++; }
    if (s_mit_tmo > SERVO_MIT_CMD_TMO_MS) {
        /* 把已经外推出去的那段位移**固化**进 s_mit_p，再清 v。
         * 直接清 v 会让 p_des 从"外推到的地方"退回"帧里的位置"，
         * 目标突然倒退 → 力矩反向冲击一下（实测在 1 kHz 帧率下约 0.2 rad
         * 的倒退量，对 kp=25 就是 5 N·m 的反向指令，直接顶到限幅）。 */
        if (s_mit_v != 0.0f) {
            s_mit_p += s_mit_v * ((float)SERVO_MIT_CMD_TMO_MS * 0.001f);
            s_mit_v = 0.0f;
        }
        s_mit_tau = 0.0f;
    }
}

uint8_t Servo_IsMit(void) { return s_mit_en; }

void Servo_GetMitState(float *tau, float *ep, float *kp, float *kd, float *p_now)
{
    if (tau)   { *tau   = s_mit_tau_out; }
    if (ep)    { *ep    = s_mit_ep; }
    if (kp)    { *kp    = s_mit_kp; }
    if (kd)    { *kd    = s_mit_kd; }
    if (p_now) { *p_now = s_mit_p_now; }
}

/* 上拍实际用到的帧龄（秒）：0~1 ms 说明外推在正常工作，
 * 接近 20 ms 说明帧率掉了（或正在超时冻结）。 */
float Servo_GetMitDt(void) { return s_mit_dt; }

/* 电流环积分/斜坡清零（原 SetCurrentMode(0->1) 的副作用，提出来给各模式复用） */
static void cur_reset_exec(void)
{
    s_cur_vd_int    = 0.0f;
    s_cur_vq_int    = 0.0f;
    s_cur_iq_ramp   = 0.0f;
    s_cur_dead_sign = 0;
}

/* 关执行层：清电流状态 + 连带关位置环与阶跃（原来写在 SetCurrentMode(0) 里） */
static void cur_stop_all(void)
{
    s_cur_id      = 0.0f;
    s_cur_iq      = 0.0f;
    s_cur_id_ref  = 0.0f;
    cur_reset_exec();
    PosLoop_Disable();
    CurStep_Abort();
}

void Servo_SetCurrentMode(uint8_t enable, float iq_ref_a)
{
    float r = iq_ref_a;

    if (r >  s_cur_iq_max) { r =  s_cur_iq_max; }
    if (r < -s_cur_iq_max) { r = -s_cur_iq_max; }
    s_cur_iq_ref = r;

    if ((enable != 0U) && (s_cur_prev == 0U)) {
        cur_reset_exec();
    }
    if (enable == 0U) {
        /* MIT / 位置 / 速度都是它的上层：src 归 NONE 后派生量自动全部归零，
         * 不再需要这里逐个写 s_mit_en / s_pos_en。 */
        cur_stop_all();
        s_pos_prev = 0U;
        s_pos_vcmd = 0.0f;
        src_enter(SERVO_SRC_NONE);
    } else if (s_src == SERVO_SRC_NONE) {
        /* 已在 SPEED / POS / MIT 时不要被降级成 TORQUE */
        src_enter(SERVO_SRC_TORQUE);
    }
    s_cur_prev = (enable != 0U) ? 1U : 0U;
}

uint8_t Servo_IsCurrentMode(void) { return s_cur_en; }

/* ---- 力矩模式的可调参数 ----
 *
 * 为什么需要斜坡：1 A 阶跃对应的加速度是 Kt·Δi/J = 0.091×1/5.47e-5 ≈ 1660 rad/s²，
 * 齿轮间隙和齿槽会被整个激起来。20 ms 一阶低通把 d(iq)/dt 压到 50 A/s 量级，
 * 又不会明显拖慢力控（20 ms 对应闭环带宽 8 Hz，高于多数力控外环）。
 * 做阶跃响应测试时把 tau 设 0，让给定直通 —— 那时要看的正是环路本身的响应。 */
void Servo_SetCurIqMax(float a)
{
    if (a < 0.05f) { a = 0.05f; }
    if (a > SERVO_CUR_IQ_MAX) { a = SERVO_CUR_IQ_MAX; }
    s_cur_iq_max = a;
    /* 目标值超过新限幅时立即夹住，否则斜坡会慢慢爬回一个非法值 */
    if (s_cur_iq_ref >  a) { s_cur_iq_ref =  a; }
    if (s_cur_iq_ref < -a) { s_cur_iq_ref = -a; }
}

float Servo_GetCurIqMax(void) { return s_cur_iq_max; }

/* 温度降额（0..1）。1 ms 任务写，20 kHz 环只读。
 * 注意：不改 s_cur_iq_max —— 那是用户设定的基准限幅，降额是乘在它上面的
 * 临时系数，温度回落就自动恢复，不该把用户参数改掉。 */
void  Servo_SetDerate(float k)
{
    if (!(k > 0.0f)) { k = 0.0f; }      /* 同时挡住 NaN */
    if (k > 1.0f)    { k = 1.0f; }
    s_derate = k;
}
float Servo_GetDerate(void) { return s_derate; }

void Servo_SetCurRampTau(float tau_s)
{
    if (tau_s < 0.0f)   { tau_s = 0.0f; }
    if (tau_s > 2.0f)   { tau_s = 2.0f; }
    s_cur_iq_tau = tau_s;
}

float Servo_GetCurRampTau(void) { return s_cur_iq_tau; }
float Servo_GetCurIqRamp(void) { return s_cur_iq_ramp; }

/* 死区电压补偿增益。理论值由 DTG 算：DeadTime=50 → 50×5.88 ns = 294 ns，
 * 占 50 µs 周期的 0.59%，折算电压 0.0059·Vbus ≈ 0.18 V @30.7 V。
 * 除以相电阻 0.4 Ω 相当于 0.45 A —— 比小力矩给定本身还大，
 * 所以力矩模式要精确必须补它。默认 0（关闭），标定后再开。 */
void Servo_SetCurDeadGain(float g)
{
    if (g < 0.0f) { g = 0.0f; }
    if (g > 3.0f) { g = 3.0f; }
    s_cur_dead_gain = g;
}

float Servo_GetCurDeadGain(void) { return s_cur_dead_gain; }

/* ---- Kt 温度补偿：1 ms 任务按电机 NTC 更新，20 kHz 环只读 ---- */
void Servo_UpdateKtComp(float t_mot_c)
{
    float k, t_ref;

    if (s_kt_comp_en == 0U) {
        s_kt_comp = 1.0f;
        return;
    }
    if (t_mot_c != t_mot_c) { return; }        /* NaN：NTC 故障，保持上次值 */

    /* 基准温度 = ψ 辨识那一刻的电机温度（PS_FLAG_PSI_T 有效时）。
     * 2026-11-20 修正：原来固定 20 °C，但辨识通常在热机后做 ——
     * 实测 84 °C 辨识得 ψ=3.894 mWb，若仍按 20 °C 基准补偿，
     * 冷机 28 °C 时 ψ_now = 3.894×0.990 = 3.855，比真实 4.19 低 8%，
     * Kt 与 BEMF 前馈同时偏小。以辨识温度作基准后，T=T_id 处 k≡1，
     * 补偿量只反映"相对于辨识时刻漂了多少"，与辨识绝对值无关。 */
    t_ref = (s_psi_t_ok != 0U) ? s_psi_t_ref : SERVO_KT_T_REF;

    k = 1.0f + SERVO_KT_ALPHA * (t_mot_c - t_ref);
    if (k < SERVO_KT_COMP_MIN) { k = SERVO_KT_COMP_MIN; }
    if (k > SERVO_KT_COMP_MAX) { k = SERVO_KT_COMP_MAX; }

    /* 变化率限制 ±0.2%/ms。
     *
     * 为什么要：上电后第一次 NTC 采样若撞上 ADC2 未稳定的码值，
     * 反算出的温度可能远高于实际（实测复位后首帧读到约 104 °C），
     * k 会被一步压到限幅下限 0.90 —— 电机还没转就先少给 10% Kt，
     * 靠 10 ms 后的正常采样才恢复。加上限速后，单帧野值最多影响 0.2%，
     * 而真实温漂是 0.012%/K、K 级要几十秒才走完，限速对它无影响。 */
    if (k > (s_kt_comp + 0.002f)) { k = s_kt_comp + 0.002f; }
    if (k < (s_kt_comp - 0.002f)) { k = s_kt_comp - 0.002f; }

    s_kt_comp = k;
}

float   Servo_GetKtComp(void)          { return s_kt_comp; }
float   Servo_GetKtNow(void)           { return s_kt_base * s_kt_comp; }
float   Servo_GetPsiTRef(void)         { return (s_psi_t_ok != 0U) ? s_psi_t_ref : SERVO_KT_T_REF; }
uint8_t Servo_PsiTRefOk(void)          { return s_psi_t_ok; }
void    Servo_SetKtCompEn(uint8_t en)  { s_kt_comp_en = (en != 0U) ? 1U : 0U;
                                         if (en == 0U) { s_kt_comp = 1.0f; } }
uint8_t Servo_GetKtCompEn(void)        { return s_kt_comp_en; }

/* 力矩估算（N·m）。Kt = 1.5·p·ψ = 1.5×14×4.333e-3 = 0.0910 N·m/A，
 * 与 mech_ident.c 的 MI2_KT 同源（那里用于把 iq 换算成转动惯量的力矩）。
 * 注意 SERVO_CUR_PSI(7.59e-3) 与实测 ψ(4.333e-3) 差 1.75 倍，那个常量
 * 只用在 BEMF 前馈上；力矩换算必须用实测值，否则力矩会虚高 75%。
 * 2026-11-19：乘 s_kt_comp 做温度补偿（见上方常量说明）。 */
/* 辨识结果写入控制律。任何异常值都只"拒绝采用"，绝不写坏环路。 */
void Servo_SetIdentPsi(float psi_wb)
{
    if (psi_wb != psi_wb) { return; }                      /* NaN */
    if ((psi_wb < SERVO_ID_PSI_MIN) || (psi_wb > SERVO_ID_PSI_MAX)) { return; }
    s_cur_psi   = psi_wb;
    s_kt_base   = 1.5f * SERVO_CUR_POLE * psi_wb;          /* Kt = 1.5·p·ψ */
    s_id_psi_ok = 1U;
}

/* 带温度的版本：t_mot_c 是辨识那一刻的电机温度，作为 Kt/前馈补偿的基准。
 *
 * 为什么不直接用"20 °C 基准 + 绝对值"：ψ 是有温度的量，同一次辨识在
 * 84 °C 与 28 °C 会给出相差 7% 的 ψ。若把两个值都当"20 °C 的 ψ"再补偿，
 * 高温辨识的那份在冷机时会被二次压价（实测低估 8%）。
 * 记录辨识温度后，T=T_id 处 k≡1，补偿只反映相对漂移 —— 与辨识在哪个温度做无关。
 *
 * t_mot_c 无效（NaN 或超出 −20~150）时退回 20 °C 基准，等同旧行为。 */
void Servo_SetIdentPsiEx(float psi_wb, float t_mot_c)
{
    Servo_SetIdentPsi(psi_wb);
    if (s_id_psi_ok == 0U) { return; }                     /* 上面被拒了 */
    if (t_mot_c != t_mot_c) { return; }
    if ((t_mot_c < -20.0f) || (t_mot_c > 150.0f)) { return; }
    s_psi_t_ref = t_mot_c;
    s_psi_t_ok  = 1U;
}

void Servo_SetIdentRl(float r_ohm, float l_h)
{
    if ((r_ohm == r_ohm) && (r_ohm >= SERVO_ID_R_MIN) && (r_ohm <= SERVO_ID_R_MAX)) {
        s_id_r = r_ohm;
    }
    if ((l_h == l_h) && (l_h >= SERVO_ID_L_MIN) && (l_h <= SERVO_ID_L_MAX)) {
        s_cur_ld  = l_h;                                   /* dq 同值（表贴） */
        s_cur_lq  = l_h;
        s_id_rl_ok = 1U;
    }
}

void Servo_GetIdentParams(float *r_ohm, float *ld_h, float *psi_wb, float *kt)
{
    if (r_ohm  != 0) { *r_ohm  = s_id_r;    }
    if (ld_h   != 0) { *ld_h   = s_cur_ld;  }
    if (psi_wb != 0) { *psi_wb = s_cur_psi; }
    if (kt     != 0) { *kt     = s_kt_base; }
}

uint8_t Servo_IdentApplied(void)
{
    return ((s_id_psi_ok != 0U) || (s_id_rl_ok != 0U)) ? 1U : 0U;
}

float Servo_GetTorqueEst(void)
{
    return s_kt_base * s_kt_comp * s_cur_iq;
}

/* 运行时改电流环增益（调 PI 用）。两个一起缩放 = 只改带宽、零点不变；
 * 单独改 Kp 或 Ki 会破坏极点抵消，是刻意的调试手段，不要长期用。 */
void Servo_SetCurGains(float kp, float ki_ts)
{
    if (kp    < 0.02f)   { kp    = 0.02f; }
    if (kp    > 20.0f)   { kp    = 20.0f; }
    if (ki_ts < 0.0005f) { ki_ts = 0.0005f; }
    if (ki_ts > 5.0f)    { ki_ts = 5.0f; }
    s_cur_kp    = kp;
    s_cur_ki_ts = ki_ts;
}

void Servo_GetCurGains(float *kp, float *ki_ts)
{
    if (kp)    { *kp    = s_cur_kp; }
    if (ki_ts) { *ki_ts = s_cur_ki_ts; }
}

void Servo_SetCurIdRef(float a)
{
    if (a >  SERVO_CUR_IQ_MAX) { a =  SERVO_CUR_IQ_MAX; }
    if (a < -SERVO_CUR_IQ_MAX) { a = -SERVO_CUR_IQ_MAX; }
    s_cur_id_ref = a;
}

void Servo_GetCurRefs(float *id_ref, float *iq_ref)
{
    if (id_ref) { *id_ref = s_cur_id_ref; }
    if (iq_ref) { *iq_ref = s_cur_iq_ref; }
}

void Servo_GetIdqMeas(float *id, float *iq)
{
    if (id) { *id = s_cur_id; }
    if (iq) { *iq = s_cur_iq; }
}


/* ---------------- 速度闭环（2 kHz 速度环 -> iq_ref -> 20 kHz 电流环） ----------------
 *
 * 为什么在 Board 层另写一套，而不是直接调模型里的速度环：
 *   生成代码的速度环输入是 hold_w，而 hold_w 只由位置环 P 产出（pos_ref-theta)*kp_p，
 *   顶层没有 w_ref 端口。要走模型的速度环就必须重新生成代码 —— 而重新生成会
 *   覆盖此前所有手工适配（real32_T、par_* 参数化、注释里的坐标系约定），
 *   历史上已经因此丢过一次补丁，代价远大于照抄 20 行算法。
 *
 * 与 fcn_three_speed_loop_deploy.m 的逐行对应关系：
 *   模型                                    本文件
 *   --------------------------------------  ----------------------------------
 *   cnt++ ; if cnt >= n_div                s_spd_cnt++ ; >= SERVO_SPD_DIV
 *   i_term += ki*err*ts                    同（ki_ts 已预先乘好 ts）
 *   i_term clamp ±i_max                    同
 *   out = kp*err + i_term ; clamp ±i_max   同
 *   其余拍 hold_iq 保持                     s_spd_iq 保持
 *
 * 唯一有意偏离：模型对积分器只做简单 clamp，iq 顶到限幅后积分仍继续累加
 * （windup）。空载小信号下两者完全一致，但一旦堵转，模型的积分会跑到饱和、
 * 松开后要几百毫秒才退出来；这里改成"饱和且误差仍指向饱和方向就冻结积分"，
 * 代价是 2 个比较，换来堵转恢复无超调。
 */
/* 增益来源：模型 par_w 的 8 倍，不是模型原值。
 *
 * 模型给 kp=0.07292272 / ki*Ts=5.0953445e-3。2026-11-15 逐档实测（30 rpm，空载）：
 *   档位    kp      纹波(rpm)  超调(%)   iq峰值(A)
 *   1x     0.073     22.5      204       0.74
 *   2x     0.146     11.2      108       0.74
 *   4x     0.292      5.6       56       0.74
 *   8x     0.583      3.2       25       0.93   <- 拐点前最优
 *   16x    1.167     22.1      122       3.00   <- iq 打到限幅，饱和-退饱和来回
 *   32x    2.334     25.4      139       3.00
 * 也就是说模型值在这台电机上偏小 8 倍：等效 J 约 2.3e-4 kg·m^2，
 * 而模型那组参数对应 J≈2.9e-5。模型整定时显然用了更小的惯量假设
 * （或把"位置环内环"当成了唯一场景）。这里的默认值以实测为准，
 * 换电机后必须重扫 —— L 键就是为此保留的。
 */
#define SERVO_SPD_KP        (0.5833818f)    /* 8 x 0.07292272 */
#define SERVO_SPD_KI_TS     (0.04076276f)   /* 8 x 5.0953445e-3 */
#define SERVO_SPD_DIV       (10U)           /* 20 kHz / 10 = 2 kHz，模型 par_w[4] */
#define SERVO_SPD_IQ_MAX    (3.0f)          /* iq 限幅 A：空载 30 rpm 约 0.2 A，
                                             * 3 A 有 15 倍余量且远低于 12 A 硬件跳闸。
                                             * 模型给 9 A，但那是带载设计值，
                                             * 空载台架上 9 A 意味着 1.4 N·m 冲击。 */
#define SERVO_SPD_WREF_MAX  (400.0f)        /* 速度指令硬限幅 rad/s，约 3820 rpm：
                                             * 覆盖额定 3000 rpm 并留 27% 余量。
                                             *
                                             * 原来是 100 rad/s（955 rpm），2026-11-16
                                             * 实测踩坑：发 3000 rpm 指令后转速稳在
                                             * 955.rpm、iq 只有 0.4 A、K 命令显示
                                             * wref=954.93 —— 那是 100 rad/s 的换算值，
                                             * 不是电机极限。静默截断比报错更难查，
                                             * 所以这里必须留足余量，让"到不了"由电压
                                             * 或超速保护明确暴露，而不是被夹在入口。 */
#define SERVO_SPD_WMAX_ABS  (10.0f)         /* 超速门限下限 rad/s（约 95 rpm）：
                                             * 编码器断线时 omega 恒 0，速度环会把 iq
                                             * 顶到限幅一路加速，必须靠这条兜住。
                                             * 空载 3 A 约 956 rad/s^2，10 ms 就到门限。 */

/* ---------------- 给定斜坡（加减速限制） ----------------
 *
 * 为什么必须有：原来给的是**阶跃**，进环那一拍 kp*err 就直接顶到 3 A 限幅，
 * 速度环实际工作在 bang-bang 区。实测超调：30 rpm 31%、3 rpm 68.8%
 * （3 rpm 还有库仑摩擦起跳的贡献）。这既不是 PI 参数能修的，也不是
 * 单纯降增益能修的 —— 降增益只会让稳态纹波变大（实测 1x 时 22.5 rpm）。
 *
 * 做法：把用户给定 s_spd_ref_raw 限速成 s_spd_ref_ramp，PI 只跟踪斜坡。
 * 斜坡在**模型坐标系**（dir 已乘）里推进，因此换向时斜坡方向自动正确。
 *
 * 默认 400 rad/s^2 = 3820 rpm/s：
 *   3000 rpm 用 0.79 s 到速（工业伺服常见量级）
 *   30 rpm   用 7.9 ms —— 低速段基本还是阶跃，那里的超调要靠摩擦补偿，
 *            不是靠压加速度。
 *
 * 改这个值要连带看超速保护：wlim = 1.5*|ref| + 10，斜坡过慢时 wlim 会更紧，
 * 但因为 ref 和 omega 都在爬，实际不会误触。
 */
#define SERVO_SPD_ACC_DFLT  (400.0f)        /* rad/s^2 ≈ 3820 rpm/s */
#define SERVO_SPD_ACC_MIN   (5.0f)          /* 27.5 rad/s^2 ≈ 263 rpm/s，再小几乎等于不动 */
#define SERVO_SPD_ACC_MAX   (20000.0f)      /* = 不限制（20 kHz 下一拍就跨完 1 rad/s） */
#define SERVO_SPD_JMAX_MAX  (200000.0f)     /* 与 30089 的协议上限一致 */
#define SERVO_SPD_TS        (SERVO_SPD_DIV * BSP_TS_CURRENT)   /* 500 us，速度环周期 */

/* 已由 s_src 派生，见文件下方状态机定义 */
static volatile float    s_spd_ref_raw;      /* 用户坐标系（编码器正方向）rad/s */
static volatile float    s_spd_kp     = SERVO_SPD_KP;
static volatile float    s_spd_ki_ts  = SERVO_SPD_KI_TS;
static uint8_t           s_spd_prev;
static float             s_spd_i_term;
static volatile float    s_spd_iq;
static uint32_t          s_spd_cnt;
static uint32_t          s_spd_exec;
static volatile float    s_spd_acc = SERVO_SPD_ACC_DFLT;  /* rad/s^2 */
static volatile float    s_spd_iqmax = SERVO_SPD_IQ_MAX;  /* iq 限幅 A，运行时可调 */

/* ---- 手写速度环的给定轨迹发生器（2026-11-19）----
 * 原斜坡是纯线性（梯形），只受 a_max 限制。这里改用 motion_profile 的 ramp_step，
 * 让同一条 2 kHz 链路支持梯形/S 曲线两种 profile：
 *   - 默认 TRAP + 大 jerk  => 与原来的线性斜坡逐周期等价，既有行为零变化；
 *   - 用户切 SCURVE        => 加速度以 |jerk|<=j_max 追随 a_max，冲击更小。
 * 独立于 s_vel：后者属于未启用的 SERVO_MODE_VELOCITY（模型三环）链路，
 * 两者不能共用状态，否则切环会互相污染斜坡。
 * a_max 仍以 s_spd_acc 为单一来源（Servo_SetSpeedAccel 兼容旧接口）。 */
static ramp_t            s_spd_ramp;
static volatile uint8_t  s_spd_profile_mode = (uint8_t)PROFILE_MODE_TRAP;
static volatile float    s_spd_jmax = 20000.0f;   /* rad/s^3，仅 S 曲线用；梯形忽略 */

/* ---- 轨迹时间优先（2026-11-21，30097）----
 * 用户给的是"从这一刻起到目标转速的过渡时间 t"，固件反算加速度：
 *   梯形  ：a = Δω / t                        （恒加速，t 秒正好到）
 *   S 曲线：a = 2Δω / t，j = 4Δω / t²
 *           S 曲线的加速度是梯形剖面：t = Δω/a + a/j = t/2 + t/2 = t，正好。
 *
 * 为什么必须**锁存**而不能每拍按剩余误差重算：
 *   若每拍 a = 剩余/t，则 a 随剩余一起减小，微分方程 dv/dt = e/t 的
 *   解是双曲型，剩余按指数衰减但永不归零 —— 实测表现是"停在半速上"。
 * 所以只在"目标变化超过 0.5 rad/s"那一拍锁一次，整段过渡用同一组 a/j。
 *
 * 为什么只在速度模式生效：位置模式下 w_ref 是 PosLoop_Step 每拍连续产出的，
 * 没有"阶跃"语义；在那里按时间重算等于每拍改加速度，反而破坏位置规划。
 */
#define SERVO_SPD_T_RELOCK   (0.5f)      /* rad/s：目标变化超过它才重锁 */
#define SERVO_SPD_T_DWMIN    (0.5f)      /* rad/s：剩余太小就不反算，用配置值 */
static volatile float    s_spd_t_ramp;   /* s，0 = 关闭（走 s_spd_acc / s_spd_jmax） */
static float             s_spd_a_latch;  /* 本段锁存的 a_max */
static float             s_spd_j_latch;  /* 本段锁存的 j_max */
static float             s_spd_w_latch;  /* 上次锁存时的目标（rad/s，模型坐标系） */
static uint8_t           s_spd_t_clamp;  /* 1 = 反算值被上下限夹过 */

/* 给定幅值的峰值保持（超速门限的基准）。
 *
 * 为什么不能直接用 |s_spd_ref_raw| 当门限：从 1000 rpm 降到 300 rpm 时，
 *   wref 一瞬间变成 31.4 rad/s，而电机还带着惯性在 104 rad/s 滑行 ——
 *   门限立刻变成 1.5×31.4+10 = 57，"正常减速"被判成超速
 *   （实测 om=104.01 / wref=31.42 / f=0x20）。
 *
 * ★ 2026-11-21 重写：不能用"按时间指数衰减"。
 *   旧实现给定下降时按 0.9999/拍衰减（τ=0.5 s）。实测复现 2100 → 140 rpm：
 *   0.226 s 后峰值从 219.9 掉到 139，门限 = 1.5×139+10 = 218.5 rad/s，
 *   而 om 还在 219.9 —— 0.24 s 就报 f=0x20，此时 wref 甚至还没离开 -2091 rpm。
 *
 *   这是必然而非偶然：减速斜率由用户给的 t 决定，最小可以到
 *   SERVO_SPD_ACC_MIN = 5 rad/s²。要保证减速全程门限不跌破实测，需要
 *       τ ≥ V0² / (a·(1.5·V0 + 10))
 *   V0=220 rad/s、a=5 时要求 τ ≥ 28.5 s，而旧值只有 0.5 s，差 57 倍。
 *
 * 新策略：不按时间衰减，按"**实测速度是否已回到给定附近**"判断。
 *   · 稳态 / 升速（aref 已到峰值）→ 峰值 = 给定，保护跟着目标走
 *   · 减速途中（om 远高于给定）  → 峰值保持不动，门限全程不收紧
 *   · 减速完成（om ≈ 给定）      → 允许收回（0.999/拍，约 1 s）
 * 真失控（给定不变而 om 持续上升）依旧撞线：门限 = 1.5×给定 + 10。
 *
 * 为什么不能看"给定是否在下降"：给定是**阶跃目标**，给 -140 时它一拍就
 * 跳到位并保持，`aref < aref_last` 只成立一拍，第二拍起就被误判成"减速已
 * 结束"。实测就是 wlim 在 192 ms 内从 339.9 掉到 243.5 而 wmeas 仍是 2100 rpm。 */
static float             s_spd_wpeak;
static volatile float    s_spd_cog;      /* 最近一拍齿槽前馈量（A，诊断） */

static uint8_t           s_ovs_cnt;      /* 超速消抖计数（20 kHz 拍） */

/* ================= 速度环增益调度（多段 PID 查表） =================
 *
 * 为什么要分段：单一组增益在这台电机上不可能同时满足零速与高速。
 *
 *   零速（真正的 0 rpm）：扰动是**齿槽转矩**（实测峰峰 1.17 A，是 30 rpm
 *     运行电流的 9 倍）。静止时齿槽被静摩擦锁住，只需积分慢慢消静差，
 *     所以这里 Kp 略降（0.25）以防 iq 在齿槽谷/峰之间来回蹭。
 *
 *   ⚠ 极低速（1~30 rpm）**不能**跟着降增益 —— 这是第一版表设计错的地方：
 *     当时把 0~30 rpm 做成 0.11→0.40 的爬升，实测 3 rpm 纹波从 0.451
 *     恶化到 1.114 rpm（+147%）。原因很直接：爬行时摩擦死区 + 齿槽让系统
 *     呈"黏滑"特性，Kp 越小，越需要更长时间才攒够力矩越过死区，
 *     一旦越过又冲过头 —— 纹波就是这么来的。
 *     所以 0.21 rad/s（2 rpm）往上一律取 0.55~0.583，只在真正的 0 点降。
 *
 *   中速（100~500 rpm）：扰动是黏滞摩擦（线性），反馈信噪比最好，
 *     实测最优 Kp=0.583（对应 wc = Kp·Kt/J = 154 Hz）。
 *
 *   高速（1000 rpm 以上）：BEMF 占掉大部分电压预算（实测 2496 rpm 时
 *     |v|=16.50 V 正好等于圆限幅 0.5375·vbus），此时 Kp 过大容易在限幅
 *     附近产生极限环。降 Kp 让 PI 工作在限幅以下，剩下的跟踪误差由
 *     加速度前馈与摩擦前馈承担。
 *
 * 查表变量用 |s_spd_ref_ramp|（斜坡后的给定），不是 w_ref、也不是 w_meas：
 *   - 用 w_ref：给 3000 rpm 时一开始就跳到高速段增益，而电机还在零速，
 *     低速起动需要的增益拿不到 → 起动无力。
 *   - 用 w_meas：反馈式调度，增益随实测速度变，给定与反馈相差大时
 *     增益来回跳，可能出现"调度引起"的振荡。
 *   - 用 w_ramp：它从实测速度起步、按 accel 缓慢爬升，**起动时自动落在
 *     低速段、到速后落到高速段**，正好是想要的时序，且单调平滑。
 *
 * 段间线性插值而不是阶跃切换：Kp 阶跃会让 iq 瞬间跳 Kp·err，形成冲击；
 * 插值后每拍变化 <1e-3，等效连续。
 */
typedef struct {
    float w;        /* 段点：|w_ramp| rad/s */
    float kp;       /* A/(rad/s) */
    float kits;     /* Ki·Ts */
} SpdSchedPt;

#define SPD_SCHED_N   (7U)

static const SpdSchedPt s_sched[SPD_SCHED_N] = {
    /*  w[rad/s]   rpm      kp      kits      依据 */
    {   0.00f,             0.250f, 0.0300f },  /* 0：静止保持，略降防抖 */
    {   0.21f,   /*   2 */ 0.550f, 0.0400f },  /* 进入爬行立刻拉回最优 */
    {  10.47f,   /* 100 */ 0.583f, 0.0408f },  /* 实测最优（154 Hz） */
    {  52.36f,   /* 500 */ 0.583f, 0.0408f },  /* 平坦段 */
    /* 高速段：**降 Kp 但保 Ki**。
     * 第一版把 Ki 跟着 Kp 一起降（0.0408→0.036→0.030），
     * 实测 1500 rpm 稳态误差 11.77 rpm（0.78%）—— 增益降下来了、振荡没了，
     * 但积分消静差的能力也跟着降了。
     * 这里的取舍是：Kp 降是为了留相位裕度（固定 0.583 在 1500 rpm 实测
     * 已经在振荡：iq 均值 0.906 A、峰值 2.21 A、纹波 12.5 rpm），
     * 而积分只影响静差、几乎不影响相位裕度，所以它可以保持甚至略升。 */
    /* 高速段只降 14%（0.583→0.500），Ki 略升。
     * 为什么不敢降更多：1500 rpm 上同一组固定增益两次测得纹波 12.5 与 2.53 rpm、
     * iq 均值 0.906 与 1.78 A —— 这个工况本身重复性差（可能是机械侧周期性阻力
     * 或共振点附近），不足以支撑"降多少"的定量判断。宁可保守。 */
    { 104.72f,   /*1000 */ 0.550f, 0.0430f },  /* 电压余量变紧，Ki 略升 */
    { 209.44f,   /*2000 */ 0.520f, 0.0420f },  /* BEMF 已占大半 */
    { 314.16f,   /*3000 */ 0.500f, 0.0410f },  /* 接近电压上限 */
};

static volatile uint8_t s_sched_en = 1U;    /* 1 = 查表（默认），0 = 用手动增益 */
static volatile float    s_sched_kp;        /* 最近一次查到的值（诊断） */
static volatile float    s_sched_kits;

/* 线性插值查表。w 取绝对值，超出末段则钳在末段值。 */
static void sched_lookup(float wabs, float *kp, float *kits)
{
    uint8_t i;

    if (wabs <= s_sched[0].w) {
        *kp   = s_sched[0].kp;
        *kits = s_sched[0].kits;
        return;
    }
    for (i = 1U; i < SPD_SCHED_N; i++) {
        if (wabs <= s_sched[i].w) {
            const float w0 = s_sched[i - 1U].w;
            const float w1 = s_sched[i].w;
            const float t  = (wabs - w0) / (w1 - w0);

            *kp   = s_sched[i - 1U].kp   + t * (s_sched[i].kp   - s_sched[i - 1U].kp);
            *kits = s_sched[i - 1U].kits + t * (s_sched[i].kits - s_sched[i - 1U].kits);
            return;
        }
    }
    *kp   = s_sched[SPD_SCHED_N - 1U].kp;
    *kits = s_sched[SPD_SCHED_N - 1U].kits;
}

void Servo_SetSchedEnable(uint8_t en)
{
    s_sched_en = (en != 0U) ? 1U : 0U;
    /* 切回手动时，把查表值抄进手动值，避免"关掉瞬间增益跳到旧的手动值" */
    if ((en == 0U) && (s_sched_kp > 0.0f)) {
        Servo_SetSpeedGains(s_sched_kp, s_sched_kits);
    }
}

uint8_t Servo_GetSchedEnable(void) { return s_sched_en; }

void Servo_GetSchedGains(float *kp, float *kits)
{
    if (kp)   { *kp   = s_sched_kp; }
    if (kits) { *kits = s_sched_kits; }
}

/* ---------------- 速度环前馈与抗饱和 ----------------
 *
 * 三个量都是"让 PI 少干活"，不是替代 PI：
 *
 *  ① 加速度前馈  iq_ff = (J/Kt)·(dw_ref/dt)
 *     斜坡段 dw_ref/dt 是常数，前馈把它整段的力矩需求提前给足，
 *     PI 只需补摩擦与模型误差。J 有反推值（2.3e-4）但可能差 50%，
 *     所以乘 0.7 折扣 —— 宁可让 PI 补，也不要前馈过量造成超调。
 *
 *  ② 摩擦前馈   iq_ff += (Tc/Kt)·tanh(w_ref/0.05) + (Bv/Kt)·w_ref
 *     这是 3 rpm 超调 68.8% 的唯一解药：静摩擦约 0.028 N·m（=0.30 A），
 *     原来全靠积分慢慢爬到 0.30 A 才能起动，爬的过程里积分早已过冲。
 *     方向必须用**斜坡给定**而不是实测：起动那一拍实测还是 0，
 *     用实测就永远给不出起动转矩 —— 那正是原来的机制。
 *
 *  ③ back-calculation 抗饱和
 *     原来只"冻结"积分（条件积分），退饱和慢；改成把超出量按 Kb 回灌积分。
 *     Kb=0 时自动退回原来的条件积分（保留对比与回退手段）。
 *
 * 参数都能运行时改，`$` 命令做两点标定自动定 Tc 与 Bv。
 */
#define SERVO_FF_ACC_DFLT   (5.50e-4f)   /* J/Kt*0.7，J=6.887e-5 实测，Kt=0.0883 */   /* A/(rad/s^2)：J/Kt×0.7 = 2.3e-4/0.0936×0.7 */
/* 0.15 A：机械辨识 `)` 的实测值 —— 30/80/200 rpm 三点稳定在 0.13~0.15 A。
 * 原来写的 0.321 是"3 rpm 稳态 iq"，而 3 rpm 本身不可信（速度环积分慢摆，
 * 两次测量给出 0.19 和 −0.03）。低速项由下面的起动助推负责。 */
#define SERVO_FF_FRIC_DFLT  (0.107f)     /* 11-21 摩擦曲线 12~200rpm 回归截距 */     /* A：动摩擦（= 30~200 rpm 稳态 iq） */
#define SERVO_FF_VISC_DFLT  (2.30e-3f)   /* 由 1000rpm 稳态反推：(0.35-0.107)/104.7。
                                          * 原 2.14e-4 比实测小一个数量级，
                                          * 1000rpm 处少补约 0.3A，全靠积分项扛 */   /* A/(rad/s)：Bv/Kt，30 rpm 时才 0.0007 A，几乎可忽略 */
#define SERVO_FF_W_EPS      (0.05f)      /* rad/s：摩擦过渡宽度，越大换向越软 */
#define SERVO_FF_ACC_LPF    (0.25f)      /* 加速度前馈一阶系数（2 kHz → tau≈2 ms） */
/* 起动助推倍数：静摩擦约是动摩擦的 2 倍（实测起动 0.64 A vs 稳态 0.30 A）。
 * 标定只能测到**稳态** iq（= 动摩擦），所以起动瞬间要额外加一份。
 * 1.0 = 起动时给两倍摩擦前馈（0.32 → 0.64 A），正好跨过静摩擦。
 * 判据用"实测转速还不到给定的 30%"，稳态时自然不触发。 */
#define SERVO_FF_BOOST_DFLT (1.0f)
#define SERVO_FF_START_RATIO (0.30f)
static volatile float   s_ff_boost = SERVO_FF_BOOST_DFLT;
#define SERVO_SPD_KB_DFLT   (0.30f)      /* back-calculation 增益，0=退回条件积分 */

static volatile uint8_t s_ff_en = 1U;   /* 2026-11-21：默认改为开启。
                                          * 原来无初值(=0)，而唯一能置 1 的路径是
                                          * `$` 标定成功（fric_cal.c:246），`$` 又因 3 rpm
                                          * 工作点必然失败 —— 于是整个速度前馈
                                          * （加速度/摩擦/黏滞/起动助推）长期是死代码。
                                          * 实测证据：ff_acc 从 5.5e-4 拉到 0.5 A/(rad/s^2)
                                          * （400 rad/s^2 下应注入 200 A、必被 3 A 限幅卡死），
                                          * 斜坡段 iq_ref 只从 0.513 变到 0.697 A。 */
static volatile float   s_ff_acc  = SERVO_FF_ACC_DFLT;
static volatile float   s_ff_fric = SERVO_FF_FRIC_DFLT;
static volatile float   s_ff_visc = SERVO_FF_VISC_DFLT;
static volatile float   s_spd_kb  = SERVO_SPD_KB_DFLT;
static float            s_ff_acc_f;      /* 加速度前馈的滤波状态 */
static volatile float   s_ff_last;       /* 最近一拍前馈总量（诊断） */
static float             s_spd_ref_ramp;                  /* 斜坡后的给定（模型坐标系） */
static uint8_t           s_spd_ramp_init;                 /* 0 = 下一拍用实测速度起步 */

void Servo_SetSpeedMode(uint8_t enable, float w_ref_rad_s)
{
    float r = w_ref_rad_s;

    if (r >  SERVO_SPD_WREF_MAX) { r =  SERVO_SPD_WREF_MAX; }
    if (r < -SERVO_SPD_WREF_MAX) { r = -SERVO_SPD_WREF_MAX; }
    s_spd_ref_raw = r;

    if ((enable != 0U) && (s_spd_prev == 0U)) {
        s_spd_i_term = 0.0f;
        s_spd_iq     = 0.0f;
        s_spd_cnt    = 0U;
        /* 峰值保持的起点取"给定"与"进环那一刻的实测余速"的较大者。
         *
         * 2026-11-21 修（实测复现）：原来只取 |给定|。带余速重新进环时
         * （界面点停止 → 电机还在滑行 → 再给 0），|给定|=0 会把门限算成
         * 1.5×0 + 10 = 10 rad/s（95 rpm），而余速还有 262 rpm，
         * 0.4 ms 内就置 BSP_FAULT_OVERSPEED：
         *   fault 出现在 27 ms、rpm=261.9 → MOE 关 → 之后每帧 30019 都被拒
         *   （用户看到的就是"高速切到零速附近报状态错误"）。
         * 取 max 后：静止起转仍从 |给定| 起（失控保护不变），带余速进环则
         * 从余速起，门限跟着真实转速走；峰值仍按 0.5 s 时间常数衰减，
         * 停止后 3 s 门限自然收回 10 rad/s，长时间不停转仍会被抓住。 */
        {
            const float w_now = fabsf((float)s_dir_sign * BspServo_GetOmegaMech());
            const float w_cmd = fabsf(r);

            s_spd_wpeak = (w_now > w_cmd) ? w_now : w_cmd;
        }
        /* 斜坡从"当前实测速度"起步：直接给 0 的话，进环那一拍就是
         * 0 → ref 的阶跃，等于没加斜坡。实际速度在 ISR 里取，因为这里
         * 拿到的 omega 可能是很久以前的（且 BspServo_GetOmegaMech 会推进低通）。 */
        s_spd_ramp_init = 0U;
        /* 执行层（电流环）由 s_src 派生，这里只清它的积分：
         * 先起电流环再放 MOE，放开那一拍积分从 0 起步。 */
        cur_reset_exec();
    }
    if (enable == 0U) {
        s_spd_i_term = 0.0f;
        s_spd_iq     = 0.0f;
        if (s_spd_prev != 0U) {
            if (s_src == SERVO_SRC_SPEED) { src_enter(SERVO_SRC_NONE); }
            cur_stop_all();
            s_pos_prev = 0U;
            s_pos_vcmd = 0.0f;
        }
    } else {
        src_enter(SERVO_SRC_SPEED);
    }
    s_spd_prev = (enable != 0U) ? 1U : 0U;
}

uint8_t Servo_IsSpeedMode(void) { return s_spd_en; }

/* ================= 位置环（手写，2026-11-17）================= */

/* ---- 位置环坐标基准（2026-11-21）------------------------------------------
 *
 * 需求：**每次进入位置模式**，位置反馈变成当时的**编码器单圈角**（0~2π），
 *       数值原样保留 —— 单圈角是 3.14 就报 3.14，**不是归零**。
 *       要消掉的只是"上电以来累积了多少圈"（否则会跑到上百 rad）。
 *
 * 为什么不能直接把反馈换成单圈角 theta：
 *   theta 在 2π→0 处回绕。位置环拿到 6.28→0 会认为"还差 6.28 rad"，
 *   直接满速反转一整圈 —— 比原来的问题更糟。
 *
 * 做法：位置环坐标 p_pos = theta_multi − s_pos_base，对齐时
 *       s_pos_base = theta_multi − theta   ⇒   p_pos = theta ∈ [0, 2π)
 *   之后 p_pos 随 theta_multi 连续变化，跨 0/2π 不跳。
 *
 * 只影响位置环的输入/输出坐标，**不改 theta_multi**，所以：
 *   · 齿槽前馈 CogFf_Get(theta_multi) 的相位不受影响，`}` 不用重做
 *   · MIT / 力矩 / 速度模式不受影响
 *   · 30090 的 pos（单圈、含零点）不变
 *
 * 示教点的影响：单圈内的示教值稳定（p_now 与编码器角一一对应，跨切换、
 * 跨上电都一样）；只有跨圈会差 2π 的整数倍，由上位机自己记圈数。
 *
 * 置 0 可回到原来的"上电起算多圈坐标"。
 */
#define SERVO_POS_COORD_AT_ENTRY   (1)

/* 跨上下文：命令/1 ms 任务写，20 kHz ISR 读，必须 volatile，
 * 否则编译器可能把它提升到循环外，表现为"切了模式但坐标没变"。 */
static volatile float s_pos_base;

/* 位置环坐标（编码器方向，进入时 = 单圈角，之后连续） */
static float pos_coord_now(void)
{
#if (SERVO_POS_COORD_AT_ENTRY != 0)
    return BspServo_GetThetaMulti() - s_pos_base;
#else
    return BspServo_GetThetaMulti();
#endif
}

/* 把坐标对齐到当前单圈角，返回**旧基准**（调用方用它平移目标）。
 *
 * 只能在命令 / 1 ms 上下文调用：这里要用 BspServo_GetThetaCached()，
 * 不能调 BspServo_GetThetaRad() —— 后者会推进测速、pos_sum、theta_multi，
 * 只允许在 20 kHz 控制节拍里跑（见 bsp_servo.c 的注释）。
 *
 * ⚠ 必须**关中断**读这两个角（2026-11-21 审出）。
 *   theta_multi 与 theta（单圈）都在 20 kHz 的 BspServo_GetThetaRad() 里更新。
 *   若分两次读且 ISR 正好插在中间，而那一拍又恰好跨过 2π（单圈角回绕），
 *   两者就不再是同一拍的量，算出的 s_pos_base 会差**整整 2π**，
 *   位置环一使能就朝反方向冲一圈。
 *   关中断窗口约 20 个周期（0.12 µs @170 MHz），且只在进入位置模式时执行一次。 */
/* 上一次下发的原始目标（未折算）。
 * keepalive 以 10 Hz 重发同一目标，若每次都按当前 now 重算圈基准，
 * now 一旦跨过 π，turn 就跳到下一圈、目标被推远一整圈 ——
 * 实测下发 3.1416 时电机在 3.2 附近来回乱走。
 * 相同输入直接忽略，保证"一个目标只折算一次"。 */
static float s_pos_cmd_last = -1.0e9f;
/* 上一次折算后的**内部目标**（连续坐标）。用于区分两种"输入值相同"：
 *   · keepalive 重发：实测仍贴着目标      -> 忽略（必须忽略，否则跨 π 跳圈）
 *   · 到位后被外力推走、用户又下发同一值  -> 必须重新定位
 * 判据：位置环处于 DONE 且实测已偏离该目标 50 mrad 以上。 */
static float s_pos_tgt_last = 0.0f;

/* 把任意角归一到 [0, 2π) */
static float norm_2pi(float a)
{
    float r = fmodf(a, BSP_TWO_PI);
    if (r < 0.0f) { r += BSP_TWO_PI; }
    return r;
}

static float pos_coord_align(void)
{
#if (SERVO_POS_COORD_AT_ENTRY != 0)
    const float    old = s_pos_base;
    const uint32_t pm  = __get_PRIMASK();
    float m, frac;

    __disable_irq();
    m = BspServo_GetThetaMulti();
    __set_PRIMASK(pm);

    /* 直接对 theta_multi 取 0~2π 余数，**不再用 m - theta_single**。
     *
     * 原来 base = m - s（s = 单圈角），数学上等于"圈数 x 2π"，两者本该等价。
     * 但 2026-11-22 实测出现过 p_now = 25.675（超过 4 圈）—— 说明 m 与 s
     * 在那一刻不是同一拍/同源的量，一相减就把圈数丢了（base≈0 → p_pos≈m），
     * 于是"进位置模式后 p_now 应在 0~2π"这个前提失效，用户下发 0 就要
     * 反向走 4 圈，一路冲到跟随误差超限（0x0100）。
     *
     * fmodf 只依赖一个量，天然同拍，结果必然落在 [0, 2π)，
     * 且丢掉关中断读第二个量的窗口（更短、更安全）。 */
    frac = fmodf(m, BSP_TWO_PI);
    if (frac < 0.0f) { frac += BSP_TWO_PI; }

    s_pos_base = m - frac;

    /* 新坐标系：允许下一次下发重新折算圈基准 */
    s_pos_cmd_last = -1.0e9f;
    s_pos_tgt_last = frac;
    return old;
#else
    return 0.0f;
#endif
}

void Servo_SetPosMode(uint8_t enable, float p_target_rad)
{
    const float ds = (float)s_dir_sign;

    /* p_target_rad 已完全废弃（2026-11-22）：起环一律"保持当前位置"，
     * 定位请调用方在起环后走 Servo_SetPosTarget()（它负责 0~2π 折算）。
     * 保留形参只为不动调用方。 */
    (void)p_target_rad;

    if (enable != 0U) {
        /* 每次使能/重起环都允许下一次 SetPosTarget 重新折算。
         * 否则"封锁 -> 重新使能"这条路上，目标已被重置成当前位置，
         * 而 s_pos_cmd_last 还留着上次的输入值，同样的目标会被去重忽略。 */
        s_pos_cmd_last = -1.0e9f;

        if (s_spd_prev != 0U) {
            /* 速度环必须显式关掉：ISR 的给定来源分支里 s_spd_en 优先于
             * s_pos_en，两个都开会得到"位置环使能了但完全不生效"，
             * 而且速度环积分还带着旧给定 —— 极难从现象反推。 */
            Servo_SetSpeedMode(0U, 0.0f);
        }
        if (s_pos_prev == 0U) {
            /* 起环：先把坐标对齐到单圈角，再把调用方给的目标平移到新坐标。
             *
             * p_target_rad 的约定是"**本坐标系的绝对位置**"（不是裸编码器角）。
             * 对齐后坐标系整体平移了 (new_base − old_base)，所以目标要同步平移，
             * 否则会错位 —— 2026-11-21 实测：do_enable 原来传裸
             * BspServo_GetThetaMulti()，第二次起环的目标偏了整整 −2π，
             * 一使能就反向冲一圈。现在调用方一律用 Servo_GetPosCoordNow()。 */
            /* 起环只做一件事：把坐标归一到 [0, 2π)，并以**当前位置**为起点。
             *
             * 2026-11-22 起不再做 "p_target -= (new_base - old_base)" 的平移。
             * 那个平移的前提是"调用方传的是旧坐标"，但 align 之后 p_pos
             * 重新从单圈角起算，旧坐标里的整圈部分全部作废 —— 实测转 12 圈后
             * 进位置模式，目标被平移成 100.1 而实际位置只有 25.7，差 74 rad。
             * 删掉平移又会让"没转过整圈"的路径错位，两头都不对。
             *
             * 根因是"目标坐标系"本身没有定义清楚。现在改成：
             *   · 起环：目标 = 当前位置（HOLD），不接收外部目标；
             *   · 要定位：调用方在起环**之后**调 Servo_SetPosTarget()，
             *     那个入口会把输入按"当前圈的 0~2π"折算（见其实现）。
             * 这样目标永远只有一个语义，不存在新旧坐标混用。 */
            (void)pos_coord_align();
            (void)p_target_rad;         /* 起环不再使用外部目标 */

            /* 起环：以**当前实测位置**为起点。
             * 若直接把 p_target 交给位置环，使能那一拍的误差就是
             * (p_target − p_now)，乘 25 直接顶到 v_max —— 相当于每次使能
             * 都先来一次满速冲击。 */
            {
                const float p_start = pos_coord_now();
                PosLoop_Enable(ds * p_start);
                /* Enable 把目标设成当前位置，同步记录，供去重逻辑判断
                 * "是否已被推离目标"。 */
                s_pos_tgt_last = p_start;
            }

            /* 速度环状态一并清零：位置环靠它执行，积分带着上次运行的
             * 残值会让第一拍就顶一个力矩。 */
            s_spd_i_term    = 0.0f;
            s_spd_iq        = 0.0f;
            s_spd_cnt       = 0U;
            /* 同上（2026-11-21）：位置环起环时 v_cmd 从 0 长起来，若峰值
             * 也从 0 起，则从速度模式带余速切过来的那一拍门限只有 10 rad/s，
             * 余速 262 rpm 就会立刻被判超速。用实测余速作起点。 */
            s_spd_wpeak     = fabsf((float)s_dir_sign * BspServo_GetOmegaMech());
            s_spd_ramp_init = 0U;
            s_pos_vcmd      = 0.0f;
            cur_reset_exec();          /* 执行层由 src=POS 派生，只清积分 */
        }
    } else {
        PosLoop_Disable();
        s_pos_vcmd = 0.0f;
        if ((s_pos_prev != 0U) && (s_spd_prev == 0U)) {
            Servo_SetCurrentMode(0U, 0.0f);
        }
    }

    s_pos_prev = (enable != 0U) ? 1U : 0U;
    if (enable != 0U) {
        src_enter(SERVO_SRC_POS);          /* POS 派生 spd+cur，三层一起开 */
    } else if (s_src == SERVO_SRC_POS) {
        src_enter(SERVO_SRC_NONE);
    }
}

uint8_t Servo_IsPosMode(void) { return s_pos_en; }

/* 复位"同一目标去重"锁存（2026-11-22）。
 *
 * 为什么需要：位置环清零故障时 PosLoop_ClearFault() 会把目标重置成
 * **当前位置**（s_hold_req + s_restart），但 s_pos_cmd_last 还留着上一次的
 * 输入值。用户随后重发**同一个目标**（例如又一次输入 0.0000）时，
 * Servo_SetPosTarget 会判"输入没变"直接 return —— 目标根本没设进去，
 * 现象是"清故障后下发没反应"，或者位置环仍按旧目标跑而二次触发 0x0100。
 * 清故障后必须允许重新折算一次。 */
void Servo_ResetPosCmdLatch(void)
{
    s_pos_cmd_last = -1.0e9f;
    s_pos_tgt_last = pos_coord_now();
}

void Servo_SetPosTarget(float p_target_rad)
{
    /* 目标按"**当前所在圈的 0~2π**"解释（2026-11-22 需求）。
     *
     * 用户输入 0 就是"回到本圈的 0 点"，行程不超过 2π，与电机此前转过
     * 多少圈无关 —— 这就是"不计入之前累计圈速"的含义。
     *
     * 做法：把内部连续位置拆成 (圈起点 + 圈内偏移)，目标 = 圈起点 + 输入。
     *   例：p_pos = 7.50（第 2 圈内 1.22），输入 0.5
     *       -> 目标 = 6.283 + 0.5 = 6.783，行程 -0.717 rad（而不是跨圈 -7.0）
     *
     * 输入不在 [0, 2π) 时按**绝对位置**处理（保留跨圈定位能力）：
     *   输入 -0.5 -> 目标 -0.5（反向小步）；输入 12.0 -> 目标 12.0（跨圈）。
     */
    /* keepalive 重发同一目标：**运动中一律忽略**，绝不重新折算。
     * 原因见下方 roundf 的注释（重算会让 now 跨过 π 时跳一整圈）。
     *
     * 例外（2026-11-22 补）：位置环已 DONE、但实测被外力推离上次目标
     * 50 mrad 以上时，必须允许重新定位 —— 否则"到位后被推开、再下发
     * 同一个值"会被静默忽略，表现为指令没反应。 */
    if (fabsf(p_target_rad - s_pos_cmd_last) <= 1.0e-6f) {
        if (PosLoop_GetState() != POS_ST_DONE) { return; }
        if (fabsf(pos_coord_now() - s_pos_tgt_last) < 0.05f) { return; }
    }
    s_pos_cmd_last = p_target_rad;

    const float now = pos_coord_now();
    float tgt;

    if ((p_target_rad >= 0.0f) && (p_target_rad < BSP_TWO_PI)) {
        /* 在"圈内偏移 = 输入值"的所有候选点里，取**离当前实测最近**的一个：
         *     tgt = 输入值 + k x 2π,   k = round((now - 输入值) / 2π)
         *
         * 这个式子同时解决三个已实测的坑：
         *
         * ① 不能用 `now - norm_2pi(now)` 做圈基准：now < 0（过冲几个 mrad）
         *    时它算出 -2π，目标被推到前面一整圈，位置环以 v_max 再跑一圈，
         *    过冲又变负 —— keepalive 10 Hz 重发，每拍跳一圈，实测 wref
         *    长期钉在 -238.7 rpm，就是在 0 附近无限绕圈。
         *
         * ② 不能用 `roundf(now/2π)*2π + 输入`：那个"圈基准"只看 now，
         *    与输入值无关。输入 6.0 且实测越过 2π 一点（6.06）时，
         *    roundf 进位到下一圈 -> 目标 = 12.28，白走一整圈。
         *    实测 C 段：起点 2.71 下发 6.0，累计弧长 12.06 rad（近两圈）。
         *
         * ③ 最短行程：k 取最近，行程必然 <= π。输入 0 而实测在 6.2 时
         *    目标 = 6.283（正向走 0.08），不会反向走 6.2。
         *
         * 只要 now 变化不超过 π，k 就不变 —— 所以 keepalive 重发时目标稳定。 */
        tgt = p_target_rad + roundf((now - p_target_rad) / BSP_TWO_PI) * BSP_TWO_PI;
    } else {
        tgt = p_target_rad;                 /* 其它：绝对位置，允许跨圈 */
    }

    s_pos_tgt_last = tgt;
    PosLoop_SetTarget((float)s_dir_sign * tgt);
}

void Servo_SetPosHold(void) { PosLoop_Hold(); }

void Servo_GetPosState(float *p_ref, float *p_err, float *v_ff,
                       float *p_tgt, float *vmax, uint8_t *st)
{
    const float ds = (float)s_dir_sign;

    /* 对外一律用**编码器坐标系**，与 30100 的 p_now / theta_multi 同号。
     * dir 只在控制内部出现，诊断输出再翻回去，否则 dir=-1 时
     * 上位机看到的位置误差符号是反的。
     * 注意：这里报的是位置环内部量，坐标系已按 s_pos_base 对齐过，
     * 与裸 theta_multi 差一个常数（见 pos_coord_now 的说明）。 */
    if (p_ref) { *p_ref = ds * PosLoop_GetRef(); }
    if (p_err) { *p_err = ds * PosLoop_GetErr(); }
    if (v_ff)  { *v_ff  = ds * PosLoop_GetVff(); }
    if (p_tgt) { *p_tgt = ds * PosLoop_GetTarget(); }
    if (vmax)  { *vmax  = PosLoop_GetVmax(); }
    if (st)    { *st    = PosLoop_GetState(); }
}

/* 位置环诊断快照：30100 AppPosStatus 的唯一数据源（2026-11-21 新增）。
 * p_now 取 theta_multi 而不是位置环内部的 ds*theta_multi —— 对外一律用编码器
 * 坐标系，与 30090.pos / 30040.enc_sum 同号，dir=-1 时上位机不会看反。
 * 其余位置量由 PosLoop 内部按模型坐标系保存，这里用 ds 翻回来。 */
void Servo_GetPosDiag(ServoPosDiag_t *d)
{
    const float ds = (float)s_dir_sign;

    if (d == 0) { return; }

    /* p_now / p_tgt / p_ref 一律归一到 [0, 2π)：用户看到什么就能输入什么。
     * err 保持**真实差值**不归一化，否则 -3.14 的误差会显示成 +3.14，
     * 现象是"越走误差越大"，没法看。 */
    d->p_now     = norm_2pi(pos_coord_now());
    d->p_tgt     = norm_2pi(ds * PosLoop_GetTarget());
    d->p_tgt_c   = norm_2pi(ds * PosLoop_GetTargetComp());
    d->p_ref     = norm_2pi(ds * PosLoop_GetRef());
    d->err       = ds * PosLoop_GetErr();
    d->i_term    = ds * PosLoop_GetITerm();
    d->travel    = PosLoop_GetTravel();          /* 行程取绝对值，与方向无关 */
    d->kp        = PosLoop_GetKp();
    d->vmax      = PosLoop_GetVmax();
    d->errmax    = PosLoop_GetErrMax();
    d->fault_cnt = PosLoop_GetFaultCnt();
    d->st        = PosLoop_GetState();
    d->settled   = PosLoop_IsSettled();
    d->comp_en   = PosLoop_GetCompEn();
}

float Servo_GetPosCoordNow(void) { return pos_coord_now(); }

float Servo_GetPosKp(void)     { return PosLoop_GetKp(); }
void  Servo_SetPosKp(float kp) { PosLoop_SetKp(kp); }


void Servo_SetSpeedGains(float kp, float ki_ts)
{
    /* 上限按"模型值的 32 倍"留：模型 Kp=0.0729 对应 J≈5e-4 时只有 2.4 Hz，
     * 若实测电机惯量小得多，需要放大到几十倍才能看到正常带宽。
     * 下限不为 0：全 0 会让速度环变成纯开环，现象是"给了指令电机不转也不报错"。 */
    if (kp    < 0.002f)   { kp    = 0.002f; }
    if (kp    > 20.0f)    { kp    = 20.0f; }
    if (ki_ts < 0.00005f) { ki_ts = 0.00005f; }
    if (ki_ts > 5.0f)     { ki_ts = 5.0f; }
    s_spd_kp    = kp;
    s_spd_ki_ts = ki_ts;
}

void Servo_GetSpeedGains(float *kp, float *ki_ts)
{
    if (kp)    { *kp    = s_spd_kp; }
    if (ki_ts) { *ki_ts = s_spd_ki_ts; }
}

void Servo_SetSpeedAccel(float acc_rad_s2)
{
    /* 上限放到"等效不限制"：调试时想看纯 PI 的阶跃响应就设到 20000。
     * 下限不为 0：全 0 会让给定永远不动，现象是"进环后电机纹丝不动也不报错"，
     * 与"参数没生效"完全无法区分。 */
    if (acc_rad_s2 < SERVO_SPD_ACC_MIN) { acc_rad_s2 = SERVO_SPD_ACC_MIN; }
    if (acc_rad_s2 > SERVO_SPD_ACC_MAX) { acc_rad_s2 = SERVO_SPD_ACC_MAX; }
    s_spd_acc = acc_rad_s2;
}

float Servo_GetSpeedAccel(void) { return s_spd_acc; }

/* 速度给定轨迹配置（2026-11-19）：mode 0=梯形 1=S 曲线；j_max rad/s^3。
 * 运行中改这两项不重建 ramp（保留 v/a），不会造成速度跳变。 */
void Servo_SetSpeedProfile(uint8_t mode, float j_max)
{
    s_spd_profile_mode = (mode == (uint8_t)PROFILE_MODE_SCURVE)
                       ? (uint8_t)PROFILE_MODE_SCURVE
                       : (uint8_t)PROFILE_MODE_TRAP;
    if (j_max > 0.0f) { s_spd_jmax = j_max; }
}
void Servo_GetSpeedProfile(uint8_t *mode, float *j_max)
{
    if (mode)  { *mode  = s_spd_profile_mode; }
    if (j_max) { *j_max = s_spd_jmax; }
}

/* ---- 轨迹时间优先（30097）----
 * t_ramp_s <= 0 关掉；> 0 时按"从这一刻起到目标的时间"反算 a/j。
 * 只存 RAM：这是个调试/整定参数，掉电后回落到 30019 的 acc + 30089 的 j。 */
void Servo_SetSpeedTrajTime(float t_ramp_s)
{
    if (t_ramp_s != t_ramp_s) { return; }        /* NaN 拒 */
    if (t_ramp_s < 0.0f)      { t_ramp_s = 0.0f; }
    if (t_ramp_s > 60.0f)     { t_ramp_s = 60.0f; }
    s_spd_t_ramp  = t_ramp_s;
    s_spd_w_latch = 1.0e9f;      /* 强制下一拍重锁，改完立即按新时间算 */
}
float Servo_GetSpeedTrajTime(void) { return s_spd_t_ramp; }

/* 实际生效的 a/j，以及按当前剩余量估的剩余时间。
 * t_left 的定义：梯形 Δ/a；S 曲线 2Δ/a（S 曲线的平均加速度约为峰值的 1/2）。
 * flags：bit0 配置了时间模式、bit1 反算值被夹、bit2 当前是 S 曲线、
 *        bit3 配置了时间模式但当前在位置环被屏蔽（不生效）。
 *        分开报是为了区分"没配"和"配了但被忽略"。 */
void Servo_GetSpeedTrajActual(float *a, float *j, float *t_left,
                              uint8_t *flags, uint8_t *mode)
{
    /* 两个量必须在同一坐标系里比：s_spd_ref_raw 是用户坐标系（编码器正方向），
     * s_spd_ref_ramp 是模型坐标系（dir 已乘）。少这一次换算，dir=-1 时
     * 剩余量会算成"两者相加"，t_left 直接翻倍。 */
    const float dw = fabsf(((float)s_dir_sign * s_spd_ref_raw) - s_spd_ref_ramp);

    if (a)      { *a = s_spd_a_latch; }
    if (j)      { *j = s_spd_j_latch; }
    if (mode)   { *mode = s_spd_profile_mode; }
    if (t_left) {
        if (s_spd_a_latch > 1.0e-3f) {
            *t_left = dw / s_spd_a_latch;
            if (s_spd_profile_mode == (uint8_t)PROFILE_MODE_SCURVE) {
                *t_left *= 2.0f;
            }
        } else {
            *t_left = 0.0f;
        }
    }
    if (flags) {
        const uint8_t t_on = (s_spd_t_ramp > 1.0e-4f) ? 1U : 0U;

        *flags = (uint8_t)(t_on
                         | (s_spd_t_clamp ? 2U : 0U)
                         | ((s_spd_profile_mode == (uint8_t)PROFILE_MODE_SCURVE) ? 4U : 0U)
                         | ((t_on && (s_pos_en != 0U)) ? 8U : 0U));
    }
}

/* 对外统一用**用户坐标系**（编码器正方向），与 Servo_GetSpeedState 的 w_ref 一致。
 * 内部 s_spd_ref_ramp 在模型坐标系里推进（那里 dir 已乘），所以出口要还原一次 ——
 * 否则 dir=-1 时上位机会看到"给定 +30、斜坡 -30"，误判成振荡。 */
float Servo_GetSpeedRampRef(void) { return (float)s_dir_sign * s_spd_ref_ramp; }

void Servo_SetSpeedIqMax(float a)
{
    /* 上限 9 A = 模型给的设计值，也是"空载 1.4 N·m 冲击"的边界；
     * 下限 0.1 A：再小连静摩擦都克服不了，现象是"进环了但电机不转"。 */
    if (a < 0.1f) { a = 0.1f; }
    if (a > 9.0f) { a = 9.0f; }
    s_spd_iqmax = a;
}

float Servo_GetSpeedIqMax(void) { return s_spd_iqmax; }

void Servo_SetFeedforwardBoost(float boost)
{
    /* 0 = 不做起动助推（退回只补动摩擦）。上限 3：再多就不是补偿静摩擦，
     * 而是给了 4 倍摩擦，低速会明显冲一下。 */
    if (boost < 0.0f) { boost = 0.0f; }
    if (boost > 3.0f) { boost = 3.0f; }
    s_ff_boost = boost;
}

float Servo_GetFeedforwardBoost(void) { return s_ff_boost; }

void Servo_SetFeedforward(uint8_t en, float acc_gain, float fric_a, float visc)
{
    /* 上下限都留了"关掉"的位置：acc=0 且 fric=0 等价于关闭，但保留 en 标志，
     * 这样上位机可以一键关掉再打开，不用重设三个数。 */
    if (acc_gain < 0.0f)    { acc_gain = 0.0f; }
    if (acc_gain > 2.0e-2f) { acc_gain = 2.0e-2f; }
    if (fric_a < 0.0f)      { fric_a = 0.0f; }
    if (fric_a > 1.5f)      { fric_a = 1.5f; }      /* 1.5 A ≈ 0.14 N·m，再大就不是"补偿"了 */
    if (visc < 0.0f)        { visc = 0.0f; }
    if (visc > 5.0e-3f)     { visc = 5.0e-3f; }

    s_ff_acc  = acc_gain;
    s_ff_fric = fric_a;
    s_ff_visc = visc;
    s_ff_en   = (en != 0U) ? 1U : 0U;
}

void Servo_GetFeedforward(uint8_t *en, float *acc_gain, float *fric_a, float *visc)
{
    if (en)       { *en       = s_ff_en; }
    if (acc_gain) { *acc_gain = s_ff_acc; }
    if (fric_a)   { *fric_a   = s_ff_fric; }
    if (visc)     { *visc     = s_ff_visc; }
}

void Servo_SetSpeedKb(float kb)
{
    /* 0 = 退回条件积分（老逻辑），>0 = back-calculation。
     * 上限 3：再大等于把积分直接等于饱和差值，会振荡。 */
    if (kb < 0.0f) { kb = 0.0f; }
    if (kb > 3.0f) { kb = 3.0f; }
    s_spd_kb = kb;
}

float Servo_GetSpeedKb(void)   { return s_spd_kb; }
float Servo_GetSpeedFF(void)   { return s_ff_last; }

/* 最近一拍的齿槽前馈量（A），诊断用 */
float Servo_GetSpeedCog(void)  { return s_spd_cog; }

/* ================= 5/7 次谐波电流抑制（MSRF，多同步旋转坐标系） =================
 *
 * 为什么是 5/7 次：三相电机反电势非正弦、逆变器死区、绕组分布不对称，
 * 主要产生 5 次（负序）与 7 次（正序）相电流谐波。它们在**静止 αβ 系**里是
 * 5ωe / 7ωe 的正弦量，但变到**基波 dq 系**之后都会折叠成 6ωe 的同一个脉动，
 * 表现为 iq 上的 6 次转矩脉动（14 极对、30 rpm 时电频率 7 Hz，6 次即 42 Hz）。
 *
 * 做法：不做"提取 6 次再反推"，而是把实测 αβ 电流分别变到
 *   -5θe（负序，5 次）与 +7θe（正序，7 次）两个坐标系，
 * 在那里 5/7 次分量变成**直流量**，各用一组 PI 压到 0，
 * 反变换回 αβ 后叠加到电压指令上。好处是频率随转速自动跟随，
 * 不需要像谐振控制器那样在变速时重算系数（谐振器离散化还有频率畸变）。
 *
 * 代价：每拍多 4 次 sinf/cosf + 2 组 PI，约 600 cycle（20 kHz 预算 8500）。
 *
 * 积分限幅取 0.15·vbus：谐波补偿是"锦上添花"，绝不能把电压预算吃光 ——
 * 主环还要留着压基波，尤其高速段 BEMF 已经占了大部分。
 */
static volatile uint8_t s_harm_en;
static volatile float   s_harm_kp    = 0.10f;    /* V/A，约为基波 Kp(0.713) 的 1/7 */
static volatile float   s_harm_ki_ts = 0.004f;   /* 每拍积分增量 */
static float    s_h5_vd_int, s_h5_vq_int;
static float    s_h7_vd_int, s_h7_vq_int;
static volatile float s_h5_id, s_h5_iq, s_h7_id, s_h7_iq;

/* ---------------- 高速波形记录（谐波分析用） ----------------
 * 100 Hz 遥测看不了 6 次谐波：30 rpm 时理论 34 Hz 已经贴着 Nyquist，
 * 更高转速直接混叠。所以这里在 20 kHz 节拍里直接采 iq，存 1024 点（51 ms），
 * 分辨率 19.5 Hz、覆盖 0~10 kHz，任何转速的 6 次谐波都落在范围内。
 * 数据经命令层以 ASCII hex 逐行送出（每点 4 位十六进制，131 % 缩放）。
 */
#define REC_N   (1024U)

static volatile uint8_t s_rec_en;
static volatile uint16_t s_rec_n;
static int16_t s_rec_buf[REC_N];

/* 录制分频：每 div 个控制拍存一点。
 *
 * 为什么需要：1024 点 @20 kHz 只有 51 ms。100 rpm 的电周期是 43 ms，
 * 也就是说整段只覆盖 1.2 个周期，140 Hz（6 次）的幅值估计完全不可信 ——
 * 上一轮死区标定时 6 次分量就是这么测不出来的。
 *
 * div=5 → 4 kHz，1024 点 = 256 ms ≈ 5.9 个电周期，分辨率 3.9 Hz，
 * 116.7 / 140.0 / 163.3 Hz（5/6/7 次）可以干净分开。
 * div=1 保持原行为（20 kHz），5/7 次谐波抑制那套脚本继续能用。 */
static volatile uint8_t s_rec_div = 1U;
static uint8_t s_rec_cnt;

/* 录制通道：0 = iq（原行为），1 = 相电流 iu + 电角度 theta，2 = vq 积分项。
 *
 * 为什么需要第二通道：要判断"电流过零附近是否出现死区平台"，
 * 必须看 abc 原始波形；而要把它按周期叠加降噪，就必须同时拿到**同一拍的电角度**。
 * 只存电流、用时间反推角度是不行的 —— 100 rpm 的转速波动有 9%，
 * 6 个周期攒下来相位会糊掉，平均等于白做。
 *
 * 2026-11-20 加第三通道（vq 积分，mV）：它是判断"BEMF 前馈是否与温度一致"
 * 的直接证据。vq 总和 = PI + 前馈，稳态下 PI→0，总和只由真实 BEMF 决定，
 * 所以看总和分辨不出前馈准不准；**积分项**才是"前馈估计误差的镜像"：
 *   · 前馈按固定 ψ 给（改前）：升温后前馈高估，int 被压成负值，
 *     2000 rpm / 84 °C 约 −0.98 V；
 *   · 前馈乘 ktc 后（改后）：int 回到 0 附近，温升只让前馈绝对值跟着变小。
 * 单位 mV（×1000），±32.7 V 量程，int16 不溢出。
 *
 * 角度编码：2π → 65536，存成 int16 的位模式（截断）。主机按 uint16 解，
 * × 2π/65536 还原。th_e 可能到 88 rad，乘 10430 后截断到 16 位正好等于
 * "mod 2π 再编码"，所以不需要额外的取模运算。 */
static volatile uint8_t s_rec_ch;
static int16_t s_rec_th[REC_N];

void Servo_SetRecordChannel(uint8_t ch) { s_rec_ch = (ch > 3U) ? 3U : ch; }
uint8_t Servo_GetRecordChannel(void)    { return s_rec_ch; }
int16_t Servo_GetRecordThetaAt(uint16_t i) { return (i < REC_N) ? s_rec_th[i] : 0; }

void Servo_StartRecord(void)
{
    s_rec_n   = 0U;
    s_rec_cnt = 0U;
    s_rec_en  = 1U;
}

/* 提前停止：只清使能，s_rec_n 保留已采点数 —— 半截波形仍然可读，
 * 这正是"发现波形不对想立刻停下看"的用法。 */
void Servo_StopRecord(void)
{
    s_rec_en = 0U;
}

/* 1..100。分频越大，等效采样率越低、窗口越长。 */
void Servo_SetRecordDiv(uint8_t div)
{
    if (div < 1U)   { div = 1U; }
    if (div > 100U) { div = 100U; }
    s_rec_div = div;
}

uint8_t Servo_GetRecordDiv(void) { return s_rec_div; }

uint8_t  Servo_IsRecording(void)      { return s_rec_en; }
uint16_t Servo_GetRecordN(void)       { return s_rec_n; }
int16_t  Servo_GetRecordAt(uint16_t i){ return (i < REC_N) ? s_rec_buf[i] : 0; }

void Servo_SetHarmonic(uint8_t enable)
{
    if (enable == 0U) {
        /* 关掉必须清积分，否则下次开起来会带着上次的残留量冲击 */
        s_h5_vd_int = 0.0f; s_h5_vq_int = 0.0f;
        s_h7_vd_int = 0.0f; s_h7_vq_int = 0.0f;
        s_h5_id = 0.0f; s_h5_iq = 0.0f;
        s_h7_id = 0.0f; s_h7_iq = 0.0f;
    }
    s_harm_en = (enable != 0U) ? 1U : 0U;
}

uint8_t Servo_IsHarmonic(void) { return s_harm_en; }

void Servo_GetHarmonicIdq(float *id5, float *iq5, float *id7, float *iq7)
{
    if (id5) { *id5 = s_h5_id; }
    if (iq5) { *iq5 = s_h5_iq; }
    if (id7) { *id7 = s_h7_id; }
    if (iq7) { *iq7 = s_h7_iq; }
}

void Servo_GetSpeedState(float *w_ref, float *iq_cmd, float *i_term, uint32_t *exec,
                         float *w_ramp)
{
    if (w_ref)  { *w_ref  = s_spd_ref_raw; }
    if (iq_cmd) { *iq_cmd = s_spd_iq; }
    if (i_term) { *i_term = s_spd_i_term; }
    if (exec)   { *exec   = s_spd_exec; }
    /* 斜坡后的给定：与 w_ref 一起看才能分辨"给定没变"和"给定变了但斜坡还没走完" */
    if (w_ramp) { *w_ramp = Servo_GetSpeedRampRef(); }
}

/*
 * 速度环单拍。20 kHz 上下文调用，内部 10 分频到 2 kHz。
 * w_ref / w_meas 必须同坐标系（都用模型坐标系，即已乘 dir_sign）——
 * 由调用方保证，因为 dir 只对"电角度/力矩"有意义，对编码器原始计数没有。
 */
/* ---------------- 速度环陷波（2026-11-21）----------------
 *
 * 用途：抑制机械谐振。谐波减速器 + 负载的扭转谐振通常在 100~400 Hz，
 * 速度环在谐振点附近增益不足会把扰动放大成持续振荡（iq 上看得到、
 * 但降 Kp 又牺牲带宽）。陷波把**反馈**在 f0 附近的增益压掉，
 * 环路其它频段不动。
 *
 * 为什么加在反馈上而不是误差上：加在误差上等于同时改变给定响应，
 * 会把参考信号在 f0 处一起削掉；加在反馈上是标准做法，只削环路增益。
 *
 * 深度用"混合"实现而不是直接改 Q：y = (1−k)·x + k·H(z)·x，
 * 极点仍是同一个二阶系统，分子被打浅 —— 深度可连续调（0 dB = 直通），
 * 不会因为 Q 太大而在 f0 附近产生相位突变。
 *
 * 系数只在参数变化时算一次（2 kHz 里不做 sin/cos），运行时每拍 5 乘 4 加。
 * 参数只存 RAM：落盘要动 ParamBlob 版本（会作废现有 C/Z 标定），
 * 等双页乒乓一起做。上电默认关闭。 */
static volatile uint8_t s_nf_en;
static volatile float   s_nf_f0     = 150.0f;   /* Hz */
static volatile float   s_nf_depth  = 12.0f;    /* dB */
static volatile float   s_nf_q      = 2.0f;
static float s_nf_b0, s_nf_b1, s_nf_b2, s_nf_a1, s_nf_a2;
static float s_nf_x1, s_nf_x2, s_nf_y1, s_nf_y2;

static void notch_recalc(void)
{
    const float fs = 1.0f / SERVO_SPD_TS;              /* 2 kHz */
    float f0 = s_nf_f0, q = s_nf_q, dep = s_nf_depth;

    if (f0 < 5.0f)        { f0 = 5.0f; }
    if (f0 > 0.45f * fs)  { f0 = 0.45f * fs; }         /* 留到 900 Hz，避开 Nyquist */
    if (q < 0.3f)         { q = 0.3f; }
    if (q > 10.0f)        { q = 10.0f; }
    if (dep < 0.0f)       { dep = 0.0f; }
    if (dep > 40.0f)      { dep = 40.0f; }

    {
        const float w0 = 6.28318531f * f0 / fs;
        const float cw = cosf(w0);
        const float al = sinf(w0) / (2.0f * q);
        const float a0 = 1.0f + al;

        /* 归一化（a0 = 1）的完整陷波 */
        const float b0n = 1.0f / a0;
        const float b1n = -2.0f * cw / a0;
        const float b2n = 1.0f / a0;
        const float a1n = -2.0f * cw / a0;
        const float a2n = (1.0f - al) / a0;

        /* 深度混合：k=0 直通，k=1 完整陷波 */
        const float k = 1.0f - powf(10.0f, -dep / 20.0f);

        s_nf_b0 = (1.0f - k) + k * b0n;
        s_nf_b1 = k * b1n;
        s_nf_b2 = k * b2n;
        s_nf_a1 = a1n;
        s_nf_a2 = a2n;
    }

    s_nf_x1 = s_nf_x2 = s_nf_y1 = s_nf_y2 = 0.0f;     /* 改参数必须清状态 */
}

void Servo_SetNotch(uint8_t en, float f0_hz, float depth_db, float q)
{
    const float fs = 1.0f / SERVO_SPD_TS;

    /* 先夹再存：回读必须是**实际生效**的值。原来只在 recalc 的局部变量里夹，
     * 结果上位机写 f0=5000 读回 5000、而环路里跑的是 900 —— 正是"两处不同源"。 */
    if (f0_hz < 5.0f)       { f0_hz = 5.0f; }
    if (f0_hz > 0.45f * fs) { f0_hz = 0.45f * fs; }
    if (q < 0.3f)           { q = 0.3f; }
    if (q > 10.0f)          { q = 10.0f; }
    if (depth_db < 0.0f)    { depth_db = 0.0f; }
    if (depth_db > 40.0f)   { depth_db = 40.0f; }

    s_nf_f0    = f0_hz;
    s_nf_depth = depth_db;
    s_nf_q     = q;
    notch_recalc();
    s_nf_en    = (en != 0U) ? 1U : 0U;
}

void Servo_GetNotch(uint8_t *en, float *f0_hz, float *depth_db, float *q)
{
    if (en)       { *en       = s_nf_en; }
    if (f0_hz)    { *f0_hz    = s_nf_f0; }
    if (depth_db) { *depth_db = s_nf_depth; }
    if (q)        { *q        = s_nf_q; }
}

static float notch_apply(float x)
{
    const float y = s_nf_b0 * x + s_nf_b1 * s_nf_x1 + s_nf_b2 * s_nf_x2
                  - s_nf_a1 * s_nf_y1 - s_nf_a2 * s_nf_y2;

    s_nf_x2 = s_nf_x1; s_nf_x1 = x;
    s_nf_y2 = s_nf_y1; s_nf_y1 = y;
    return y;
}

static float Servo_SpeedLoopStep(float w_ref, float w_meas)
{
    s_spd_cnt++;
    if (s_spd_cnt >= SERVO_SPD_DIV) {
        float acc_ref = 0.0f;

        s_spd_cnt = 0U;
        s_spd_exec++;

        /* 陷波只作用在反馈上，且在 2 kHz 的环内节拍里跑（fs = 1/TS） */
        if (s_nf_en != 0U) { w_meas = notch_apply(w_meas); }

        /* ---- ① 给定斜坡：阶跃 → 受 accel 限制的斜坡 ----
         * 首次进入用实测速度起步（不是 0），否则"加斜坡"只是把阶跃推迟一拍。
         * 2026-11-19：改用 ramp_step，支持梯形(默认，等价原线性斜坡)/S 曲线。 */
        if (s_spd_ramp_init == 0U) {
            s_spd_ref_ramp  = w_meas;
            ramp_init(&s_spd_ramp, (profile_mode_t)s_spd_profile_mode,
                      SERVO_SPD_TS, s_spd_acc, s_spd_jmax);
            s_spd_ramp.v = w_meas;   /* 从实测速度起步，切环不跳变 */
            s_spd_ramp_init = 1U;
        }
        {
            const float prev = s_spd_ref_ramp;

            /* a_max / j_max / mode 可运行时改：只更新 ramp 参数，不 ramp_init，
             * 保留 v/a，运行中改配置不会产生速度阶跃。
             *
             * 2026-11-21：新增"时间优先"（30097）。时间模式打开时用反算并
             * **锁存**的 a/j 覆盖配置值；关掉时立刻回到 s_spd_acc / s_spd_jmax，
             * 所以运行中切换不会残留上一段的加速度。 */
            if ((s_spd_t_ramp > 1.0e-4f) && (s_pos_en == 0U)) {
                if (fabsf(w_ref - s_spd_w_latch) > SERVO_SPD_T_RELOCK) {
                    const float dw = fabsf(w_ref - s_spd_ref_ramp);
                    const float t  = s_spd_t_ramp;
                    float a, j;

                    s_spd_w_latch   = w_ref;
                    s_spd_t_clamp   = 0U;

                    if (dw < SERVO_SPD_T_DWMIN) {
                        /* 斜坡已到位（稳态）。这里**不能**直接回退到 s_spd_acc/jmax：
                         * 那样上位机改完 t 回读到的 a_eff/j_eff 永远是配置值，
                         * 看着就是"时间修改不起作用"
                         * （实测：界面 t=0.40、固件 t=60、回读 a_eff=48 rpm/s=cfg）。
                         *
                         * 改成按"从零到目标"的名义 Δω 反算，让回读始终与 t 自洽。
                         * 不影响实际轨迹：真正驱动斜坡的是重锁后锁存的 latch，
                         * 而目标一变（> SERVO_SPD_T_RELOCK）就会按**真实剩余量**
                         * 重新反算并覆盖它。
                         * 目标本身接近 0（停机）时没有名义行程，仍回退配置值。 */
                        const float dw_nom = fabsf(w_ref);

                        if (dw_nom < SERVO_SPD_T_DWMIN) {
                            a = s_spd_acc;
                            j = s_spd_jmax;
                        } else if (s_spd_profile_mode == (uint8_t)PROFILE_MODE_SCURVE) {
                            a = 2.0f * dw_nom / t;
                            j = 4.0f * dw_nom / (t * t);
                        } else {
                            a = dw_nom / t;
                            j = s_spd_jmax;
                        }
                    } else if (s_spd_profile_mode == (uint8_t)PROFILE_MODE_SCURVE) {
                        a = 2.0f * dw / t;
                        j = 4.0f * dw / (t * t);
                    } else {
                        a = dw / t;
                        j = s_spd_jmax;
                    }

                    if (a < SERVO_SPD_ACC_MIN) { a = SERVO_SPD_ACC_MIN; }
                    if (a > SERVO_SPD_ACC_MAX) { a = SERVO_SPD_ACC_MAX; s_spd_t_clamp = 1U; }
                    if (j < 1.0f)              { j = 1.0f; }
                    if (j > SERVO_SPD_JMAX_MAX){ j = SERVO_SPD_JMAX_MAX; s_spd_t_clamp = 1U; }

                    s_spd_a_latch = a;
                    s_spd_j_latch = j;
                }
                s_spd_ramp.a_max = s_spd_a_latch;
                s_spd_ramp.j_max = s_spd_j_latch;
            } else {
                /* 位置模式下内环斜坡的 a_max 不能取 s_spd_acc（2026-11-21 修）。
                 *
                 * 位置环的输出是**速度给定**，同样要过这道斜坡。而位置规划器
                 * 已经按 POS_AMAX（200 rad/s²）推进，若 s_spd_acc 被速度模式
                 * 调小（例如 20），内环跟不上规划 → 误差持续累积 → 超过
                 * POS_ERRMAX 持续 100 ms → 落 FOLLOW 故障，电机停在半路。
                 *
                 * 位置环自身的输出连续（规划器限了加速度，起点用实测速度），
                 * 不存在阶跃，所以斜坡在这里只需要"不成为瓶颈"：取两者较大值，
                 * 既保留平滑与前馈 acc 的作用，又不会被速度模式的配置卡住。
                 * 速度模式行为不变（s_pos_en == 0 时仍是 s_spd_acc）。 */
                float a_use = s_spd_acc;

                if ((s_pos_en != 0U) && (a_use < POS_AMAX_DFLT)) {
                    a_use = POS_AMAX_DFLT;
                }
                s_spd_w_latch   = w_ref;
                s_spd_a_latch   = a_use;
                s_spd_j_latch   = s_spd_jmax;
                s_spd_t_clamp   = 0U;
                s_spd_ramp.a_max = a_use;
                s_spd_ramp.j_max = s_spd_jmax;
            }
            s_spd_ramp.mode  = (profile_mode_t)s_spd_profile_mode;
            s_spd_ref_ramp = ramp_step(&s_spd_ramp, w_ref);

            acc_ref = (s_spd_ref_ramp - prev) / SERVO_SPD_TS;
        }

        /* ---- ② 前馈 ----
         * 加速度项要低通：斜坡结束时 acc_ref 从常数突降为 0，
         * 不滤的话前馈瞬间消失会让 iq 掉一个台阶，转速跟着跌落一下
         * （现象是"到速瞬间抖一下"）。 */
        {
            float ff = 0.0f;

            s_ff_acc_f += SERVO_FF_ACC_LPF * (acc_ref - s_ff_acc_f);

            if (s_ff_en != 0U) {
                ff += s_ff_acc * s_ff_acc_f;
                ff += s_ff_fric * tanhf(s_spd_ref_ramp / SERVO_FF_W_EPS);
                ff += s_ff_visc * s_spd_ref_ramp;

                /* 起动助推：实测转速还不到给定 30% 时，额外补一份摩擦前馈。
                 * 静摩擦是动摩擦的 2 倍，只补动摩擦的话起动仍靠积分爬，
                 * 爬的过程就是实测 68.8% 超调的来源（仿真确认：只加摩擦前馈
                 * 超调 68.5% 不变，加助推后降到 27.7%）。
                 * 稳态时 w_meas ≈ w_ramp，判据自然失效，不会过补偿。 */
                if (fabsf(w_meas) < (SERVO_FF_START_RATIO * fabsf(s_spd_ref_ramp))) {
                    ff += s_ff_fric * s_ff_boost * tanhf(s_spd_ref_ramp / SERVO_FF_W_EPS);
                }
            }
            s_ff_last = ff;
        }

        /* ---- ③ PI + 抗饱和（增益按工作点查表）---- */
        {
            const float err   = s_spd_ref_ramp - w_meas;
            const float imax  = s_spd_iqmax;
            const float ff    = s_ff_last;
            float kp, kits;
            float u_raw, u_sat;

            if (s_sched_en != 0U) {
                sched_lookup(fabsf(s_spd_ref_ramp), &kp, &kits);
                /* 记下来供 K 命令回显：遇到"增益看着对但响应不对"时，
                 * 第一件要确认的事就是实际查到了哪一组。 */
                s_sched_kp   = kp;
                s_sched_kits = kits;
            } else {
                kp   = s_spd_kp;
                kits = s_spd_ki_ts;
            }

            /* 前馈加在 PI 之外，所以饱和判断必须用"含前馈"的总量：
             * 前馈自己把输出顶到限幅时，积分同样要停止累积。 */
            u_raw = (kp * err) + s_spd_i_term + ff;
            u_sat = u_raw;
            if (u_sat >  imax) { u_sat =  imax; }
            if (u_sat < -imax) { u_sat = -imax; }

            if (s_spd_kb > 0.0f) {
                /* back-calculation：把"被削掉的量"按 Kb 回灌积分。
                 * Kb 太大（>1）会让积分被前馈拖着走，太小退化成冻结积分；
                 * 0.3 对应跟踪时间常数 Tt = Ts/Kb ≈ 1.7 ms。 */
                s_spd_i_term += (kits * err) + (s_spd_kb * (u_sat - u_raw));
            } else {
                /* Kb=0：退回原来的条件积分（饱和且误差同向时冻结） */
                if (!((u_sat != u_raw) && ((err * u_raw) > 0.0f))) {
                    s_spd_i_term += kits * err;
                }
            }
            if (s_spd_i_term >  imax) { s_spd_i_term =  imax; }
            if (s_spd_i_term < -imax) { s_spd_i_term = -imax; }

            /* 齿槽前馈：按机械角查表，直接抵消位置相关的周期扰动。
             * 它不进入积分（积分是独立变量），所以 PI 不会把它"学掉"。
             * 速度门控在 CogFf_Get 内部：<20 rad/s 全量、>52 rad/s 关闭。 */
            /* 齿槽前馈：**不按模式判断**，靠 CogFf_Get 内部的速度门控。
             *
             * 门控是 |w_ramp| 的函数：w=0 → g=0；10~191 rpm 全量；两端线性退出。
             * 于是两种工况天然分开：
             *   位置环静止保持 → s_spd_ref_ramp=0 → g=0 → 自动不补。
             *     这是必须的：静止时齿槽是一个**常值**力矩，速度环 PI 的积分
             *     会把它吃掉（实测保持误差 222 µrad，远小于齿槽直接作用量
             *     0.103 N·m / kp25 = 4.1 mrad，说明积分确实在兜）。
             *     此时若强补，前馈注入的位置相关扰动没有积分缓冲，与位置刚度
             *     对着干 —— MIT 那轮强制传 COG_FF_W_FULL 就是这个下场
             *     （保持峰峰 7.2 → 14.2 mrad，见 MIT模式_实现与实测.md 9.3）。
             *   位置环定位运动 → w≠0（实测峰值 17 rad/s ≈162 rpm）→ 全量补偿。
             *     此时它和纯速度模式走的是**同一个 Servo_SpeedLoopStep、
             *     同一个门控、同一张表**，所以速度模式实测的 +21%(30rpm) /
             *     +40%(100rpm) 直接适用；外环换成位置 P 环不改变内环能力。
             *
             * 曾经写成 `if (s_pos_en == 0U)` 才补 —— 那是过度收窄：把位置环
             * 的定位段也一起排除了，而静止段的"不补"本来就被门控免费提供。 */
            s_spd_cog = CogFf_Get(BspServo_GetThetaMulti(), s_spd_ref_ramp);

            s_spd_iq = (kp * err) + s_spd_i_term + ff + s_spd_cog;
            if (s_spd_iq >  imax) { s_spd_iq =  imax; }
            if (s_spd_iq < -imax) { s_spd_iq = -imax; }
        }
    }
    return s_spd_iq;
}


/* ---------------- 速度模式 API ---------------- */
void Servo_SetMode(uint8_t mode)
{
    if (mode == s_mode) {
        return;
    }
    if (mode == SERVO_MODE_VELOCITY) {
        /* 以当前实际位置重建给定：位置环 P=50.265，跳变会直接变成满速指令 */
        VelProfile_Enable(&s_vel, BspServo_GetThetaMulti());
        s_pos_ref = VelProfile_GetPosRef(&s_vel);
    } else {
        VelProfile_Disable(&s_vel, BspServo_GetThetaMulti());
        s_pos_ref  = BspServo_GetThetaMulti();
        s_pos_w_ff = 0.0f;
    }
    s_mode = mode;
}

void  Servo_SetVelRef(float rad_s)        { VelProfile_SetTarget(&s_vel, rad_s); }
void  Servo_SetVelProfileMode(uint8_t m)  { VelProfile_SetMode(&s_vel, m); }
void  Servo_SetVelLimits(float a, float j){ VelProfile_SetLimits(&s_vel, a, j); }
float Servo_GetVelRef(void)               { return VelProfile_GetVref(&s_vel); }
float Servo_GetPosRef(void)               { return s_pos_ref; }
uint32_t Servo_GetVelClampCount(void)     { return VelProfile_GetClampCount(&s_vel); }

/* 最近一拍的三相电流 / Iq 给定 / 占空比快照；给遥测用，不参与控制 */
void Servo_GetSample(float *iu, float *iv, float *iw, float *iq_ref, float *duty)
{
    /* 三个 float 必须**原子**取出。
     *
     * s_iu/s_iv/s_iw 是在 20 kHz 的 Servo_ControlTick 里逐个更新的。若遥测
     * 任务（100 Hz）在"iu 已更新、iw 还没更新"的窗口里被 ISR 抢占，
     * 读到的三相就是新旧混合 —— Clarke/Park 之后 dq 会出现假尖峰。
     *
     * 实测症状：ISR 内部算的 iq_now 在 id=2 A 时 sd 只有 0.033 A，
     * 而遥测用这三个快照算出的 id，sd 是 0.13 A，差 4 倍。
     * 之前把这 0.13 当成"电流采样噪声大"，方向就找错了 ——
     * 采样本身是干净的，问题在读侧。
     *
     * 关中断窗口约 20 个周期（5 个 float 存取），对 50 µs 控制周期
     * 的影响可忽略；BspServo_GetPosSum 读 64 位累积量时用的是同一套办法。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (iu)     { *iu = s_iu; }
    if (iv)     { *iv = s_iv; }
    if (iw)     { *iw = s_iw; }
    if (iq_ref) { *iq_ref = s_iq_ref; }
    if (duty)   { duty[0] = s_duty[0]; duty[1] = s_duty[1]; duty[2] = s_duty[2]; }

    __set_PRIMASK(primask);
}

uint32_t Servo_GetInjectedMissCount(void) { return s_inj_miss; }
uint32_t Servo_GetMaxStepCycles(void)     { return s_max_cycles; }

/* ---------------- 注入转换完成：两路电流都新鲜才执行控制律 ---------------- */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        s_adc1_irq++;
        s_inj_flags |= 0x01U;
    } else if (hadc->Instance == ADC2) {
        s_adc2_irq++;
        s_inj_flags |= 0x02U;
    }

    if (s_inj_flags == 0x03U) {
        s_inj_flags = 0U;
        Servo_ControlTick();
    }
}

/* ---------------- 20kHz 控制节拍 ---------------- */
void Servo_ControlTick(void)
{
    uint32_t t0 = DWT->CYCCNT;
    uint32_t t1 = t0, t2 = t0, t3 = t0, t4 = t0, t5 = t0;
    float iu, iw;

    /* MIT 给定外推的时基：每拍无条件自增。
     * 放在分支之前而不是 MIT 分支里 —— 否则非 MIT 期间不推进，
     * 再次使能时"帧龄"会是一个巨大的值，外推一次就飞出去。 */
    s_mit_tick++;

    /* 相邻节拍间隔：dt_avg 就是真实控制周期，dt_min 反映触发抖动 */
    if (s_dt_cnt != 0U) {
        uint32_t gap = t0 - s_cyc_prev;
        if (gap < s_dt_min) {
            s_dt_min = gap;
        }
        s_dt_sum += gap;          /* 64 位累加：32 位在 25 s 就会溢出 */
    }
    s_cyc_prev = t0;
    s_dt_cnt++;

    /*
     * 位置/速度感知：每拍无条件推进一次，故障时也要知道转子在哪。
     * 这是 BspServo_GetThetaRad() 的**唯一**调用点——该函数内部会更新测速低通
     * 与多圈累积，多调一次就会用 50us 的分母去消化 1ms 的角差。
     */
    const float theta_now = BspServo_GetThetaRad();
    /* 多圈机械角：只给模型位置环的 theta_pos 用（theta_now 仍用于电角度，
     * 必须是单圈）。两者同零点、同步推进，所以 pos_ref - theta_multi 是真实误差。 */
    const float theta_multi = BspServo_GetThetaMulti();
    t1 = DWT->CYCCNT;

    BspServo_ReadInjected(&iu, &iw);
    t2 = DWT->CYCCNT;

    /* 双电阻采样：U 相实测、V 相由 Iu+Iw 重构、W 相实测（与下方赋值同一约定） */
    s_iu = iu;
    s_iv = -(iu + iw);
    s_iw = iw;

    /* 缺相检测累积（内部 10 kHz 抽取）。指令角：开环用虚拟角 s_ol_theta，
     * 闭环用实测多圈机械角 × 极对数 —— 不能用单圈角（越过 2π 会跳变）。 */
    {
        const float th_cmd = (s_ol_en != 0U)
                           ? s_ol_theta
                           : (SERVO_CUR_POLE * (float)s_dir_sign * theta_multi);
        Servo_PhaseMonTick(iu, iw, th_cmd);
    }

    if (BspServo_GetFault() != 0U) {
        /* 故障：不跑控制律，占空比回中并保持主输出封锁 */
        three_loop_deploy_U.en = 0.0;
        Servo_SetCurrentMode(0U, 0.0f);  /* 电流模式一并退出，避免 en=1/moe=0 的含糊状态 */
        BspServo_SetDuty(0.5, 0.5, 0.5);
        BspServo_PwmEnable(0U);
        s_duty[0] = s_duty[1] = s_duty[2] = 0.5f;
        t3 = t4 = DWT->CYCCNT;          /* 故障路径不跑控制律，分段计时归零 */
    } else if (s_ol_en != 0U) {
        /*
         * 虚拟电角度下的 dq 电流闭环：id -> s_ol_idref，iq -> 0。
         * 关键区别：Park/逆 Park 用的 θ 是 s_ol_theta（虚拟），**不读编码器**，
         * 所以完全不需要零点标定就能建立一个确定的磁场方向。
         * 电流由 PI 限制在几安，不像纯电压开环那样"电压小推不动、电压大就过流"。
         *
         *   ialpha = iu
         *   ibeta  = (iu + 2*iv)/sqrt(3)，代入 iv = -(iu+iw)
         *          = -(iu + 2*iw)/sqrt(3)
         */
        /* 开环匀速拖动：虚拟角在 ISR 里累加（原因见 Servo_SetOLSpin 注释）。
         * 这里**不读编码器**，所以编码器零点/线性度全错也能把转子拖起来 ——
         * 这是编码器自校准唯一可用的驱动通道。 */
        if (s_ol_spin != 0U) {
            float th = s_ol_theta + s_ol_w_e * SERVO_OL_TS;
            if (th >= 6.28318531f)     { th -= 6.28318531f; }
            else if (th < 0.0f)        { th += 6.28318531f; }
            s_ol_theta = th;
        }
        const float te = s_ol_theta;
        const float cs = cosf(te);
        const float sn = sinf(te);
        const float vb = BspServo_GetVbus();
        float ialpha = iu;
        float ibeta  = -0.57735027f * iu - 1.15470054f * iw;
        float id_now =  cs * ialpha + sn * ibeta;
        float iq_now = -sn * ialpha + cs * ibeta;
        float ed = s_ol_idref - id_now;
        float eq = 0.0f - iq_now;
        float vd, vq, vmax, vmag, valpha, vbeta, da, db, dc;

        s_ol_vd_int += ed * SERVO_OL_KI_TS;
        s_ol_vq_int += eq * SERVO_OL_KI_TS;
        vd = SERVO_OL_KP * ed + s_ol_vd_int;
        vq = SERVO_OL_KP * eq + s_ol_vq_int;

        vmax = (vb > 1.0f) ? (vb * s_ol_vlim) : 1.0f;   /* 运行时可变，见 Servo_SetOLVlim */
        vmag = sqrtf(vd * vd + vq * vq);
        if ((vmag > vmax) && (vmag > 0.0f)) {          /* 圆限幅 + 抗饱和 */
            float k = vmax / vmag;
            vd *= k;
            vq *= k;
            s_ol_vd_int *= 0.98f;
            s_ol_vq_int *= 0.98f;
        }

        valpha = cs * vd - sn * vq;
        vbeta  = sn * vd + cs * vq;

        da = 0.5f + (valpha / vb);
        db = 0.5f + ((-0.5f * valpha + 0.86602540f * vbeta) / vb);
        dc = 0.5f + ((-0.5f * valpha - 0.86602540f * vbeta) / vb);

        /* 死区补偿（移植自 FM_XC）：按各相电流方向补 Δd ≈ 1.34% */
        BspServo_SetDuty(DeadbandComp_Apply(da, s_iu),
                         DeadbandComp_Apply(db, s_iv),
                         DeadbandComp_Apply(dc, s_iw));
        s_duty[0] = da;
        s_duty[1] = db;
        s_duty[2] = dc;
        s_iq_ref  = 0.0f;
        /* 快照限幅后的 vd/vq：R 辨识要用 |v|，而 v 必须是实际送给调制的那个值
         * （限幅前的 PI 输出在饱和时会偏大，直接拿去算 R 会系统性偏高）。
         * 同时更新 s_cur_id/iq —— 原来只有电流环分支才写，导致开环模式下
         * Servo_GetIdqMeas() 永远返回 0，R 辨识会算出 R=0/0。 */
        s_ol_vd   = vd;
        s_ol_vq   = vq;
        s_cur_id  = id_now;
        s_cur_iq  = iq_now;
        t3 = t4 = DWT->CYCCNT;          /* 不跑模型，model/duty 分段不参与预算统计 */
    } else if (s_olv_en != 0U) {
        /*
         * 虚拟角下的 d 轴正弦电压注入：θ 固定 0，vd = amp·sin(ph)，vq = 0。
         * 不走电流环（PI 会把这种激励压掉），直接算占空比。
         * 记录每拍 id 给 L 拟合用；θ=0 时 Park 退化 => id = iα = iu。
         */
        const float vb = BspServo_GetVbus();

        s_olv_ph += s_olv_dph;
        if (s_olv_ph >= BSP_TWO_PI) { s_olv_ph -= BSP_TWO_PI; }

        {
            const float vdin = s_olv_amp * sinf(s_olv_ph);

            if (vb > 5.0f) {
                const float da = 0.5f + (vdin / vb);
                const float db = 0.5f - (0.5f * vdin / vb);
                const float dc = db;

                BspServo_SetDuty(da, db, dc);
                s_duty[0] = da;
                s_duty[1] = db;
                s_duty[2] = dc;

                /* θ=0：id = iu，iq = (iu + 2·iv)/√3 */
                s_cur_id = s_iu;
                s_cur_iq = 0.57735027f * s_iu + 1.15470054f * s_iw;

                RlIdent_PushInject(s_iu, s_olv_ph);
            } else {
                BspServo_SetDuty(0.5f, 0.5f, 0.5f);
                s_duty[0] = s_duty[1] = s_duty[2] = 0.5f;
            }
        }
        s_iq_ref = 0.0f;
        t3 = t4 = DWT->CYCCNT;
    } else if (s_cur_en != 0U) {
        /*
         * 编码器电角度下的 dq 电流闭环（力矩模式）：id → 0，iq → s_cur_iq_ref。
         * θ 与 three_loop_deploy.c:51 同源，所以依赖 Z 辨识出的零点；
         * 零点是 RAM 变量，复位后必须先 C→Z 再进本模式。
         */
        const float dir  = (float)s_dir_sign;
        const float om   = dir * BspServo_GetOmegaMech();
        /* EncComp_Eval：编码器角度误差的软件补偿（电气角，随机械角变化）。
         * 只加在 Park 的 th_e 上，**不动机械角/位置环/齿槽查表**：
         *   · 位置环与齿槽表用的是 theta_multi（由整数计数重算），
         *     若在这里之外再改 theta，会把前馈相位和位置基准一起打乱；
         *   · 补偿默认关闭（EncComp_Init 里 s_en=0），关闭时该函数立即返回 0，
         *     与改动前逐位一致。 */
        const float th_e = SERVO_CUR_POLE * (dir * theta_now)
                         + SERVO_CUR_ELEC_OFF + SERVO_CUR_COMP * om
                         + s_elec_off
                         + EncComp_Eval(dir * theta_now);

        if ((BspServo_IsPwmEnabled() == 0U) || (BspServo_GetVbus() < 5.0f)) {
            /* MOE 未放开、或母线还没建立：保持中点并清积分。
             * 两条都不能省：
             *   MOE=0 期间 PI 若继续积分，放开的那一拍就是满电压 -> 电流冲击；
             *   vbus < 5 V 时 da = 0.5 + valpha/vb 会把占空比算成巨大的值（乃至 NaN），
             *   而 (uint16_t)NaN 是未定义行为。 */
            s_cur_vd_int = 0.0f;
            s_cur_vq_int = 0.0f;
            CurStep_Abort();            /* 没有输出就没法测：阶跃作废，别记一堆 0 */
            /* 速度环的积分也必须一起清：MOE=0 期间若继续积分，
             * 放开的那一拍 iq 直接是满限幅 -> 力矩冲击。 */
            s_spd_i_term = 0.0f;
            s_spd_iq     = 0.0f;
            s_spd_cnt    = 0U;
            BspServo_SetDuty(0.5f, 0.5f, 0.5f);
            s_duty[0] = s_duty[1] = s_duty[2] = 0.5f;
            s_cur_id = 0.0f;
            s_cur_iq = 0.0f;
            s_iq_ref = ((s_spd_en != 0U) || (s_pos_en != 0U)) ? 0.0f : s_cur_iq_ref;  /* 未输出时上报常驻给定 */
            t3 = t4 = DWT->CYCCNT;
        } else {
            /*
             * 超速保护。分两套判据：
             *   速度闭环：门限随给定放大（1.5|w_ref| + 10 rad/s），正常跟踪永远够不到，
             *             但只要编码器断线（omega 恒 0）或 dir 判错（速度环成正反馈），
             *             就会在 10~20 ms 内撞线 —— 这是唯一能兜住飞车的东西。
             *   力矩模式：固定 300 rpm（原本就没有速度环，空载必然一路加速）。
             * 置位后下一拍由故障分支封锁 MOE，需要 X 清除。
             */
            {
                const float aref = fabsf(s_spd_ref_raw);

                /* 峰值保持：不按时间衰减，按"实测是否已回到给定附近"决定。
                 *
                 * ⚠ 不能用"给定是否在下降"判断（第一版就是这么写的，实测失败）：
                 *   s_spd_ref_raw 是**目标**，给 -140 时它一拍就跳到位并保持，
                 *   于是 aref < aref_last 只维持一拍，第二拍起就被当成
                 *   "减速已结束"开始衰减。实测 wlim 在 192 ms 内从 339.9
                 *   掉到 243.5，而 wmeas 还是 2100 rpm —— 门限一路追着实测跑。
                 *
                 * 正确的条件：**实测速度已经回到给定附近**，才说明减速真的结束。
                 *   · 稳态 / 升速：aref == peak → 峰值 = 给定，保护跟着目标走
                 *   · 减速途中：om 远高于 aref → 峰值保持不动，门限不收紧
                 *   · 减速完成：om ≈ aref → 允许收回（0.999/拍 ≈ 1 s 衰到 13%）
                 * 真失控（给定不变而 om 上升）仍会撞线：门限 = 1.5×给定 + 10。 */
                if (aref >= s_spd_wpeak) {
                    s_spd_wpeak = aref;
                } else if (fabsf(om) < (1.3f * aref) + 5.0f) {
                    s_spd_wpeak *= 0.999f;
                    if (s_spd_wpeak < aref) { s_spd_wpeak = aref; }
                }

                {
                    const float base = s_spd_wpeak;

                    /* MIT 的超速门限跟着 |v_des| 走：MIT 本来就要按给定速度运行，
                     * 用固定的 300 rpm（力矩模式的门限）会把正常的关节运动误判成失控。
                     * 位置环同理：它的 v_cmd 已写进 s_spd_ref_raw，走同一条门限。 */
                    const float wlim = (s_ol_en != 0U)
                                     ? SERVO_OL_WLIM
                                     : (((s_spd_en != 0U) || (s_pos_en != 0U))
                                        ? (1.5f * base + SERVO_SPD_WMAX_ABS)
                                        : ((s_mit_en != 0U)
                                           ? (1.5f * fabsf(s_mit_v) + SERVO_MIT_WMAX)
                                           : SERVO_CUR_WMAX));

                    /* 消抖：连续 8 拍（0.4 ms）都超限才判故障。
                     * 减速过程中 om 会擦着门限过几拍，但不会持续；
                     * 真失控时 0.4 ms 只多跑 2 rad/s（加速度 5278 rad/s²），
                     * 对"兜住飞车"这个目的毫无影响。 */
                    if (fabsf(om) > wlim) {
                        if (s_ovs_cnt < 8U) { s_ovs_cnt++; }
                        if (s_ovs_cnt >= 8U) {
                            BspServo_RaiseFault(BSP_FAULT_OVERSPEED);
                        }
                    } else {
                        s_ovs_cnt = 0U;
                    }
                }
            }
            const float cs = cosf(th_e);
            const float sn = sinf(th_e);
            const float vb = BspServo_GetVbus();
            const float ialpha = iu;
            const float ibeta  = -0.57735027f * iu - 1.15470054f * iw;
            float id_now =  cs * ialpha + sn * ibeta;
            float iq_now = -sn * ialpha + cs * ibeta;
            /* 给定来源三选一（优先级从高到低）：
             *   速度闭环 -> 速度环输出
             *   阶跃整定 -> CurStep 接管（仅在力矩模式）
             *   常驻   -> s_cur_iq_ref（M/N 命令）
             */
            float id_ref = s_cur_id_ref;
            float iq_ref = s_cur_iq_ref;
            float ed, eq;
            float vd, vq, vmax, vmag, valpha, vbeta, da, db, dc;
            float om_e, dec_vd, dec_vq;

            /* 力矩给定斜坡（一阶低通，20 kHz 更新）。
             * CurStep 的阶跃测试会在下面覆盖 iq_ref，所以斜坡不影响整定测量。 */
            if (s_cur_iq_tau > 0.0f) {
                const float ar = SERVO_OL_TS / (s_cur_iq_tau + SERVO_OL_TS);
                s_cur_iq_ramp += ar * (s_cur_iq_ref - s_cur_iq_ramp);
                iq_ref = s_cur_iq_ramp;
            } else {
                s_cur_iq_ramp = s_cur_iq_ref;   /* tau=0：直通，用于阶跃响应测试 */
                iq_ref = s_cur_iq_ramp;
            }

            /* 给定来源：MIT 阻抗 > 速度闭环 > 电流阶跃 > 常驻给定。
             * R/L 辨识的 L 段走独立的单频注入分支，不在这里抢给定。
             * MIT 放最前：它和速度环是互斥的两种语义，不能叠加。 */
            if (s_mit_en != 0U) {
                /* MIT 阻抗控制：位置/速度误差直接换成力矩，再除以 Kt 得到 iq。
                 * 全部在**模型坐标系**里算（dir 已乘），所以 kp/kd 为正时
                 * 总是把转子往 p_des 推，与编码器安装方向无关。
                 *
                 * 位置用多圈角，跨圈不突跳；误差再限幅到 ±EP_MAX：
                 * kp 可能给到上千，p_des 打错一位就是满力矩冲出去。 */
                const float ds   = (float)s_dir_sign;
                float ep;

                /* 给定外推（一阶保持）：帧率约 1 kHz，帧之间 p_des 是常数，
                 * 轨迹跟踪时这个台阶就是纯滞后 —— 实测 2 Hz、峰值 10 rad/s 的
                 * 轨迹，误差 54 mrad，其中 50 mrad = 5 ms × 10 rad/s。
                 * 帧里本来就有 v_des，直接线性外推，不需要额外传感器，
                 * 也不放大噪声（v_des 是上位机给的期望速度，不是差分出来的）。
                 * 超时后 v_des 被清零，外推自动停止。 */
                {
                    const uint32_t age = s_mit_tick - s_mit_stamp;
                    float dt = (float)age * BSP_TS_CURRENT;

                    if (dt > SERVO_MIT_EXTRAP_MAX_S) { dt = SERVO_MIT_EXTRAP_MAX_S; }
                    s_mit_dt = dt;          /* 诊断：实际用到的帧龄 */

                    /* 目标整形（见 SERVO_MIT_SHAPE_VMAX 说明）：控制律用整形值，
                     * 不用帧里的原始 p_des。一帧 3 rad 的阶跃会变成
                     * "以 15 rad/s 平滑走过去"，而不是 kp·3rad 的满力矩冲出去。 */
#if (SERVO_MIT_SHAPE_EN != 0)
                    if (s_mit_kp <= 0.0f) {
                        /* 零刚度（盲起环）：跟随实际位置，绝不预跑 ——
                         * 否则上刚度那一拍目标已经跑到别处，等于人为造阶跃 */
                        s_mit_p_shape = theta_multi;
                        s_mit_v_shape = 0.0f;
                    } else {
                        const float ts = BSP_TS_CURRENT;
                        const float ep_s = s_mit_p - s_mit_p_shape;
                        float vmax = SERVO_MIT_SHAPE_VMAX;
                        float vs, dv, dmax;
                        const float vd = fabsf(s_mit_v);

                        /* 用户明确要高速时放宽限速，别把合法轨迹拖慢 */
                        if ((vd * 1.2f) > vmax) { vmax = vd * 1.2f; }

                        /* ① 位置差 → 期望速度（比例，限幅到 vmax） */
                        vs = ep_s * SERVO_MIT_SHAPE_KP;
                        if (vs >  vmax) { vs =  vmax; }
                        if (vs < -vmax) { vs = -vmax; }

                        /* ② 速度按 amax 逼近 —— 限住起步的加速度，
                         *    否则"速度阶跃"仍会让 iq 顶到 4 A 以上 */
                        dv   = vs - s_mit_v_shape;
                        dmax = vmax * SERVO_MIT_SHAPE_A_RATIO * ts;
                        if (dv >  dmax) { dv =  dmax; }
                        if (dv < -dmax) { dv = -dmax; }
                        s_mit_v_shape += dv;

                        /* ③ 积分出整形目标 */
                        s_mit_p_shape += s_mit_v_shape * ts;

                        /* ④ 到位吸附：消除指数收敛尾巴，避免长时间爬行 */
                        if ((fabsf(ep_s) < 0.0005f) && (fabsf(s_mit_v_shape) < 0.02f)) {
                            s_mit_p_shape = s_mit_p;
                            s_mit_v_shape = 0.0f;
                        }
                    }
#else
                    s_mit_p_shape = s_mit_p;   /* 整形关闭（SHAPE_EN=0）：直接执行原始目标 */
#endif

#if (SERVO_MIT_EXTRAP_EN != 0)
                    ep = ds * ((s_mit_p_shape + (s_mit_v * dt)) - theta_multi);
#else
                    (void)dt;
                    ep = ds * (s_mit_p_shape - theta_multi);   /* A/B 基准：零阶保持 */
#endif
                }

                const float epc  = (ep >  SERVO_MIT_EP_MAX) ?  SERVO_MIT_EP_MAX
                                 : ((ep < -SERVO_MIT_EP_MAX) ? -SERVO_MIT_EP_MAX : ep);
                const float vdes = ds * s_mit_v;
                /* ⚠ D 项必须用**单拍速度**，不能用控制环那个 om。
                 * om = 8 拍累计 + 95 Hz 低通，群延迟约 1.9 ms；在 244 Hz 处
                 * 相当于 167° 相移 —— 阻尼项直接变成**负阻尼**。
                 * 实测：kp=0、kd=0.5（纯阻尼）就会在 244 Hz 起 5 A 极限环；
                 * kd=0 时干净。换成单拍速度后该振荡消失。
                 * 代价：单拍速度有 ±0.06 rad/s 量化噪声，kd=0.5 时折合 0.03 N·m。 */
                /* 2 拍差分测速（见 s_mit_th1 处的说明） */
                float v_est;
                if (s_mit_vinit == 0U) {
                    s_mit_vinit = 1U;
                    s_mit_th1 = theta_multi;
                    s_mit_th2 = theta_multi;
                    v_est = 0.0f;
                } else {
                    v_est = (theta_multi - s_mit_th2) * (0.5f / BSP_TS_CURRENT);
                    s_mit_th2 = s_mit_th1;
                    s_mit_th1 = theta_multi;
                }
                const float om_d = (float)s_dir_sign * v_est;
                float tau = (s_mit_kp * epc) + (s_mit_kd * (vdes - om_d)) + s_mit_tau;

                /* 【2026-11-18 实测后撤回】这里曾加过齿槽前馈：
                 *     tau += Kt * CogFf_Get(theta_multi, COG_FF_W_FULL);
                 * 理由很顺：齿槽是位置函数，零速保持时也存在，表现就是"停不准"。
                 * 但 A/B 实测**反而更差**：
                 *     kp=10/kd=0.1，扫 0.8 rad 保持
                 *     齿槽 OFF：峰峰 7.2 mrad，σ 2.1 mrad
                 *     齿槽 ON ：峰峰 14.2 mrad，σ 4.2 mrad   ← 差一倍
                 *
                 * 原因：速度环里齿槽前馈是加在 PI 输出上，误差由积分吸收；
                 * 而 MIT 是纯 PD，前馈直接进力矩，表的误差（空载标的、
                 * 128 点插值）没人兜，等于往环路里注入一个位置相关的扰动，
                 * 和 kp 的位置刚度对着干。
                 * 结论：**MIT 不加齿槽前馈**，保持精度只能靠 kp 换。 */

                if (tau >  SERVO_MIT_TAU_MAX) { tau =  SERVO_MIT_TAU_MAX; }
                if (tau < -SERVO_MIT_TAU_MAX) { tau = -SERVO_MIT_TAU_MAX; }

                /* τ→iq 用温度补偿后的 Kt：否则 50 °C 时同样的 τ 只出 96.4% 力矩。
                 * 回读也乘同一个系数，保证"设定值 = 实际值"不被补偿破坏。 */
                iq_ref = tau / (s_kt_base * s_kt_comp);

                /* 力矩模式没有速度环，电流限幅必须自己兜：
                 * τ 限到 1.5 N·m 相当于 16 A，而 iq 限幅只有 5 A。 */
                if (iq_ref >  s_cur_iq_max) { iq_ref =  s_cur_iq_max; }
                if (iq_ref < -s_cur_iq_max) { iq_ref = -s_cur_iq_max; }

                id_ref        = 0.0f;
                s_mit_ep      = epc;
                s_mit_p_now   = theta_multi;
                s_mit_tau_out = iq_ref * (s_kt_base * s_kt_comp);   /* 实际给出的力矩 */
            } else if ((s_spd_en != 0U) || (s_pos_en != 0U)) {
                float w_ref_model;

                if (s_pos_en != 0U) {
                    const float ds = (float)s_dir_sign;

                    /* 与速度环**严格同拍**推进：Servo_SpeedLoopStep 每 10 拍
                     * 才执行一次，位置环在它执行前的最后一拍算，规划的 ts
                     * （500 us）才与实际控制周期一致。每拍都算会让规划跑快
                     * 10 倍，定位时间对不上、末端过冲。 */
                    if (s_spd_cnt >= (SERVO_SPD_DIV - 1U)) {
                        /* 用位置环坐标（= theta_multi − s_pos_base），不是裸的
                         * theta_multi：进位置模式那一拍已对齐到单圈角，
                         * 这样反馈值不会累积到上百 rad。
                         * 这里只做一次 volatile 读 + 减法，20 kHz 下无压力。 */
                        s_pos_vcmd = PosLoop_Step(ds * pos_coord_now(), om);

                        /* 跟随误差超限：位置环已停止给速度，这里落故障位。
                         * 放 ISR 而非 1 ms 任务，是为了检出与封锁同拍生效 ——
                         * 堵转时多给 1 ms 就多走 v_max×1ms。 */
                        if (PosLoop_GetState() == (uint8_t)POS_ST_ERR) {
                            BspServo_RaiseFault(BSP_FAULT_FOLLOW);
                        }
                    }
                    w_ref_model   = s_pos_vcmd;         /* 已是模型坐标系 */
                    s_spd_ref_raw = ds * s_pos_vcmd;    /* 超速判据用绝对值，符号仅诊断 */
                } else {
                    /* 坐标系：om 已乘 dir_sign（模型坐标系），给定也必须换算过去，
                     * 否则 dir=-1 时正 iq 让 om 变小 -> 误差变大 -> 正反馈飞车。 */
                    w_ref_model = (float)s_dir_sign * s_spd_ref_raw;
                }

                iq_ref = Servo_SpeedLoopStep(w_ref_model, om);
                /* 速度模式默认 id=0；ψ/R/L 联合辨识需要用 id 扫描打破
                 * "iq 被摩擦锁在 ωe 上"的共线性，所以这里让辨识模块接管。
                 * 非活动时 FluxIdent_GetIdRef() 返回 0，行为与原来完全一致。 */
                id_ref = FluxIdent_GetIdRef();
            } else {
                /* 不在速度环：齿槽前馈诊断量归零。
                 * 必须放在这里而不是循环开头 —— 这段是 20 kHz，而
                 * s_spd_cog 每 10 拍（2 kHz）才更新一次，在开头清零会让
                 * `:` 读到的 now= 有 90% 是 0，看着像前馈没工作。 */
                s_spd_cog = 0.0f;
                if (MechIdent_Refs(&iq_ref) != 0U) {
                    /* 机械辨识（扫频/惯量）优先级最高：它要独占 iq 给定，
                     * 被阶跃或常驻给定改写会让同步解调拿到错误的激励基。 */
                    id_ref = 0.0f;
                } else {
                    /* 阶跃整定接管 id/iq；**未活动时不动给定**（CurStep_Refs 只返回 0）。
                     * 这里曾经因为 CurStep_Refs 会顺手写 0，导致力矩模式
                     * 给定被抹掉、环路里根本没有目标电流。 */
                    (void)CurStep_Refs(&id_ref, &iq_ref);
                }
            }

            /* 温度降额：这里是**所有给定来源汇合后的唯一收口**，所以放在这里
             * 一次就盖住力矩/速度/位置/辨识全部模式。
             * 用各模式自己的限幅乘以降额系数，而不是统一用 s_cur_iq_max ——
             * 速度环的 3 A 限幅比力矩模式的 5 A 小，统一用 5 A 会让速度模式
             * 在降额前就不受约束。
             * 辨识（扫频/惯量/ψ-R-L）也会被这个限幅截断：过热时宁可测不完整，
             * 也不让电机继续升温。 */
            if (s_derate < 1.0f) {
                /* 位置环也走速度环（s_spd_en=0、s_pos_en=1），若只判 s_spd_en
                 * 就会退回 s_cur_iq_max(5 A)，而速度环输出已限在 3 A ——
                 * 于是 derate>0.6 时 lim>3 A，降额完全不生效。
                 * 2026-11-18 修正：位置环与速度环共用同一个速度环限幅。 */
                const float lim = s_derate * (((s_spd_en != 0U) || (s_pos_en != 0U))
                                              ? s_spd_iqmax : s_cur_iq_max);

                if (iq_ref >  lim) { iq_ref =  lim; }
                if (iq_ref < -lim) { iq_ref = -lim; }
            }

            s_cur_id = id_now;
            s_cur_iq = iq_now;

            ed = id_ref - id_now;
            eq = iq_ref - iq_now;

            s_cur_vd_int += ed * s_cur_ki_ts;
            s_cur_vq_int += eq * s_cur_ki_ts;

            /* 解耦 + BEMF 前馈（与模型 dec_mode=3 一致）：
             *   ωe = 14*ωm；vd 扣 iq·ωe·Lq，vq 加 id·ωe·Ld + ωe·ψ
             *
             * 2026-11-20：ψ 项乘 s_kt_comp，与 Kt 通道同源。
             * 改动前 Kt 走 s_kt_base*s_kt_comp（iq_ref 已按温度修正），
             * 而这里用的是固定 s_cur_psi —— 同一台机器两条通道差 7.7%
             * （84 °C，ktc≈0.923）。2000 rpm 下前馈多给
             *   ωe·ψ·(1−ktc) = 2932 × 4.333e-3 × 0.077 ≈ 0.98 V，
             * 全靠 vq 积分反向顶回来。稳态转速看不出（积分能吸收），
             * 但代价是积分长期带偏置：加速/加载瞬间 vq 先撞圆限幅，
             * 高压区（BEMF 已占大半母线）还会吃掉可用裕度。
             *
             * 温度补偿必须两处一致：只补 Kt 等于"力矩算对了、电压算错了"。
             * 关闭补偿时（ktc=1）本式与原实现逐位一致，无回归风险。
             * s_kt_comp 由 1 ms 任务更新，20 kHz 里读两次的差异 <0.001%，
             * 不值得为它加缓存。 */
            om_e   = SERVO_CUR_POLE * om;
            dec_vd = iq_now * om_e * s_cur_lq;
            dec_vq = id_now * om_e * s_cur_ld + om_e * (s_cur_psi * s_kt_comp);

            vd = s_cur_kp * ed + s_cur_vd_int - dec_vd;
            vq = s_cur_kp * eq + s_cur_vq_int + dec_vq;

            /* 死区电压补偿（可选，s_cur_dead_gain 默认 0 = 关闭）。
             *
             * 死区让实际输出电压比给定少 sign(i)·Vdead，Vdead = 0.0059·Vbus
             * ≈ 0.18 V @30.7 V。除以 R=0.4 Ω 相当于 0.45 A 的偏差 ——
             * 小力矩给定时它比给定本身还大，全靠积分硬顶，表现为过零附近
             * 电流爬不动、低速力矩不线性。
             *
             * 方向判据带迟滞：iq_ref > 30 mA 判正、< −30 mA 判负、
             * |iq_ref| < 10 mA 才归零。没有迟滞的话符号在过零点抖动，
             * 补偿自己就变成振荡源。
             *
             * 补在 vq 上而不是三相电压上，是因为我们只需要改善 dq 域的
             * 力矩线性度；逐相补偿要做六扇区判断，收益不值这份复杂度。 */
            if (s_cur_dead_gain > 0.0f) {
                if      (iq_ref >  0.03f)        { s_cur_dead_sign =  1; }
                else if (iq_ref < -0.03f)        { s_cur_dead_sign = -1; }
                else if (fabsf(iq_ref) < 0.01f)  { s_cur_dead_sign =  0; }

                vq += s_cur_dead_gain * (SERVO_CUR_DEAD_FRAC * vb)
                    * (float)s_cur_dead_sign;
            }

            vmax = (vb > 1.0f) ? (vb * SERVO_CUR_VLIM_FRAC) : 1.0f;
            vmag = sqrtf(vd * vd + vq * vq);
            if ((vmag > vmax) && (vmag > 0.0f)) {          /* 圆限幅 + 抗饱和 */
                const float k      = vmax / vmag;
                const float vd_sat = vd * k;
                const float vq_sat = vq * k;

                /* back-calculation（比例式退饱和），与速度环同一套做法。
                 *
                 * 原来是 `int *= 0.98` 逐拍衰减，有两个问题：
                 *   1) 衰减量与被削掉多少无关 —— 只超限 1% 和超限 90% 都乘 0.98，
                 *      轻度过载时积分被过度压制（恢复慢），重度过载时又削得不够快；
                 *   2) 每拍无条件乘 0.98 相当于给积分加了一个 98%/50µs 的泄漏，
                 *      即使只在一瞬间轻微超限，也会把之前积累的积分持续往下拉。
                 * 改成把"被削掉的量"按 Kb 回灌：削多少补多少，且 Kb<1 保证稳定。 */
                s_cur_vd_int += SERVO_CUR_KB * (vd_sat - vd);
                s_cur_vq_int += SERVO_CUR_KB * (vq_sat - vq);

                vd = vd_sat;
                vq = vq_sat;
            }

            CurStep_Push(id_now, iq_now, vd, vq);          /* 记录本拍（含限幅后的 v） */
            /* 机械辨识的测量通道：未滤波单拍速度 + 多圈角 */
            MechIdent_Step(BspServo_GetOmegaFast(), iq_now, theta_multi);

            /* ψ/R/L 联合辨识：记录本拍实测。
             * vd/vq 必须是**限幅后**的值 —— 电压顶死时限幅前的 PI 输出偏大，
             * 用它解方程会把 R 与 ψ 一起算高。此时 vd/vq 已过圆限幅。 */
            FluxIdent_Step(id_now, iq_now, vd, vq, om * SERVO_CUR_POLE);

            valpha = cs * vd - sn * vq;
            vbeta  = sn * vd + cs * vq;

            /*
             * 5/7 次谐波抑制：在 αβ 域叠加补偿电压。
             * 注意这里用的是**实测** ialpha/ibeta（不是给定），
             * 因为要压的是真实存在的谐波电流，不是指令里的。
             */
            if (s_harm_en != 0U) {
                /* 先把电角度折回 [0,2π)：th_e 最大约 90 rad，直接乘 5/7 之后
                 * 到 600 rad 量级，sinf 的参数归约会损失有效位 */
                const float thw = th_e - BSP_TWO_PI
                                * (float)(int32_t)(th_e * 0.159154943f);
                const float c5 = cosf(5.0f * thw);
                const float s5 = sinf(5.0f * thw);
                const float c7 = cosf(7.0f * thw);
                const float s7 = sinf(7.0f * thw);
                const float kp   = s_harm_kp;
                const float kits = s_harm_ki_ts;
                const float ilim = 0.15f * vb;

                /* 5 次是负序：θ = −5θe ⇒ cos(−5θ)=c5, sin(−5θ)=−s5 */
                const float id5 =  c5 * ialpha - s5 * ibeta;
                const float iq5 =  s5 * ialpha + c5 * ibeta;
                /* 7 次是正序：θ = +7θe */
                const float id7 =  c7 * ialpha + s7 * ibeta;
                const float iq7 = -s7 * ialpha + c7 * ibeta;

                /*
                 * ⚠ 误差必须取负：目标是让这些分量**归零**，
                 * 所以 PI 要输出抵消电压。写成 +id5 就是正反馈 ——
                 * 实测直接触发过流（f=0x01，峰值 16.6 A）并封锁。
                 */
                const float e5d = -id5;
                const float e5q = -iq5;
                const float e7d = -id7;
                const float e7q = -iq7;

                float vd5, vq5, vd7, vq7;

                s_h5_vd_int += e5d * kits;
                s_h5_vq_int += e5q * kits;
                s_h7_vd_int += e7d * kits;
                s_h7_vq_int += e7q * kits;

                if (s_h5_vd_int >  ilim) { s_h5_vd_int =  ilim; }
                if (s_h5_vd_int < -ilim) { s_h5_vd_int = -ilim; }
                if (s_h5_vq_int >  ilim) { s_h5_vq_int =  ilim; }
                if (s_h5_vq_int < -ilim) { s_h5_vq_int = -ilim; }
                if (s_h7_vd_int >  ilim) { s_h7_vd_int =  ilim; }
                if (s_h7_vd_int < -ilim) { s_h7_vd_int = -ilim; }
                if (s_h7_vq_int >  ilim) { s_h7_vq_int =  ilim; }
                if (s_h7_vq_int < -ilim) { s_h7_vq_int = -ilim; }

                vd5 = (kp * e5d) + s_h5_vd_int;
                vq5 = (kp * e5q) + s_h5_vq_int;
                vd7 = (kp * e7d) + s_h7_vd_int;
                vq7 = (kp * e7q) + s_h7_vq_int;

                /* 反变换：5 次 θ=−5θe ⇒ [c5 s5; −s5 c5]；7 次 θ=+7θe ⇒ [c7 −s7; s7 c7] */
                valpha += ((c5 * vd5) + (s5 * vq5)) + ((c7 * vd7) - (s7 * vq7));
                vbeta  += ((c5 * vq5) - (s5 * vd5)) + ((s7 * vd7) + (c7 * vq7));

                s_h5_id = id5; s_h5_iq = iq5;
                s_h7_id = id7; s_h7_iq = iq7;

                /* 叠加后可能超出 SVPWM 线性区：按圆限幅等比缩一次。
                 * 主环在 dq 域已限到 0.5375·vbus，这里防的是"谐波补偿把它顶出去"，
                 * 顶出去会削顶 -> 又产生新的低次谐波，等于白干。 */
                {
                    const float vm2   = (valpha * valpha) + (vbeta * vbeta);
                    const float vlim2 = vb * vb * 0.2887f;   /* (0.5375)² ≈ 0.2889 */

                    if ((vm2 > vlim2) && (vm2 > 1.0e-9f)) {
                        const float k = sqrtf(vlim2 / vm2);
                        valpha *= k;
                        vbeta  *= k;
                    }
                }
            }

            da = 0.5f + (valpha / vb);
            db = 0.5f + ((-0.5f * valpha + 0.86602540f * vbeta) / vb);
            dc = 0.5f + ((-0.5f * valpha - 0.86602540f * vbeta) / vb);

            /* 死区补偿：先喂本拍转速做门控（低速补、高速关），
             * 再对三相分别施加。门控理由见 deadband_comp.h 的 DBC_W_FULL。 */
            DeadbandComp_SetSpeed(om);
            BspServo_SetDuty(DeadbandComp_Apply(da, s_iu),
                             DeadbandComp_Apply(db, s_iv),
                             DeadbandComp_Apply(dc, s_iw));
            s_duty[0] = da;
            s_duty[1] = db;
            s_duty[2] = dc;
            s_iq_ref  = iq_ref;

            if (s_rec_en != 0U) {
                if (++s_rec_cnt >= s_rec_div) {
                    s_rec_cnt = 0U;
                    if (s_rec_n < REC_N) {
                        if (s_rec_ch == 0U) {
                            s_rec_buf[s_rec_n] = (int16_t)(iq_now * 131.0f);  /* ±250 A 不溢出 */
                        } else if (s_rec_ch == 2U) {
                            /* vq 积分项，mV。±32.7 V 量程 */
                            s_rec_buf[s_rec_n] = (int16_t)(s_cur_vq_int * 1000.0f);
                        } else if (s_rec_ch == 3U) {
                            /* 编码器角度误差测量通道（2026-11-21）：
                             *   buf = d 轴 PI 积分（mV）—— 稳态 id=0 时它等于
                             *         ωe·ψ·sin(δ)，δ 是"估计角 − 真实角"的电角度误差；
                             *   th  = 单圈机械角（2π→65536），用来按位置分箱。
                             * 注意 rec_th_units() 对所有通道都按 2π 映射，所以
                             * 这里存机械角，主机拿到的就是机械角，不是电角度。 */
                            s_rec_buf[s_rec_n] = (int16_t)(s_cur_vd_int * 1000.0f);
                            s_rec_th[s_rec_n]  =
                                (int16_t)(uint16_t)(theta_now * (65536.0f / 6.283185307179586f));
                        } else {
                            /* 相电流 + 同拍电角度（2π→65536 编码） */
                            s_rec_buf[s_rec_n] = (int16_t)(s_iu * 131.0f);
                            s_rec_th[s_rec_n]  =
                                (int16_t)(uint16_t)(th_e * (65536.0f / 6.283185307179586f));
                        }
                        s_rec_n++;
                    } else {
                        s_rec_en = 0U;
                    }
                }
            }
            t3 = t4 = DWT->CYCCNT;      /* 不走模型，本分支自带 duty，不单独分段 */
        }
    } else {
        /*
         * 双电阻采样相序（关键）：
         *   OPA1/ADC1_IN3 实测 U 相，OPA2/ADC2_IN3 实测 W 相，OPA3/ADC3 板上悬空。
         *   生成代码 Clarke 只用 ia、ib（i_beta = 0.57735*ia + 1.15470*ib），
         *   隐含 ic = -ia-ib，因此 V 相必须由 Iu + Iw 重构。
         *   禁止把 W 相电流直接赋给 ib，否则电角度错 120 度、FOC 失效。
         */
        three_loop_deploy_U.ia = (real32_T)iu;
        three_loop_deploy_U.ib = (real32_T)(-(iu + iw));
        three_loop_deploy_U.ic = (real32_T)iw;         /* 生成代码未使用，仅作监视 */

        three_loop_deploy_U.vbus    = (real32_T)BspServo_GetVbus();
        /* dir_sign = -1 时等价于把模型里的 14*theta_m 变成 -14*theta_m。
         * 模型本身是生成代码，斜率写死为正，只能在入口取反。 */
        three_loop_deploy_U.theta_m = (real32_T)((float)s_dir_sign * theta_now);
        three_loop_deploy_U.omega_m = (real32_T)((float)s_dir_sign * BspServo_GetOmegaMech());
        /*
         * 位置环坐标系：模型已打补丁增加 theta_pos（多圈）与 w_ff（速度前馈）两个输入。
         *
         * 原来只能用单圈 theta_m，跨 2π 时 pos_ref - theta_m 会突跳 2π，所以不得不把
         * pos_ref 折叠回同一圈 —— 那只能保证 ±π 内正确，大范围绝对定位会被折到
         * 最近的等价点。实测（2026-11-15，3 rpm / 5.4 s）跨圈一次就 clamp=1、
         * f=0x01、ovc=-23.37/+22.06，直接过流跳闸。
         *
         * 现在 theta_pos 与 pos_ref 都在同一个**多圈**坐标系里（都从使能时的
         * 单圈角起步累加），直接相减就是真实误差，不需要折叠。
         *
         * 三者的 dir 都在这里统一乘：模型自己不知道编码器方向，
         * 误差 = dir·(pos_ref - theta_multi)，与 dir=+1 时同号。
         */
        {
            const float ds = (float)s_dir_sign;

            three_loop_deploy_U.theta_pos = (real32_T)(ds * theta_multi);
            three_loop_deploy_U.pos_ref   = (real32_T)(ds * s_pos_ref);
            /* 速度前馈：位置环是纯 P，稳态跟随速度 v 需要固定误差 e = v/Kp
             * （Kp=50.265，20 rad/s 时 0.40 rad ≈ 23°）。把 v_ref 直接加到
             * hold_w 上，这个滞后就被消掉了，位置误差只剩动态部分。 */
            three_loop_deploy_U.w_ff      = (real32_T)(ds * s_pos_w_ff);
        }
        /*
         * 2026-11-17：三环默认改走手写链路后，模型只剩"对比/回归"用途。
         * s_enable 为 0 时**直接跳过 step**：
         *   ① 省掉每拍模型内部的三角函数（生成代码 1331 行，占用未实测但不便宜）；
         *   ② 杜绝"某条路径意外置 s_enable=1 就让模型突然接管"的可能。
         * 输出三相中点：这是唯一安全的无输出状态（线电压 0，电机不受力）。
         *
         * 注意上面那段 ia/ib/ic/theta/omega 的赋值**保留**：BspServo_GetOmegaMech()
         * 内部会推进 95 Hz 测速低通，跳过它会改变速度环的输入特性。
         */
        if (s_enable != 0U) {
            three_loop_deploy_U.en = 1.0;

            /* 温度降额：模型路径唯一的限流入口是 par_c[10]（i_max），
             * 必须在 step 之前写。 */
            three_loop_deploy_U.par_c[10] = 9.0f * s_derate;

            three_loop_deploy_step();
            t3 = DWT->CYCCNT;

            /* 死区补偿（移植自 FM_XC）：控制律给出的 duty 是"理想上管导通比例"，
             * 实际桥臂因死区（294 ns = 0.588% 占空比，DTG=50 属 0xx 段）会少输出，
             * 按电流方向补回来。
             * 注意 s_duty[] 记录的仍是**未补偿**值，遥测看到的是控制律原始输出。 */
            BspServo_SetDuty(DeadbandComp_Apply((float)three_loop_deploy_Y.duty[0], s_iu),
                             DeadbandComp_Apply((float)three_loop_deploy_Y.duty[1], s_iv),
                             DeadbandComp_Apply((float)three_loop_deploy_Y.duty[2], s_iw));
            t4 = DWT->CYCCNT;

            /* 遥测快照：iq_used 是限幅后真正送给电流环的给定 */
            s_duty[0] = (float)three_loop_deploy_Y.duty[0];
            s_duty[1] = (float)three_loop_deploy_Y.duty[1];
            s_duty[2] = (float)three_loop_deploy_Y.duty[2];
            s_iq_ref  = (float)three_loop_deploy_Y.iq_used;
        } else {
            three_loop_deploy_U.en = 0.0;
            BspServo_SetDuty(0.5f, 0.5f, 0.5f);
            s_duty[0] = 0.5f;
            s_duty[1] = 0.5f;
            s_duty[2] = 0.5f;
            s_iq_ref  = 0.0f;
            t4 = DWT->CYCCNT;
            t3 = t4;
        }
    }

    /* 预读下一帧编码器：DMA 异步，当前 ISR 只发起、不等待 */
    (void)BspServo_StartEncoderDma();
    t5 = DWT->CYCCNT;

    /* 分段峰值：定位 20 kHz 预算被谁吃掉（只记峰值，成本约 5 个比较） */
    UPD_MAX(s_max_theta,   t1 - t0);
    UPD_MAX(s_max_readinj, t2 - t1);
    UPD_MAX(s_max_step,    t3 - t2);
    UPD_MAX(s_max_duty,    t4 - t3);
    UPD_MAX(s_max_enc,     t5 - t4);

    uint32_t dt = DWT->CYCCNT - t0;
    if (dt > s_max_cycles) s_max_cycles = dt;
    s_tick_count++;
}

/* ---------------- 1ms 慢速任务 ---------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        Servo_SlowTask1ms();
    }
}

/*
 * 运行态诊断：开机 3 s 后打一次，之后永久静默。
 * 为什么是 3 s：DWT 峰值需要跑过完整加减速与换相周期才有代表性；
 * 为什么要重试：100 Hz 遥测 DMA 每 10 ms 占 UART 1.44 ms，撞上就发不出去。
 */

/* ---------------- 辨识/标定参数的 Flash 持久化 ----------------
 *
 * 保存时机不是"命令执行完"，而是"值真的变了"：
 *   C 的标定是异步的（ReadInjected 里累加 200 ms 才把 cal 置 1），
 *   Z 是 9 s 状态机，在命令处理里立刻存只会存到旧值。
 * 所以这里每 500 ms 拿实际值比一次；ParamStore_Save 内部还会与 Flash 内容
 * memcmp，相同就直接返回，稳态下不会产生任何擦写。
 *
 * 只在 MOE=0 时写：单 bank 擦除会让 CPU 停顿 20~40 ms，
 * 占空比寄存器停在最后一次写入值 —— 等于桥臂输出被冻结在某个固定矢量上。
 */
uint8_t Servo_ParamSaveNow(void)
{
    ParamBlob pb;
    const MotorIdentResult *r;

    /* 三道前置条件，任一不满足都直接失败而不是存半套：
     *   有输出 -> 擦除期间 CPU 停顿会让桥臂冻结在固定矢量
     *   未标定/未对齐 -> 存下去的是默认值，下次上电会被误当成"已标定" */
    if (BspServo_IsPwmEnabled() != 0U)        { return 0U; }
    if (BspServo_IsCurrentCalibrated() == 0U) { return 0U; }
    if (BspServo_IsEncoderZeroed() == 0U)     { return 0U; }

    r = MotorIdent_GetResult();

    memset(&pb, 0, sizeof(pb));
    pb.flags     = PS_FLAG_BIAS | PS_FLAG_ZERO;
    pb.zero_rad  = BspServo_GetEncoderZero();
    pb.dir       = Servo_GetDirSign();
    pb.ratio_int = r->ratio_int;
    pb.ratio     = r->ratio;
    BspServo_GetCurrentBias(&pb.bias_u, &pb.bias_w);
    Servo_GetSpeedGains(&pb.spd_kp, &pb.spd_ki_ts);

    /* R/L 只有真的辨识过才带上标记，否则存 0 会在下次上电被当成有效值 */
    {
        const RlIdentResult *rl = RlIdent_GetResult();
        if ((rl->err == RL_IDENT_OK) && (rl->r_ohm > 0.0f) && (rl->l_h > 0.0f)) {
            pb.flags |= PS_FLAG_RL;
            pb.r_ohm  = rl->r_ohm;
            pb.l_h    = rl->l_h;
        }
    }

    /* 磁链：+ 辨识出有效值才存。psi 决定 Kt，进而决定 J 与所有转矩换算，
     * 存 0 会让下次上电算出 Kt=0（除零/无穷大），必须靠 flag 保护。
     * 同时带上 ψ 的温度基准：不带的话，热机辨识的值下次上电会被当冷态值再补一遍。 */
    {
        const FluxIdentResult *fi = FluxIdent_GetResult();

        if ((fi->err == FI_OK) && (fi->psi_wb > 1.0e-4f)) {
            pb.flags  |= PS_FLAG_PSI;
            pb.psi_wb  = fi->psi_wb;
            if (s_psi_t_ok != 0U) {
                pb.flags    |= PS_FLAG_PSI_T;
                pb.psi_t_ref = s_psi_t_ref;
            }
        }
    }

    /* 摩擦前馈：只在前馈真的开着且幅值非 0 时才存 ——
     * 存一个 0 会在下次上电被当成"标定过、摩擦为零"，比不存更误导。 */
    {
        uint8_t ffen = 0U;
        float   fa = 0.0f, fv = 0.0f, fvs = 0.0f;

        Servo_GetFeedforward(&ffen, &fa, &fv, &fvs);
        if ((fv > 0.01f) && (fvs >= 0.0f)) {
            pb.flags     |= PS_FLAG_FRIC;
            pb.fric_a     = fv;
            pb.fric_visc  = fvs;
        }
    }

    return ParamStore_Save(&pb);
}

static void Param_PersistTask1ms(void)
{
    static uint16_t s_cnt;

    if (++s_cnt < 500U) { return; }
    s_cnt = 0U;
    (void)Servo_ParamSaveNow();     /* 内容没变时内部 memcmp 会直接返回，不擦页 */
}

/*
 * 上电恢复。调用点必须在 BspServo_Init() 之后：
 *   - 零漂要写进 BSP 的 bias 变量，并置 cal=2；
 *   - 零点要走 BspServo_SetEncoderZero()，它会归一化并置 zero_ok（Z 的联锁就靠它）。
 * 顺序也有讲究：先设 dir 再写零点。零点换算依赖 dir 的符号，
 * 反过来会让下一次辨识的基准坐标系与这次不一致。
 */
static uint8_t param_load_and_apply(void)
{
    ParamBlob pb;

    if (ParamStore_Load(&pb) == 0U) { return 0U; }

    if ((pb.flags & PS_FLAG_ZERO) != 0U) {
        Servo_SetDirSign(pb.dir);
        BspServo_SetEncoderZero(pb.zero_rad);
        MotorIdent_SetCached(pb.ratio_int, pb.ratio, pb.dir);
    }
    if ((pb.flags & PS_FLAG_BIAS) != 0U) {
        BspServo_SetCurrentBias(pb.bias_u, pb.bias_w);
    }
    if (pb.spd_kp > 0.0f) {
        Servo_SetSpeedGains(pb.spd_kp, pb.spd_ki_ts);
    }
    if ((pb.flags & PS_FLAG_RL) != 0U) {
        RlIdent_SetCached(pb.r_ohm, pb.l_h);
        /* 控制律接管：L -> dq 解耦，R -> 记录（越界由 SetIdentRl 内部拒绝） */
        Servo_SetIdentRl(pb.r_ohm, pb.l_h);
    }
    if ((pb.flags & PS_FLAG_PSI) != 0U) {
        /* 控制律接管：psi -> Kt 与 BEMF 前馈。
         * 改的是运行时变量，不必再动 three_loop_params.m 或重生成模型；
         * 越界值（含历史上那个 1.75 倍的野值）会被 SetIdentPsi 直接拒绝。
         * v3 老参数没有 PS_FLAG_PSI_T，走 SetIdentPsi（基准退回 20 °C），
         * 与升级前的行为逐位一致。 */
        FluxIdent_SetCached(pb.r_ohm, pb.l_h, pb.psi_wb);
        if ((pb.flags & PS_FLAG_PSI_T) != 0U) {
            Servo_SetIdentPsiEx(pb.psi_wb, pb.psi_t_ref);
        } else {
            Servo_SetIdentPsi(pb.psi_wb);
        }
    }
    if ((pb.flags & PS_FLAG_FRIC) != 0U) {
        /* 只恢复幅值与黏滞，"是否启用"沿用当前值 ——
         * 上电默认是关的，让用户显式开，避免旧参数在新接线上突然生效。 */
        uint8_t ffen = 0U;
        float   fa = 0.0f, fv = 0.0f, fvs = 0.0f;

        Servo_GetFeedforward(&ffen, &fa, &fv, &fvs);
        Servo_SetFeedforward(ffen, fa, pb.fric_a, pb.fric_visc);
    }
    return 1U;
}

/* ================= 缺相检测（两相采样，2026-11-21）=================
 *
 * 为什么必须做：本板只采 U、W 两相，V 相是 KCL 重构出来的 `iv = -(iu+iw)`。
 * 只要有一相真的没有电流，**重构值也就跟着是 0**——从 KCL 上完全看不出来。
 * 现场因此踩过两次：Z 辨识报 ratio_bad / STUCK，查半天才发现是 V 相断路。
 *
 * 判据（只用两相电流，物理可证）：
 *   健康三相：iu、iv、iw 互差 120°，iu+iw = -iv，一个完整电周期内
 *             rms(iu+iw) 与 rms(iu) 同量级 —— 实测比值 0.963、corr=-0.52
 *   V 相断路：电流只能在 U-W 之间流动，iu ≡ -iw ⇒ iu+iw ≡ 0
 *             —— 实测比值 0.091、corr=-0.997（两次故障都是这个数）
 *   U 相断路：iu ≡ 0 而 iw 正常；W 相断路：iw ≡ 0（可用单相 RMS 比直接判）
 *
 * 唯一会误判的工况：**静止矢量恰好在 V 相过零点**（θ 使 iv=0）。那时
 * iu+iw 确实恒为 0，与 V 相断路在物理上不可区分 —— 这是两相采样的固有限制，
 * 不是判据的缺陷。所以再加一道"角度扫开"门：窗口内**指令电角度**至少扫过
 * 1.5 rad（86°）才认为 iu+iw 的 RMS 有统计意义；纯 DC 保持直接判"不可判"
 * （code=4），不误报。
 *
 * 为什么用指令角而不是实测角：V 相断路时转子几乎不动，用实测角永远扫不开，
 * 恰好把最该看护的工况排除在外。
 *
 * 窗口 200 ms / 10 kHz 抽取，累积量只有三个平方和加一个扫角；开销约 0.4 Mcyc/s
 * （170 MHz 的 0.25%），全在 20 kHz 节拍里，不占 1 ms 任务。
 */
#define PM_DECIM        (2U)        /* 20 kHz 抽到 10 kHz */
#define PM_WIN          (2000U)     /* 200 ms 窗口（10 kHz × 0.2 s） */
/* 激励门槛。0.35 A 试过，太严：100 rpm 空载 iq≈0.27 A，相电流 RMS 只有 0.19 A，
 * 于是正常工况一直判"不可判"——检测器等于没有。降到 0.15 A：
 * 传感器噪声（几十 mA）算出的 rms(iu+iw) 反而比单相大 √2 倍（不相关噪声相加），
 * 比值≈1.4，不会误判成缺相；而 0.15 A 对应相电流峰值 0.21 A，已能涵盖轻载。 */
#define PM_MIN_RMS_A    (0.15f)
#define PM_OPEN_RATIO   (0.30f)     /* rms(iu+iw)/rms(iu or iw) 低于它 → V 缺流 */
#define PM_OPEN_GAIN    (0.20f)     /* 单相 RMS 比值低于它 → 该相缺流 */
#define PM_MIN_SWEEP    (1.5f)      /* 窗口内指令电角度至少扫过 1.5 rad */

static float    s_pm_iu2, s_pm_iw2, s_pm_is2;
static float    s_pm_sweep;
static float    s_pm_th_prev;
static uint8_t  s_pm_first;
static uint8_t  s_pm_dec;
static uint16_t s_pm_n;
/* 诊断：本窗口原始结论 / 消抖后结论 / 比值 */
static volatile uint8_t s_pm_raw;       /* 0 正常 1 U 2 V 3 W 4 不可判 */
static volatile uint8_t s_pm_code;      /* 消抖后：0 正常 1 U 2 V 3 W */
static volatile uint8_t s_pm_ratio;     /* rms(iu+iw)/max(rms) ×100，健康≈100 */
static volatile uint8_t s_pm_bal;       /* min/max 单相 RMS ×100，健康≈100 */
static uint8_t  s_pm_bad_cnt, s_pm_ok_cnt;
static uint8_t  s_pm_trip_en;           /* 1 = 判定缺相后置故障（默认关） */

void Servo_SetPhaseTrip(uint8_t en) { s_pm_trip_en = (en != 0U) ? 1U : 0U; }
uint8_t Servo_GetPhaseTrip(void)    { return s_pm_trip_en; }

uint32_t Servo_GetPhaseMon(void)
{
    return (uint32_t)s_pm_code
         | ((uint32_t)s_pm_raw   << 8)
         | ((uint32_t)s_pm_ratio << 16)
         | ((uint32_t)s_pm_bal   << 24);
}

static uint8_t pm_u8_100(float x)
{
    float v = x * 100.0f;
    if (!(v > 0.0f)) { return 0U; }
    if (v > 255.0f)  { return 255U; }
    return (uint8_t)(v + 0.5f);
}

void Servo_PhaseMonTick(float iu, float iw, float th_cmd)
{
    /* 没有输出时电流恒为 0，累积毫无意义，还会把上一次的窗口拖脏 */
    if (BspServo_IsPwmEnabled() == 0U) {
        s_pm_n     = 0U;
        s_pm_dec   = 0U;
        s_pm_first = 0U;
        s_pm_iu2 = s_pm_iw2 = s_pm_is2 = 0.0f;
        s_pm_sweep = 0.0f;
        s_pm_raw   = 4U;
        return;
    }
    if (++s_pm_dec < PM_DECIM) { return; }
    s_pm_dec = 0U;

    {
        const float isum = iu + iw;
        s_pm_iu2 += iu * iu;
        s_pm_iw2 += iw * iw;
        s_pm_is2 += isum * isum;
    }

    if (s_pm_first == 0U) {
        s_pm_first  = 1U;
        s_pm_th_prev = th_cmd;
    } else {
        float d = th_cmd - s_pm_th_prev;
        /* 折回 [-π, π]：闭环用的是多圈角（连续），开环用虚拟角（连续），
         * 但 s_dir_sign 翻转或 Z 结束重设零点时会出现大跳变，别把它算成扫角。 */
        if (d >  3.14159265f) { d -= 6.28318531f; }
        if (d < -3.14159265f) { d += 6.28318531f; }
        if (d > -1.57079633f && d < 1.57079633f) {   /* 单拍 >90° 视为跳变，忽略 */
            s_pm_sweep += (d > 0.0f) ? d : -d;
        }
        s_pm_th_prev = th_cmd;
    }

    if (++s_pm_n < PM_WIN) { return; }

    /* ---- 窗口结算 ---- */
    {
        const float n  = (float)s_pm_n;
        const float ru = sqrtf(s_pm_iu2 / n);
        const float rw = sqrtf(s_pm_iw2 / n);
        const float rs = sqrtf(s_pm_is2 / n);
        const float rmax = (ru > rw) ? ru : rw;
        const float rmin = (ru > rw) ? rw : ru;
        uint8_t code;

        if (rmax < PM_MIN_RMS_A) {
            code = 4U;                                  /* 无激励 */
        } else if (s_pm_sweep < PM_MIN_SWEEP) {
            code = 4U;                                  /* 角度没扫开，不可判 */
        } else if (rmin < (PM_OPEN_GAIN * rmax)) {
            code = (ru < rw) ? 1U : 3U;                 /* U / W 缺流 */
        } else if (rs < (PM_OPEN_RATIO * rmax)) {
            code = 2U;                                  /* V 缺流 */
        } else {
            code = 0U;
        }

        s_pm_raw   = code;
        s_pm_ratio = pm_u8_100((rmax > 1.0e-6f) ? (rs / rmax) : 0.0f);
        s_pm_bal   = pm_u8_100((rmax > 1.0e-6f) ? (rmin / rmax) : 0.0f);

        /* 消抖：连续 2 个窗口（400 ms）才认定，连续 3 个窗口正常才清除。
         * 用窗口数而不是拍数：判据本身是统计量，拍级消抖没有意义。 */
        if ((code == 0U) || (code == 4U)) {
            if (s_pm_ok_cnt < 3U) { s_pm_ok_cnt++; }
            s_pm_bad_cnt = 0U;
            if (s_pm_ok_cnt >= 3U) { s_pm_code = 0U; }
        } else {
            if (s_pm_bad_cnt < 3U) { s_pm_bad_cnt++; }
            s_pm_ok_cnt = 0U;
            if (s_pm_bad_cnt >= 2U) { s_pm_code = code; }
        }
    }

    s_pm_n     = 0U;
    s_pm_iu2 = s_pm_iw2 = s_pm_is2 = 0.0f;
    s_pm_sweep = 0.0f;
    s_pm_first = 0U;
}

/* 1 ms：只做"是否升级为故障"。默认关（trip_en=0）：缺相跳闸会直接断输出，
 * 关节负载侧会掉下来，是否接受由使用者决定；判据本身照常工作、可随时读。 */
static void phase_mon_task1ms(void)
{
    if ((s_pm_trip_en != 0U) && (s_pm_code >= 1U) && (s_pm_code <= 3U) &&
        (BspServo_IsPwmEnabled() != 0U)) {
        BspServo_RaiseFault(BSP_FAULT_PHASE_LOSS);
    }
}

void Servo_SlowTask1ms(void)
{
    BspServo_SlowTask1ms();
    /* 温度降额系数：驱动层刚刚按最新 NTC 算好，搬到本文件供 20 kHz 环读。
     * 放在 BspServo_SlowTask1ms 之后，保证用的是这一拍的温度。 */
    Servo_SetDerate(BspServo_GetDerate());
    /* Kt 温度补偿：用同一拍的电机 NTC 更新，供 20 kHz 的力矩换算读。
     * 放在 derate 之后，两者都依赖这一拍的 NTC 采样。 */
    Servo_UpdateKtComp(BspServo_GetTempMotor());
    phase_mon_task1ms();      /* 缺相：可选升级为故障（默认只诊断不跳闸） */
    Servo_MitTask1ms();       /* MIT 命令超时：停发就冻结 v_des/τ_ff */
    ServoCmd_Task1ms();       /* deadman 倒计时：到点只置标志，封锁在主循环做 */
    CanProto_Task1ms();       /* CAN 链路测试：空闲时直接 return，零开销 */

    /*
     * 速度模式：1 kHz 推进一次速度轨迹（梯形/S 曲线），把 v_ref 积分成位置给定。
     *   - 与模型位置环同频（模型 cnt_i>=20 @20kHz = 1 kHz），不会引入额外采样延迟
     *   - 未使能或有故障时回到待命：斜坡清零、给定贴住实际位置，
     *     这样下一次使能是"从当前位置重新起步"，不会因为给定落后/超前而抽一下
     */
    if (s_mode == SERVO_MODE_VELOCITY) {
        /* 必须用**多圈**角：pos_ref 与 theta_pos 都在这个坐标系里，
         * 用单圈会让每次跨 2π 产生 2π 的位置误差。 */
        float theta = BspServo_GetThetaMulti();
        if ((s_enable != 0U) && (BspServo_GetFault() == 0U)) {
            s_pos_ref  = VelProfile_Step(&s_vel, theta);
            /* 速度前馈 = 轨迹当前速度。这是第 4 项的核心：
             * 位置环纯 P 的稳态滞后 e = v_ref/Kp，20 rad/s 时 0.40 rad，
             * 前馈直接把这个量补掉，位置误差只剩动态部分。 */
            s_pos_w_ff = VelProfile_GetVref(&s_vel);
        } else {
            VelProfile_Disable(&s_vel, theta);
            s_pos_ref  = theta;
            s_pos_w_ff = 0.0f;
        }
    }

    /* 100 Hz 遥测上发（内部 10 分频）：Iq / Id / 母线电压 / 位置 */
    Telemetry_Task1ms();

    /* 开机 3 s 打一次周期预算诊断，之后静默 */

    /* R/L 在线辨识状态机（非活动时只有一次比较） */
    (void)RlIdent_Task1ms();

    /* 编码器自校准状态机（非活动时只有一次比较） */
    (void)EncCal_Task1ms();

    /* 执行器占用到期检查：所有辨识状态机跑完之后再判，避免用到上一拍的状态 */
    Act_Task1ms();

    /* 参数变化时写 Flash（详见 Param_PersistTask1ms） */
    Param_PersistTask1ms();

    /* 连续 3ms 没有任何控制节拍，说明注入触发链或 ADC2 注入启动失败。
     * Flash 擦写期间属于**预期内的停摆**：单 bank 模式下擦一页会让 CPU 取指
     * 停顿 20~40 ms，20 kHz 节拍必然丢几百拍。不排除这一种情况就会在每次
     * 保存参数后立刻误报 BSP_FAULT_ADC_SYNC。 */
    if (ParamStore_IsBusy() != 0U) {
        s_inj_miss  = 0U;
        s_last_tick = s_tick_count;
    } else if (s_tick_count == s_last_tick) {
        if (++s_inj_miss >= 3U) {
            BspServo_RaiseFault(BSP_FAULT_ADC_SYNC);
        }
    } else {
        s_inj_miss = 0U;
        s_last_tick = s_tick_count;
    }
}

/* ---------------- 上电初始化 ---------------- */
/*
 * 三环参数注入。
 *
 * 2026-11-15 18:37 重新生成的模型把参数从"编译期字面量"改成了顶层输入
 * par_c / par_w / par_p，并且整体换成 real32_T（单精度）。
 *   好处：改参数不用碰生成代码，也就没有"重新生成就丢补丁"的问题；
 *   代价：必须在这里填一次。不填的话三组数组全 0 ——
 *         Kp=Ki=0 时电流环没有任何输出，现象是"有电流但不转、位置环 clamp 疯涨"。
 *
 * 各数组含义（与 build_three_loop_deploy.m 的顺序一致）：
 *   par_c = [kp_i, ki_i, dt, Ld, Lq, psi, max_duty, overmod, mag_vd_max, dec_mode, i_max]
 *   par_w = [kp_w, ki_w, Ts_w, i_max, N_w]
 *   par_p = [kp_p, w_max, Ts_p, N_p]
 *
 * 数值来自 three_loop_params.m。两个约束必须守住：
 *   1) dt 必须等于**实际控制周期**（现在是 10 kHz -> 100 us）；
 *   2) N_w = Ts_w/Ts_i、N_p = Ts_p/Ts_i，否则三环执行频率会错一档。
 */
static void model_params_init(void)
{
    /* ---- 电流环 ---- */
    three_loop_deploy_U.par_c[0]  = 0.71314153f;   /* kp_i = wc*Ld, wc = 2*pi*1000 */
    three_loop_deploy_U.par_c[1]  = 2378.1856f;    /* ki_i = wc*R_eff */
    three_loop_deploy_U.par_c[2]  = 5.0e-5f;       /* dt   = 50 us (20 kHz) */
    three_loop_deploy_U.par_c[3]  = 113.5e-6f;     /* Ld */
    three_loop_deploy_U.par_c[4]  = 113.5e-6f;     /* Lq（表贴式 Ld=Lq） */
    three_loop_deploy_U.par_c[5]  = 7.59298e-3f;   /* psi [Wb] */
    three_loop_deploy_U.par_c[6]  = 0.95f;         /* max_duty */
    three_loop_deploy_U.par_c[7]  = 1.0f;          /* overmod */
    three_loop_deploy_U.par_c[8]  = 0.98f;         /* mag_vd_max */
    three_loop_deploy_U.par_c[9]  = 3.0f;          /* dec_mode: 交叉耦合 + BEMF */
    three_loop_deploy_U.par_c[10] = 9.0f;          /* i_max [A] */

    /* ---- 速度环：设计 2 kHz，50 us 下 N_w = 500/50 = 10 ---- */
    three_loop_deploy_U.par_w[0]  = 0.07292272f;   /* kp_w */
    three_loop_deploy_U.par_w[1]  = 10.190689f;    /* ki_w */
    three_loop_deploy_U.par_w[2]  = 5.0e-4f;       /* Ts_w = 500 us（积分步长） */
    three_loop_deploy_U.par_w[3]  = 9.0f;          /* i_max [A] */
    three_loop_deploy_U.par_w[4]  = 10.0f;         /* N_w */

    /* ---- 位置环：设计 1 kHz，50 us 下 N_p = 1000/50 = 20 ---- */
    three_loop_deploy_U.par_p[0]  = 50.265482f;    /* kp_p */
    three_loop_deploy_U.par_p[1]  = 120.0f;        /* w_max [rad/s] */
    three_loop_deploy_U.par_p[2]  = 1.0e-3f;       /* Ts_p = 1 ms（P 环不用 dt，仅记录） */
    three_loop_deploy_U.par_p[3]  = 20.0f;         /* N_p */
}

void Servo_Init(void)
{
    uint8_t tim6_ok = 1u;

    /*
     * 第一步先出串口横幅，早于任何 BSP 动作。
     * 这样上电现象可以被切开判定：
     *   看到横幅 + 状态行  -> 串口链路 + BSP 都正常，可以开始收二进制帧
     *   只看到横幅         -> TX 通，BSP 或 TIM6 失败（看状态行的 bsp=/tim6=）
     *   一个字都没有       -> 物理层问题（接线/485 方向/波特率/PB6 未引出）
     */
    UartLink_Init();
    Act_Init();               /* 执行器占用仲裁：必须在任何辨识可能启动之前 */
    ServoCmd_Init();          /* 打开 RXNE：E/D/C/S 单字符命令，见 servo_cmd.c */
    /* MAVLink 协议层初始化（2026-11-18）：database 门卫 + 帧接收/分发。
     * 必须在 ServoCmd_Init 之后（RXNE 已开），在控制环启动之前。 */
    AppParam_Init();
    MavHost_Init();

    three_loop_deploy_initialize();

    three_loop_deploy_U.en = 0.0;
    s_pos_ref = 0.0f;
    s_enable = 0U;

    /* 参数注入必须在 initialize 之后（initialize 会清零 U/Y/DW） */
    model_params_init();

    /* 速度轨迹：1 kHz，默认 S 曲线 + a=200 rad/s^2 + j=2000 rad/s^3，目标 0 */
    EncCal_Init();
    FricCal_Init();
    MechIdent_Init();
    FluxIdent_Init();
    CogFf_Init();
    EncComp_Init();          /* 编码器角度软件补偿：默认关闭（系数全 0） */
    /* 位置环：ts 必须等于速度环节拍（2 kHz / 500 us），
     * 因为 PosLoop_Step 只在速度环执行的那一拍被调用一次。 */
    PosLoop_Init(SERVO_SPD_TS);

    /* 预填一次调度值：否则首次 . 回显 kp=0.00，看不出实际会用多少。
     * 用局部变量再赋给 volatile 成员 —— 直接把 volatile 指针转成 float*
     * 传进去虽然能编过，但语义上是在绕过 volatile，不该这么写。 */
    {
        float k0 = 0.0f, i0 = 0.0f;
        sched_lookup(0.0f, &k0, &i0);
        s_sched_kp   = k0;
        s_sched_kits = i0;
    }
    VelProfile_Init(&s_vel, 0.001f);
    s_mode = SERVO_MODE_POSITION;
    if (BspServo_Init() != HAL_OK) {
        BspServo_RaiseFault(BSP_FAULT_BSP_INIT);
        return;                       /* 30090 的 fault 位 = 0x0008 (BSPINIT) */
    }

    /* 恢复上次辨识/标定结果。CRC 不通过就当作没存过，绝不半套应用。
     * 失败原因（1=magic 2=version 3=crc 4=flags）走 30082 只读槽 13（ps_diag），
     * 启动时不再打 ASCII。 */
    (void)param_load_and_apply();

    /* 上电就把位置环坐标对齐到单圈角（2026-11-21）。
     *
     * 必须在 param_load_and_apply() 之后：零点加载前 s_enc_zero=0，
     * 那时对齐算出来的偏置是错的。
     *
     * 为什么上电就对：30100 的 p_now 从第一帧起就应当等于 30090.pos
     * （差整数个 2π）。不对齐的话上电读到的是裸 theta_multi，与单圈角
     * 差 -s_enc_zero（本机实测 0.1231 vs 5.4755，差 5.35 rad），
     * 看起来就像"上电角度不对、进了位置模式才跳回来"。
     *
     * 之后每次进位置模式还会再对齐一次；那时 theta_multi 与单圈角都没变，
     * 平移量为 0，所以不会产生任何跳变。 */
    (void)pos_coord_align();

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        BspServo_RaiseFault(BSP_FAULT_BSP_INIT);
        tim6_ok = 0u;
    }

    /* 遥测上下文就绪后，1 ms 任务里的 10 分频才开始发帧 */
    Telemetry_Init();

    /* 先跑一次慢任务，让状态行里的 vbus 是实测值，而不是上电默认 48.0 */
    BspServo_SlowTask1ms();

    /* 遥测闸门在这里打开（原来要等 ASCII 状态行发完，现在没有那一行了） */
    if (tim6_ok != 0u) {
        Telemetry_Start();
    }
}
