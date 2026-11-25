/*
 * servo_cmd.c —— 串口单字符调试命令（首次上电联调用的最小使能入口）
 *
 * 为什么要有这一层：固件原本没有任何使能入口（Servo_SetEnable / BspServo_PwmEnable(1)
 * 均无调用者），BDTR.MOE 恒为 0，电机永远不可能通电。但"直接放开"又不行 ——
 * 电流零点未标定时 iw 有 -1.41 A 的虚假读数，闭环一使能就会输出错误力矩。
 * 所以这里做成"先标定、才允许使能"的联锁，并加 deadman 超时自动封锁。
 *
 * 命令（单字符，921600 8N1）：
 *   C  电流零点标定（要求 MOE=0、电机静止，约 200 ms）
 *   Z  电角度/极对数/方向辨识（VESC 三段式，约 9 s）
 *   I  查看上一次辨识结果
 *   E  使能（要求已标定 + 无故障）→ 置 MOE=1，同时启动 2 s deadman
 *   D  禁用（先封锁 MOE，再清使能，占空比回中）
 *   M  电流闭环 +1 A        N  电流闭环 -1 A
 *   T  电流阶跃 iq          U  电流阶跃 id          V  两轴同时
 *   W  速度闭环 正转        Q  速度闭环 反转
 *   O  循环速度档位 3/30/100/500/1500/3000 rpm（W/Q 用它；在环内按 O 直接换给定）
 *   L  速度环增益档位 0.125x..32x（Y 复位到 1x = 实测最优）
 *   Y  速度环增益 + 转速档位复位
 *   K  速度环详情（wref/ramp/wmeas/iqc/iterm/nex/kp/kiTs/acc/iqm）
 *   =  速度给定（连续）：`=300⏎`，可带小数与负号，空值=查询。走 do_speed，联锁同 W
 *   ~  加减速限制 rad/s^2：`~400⏎`，默认 400（≈3820 rpm/s），>=20000 视为不限制
 *   |  速度环出口 iq 限幅 A：`|3⏎`，默认 3，上限 9
 *   F  速度轨迹 +3 rpm      R  速度轨迹 -3 rpm
 *   P  位置模式（保持当前角度）
 *   S  状态回读（fl=1 表示 Flash 里有通过 CRC 的辨识参数）
 *   A  立即把 bias+zero+ratio+速度环增益写进 Flash（自动路径 500 ms 也会存）
 *   !  软复位（验证冷启动能否自动恢复参数）
 *   X  清故障（要求 MOE=0）
 *   ?  帮助
 *
 * 运行态互斥：辨识 / 速度闭环 / 电流闭环 / 模型三环 只能有一个在跑，
 * 切换前必须先 D。ISR 的分支优先级是 fault > 辨识 > 电流(含速度) > 模型，
 * 所以"直接发另一条命令"只会看起来生效、实际还在跑旧环路。
 *
 * 任何命令都会重置 deadman；2 s 内无命令自动封锁输出。
 */
#include "servo_cmd.h"
#include "bsp_servo.h"
#include "actuator.h"
#include "motor_ident.h"
#include "encoder_cal.h"
#include "fric_cal.h"
#include "mech_ident.h"
#include "motor_flux_ident.h"
#include "cog_ff.h"
#include "motor_rl_ident.h"
#include <math.h>       /* fabsf：力矩模式的余速门禁 */
#include "deadband_comp.h"

/* ---- 统一的"辨识占用"判断（2026-11-21 加）-------------------------------
 *
 * 三路都要查。只查 Servo_IsOpenLoop() 会漏两类：
 *   · Act_IsBusy()        —— 所有辨识都通过 Act_Acquire 登记。这条覆盖
 *                            $ ( ) < } 这类**用速度环/电流环跑**的辨识，
 *                            它们全程 s_ol_en == 0。
 *   · Servo_IsOpenLoop()  —— 开环拖动：C / Z / ^ 的主体、@ 的 R 段。
 *   · Servo_IsVoltInject()—— @ 的 L 段：它先 SetOpenLoop(0) 再开注入
 *                            （motor_rl_ident.c:307-309），此时 s_ol_en
 *                            已经是 0，只剩这个标志能说明它在跑。
 *
 * 漏掉的后果：辨识跑到一半发 30054 / 30019 能通过检查，位置环或速度环
 * 起环后，ISR 里的分支优先级是 s_pos_en > s_ol_en，辨识的给定会被整段
 * 忽略——现象是"辨识卡住不动、最后超时"或"算出一个荒谬的 L"。
 */
static uint8_t ident_busy(void)
{
    if (Act_IsBusy() != 0U)         { return 1U; }
    if (Servo_IsOpenLoop() != 0U)   { return 1U; }
    if (Servo_IsVoltInject() != 0U) { return 1U; }
    return 0U;
}
#include "cur_step.h"
#include "telemetry_tx.h"
#include "mavlink_common.h"   /* MAV_ID_* 辨识结果 kind */
#include "pos_loop.h"
#include "param_store.h"
#include "uart_link.h"
#include "mavlink_host.h"
#include "mavlink_common.h"   /* MAV_STX */
#include "can_proto.h"        /* CAN FD 内环自检 */
#include "usart.h"

#include <string.h>

#define CMD_DEADMAN_MS   2000U
#define CMD_LINE_MAX     200U
/* 命令环形缓冲。2026-11-21：32 → 256。
 * 32 B 在 ASCII 时代够用（单字符命令最多 8 B），迁到飞控协议后最长的请求是
 * 30094 AppRecord，帧长 12+144 = **156 B** —— 一旦接收期间主循环有 >350 µs 的
 * 阻塞（发送一条 156 B 应答本身就是 1.7 ms 的逐字节轮询），后面的字节就会被
 * "满了丢新字节"静默吃掉，现象是"某条请求偶发没有应答，遥测照常、固件
 * 侧的 CRC 错与 pending 顶替计数都不动"。这个缺陷是 30094 分块连读暴露出来的：
 * 64 块里随机丢 1~2 块。256 B 对应 2.8 ms 的缓冲窗口，覆盖最长阻塞。
 * 代价：RAM +224 B，索引仍是 uint8（(s_qh+1) % 256 不会溢出）。 */
#define CMD_Q_LEN        256U
#define CMD_RPM_TEST     3.0f                   /* F/R 用的转速：首次联调先给 3 rpm，
                                                 * 确认编码器方向与相序一致后再调大 */
/* W/Q 的转速档位（O 键循环）。做成表而不是参数解析：本命令层是单字符流，
 * 引入数字解析就要区分"命令里的数字"和"MAVLink 二进制里的数字"，
 * 后者的 0x30~0x39 会在任何时刻出现。表能覆盖常用点，风险最低。 */
/* 档位覆盖"低速标定"到"额定 3000 rpm"：
 * 注意 3000 rpm 在 30.75 V 母线下**物理不可达** —— BEMF 峰值 p*psi*w = 33.4 V，
 * 已超过 SVPWM 能输出的 0.5375*30.75 = 16.5 V。该档位用于验证"电压受限"行为
 * （速度环误差长期偏大、iq 顶限幅、转速停在 ~1485 rpm），
 * 要真跑到 3000 rpm 需要 vbus >= 62 V。 */
#define CMD_SPD_RPM_N     6U
static const float s_spd_rpm_tab[CMD_SPD_RPM_N] =
    { 3.0f, 30.0f, 100.0f, 500.0f, 1500.0f, 3000.0f };
static uint8_t     s_spd_rpm_idx = 1U;      /* 默认 30 rpm */

#define CMD_RPM_SPD_TEST 30.0f                  /* W/Q 用：真速度闭环的转速。
                                                 * 为什么不是 3 rpm —— 速度环带宽约 10 Hz，
                                                 * 3 rpm 时反电势与 iq 都太小，测量噪声占主导；
                                                 * 30 rpm 空载只需 ~0.2 A，BEMF 仅 0.33 V，
                                                 * 既在安全区，信噪比又足够看清阶跃。 */
#define CMD_RAD_PER_RPM  (0.10471975512f)       /* 2π/60 */
#define CMD_CUR_TEST     1.0f                   /* M/N 的 iq 给定：Kt=1.5*p*psi=0.16 N·m/A，1 A 约 0.16 N·m */
#define CMD_STEP_AMP     2.0f                   /* T/U/V 阶跃幅值 [A]：1 A 时所需相电压(0.38V)与死区误差(0.4V)同量级，
                                                   * 环路进不了线性区；2 A 对应 0.76V，信噪比够用 */

static volatile uint8_t  s_q[CMD_Q_LEN];
static volatile uint16_t s_q_ovf;        /* 环形缓冲溢出丢字节计数（诊断） */
static volatile uint8_t  s_qh;
static volatile uint8_t  s_qt;
/* 累计接收字节数（临时诊断用，不取模）。qh/qt 是环形索引，
 * 绝对值无法区分"一个字节没进"和"进了又被主循环消费完"，
 * 只有单调计数能定性。排查完随 diag 一起删。 */
static volatile uint32_t s_rx_total;
static volatile uint8_t  s_kill;
static volatile uint16_t s_deadman;
static uint8_t           s_armed;

/* 辨识注入的 d 轴电流。
 * 2026-11-20 实测三档：
 *   2.0 A → 120° 步进时电流失控，全程峰值 −16.5 A，0.1 s 就 IDENT FAIL fault_run；
 *   1.0 A → 不报错但 45 s 跑不完（空载拖不动，步进段速度建立不起来）；
 *   1.5 A → 折中，约 0.27 N·m，空载能拖动而冲击低于 12 A 阈值。
 * 根因不是阈值太小（12 A 已经很高），而是电角度未定时 2 A 的 d 轴电流
 * 会落到 q 轴上产生大转矩，空载 J 只有 7.7e-5，几十毫秒就冲到几千 rpm，
 * 反电动势把电流环顶到失控。所以只能从注入幅值上压。 */
#define CMD_ALIGN_ID     2.0f
#define CMD_RAD2DEG      57.2957795131f

/* AC6 -O3 会把 put_* 在每个调用点整段展开：本文件有 20+ 个格式化点，
 * 实测把 servo_cmd.o 撑到 42 KB（占整个固件的 45%）。这些函数只在主循环/
 * 1 ms 上下文跑，调用开销可忽略，一律禁止内联。 */
#define CMD_NOINLINE  __attribute__((noinline))

/* 前置声明：do_align()（前面）和 ServoCmd_ExecAction()（后面）都要用，
 * 定义在 ExecAction 之前，所以这里先声明。 */
static void push_reject(uint8_t kind, uint8_t err);

/* USART 错误标志（ORE/FE/NE/PE）累计。ASCII 通道删除后仍保留：
 * ORE 死锁是这块板反复踩到的故障（见 stm32g4xx_it.c 的 RXNE/ORE 处理），
 * 有一个单调计数就能判断"收不到命令"是不是溢出造成的。 */
static volatile uint16_t s_uart_err;

/* MIT 环联锁拒绝原因（0x02 二进制帧已删除，但 mit_engage 是 MAVLink 30065
 * 与 ServoCmd_CtrlMit 共用的核心，这两个计数仍被它写入，供 30091 诊断读回）。 */
static volatile uint16_t s_mit_bad;
static volatile uint8_t  s_mit_why;

void ServoCmd_NoteUartErr(void)
{
    if (s_uart_err < 0xFFFFU) { s_uart_err++; }
}

/* ---------------- 电流环增益缩放（调 PI 用） ----------------
 * 标称：Kp = wc*Ld = 0.5341，Ki*Ts = 0.12566（wc = 2π·1000，Ts = 50us，L=85µH 实测）
 * 2026-11-17 由 0.71314153 / 0.11890653 按**实测电感**重算：原值按设计 L=113.5 µH，
 * 实际带宽跑到 1335 Hz（设计 1000），且零点/极点比掉到 0.71、没对消干净。
 * ⚠ 必须与 servo_isr.c 的 SERVO_CUR_KP / SERVO_CUR_KI_TS 一致，否则 G/H 档位会跳变。
 * G 键循环 Kp 比例，H 键循环 Ki 比例。两者同比缩放 = 只改带宽、零点不动，
 * 这是"整定带宽"的正确姿势；单独改某一个会破坏极点抵消，仅作调试对比。
 */
#define CMD_GAIN_BASE_KP     0.5341f
#define CMD_GAIN_BASE_KITS   0.12566f
#define CMD_GAIN_N           6U
static const float s_gain_scale[CMD_GAIN_N] = { 0.5f, 0.75f, 1.0f, 1.25f, 1.5f, 2.0f };
static uint8_t s_gk = 2U;                 /* 默认 1.00x */
static uint8_t s_gi = 2U;

static CMD_NOINLINE void apply_gains(void)
{
    Servo_SetCurGains(CMD_GAIN_BASE_KP * s_gain_scale[s_gk],
                      CMD_GAIN_BASE_KITS * s_gain_scale[s_gi]);
}

/* ---------------- 速度环增益缩放（L 键循环） ----------------
 * 为什么要缩放而不是直接改常数：速度环的合适 Kp 与**转动惯量**成正比
 * （Kp ≈ J·wc²/(Kt·|1+j·wc/ωz|)），而 J 没有实测值。
 * 模型值 Kp=0.072923 在 J=5e-4 下只给 2.4 Hz 带宽、300 ms 建立；
 * 若这台电机的真实惯量小一个数量级，同一组参数就会变成高增益环。
 *
 * 2026-11-15 实机（30 rpm，逐档实测稳态纹波）：
 *   kp=0.07(1x) 纹波 23.6 rpm 超调 193%   <- 模型值在这台电机上太小
 *   kp=0.15(2x) 纹波 11.2 rpm 超调 108%
 *   kp=0.29(4x) 纹波  5.5 rpm 超调  54%
 *   kp=0.58(8x) 纹波  2.8 rpm 超调  24%
 * 规律是"增益越低、纹波越大"，说明低增益下积分器在慢慢摆（不是高增益自激）。
 * 所以档位必须能往上走到 32x，才能找到纹波回升的拐点。
 *
 * 档位取 2 的幂：等比缩放同时抬高 Kp 与 Ki·Ts，零点不动（ωz 保持 22 Hz），
 * 只有带宽变 —— 这样"扫档"只改变一个自由度，结论才不会互相干扰。
 */
#define CMD_SPD_N   9U
static const float s_spd_scale[CMD_SPD_N] =
    { 0.125f, 0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f };
static uint8_t s_spd_k = 3U;                 /* 默认 1.00x = 模型值 */

/* 基准是**实测最优值**（8 x 模型值），不是模型原值 —— 见 servo_isr.c 的实测表。
 * 这样 Y 复位后就是可直接用的档位，L 只在需要重新扫时用。 */
#define CMD_SPD_BASE_KP     (0.5833818f)
#define CMD_SPD_BASE_KITS   (0.04076276f)

/* 上电时把档位索引与"实际生效的 kp"对齐。
 * 为什么需要：Flash 里存的是 kp 数值，而 s_spd_k 是 RAM 里的档位索引。
 * 若两者不同步（比如上次存的是 4x，复位后索引仍是 1x），
 * 复位后第一次按 L 会从 1x 往上跳，增益突然掉一半。 */
static void sync_spd_gain_index(void)
{
    static uint8_t s_done;
    float kp = 0.0f, kits = 0.0f;
    uint8_t i;

    if (s_done != 0U) { return; }
    s_done = 1U;

    Servo_GetSpeedGains(&kp, &kits);
    for (i = 0U; i < CMD_SPD_N; i++) {
        float ref = CMD_SPD_BASE_KP * s_spd_scale[i];
        if ((kp > (ref - 0.01f * ref)) && (kp < (ref + 0.01f * ref))) {
            s_spd_k = i;
            return;
        }
    }
}

static CMD_NOINLINE void apply_spd_gains(void)
{
    Servo_SetSpeedGains(CMD_SPD_BASE_KP * s_spd_scale[s_spd_k],
                        CMD_SPD_BASE_KITS * s_spd_scale[s_spd_k]);
}


static void do_disable(const char *why)
{
    (void)why;
    /* 顺序：先封 MOE（立即断输出），再清使能，最后占空比回中 */
    BspServo_PwmEnable(0U);
    Servo_SetEnable(0U);
    Servo_SetOpenLoop(0U, 0.0f, 0.0f);      /* 万一在辨识过程中，一并退出 */
    Servo_SetSpeedMode(0U, 0.0f);           /* 速度闭环先退，它会顺带关电流模式 */
    /* 位置环必须显式退（2026-11-22 补）。原来只退了速度/电流/开环，
     * s_pos_prev 一直是 1，后果：
     *   deadman 封锁 -> s_armed=0 但 s_pos_prev=1
     *   -> 下次 30019 走 do_pos 的"起环"分支
     *   -> Servo_SetPosMode() 内部因 s_pos_prev!=0 跳过对齐与重设目标
     *   -> 紧接着的 Servo_SetPosTarget(同一目标) 被去重忽略
     *   -> 电机停在原地（实测：goto 6.20 后 p_now 仍是 3.60、travel=0.0005）
     * 表现是"偶发：下发了目标但电机不动"，取决于 deadman 是否刚好触发。 */
    Servo_SetPosMode(0U, 0.0f);             /* 位置环一并退出（会清 s_pos_prev） */
    Servo_SetCurrentMode(0U, 0.0f);         /* 电流闭环模式一并退出 */
    MotorIdent_Abort();                     /* 内部会再次确认封锁，幂等 */
    RlIdent_Abort();                        /* R/L 辨识若在跑，一并退出 */
    EncCal_Abort();                         /* 编码器校准若在跑，一并退出 */
    Act_ForceRelease();                     /* 执行器占用令牌一并收回 */
    FricCal_Abort();                        /* 摩擦标定若在跑，一并退出 */
    MechIdent_Abort();                      /* 机械辨识若在跑，一并退出 */
    FluxIdent_Abort();                      /* ψ/R/L 辨识若在跑，一并退出 */
    BspServo_SetDuty(0.5f, 0.5f, 0.5f);
    s_armed = 0U;
    s_deadman = 0U;

}

/* ---------------- 电流闭环（力矩模式）：M = +1 A，N = -1 A ----------------
 * 这是"先把电流环闭起来"的最小入口：θ 用编码器电角度，id 给定恒为 0，
 * iq 由 CMD_CUR_TEST 给定，走 Board/servo_isr.c 里 s_cur_* 那条分支。
 *
 * 三道联锁（缺一不可，都是实测踩出来的）：
 *   1. 电流零漂已标定（C）—— 否则 PI 把偏置当真实电流一路积分
 *   2. 电角度已对齐（Z）  —— θ 来自编码器，零点不对时力矩方向是随机的
 *   3. 无故障位            —— 故障时 PwmEnable 会静默拒绝置 MOE
 */
static void do_current(float iq)
{

    if (BspServo_GetFault() != 0U) {
        return;
    }
    if (BspServo_IsCurrentCalibrated() == 0U) {
        return;
    }
    if (BspServo_IsEncoderZeroed() == 0U) {
        return;
    }
    if (ident_busy() != 0U) {
        /* 辨识正在占用控制链，ISR 会优先走它，
         * 此时打开电流模式只会放 MOE 而不受控。 */
        return;
    }
    if (Servo_IsSpeedMode() != 0U) {
        /* 速度闭环下 iq_ref 由速度环每 2 kHz 覆盖，此时改 s_cur_iq_ref 不会有任何效果，
         * 却会让用户以为力矩已切换 —— 明确拒绝比静默失效好。 */
        return;
    }

    if (s_armed != 0U) {
        if (Servo_IsCurrentMode() == 0U) {
            return;
        }
        /* MIT 是电流模式的子模式，ISR 里它的优先级高于 s_cur_iq_ref。
         * 此时改 iq_ref 会被 MIT 每拍覆盖 —— 回显"updated"却完全没有效果，
         * 属于最难查的静默失效，所以必须显式拒绝（2026-11-18）。 */
        if (Servo_IsMit() != 0U) {
            return;
        }
        /* 已在电流模式：只改给定，不重开 MOE（避免一次无谓的通断冲击） */
        Servo_SetCurrentMode(1U, iq);
        s_deadman = CMD_DEADMAN_MS;
        return;
    }

    /* 余速门禁（2026-11-21）
     *
     * 力矩模式的超速门限是**固定值**（servo_isr.c 的 SERVO_CUR_WMAX = 31.415927
     * rad/s ≈ 300 rpm，= 10π），它没有像速度环那样把"进环余速"纳入峰值保持。
     * 于是带 1000 rpm 余速切进来（set_torque 会自己起电流环）会在 0.4 ms 内
     * 置 BSP_FAULT_OVERSPEED，整机封锁 —— 现象是"一切力矩模式就报故障"，
     * 而真实原因只是电机还在滑行。
     *
     * 这里选择**在入口拒绝**而不是放宽门限：那条 300 rpm 是力矩模式唯一的
     * 失控保护（没有速度环，空载给一点 iq 就会一路加速），放宽它等于拆掉它。
     * 拒绝 + 明确提示，用户停稳后再切即可。留 10% 余量（270 rpm）。
     *
     * 注意用 |omega| 而不是带符号值：这里只关心幅值，坐标系无关。
     */
#define CMD_CUR_WMAX_RADS   (31.415927f)   /* 必须与 servo_isr.c 的 SERVO_CUR_WMAX 一致 */
    {
        const float w_now = fabsf(BspServo_GetOmegaMech());

        if (w_now > (0.9f * CMD_CUR_WMAX_RADS)) {
            return;
        }
    }

    /* 进入顺序：关模型三环 -> 贴位置给定 -> 起电流模式 -> 最后放 MOE。
     * 先起电流模式再放 MOE 是刻意的：ISR 里 MOE=0 期间该分支只保持中点并清积分，
     * 放开 MOE 的那一拍直接从零积分起步，不会有预积分冲击。 */
    Servo_SetEnable(0U);
    Servo_SetMode(SERVO_MODE_POSITION);
    Servo_SetPosRef(BspServo_GetThetaMulti());   /* 多圈：与模型的 theta_pos 同坐标系 */
    Servo_SetCurrentMode(1U, iq);
    BspServo_PwmEnable(1U);

    if (BspServo_IsPwmEnabled() == 0U) {
        Servo_SetCurrentMode(0U, 0.0f);
        return;
    }

    s_armed   = 1U;
    s_deadman = CMD_DEADMAN_MS;
    /* 这里原来写死字符串 "moe=1"。它会永远报 1，即使 PWM 实际没打开 ——
     * 2026-11-21 排查录波 "no output" 时就被它带偏：`>` 回显 moe=1，
     * 而紧接着 `&` 报 no output，看着像自相矛盾，实际是回显在说谎。
     * 回显必须读真实状态，否则排查时间全花在怀疑工具上。 */
}

/* ---------------- 速度闭环：W = +30 rpm，Q = -30 rpm ----------------
 * 与 F/R 的区别（这是本节存在的理由）：
 *   F/R -> Servo_SetMode(VELOCITY) -> 速度轨迹积分成 pos_ref -> 模型位置环
 *         速度**误差**不进 PI，靠位置环 P 把"位置落后量"换成转速，
 *         本质是"给位置环喂一条匀速度斜坡"，不是速度闭环。
 *   W/Q -> Servo_SetSpeedMode() -> (w_ref - w_meas) 直接进 PI -> iq_ref -> 电流环
 *         速度反馈真正闭合，能压住摩擦/齿槽/负载扰动。
 */
static void do_speed(float rpm)
{
    const float w = rpm * CMD_RAD_PER_RPM;

    if (BspServo_GetFault() != 0U) {
        return;
    }
    if (BspServo_IsCurrentCalibrated() == 0U) {
        return;
    }
    if (BspServo_IsEncoderZeroed() == 0U) {
        /* 速度环本身只需要 omega，但力矩要靠电角度——零点不对时
         * "正向 iq"可能产生反向力矩，速度环会一路推成飞车。 */
        return;
    }
    if (ident_busy() != 0U) {
        return;
    }

    if (s_armed != 0U) {
        if (Servo_IsSpeedMode() == 0U) {
            return;
        }
        /* 已在速度闭环：只改给定，不重开 MOE（MOE 通断会带来一次冲击） */
        Servo_SetSpeedMode(1U, w);
        s_deadman = CMD_DEADMAN_MS;
        return;
    }

    /* 进入顺序：关模型三环 -> 切模式（内部把 pos_ref 贴到当前角）->
     * 起速度闭环（内部起电流环并清两套积分）-> 最后放 MOE。
     * 顺序不可颠倒：先起环路再放 MOE，放开那一拍积分从 0 起步。 */
    Servo_SetEnable(0U);
    Servo_SetMode(SERVO_MODE_SPEED);
    apply_spd_gains();          /* 进入时套用当前档位（默认 1.00x = 模型值） */
    Servo_SetSpeedMode(1U, w);
    BspServo_PwmEnable(1U);

    if (BspServo_IsPwmEnabled() == 0U) {
        Servo_SetSpeedMode(0U, 0.0f);
        return;
    }

    s_armed   = 1U;
    s_deadman = CMD_DEADMAN_MS;
}

/* 四种运行态互斥：辨识(openloop) / 速度闭环 / 电流闭环 / 模型三环。
 * ISR 的分支优先级是 fault > openloop > 电流(含速度) > 模型，
 * 所以从"低优先级态"发命令切"高优先级态"时，必须先把高优先级态关掉，
 * 否则命令看起来生效了（回显 OK），实际 ISR 还在跑旧环路。 */

/* 位置环入口余速门禁（A13/F2，长测 2026-11-23）：
 * 带余速切位置环时，位置环 v_max(POS_VMAX_DFLT=25 rad/s≈239 rpm) 没能力在单次
 * 折算行程(≤π)内刹住，实测 800 rpm 余速切进来会单向多跑 4~6 圈并锁 0x0100 跟随误差，
 * 且不报警、不拒绝、直接满速跑。这里在入口拒绝（留 10% 余量 ≈215 rpm），
 * 与力矩模式的 CMD_CUR_WMAX_RADS 门禁同策略。用 |omega| 只关心幅值。 */
#define CMD_POS_WMAX_RADS   (25.0f)   /* 电机端口径 v_max = 输出端 POS_VMAX_DFLT(2.78 rad/s) × 9 */
static uint8_t pos_entry_blocked(void)
{
    const float w_now = fabsf(BspServo_GetOmegaMech());
    return (w_now > (0.9f * CMD_POS_WMAX_RADS)) ? 1U : 0U;
}

static void do_pos(uint8_t hold, float rad)
{

    if (BspServo_GetFault() != 0U) {
        return;
    }
    if (BspServo_IsCurrentCalibrated() == 0U) {
        return;
    }
    if (BspServo_IsEncoderZeroed() == 0U) {
        return;
    }
    if (ident_busy() != 0U) {
        return;
    }

    /* 已在位置环内：只改目标，不重开 MOE —— MOE 通断会带来一次冲击 */
    if ((s_armed != 0U) && (Servo_IsPosMode() != 0U)) {
        if (hold != 0U) { Servo_SetPosHold(); }
        else            { Servo_SetPosTarget(rad); }
        s_deadman = CMD_DEADMAN_MS;

        {
            float pr = 0.0f, pe = 0.0f, vf = 0.0f, pt = 0.0f, vm = 0.0f;
            uint8_t st = 0U;

            Servo_GetPosState(&pr, &pe, &vf, &pt, &vm, &st);
            /* 位置用 4 位小数：3 位只有 1 mrad 量化，而位置环的重复性是 30 urad 级，
             * 用 1 mrad 的读数当基准会把"定位偏差"整体算错 0.5 mrad。 */
        }
        return;
    }

    if (s_armed != 0U) {
        /* 从速度/力矩模式切过来：必须先停原来那条链，否则 ISR 里
         * s_spd_en 优先，位置环看着"已使能"但完全不生效。 */
        do_disable("switch to POS");
    }

    /* A13/F2：带余速起位置环会多跑数圈并锁 0x0100。拒绝（电机保持滑行），
     * 不置 s_armed，等用户停稳再切。已在环内改目标（上面的 early-return 分支）不受影响。 */
    if (pos_entry_blocked() != 0U) {
        return;
    }

    /* 进入顺序与 do_speed 一致：关模型三环 → 切模式 → 套增益 →
     * 起位置环（内部起电流环、清两套积分）→ 最后放 MOE。 */
    Servo_SetEnable(0U);
    Servo_SetMode(SERVO_MODE_POS_LOOP);
    apply_spd_gains();
    /* 起环一律"保持当前位置"，目标在 MOE 打开之后再给。
     * 顺序不能反：位置环在 ISR 里已经跑起来了，若在 PwmEnable 之前设目标，
     * 规划位置会先跑出去而电机还没通电，规划与实际一拉开就是一次跟随误差。
     * 2026-11-22 起 Servo_SetPosMode() 不再接收外部目标（其参数已废弃）。 */
    Servo_SetPosMode(1U, Servo_GetPosCoordNow());
    BspServo_PwmEnable(1U);

    if (BspServo_IsPwmEnabled() == 0U) {
        Servo_SetPosMode(0U, 0.0f);
        return;
    }

    s_armed   = 1U;
    s_deadman = CMD_DEADMAN_MS;

    if (hold == 0U) {
        /* 定位到上位机给的角度。Servo_SetPosTarget 会把它按"当前圈的 0~2π"
         * 折算：输入 0 就是回到本圈 0 点，与之前累计转了多少圈无关。 */
        Servo_SetPosTarget(rad);
    }

    {
        float pr = 0.0f, pe = 0.0f, vf = 0.0f, pt = 0.0f, vm = 0.0f;
        uint8_t st = 0U;

        Servo_GetPosState(&pr, &pe, &vf, &pt, &vm, &st);
    }
}

static uint8_t do_enable(void)
{
    if (s_armed != 0U) {
        s_deadman = CMD_DEADMAN_MS;
        return 1U;                 /* 已使能视为成功（幂等） */
    }

    if (BspServo_GetFault() != 0U) {
        return 0U;
    }
    if (BspServo_IsCurrentCalibrated() == 0U) {
        /* 联锁：没标定就不许通电，否则电流环把零漂当真实电流去积分 */
        return 0U;
    }
    if (BspServo_IsEncoderZeroed() == 0U) {
        /* 2026-11-17 补：原来 E 只查 C 不查 Z。位置环要出力矩就得靠电角度，
         * 零点没标时"正 iq"可能出反向力矩 —— 一使能就往一边顶。
         * 速度环（do_speed）和位置定位（do_pos）早就有这道，E 是漏的。 */
        return 0U;
    }

    /*
     * 2026-11-17：三环默认改走**手写链路**。
     *
     * 原来这里把模型三环的 en 置 1、把 pos_ref 贴到当前角，由 three_loop_deploy
     * 做位置保持。现在改成：
     *     手写位置环（规划 + P + 前馈）→ 手写速度环 → 手写电流环
     * 模型 en 不再置 1，模型分支因此永不参与（见 servo_isr.c 的 en 判断）。
     *
     * 为什么必须做这一步：模型三环的参数还是 11-15 的整定值（L=113.5 µH、
     * psi=7.593 mWb），与 11-17 实测校正后的手写链已漂移 33%/75%；
     * 同一台机器跑两套没对齐的控制器，位置模式的行为和速度模式对不上。
     * 详见 架构评审_现状与缺口.md 与 电流环参数对比_模型vs手写.md。
     */
    if (ident_busy() != 0U) {
        /* 辨识正在跑时必须拒绝（2026-11-21 修）。
         *
         * 为什么上面几道拦不住：辨识期间 MOE 是由辨识模块自己
         * BspServo_PwmEnable(1) 打开的，而 s_armed 仍是 0，所以
         * "already ON" 那道判断不会命中。
         *
         * 放行的后果是两边都以为自己有权控制：ISR 的分支优先级是
         * s_pos_en > s_ol_en，位置环一使能，辨识的给定就被整段忽略，
         * 最终超时或算出荒谬值；而且辨识收尾还会 PwmEnable(0)，
         * 把刚打开的 MOE 关掉，留下 s_armed=1 / MOE=0 的不一致状态
         * ——上位机看到"已使能"但电机不动。
         *
         * 正确做法：等它跑完，或发 halt（do_disable 会 Abort 全部辨识）。 */
        return 0U;
    }

    /* A13/F2：带余速使能（起位置环）会多跑数圈并锁 0x0100。拒绝，等停稳再使能。 */
    if (pos_entry_blocked() != 0U) {
        return 0U;
    }

    Servo_SetEnable(0U);                 /* 模型三环保持关闭 */
    Servo_SetMode(SERVO_MODE_POS_LOOP);
    apply_spd_gains();                   /* 位置环复用速度环，套当前档位 */
    /* 目标 = 当前实测（误差≈0）。
     * 必须用 Servo_GetPosCoordNow()，不能用 BspServo_GetThetaMulti()：
     * 位置环坐标带 s_pos_base 偏置（每次进位置模式对齐到单圈角），
     * 传裸多圈角会让 Servo_SetPosMode 内部的平移多减一个 old_base，
     * 第二次起环的目标就会偏出整圈（2026-11-21 实测偏 -2π）。 */
    Servo_SetPosMode(1U, Servo_GetPosCoordNow());
    BspServo_PwmEnable(1U);              /* 再放开主输出 */

    /* PwmEnable 是 void：故障位非 0 时它会静默拒绝置 MOE，必须回读确认 */
    if (BspServo_IsPwmEnabled() == 0U) {
        Servo_SetPosMode(0U, 0.0f);
        Servo_SetEnable(0U);
        s_armed = 0U;
        return 0U;
    }

    s_armed   = 1U;
    s_deadman = CMD_DEADMAN_MS;

    return 1U;
}

static void do_cal(void)
{
    if (s_armed != 0U) {
        return;
    }
    if (BspServo_StartCurrentCal() == 0U) {
        return;
    }
}

static void do_align(void)
{
    MotorIdentErr e;

    if (s_armed != 0U) {
        return;
    }

    e = MotorIdent_Start(Servo_GetAlignId());
    if (e != MOTOR_IDENT_OK) {
        /* Z 以前也是"只 emit ASCII"：ASCII 关闭后，前置不满足的表现是
         * "action(5) 回 status=0，然后 50 s 一条消息都没有"。
         * 这里补一帧失败的 kind=8，上位机立刻能拿到错误码。 */
        push_reject(MAV_ID_ALIGN, (uint8_t)e);
        return;
    }
}

/* TIM6 1 ms：驱动辨识状态机，结束时打印一次结果 */
static void ident_task1ms(void)
{
    const MotorIdentResult *r;

    if (MotorIdent_Task1ms() == 0U) { return; }

    r = MotorIdent_GetResult();

    if (MotorIdent_GetState() == MOTOR_IDENT_DONE) {
        {
            float v[8];
            /* v[0] 是"极对数"，必须是整数：拖拽法算出的 ratio 是带小数的
             * 实测比值（典型 13.92），它只用于判可信度，不该显示成人看的极对数。
             * 小数部分已经在 v[7]=resid 里，上游想评估测量质量看那一栏。 */
            v[0] = (float)r->ratio_int;      v[1] = (float)r->dir;
            v[2] = r->offset_rad * CMD_RAD2DEG;
            v[3] = r->zero_mech * CMD_RAD2DEG;
            v[4] = (float)r->n_step;         v[5] = (float)r->n_sweep;
            v[6] = (float)r->n_fail;         v[7] = r->resid;
            Telemetry_PushIdentResult(MAV_ID_ALIGN, 1u, r->n_step, v, r->resid);
        }
    } else {
        /* 2026-11-20：原来只打 "ratio_bad" 三个字，看不出失败的具体形态 ——
         * 是"每步几乎没动"（dmech 近 0）、"动了一半"（dmech≈4.3°，ratio≈28）、
         * 还是"动过头/打滑"（min/max 离散大）。只能靠反复推测，代价太高。
         *
         * 判据换算：正常 120° 电角步进对应机械角
         *   dmech = (2π/3)/14 = 0.1496 rad = 8.571°
         * 所以 mdeg 三位数（8571）是正确的，几十表示基本没动，
         * 4000 附近表示只走到一半，min/max 相差数倍表示打滑。
         * ratio 直接取整打印（put_f3 上限 ±1000，ratio 可能上万会打 "nan"）。 */
        /* 失败也必须推 30092（2026-11-21）。原来失败只走 emit()，ASCII 关闭后
         * 整个失败路径完全不可见：上位机只能等到超时，报一句"Z 失败"，
         * 却不知道是"没动""动一半"还是"打滑"。这正是迁移里最该避免的形态。
         * 约定：ok=0，v[0]=错误码（同 MotorIdentErr），其余字段照常给实测值，
         * 上位机据此直接打出与 ASCII 版一致的诊断。 */
        {
            float v[8];
            v[0] = (float)r->err;
            v[1] = r->dmech_avg * CMD_RAD2DEG * 1000.0f;   /* mdeg */
            v[2] = r->dmech_min * CMD_RAD2DEG * 1000.0f;
            v[3] = r->dmech_max * CMD_RAD2DEG * 1000.0f;
            v[4] = r->ratio;
            v[5] = (float)r->n_step;
            v[6] = (float)r->n_fail;
            v[7] = r->resid;
            Telemetry_PushIdentResult(MAV_ID_ALIGN, 0u, r->n_step, v, r->resid);
        }
    }
}

/* 四种机械辨识的统一入口。
 * 为什么不用 `_<n>` 数值命令：实机发现引导字符与数字在同一批 Poll 里时，
 * 偶尔会出现数字丢失（命令走了查询分支），表现为"发了 _1 却没启动"。
 * 单字符不可能丢（一个字节就是一条命令），所以改用 4 个专用字符。 */
static void flux_task1ms(void)
{
    static FluxIdentState s_prev = FI_IDLE;
    const FluxIdentState   st = FluxIdent_GetState();
    const FluxIdentResult *r;
    float v[8];

    /* 只在 DONE / FAIL 那一拍动作一次（ASCII 逐行补发已删除）。
     *
     * ★ 顺序不能反：必须**先**判断"状态有没有变"，变了就立刻写 s_prev，
     *   然后再判断是不是终结态。写成
     *       if ((st == s_prev) || (不是终结态)) { return; }
     *   s_prev 只会在终结态才被更新，于是**第二次辨识结束时 st 与
     *   s_prev 都是 DONE → 直接 return，结果帧永远不推**（2026-11-21 踩过：
     *   表现为第一次辨识正常、之后每次"卡住"没有 30092）。 */
    if (st == s_prev) { return; }
    s_prev = st;
    if ((st != FI_DONE) && (st != FI_FAIL)) { return; }

    r = FluxIdent_GetResult();

    if (st == FI_FAIL) {
        /* 失败帧：v[0]=错误码，v[1]=已采点数，v[2..4]=已算出的 psi/R/L，
         * v[5]=组间偏差%，v[6..7]=失败瞬间的实测转速与给定。
         * 判读：v[1]=0  → 第一点就没数据（多为速度环起不来 / 使能被拒）；
         *       v[1]>=1 → 副组才失败，看 v[6] 是否接近 v[7]。 */
        v[0] = (float)r->err;
        v[1] = (float)r->n_done;
        v[2] = r->psi_wb * 1000.0f;   /* psi_mWb */
        v[3] = r->r_ohm * 1000.0f;    /* R_mOhm */
        v[4] = r->l_h * 1.0e6f;       /* L_uH */
        v[5] = r->psi_dev_pct;
        v[6] = r->dbg_om;
        v[7] = r->dbg_wref;
        Telemetry_PushIdentResult(MAV_ID_FLUX, 0u, r->n_done, v, r->psi_dev_pct);
        return;
    }

    /* 成功：psi 立即接管 Kt 与 BEMF 前馈（不再等重编译）。
     * 必须带上"辨识这一刻的电机温度"：psi 是有温度的量，热机辨识出的
     * 3.89 mWb 若被当成 20 degC 的基准，冷机时会被二次补偿压到 3.85（-8%）。 */
    Servo_SetIdentPsiEx(r->psi_wb, BspServo_GetTempMotor());

    v[0] = r->psi_wb * 1000.0f;       /* psi_mWb */
    v[1] = r->kt * 1000.0f;           /* kt_mNm_per_A */
    v[2] = BspServo_GetTempMotor();   /* Tref_C */
    v[3] = r->r_ohm * 1000.0f;        /* R_mOhm */
    v[4] = r->l_h * 1.0e6f;           /* L_uH */
    v[5] = r->ke_v_s_rad;
    v[6] = r->psi_dev_pct;
    v[7] = 0.0f;
    Telemetry_PushIdentResult(MAV_ID_FLUX, 1u, r->n_pts, v, r->psi_dev_pct);
}

/* 机械辨识结果：DONE / FAIL 那一拍推一次 30092。
 *
 * 原来要等 ASCII 逐行发完（s_mi_phase 走完 30 行）才推，删掉逐行输出后
 * 提前到跃迁拍 —— 顺带修掉一个隐患：以前某行 DMA 发不出去时，
 * 结果帧会一直不推，上位机等到超时。 */
static void mech_task1ms(void)
{
    static MechIdentState s_prev = MI2_IDLE;
    const MechIdentState  st = MechIdent_GetState();
    const MechIdentResult *r;
    float   v[8];
    uint8_t kk;

    /* 同 flux_task1ms：先跟踪状态变化，再判终结态。
     * 否则第二次之后 s_prev 停在 DONE，永远不再推结果帧。 */
    if (st == s_prev) { return; }
    s_prev = st;
    if ((st != MI2_DONE) && (st != MI2_FAIL)) { return; }

    r = MechIdent_GetResult();

    if (st == MI2_FAIL) {
        /* mode 在 MechIdent_Start 里、所有前置检查通过之后才赋值，
         * 所以 FAIL 时它一定是 1..4，可据此选对 kind —— 否则上位机
         * 只知道"机械辨识失败"，分不清是扫频/摩擦/惯量/齿槽。 */
        switch (r->mode) {
        case 1U:  kk = MAV_ID_SWEEP;      break;
        case 2U:  kk = MAV_ID_FRIC_CURVE; break;
        case 4U:  kk = MAV_ID_COG;        break;
        default:  kk = MAV_ID_INERTIA;    break;
        }
        v[0] = (float)r->err;
        v[1] = (float)r->mode;
        v[2] = (float)r->cog_n;
        v[3] = (float)r->fric_n;
        v[4] = (float)r->sweep_n;
        v[5] = r->j_eq * 1.0e4f;
        v[6] = r->visc;
        v[7] = 0.0f;
        Telemetry_PushIdentResult(kk, 0u, 0u, v, 0.0f);
        return;
    }

    switch (r->mode) {
    case 1U:        /* 扫频：wmean=0 说明差分通道坏了，否则是"真的没响应" */
        v[0] = r->dbg_wmean; v[1] = r->dbg_wamp; v[2] = r->dbg_iamp;
        v[3] = 0.0f; v[4] = 0.0f; v[5] = 0.0f; v[6] = 0.0f; v[7] = 0.0f;
        Telemetry_PushIdentResult(MAV_ID_SWEEP, 1u, r->sweep_n, v, 0.0f);
        break;

    case 2U:        /* 摩擦曲线（诊断用，不是标定入口） */
        v[0] = r->fric_a; v[1] = r->visc; v[2] = (float)r->fric_n;
        /* v[3]/v[4]：静摩擦 iq 与静/动摩擦比。标定参数请用 $（action 9）：
         * 三点正反、重复性 0.3%；这里单向 8 点重复性只有 ±12%~21%。 */
        v[3] = r->fric_iq_lo; v[4] = r->fric_ratio;
        v[5] = 0.0f; v[6] = 0.0f; v[7] = 0.0f;
        Telemetry_PushIdentResult(MAV_ID_FRIC_CURVE, 1u, r->fric_n, v, 0.0f);
        break;

    case 3U:        /* 惯量 */
        v[0] = r->j_eq * 1.0e4f;   /* J x 1e4，避免小数量级 */
        v[1] = r->acc1;  v[2] = r->acc2;
        v[3] = r->tq1;   v[4] = r->tq2;
        v[5] = 0.0f; v[6] = 0.0f; v[7] = 0.0f;
        Telemetry_PushIdentResult(MAV_ID_INERTIA, 1u, 0u, v, 0.0f);
        break;

    case 4U:        /* 齿槽：fwd 与 rev 差太多说明两向不对称，此时"平均"本身就可疑 */
        v[0] = r->cog_pp; v[1] = r->cog_pp_fwd; v[2] = r->cog_pp_rev;
        v[3] = r->cog_dc; v[4] = 0.0f; v[5] = 0.0f; v[6] = 0.0f; v[7] = 0.0f;
        Telemetry_PushIdentResult(MAV_ID_COG, 1u, r->cog_n, v, 0.0f);

        /* 测完顺手装进前馈缓存（不写 Flash，要落盘用 A）。 */
        (void)CogFf_LoadFromIdent();
        break;

    default:
        break;
    }
}

/* $ 的结果打印：摩擦标定结束那一刻打一次 */
static void fric_task1ms(void)
{
    static FricCalState s_prev = FC_IDLE;
    FricCalState st = FricCal_GetState();
    const FricCalResult *r;

    if ((st == s_prev) || ((st != FC_DONE) && (st != FC_FAIL))) {
        s_prev = st;
        return;
    }
    s_prev = st;

    r = FricCal_GetResult();
    {
        if (r->err == FC_OK) {
            {
                float v[8];
                v[0] = r->iq1;   v[1] = r->iq2;
                v[2] = r->visc;  v[3] = r->tc_nm;
                v[4] = r->rpm1;  v[5] = r->rpm2;
                v[6] = 0.0f;     v[7] = 0.0f;
                Telemetry_PushIdentResult(MAV_ID_FRIC_2PT, 1u, 0u, v, 0.0f);
            }
        } else {
            /* 失败也推 30092（2026-11-21 补）。
             *
             * 原来只有成功分支推送，失败时上位机只能等到超时，既不知道是哪一类
             * 失败，也看不到 iq1/iq2 —— 而 SPREAD 的判据恰恰是"iq2 与 iq1 的关系"，
             * 没有这两个数就只能猜。Z 那次已经踩过同样的坑（失败静默）。
             * 约定与 kind=8 一致：ok=0 时 v[0] = 错误码。
             */
            {
                float v[8];

                v[0] = (float)r->err;
                v[1] = r->iq1;
                v[2] = r->iq2;
                v[3] = r->rpm1;
                v[4] = r->rpm2;
                v[5] = r->visc;
                v[6] = 0.0f;
                v[7] = 0.0f;
                Telemetry_PushIdentResult(MAV_ID_FRIC_2PT, 0u, 0u, v, 0.0f);
            }
        }
    }
}

/* ^ 的结果打印：编码器自校准结束那一刻打一次 */
static void enc_cal_task1ms(void)
{
    static EncCalState s_prev = EC_IDLE;
    EncCalState st = EncCal_GetState();
    const EncCalResult *r;

    if ((st == s_prev) || ((st != EC_DONE) && (st != EC_FAIL))) {
        s_prev = st;
        return;
    }
    s_prev = st;

    r = EncCal_GetResult();
    {
        if (r->err == EC_OK) {
            {
                float v[8];
                v[0] = (float)r->turns_x10 * 0.1f;   /* 累计圈数 */
                v[1] = r->rpm_avg;
                v[2] = r->rpm_min;
                v[3] = r->rpm_max;
                v[4] = (float)r->chip_status;
                v[5] = (float)r->elapsed_ms;
                v[6] = (float)r->ol_mode;
                v[7] = r->target_rpm;
                Telemetry_PushIdentResult(MAV_ID_ENC_CAL, 1u,
                                          (uint16_t)(r->turns_x10 / 10u), v, 0.0f);
            }
        } else {
        }
    }
}

/* @ 的结果打印：状态机结束那一拍打一次 */
#define CMD_RL_TEST_A   2.0f

static void rl_ident_task1ms(void)
{
    static RlIdentState s_prev = RL_IDENT_IDLE;
    RlIdentState st = RlIdent_GetState();
    const RlIdentResult *r;

    if ((st == s_prev) ||
        ((st != RL_IDENT_DONE) && (st != RL_IDENT_FAIL))) {
        s_prev = st;
        return;
    }
    s_prev = st;

    r = RlIdent_GetResult();
    {
        if (r->err == RL_IDENT_OK) {
            /* R/L 由注入法单独测得，写入解耦与 R 记录 */
            Servo_SetIdentRl(r->r_ohm, r->l_h);
            {
                float v[8];
                v[0] = r->r_ohm * 1000.0f;   /* R_mOhm */
                v[1] = r->l_h * 1.0e6f;      /* L_uH */
                v[2] = r->z_ohm;  v[3] = r->i_amp;
                v[4] = r->v_avg;  v[5] = r->i_avg;
                v[6] = r->inj_amp; v[7] = 0.0f;
                Telemetry_PushIdentResult(MAV_ID_RL, 1u, r->l_points, v, 0.0f);
            }
        } else {
            /* 失败也上报（2026-11-21 补）。原来是**只 emit ASCII**，
             * 而 X72_ASCII_DEBUG=0 → 失败完全静默，上位机只能报"超时/失败"，
             * 拿不到错误码，也不知道卡在 R 段还是注入段。
             * rl_fail() 不清 s_res 的测量字段，所以这里 v[1..7] 是**真实现场**：
             *   · v[1]=0（R 还是 0）      → 连 R 段都没跑，多半 ERR_CURRENT/PWM
             *   · v[1] 有值但 v[2]=0       → R 段过了，注入段失败（看 v[6]=i_avg）
             *   · v[4]=0（iamp，注入电流） → 电压注了但没电流，查功率级/接线
             * 失败帧语义：v[0] 换成错误码（与 Z / $ 的约定一致，接收侧靠 ok 区分）。 */
            float v[8];
            v[0] = (float)r->err;
            v[1] = r->r_ohm * 1000.0f;   /* R_mOhm */
            v[2] = r->l_h * 1.0e6f;      /* L_uH */
            v[3] = r->z_ohm;
            v[4] = r->i_amp;             /* 注入产生的电流幅值 */
            v[5] = r->v_avg;
            v[6] = r->i_avg;
            v[7] = r->inj_amp;
            Telemetry_PushIdentResult(MAV_ID_RL, 0u, r->l_points, v, 0.0f);

        }
    }
}

void ServoCmd_Init(void)
{
    uint16_t i;          /* 必须是 16 位：CMD_Q_LEN 已涨到 256，
                          * uint8_t 的 "i < 256" 恒成立 → 死循环 */

    for (i = 0U; i < CMD_Q_LEN; i++) { s_q[i] = 0U; }
    s_qh = 0U;
    s_qt = 0U;
    s_q_ovf = 0U;
    s_kill = 0U;
    s_deadman = 0U;
    s_armed = 0U;

    /*
     * 直接开 RXNE 中断，**不调用 HAL_UART_Receive_IT**：
     * HAL 的接收会把 huart1.gState 置为 BUSY_RX，而 100 Hz 遥测用同一个 huart1
     * 做 DMA 发送，两者互相把对方的 gState 检查判死（发送侧会一直 drop）。
     * 所以接收走裸寄存器，发送继续走 HAL DMA，互不干扰。
     */
    /*
     * 2026-11-20：这里**还要自己使能 NVIC**，不能只依赖 HAL_UART_MspInit()。
     *
     * 实测故障：ASCII 命令全哑、MAVLink 也收不到，但遥测正常。
     * 用 30040 的 elec_angle 做诊断字段读回：
     *     CR1 = 0x2D（UE/RE/TE/RXNEIE 全在）、gState=READY，
     *     但 NVIC->ISER[1] 的 bit5(IRQ37) = 0，且发字节后 ISR 的 ORE=1
     *     —— 字节到了 RDR 却没人取，溢出丢弃，qh 一直是 0。
     * 也就是说：外设侧全部就绪，唯独 NVIC 里中断没开。
     * MspInit 里确实有 HAL_NVIC_EnableIRQ(USART1_IRQn)，但它在
     * "huart1.gState != HAL_UART_STATE_RESET 就跳过 MspInit" 的路径下不会执行。
     * 在命令层再使能一次是幂等的，而且把"能不能收命令"这件事
     * 从 HAL 的初始化时机里解耦出来。
     */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

    /* 清一次可能残留的 ORE/FE/NE/PE：按 RM0440 的要求"读 ISR 再读 RDR"。
     * 不清的话 RXNE 会被 ORE 抑制，即使中断开了也收不到后续字节。 */
    {
        volatile uint32_t isr_tmp = USART1->ISR;
        volatile uint32_t rdr_tmp = USART1->RDR;
        (void)isr_tmp;
        (void)rdr_tmp;
    }
}

void ServoCmd_Feed(uint8_t b)
{
    uint8_t n = (uint8_t)((s_qh + 1U) % CMD_Q_LEN);

    s_rx_total++;                    /* 诊断计数：进没进中断，看它是否单调增 */

    if (n != s_qt) {                 /* 满了就丢新字节，绝不覆盖未处理的 */
        s_q[s_qh] = b;
        s_qh = n;
    } else if (s_q_ovf < 0xFFFFU) {
        /* 2026-11-21：溢出必须留痕。原来只是静默丢弃 —— 一帧 156 B 的请求
         * 被吃掉中间几个字节，表现成"CRC 错为 0、pending 顶替为 0、就是没有
         * 应答"，只能靠猜。计数在 30082 的只读槽 q_ovf。 */
        s_q_ovf++;
    }
}

/* 一行一发送：DMA 忙就保持 pending，下一毫秒再试 */
static void trace_pump(void);          /* 定义在下面：逐行送结果与波形 */

void ServoCmd_Task1ms(void)
{
    /* ---- ASCII 输出自愈（2026-11-17）----
     * pending 超过 50 ms 说明发送侧卡住了（DMA 状态机异常、行超长被拒等）。
     * 必须强制放行：否则后续每一条回显都在 emit() 里被丢弃，而主循环完全
     * 正常（看门狗不会复位），现象就是"遥测照发、命令全无响应、连 ! 都不理"。
     * 这个形态今天踩到三次，每次只能烧录恢复。 */
    /* ---- RX 自愈 ----
     * RXNEIE 被意外清掉时接收会永久停摆而 TX 一切正常，重新使能是幂等的。
     *
     * ⚠ 这里**不要**顺手清 ORE（2026-11-21 试过，是错的）：
     *   清 ORE 必须读 RDR，而 RDR 里的字节属于中断。在 1 ms 任务里读一次，
     *   就会把 ISR 还没取走的字节抢掉 —— 帧被截断、CRC 全错，表现是
     *   "命令时通时不通"，比原来的问题更难查。
     *   ORE 的清理放在 USART1_IRQHandler 里做（读 ISR 快照 → 读 RDR），
     *   那里本来就要取字节，不会丢。 */
    if ((USART1->CR1 & USART_CR1_RXNEIE) == 0U) {
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    }

    FricCal_Task1ms();          /* 摩擦标定状态机（只在 $ 期间活动） */
    fric_task1ms();             /* 结果打印 */
    MechIdent_Task1ms();        /* 机械辨识状态机（只在 ()<>} 期间活动） */
    mech_task1ms();             /* 机械辨识结果逐行输出 */
    FluxIdent_Task1ms();        /* ψ/R/L 联辨识状态机（只在 + 期间活动） */
    flux_task1ms();             /* 其结果逐行输出 */

    ident_task1ms();            /* 极对数/零点辨识（只在 Z 期间活动） */
    enc_cal_task1ms();          /* 编码器自校准结果打印 */
    rl_ident_task1ms();         /* R/L 辨识结果打印（只在 @ 结束后一次） */

    if (s_armed == 0U) { return; }
    if (s_deadman == 0U) { s_kill = 1U; return; }
    s_deadman--;
    if (s_deadman == 0U) { s_kill = 1U; }
}

/* MIT 装配核心（ServoCmd_CtrlMit 与 30065 共用）。
 * 返回 1=在环（首次装配成功或本就是 MIT 只更新给定），0=联锁拒绝。
 * 拒绝原因写 s_mit_why，供 30091 诊断读回。
 * 注意：调用方负责确定给定来源与是否自动封锁切环；这里只管 MIT 环自身装配。 */
static uint8_t mit_engage(float p, float v, float kp, float kd, float t)
{
    if (Servo_IsMit() != 0U) {
        /* 防御：MIT 是电流环子模式，正常由 do_disable → SetCurrentMode(0)
         * 连带清除。若出现 en=1/armed=0 的残留，只更新给定会绕过联锁。 */
        if (s_armed == 0U) {
            Servo_SetMitEn(0U);
            Servo_SetCurrentMode(0U, 0.0f);
            s_mit_bad++;
            s_mit_why = 6U;
            return 0U;
        }
        /* 已在 MIT 环：只更新给定，不重开 MOE（每帧续命令超时） */
        Servo_SetMitCmd(p, v, kp, kd, t);
        s_deadman = CMD_DEADMAN_MS;
        return 1U;
    }

    /* 第一帧负责把环路拉起来，并把 p_des 先写好 —— 顺序必须是
     * "先给给定、再使能"，否则使能那一拍会用上一帧（或 0）的给定。 */
    if (BspServo_GetFault() != 0U)            { s_mit_bad++; s_mit_why = 1U; return 0U; }
    if (BspServo_IsCurrentCalibrated() == 0U) { s_mit_bad++; s_mit_why = 2U; return 0U; }
    if (BspServo_IsEncoderZeroed() == 0U)     { s_mit_bad++; s_mit_why = 3U; return 0U; }
    if (ident_busy() != 0U)                   { s_mit_bad++; s_mit_why = 4U; return 0U; }

    /* A14/F3（长测 2026-11-23）：带高速余速切 MIT 必锁 0x0020 且模式回 NONE。
     * MIT 无速度环、无轨迹规划，ep 被限到 ±1 rad 后满力矩输出，超速门限
     * SERVO_CUR_WMAX(300 rpm) 会立即触发。与 do_current 同一门限（留 10% 余量）。 */
    {
        const float w_now = fabsf(BspServo_GetOmegaMech());
        if (w_now > (0.9f * CMD_CUR_WMAX_RADS)) { s_mit_bad++; s_mit_why = 7U; return 0U; }
    }

    Servo_SetEnable(0U);              /* 关模型三环 */
    Servo_SetMode(SERVO_MODE_POSITION);
    Servo_SetCurrentMode(1U, 0.0f);   /* 起电流环（会清积分） */
    Servo_SetMitCmd(p, v, kp, kd, t);
    Servo_SetMitEn(1U);
    BspServo_PwmEnable(1U);

    /* 与 do_current/do_speed/do_pos 一致：PwmEnable 在故障位非 0 时静默
     * 拒绝置 MOE，必须回读确认。否则返回 active=12 但 MOE=0，上位机误判
     * "MIT 已使能"，实际桥臂一直封锁。 */
    if (BspServo_IsPwmEnabled() == 0U) {
        Servo_SetMitEn(0U);
        Servo_SetCurrentMode(0U, 0.0f);
        s_armed = 0U;
        s_mit_bad++;
        s_mit_why = 6U;
        return 0U;
    }

    s_armed   = 1U;
    s_deadman = CMD_DEADMAN_MS;
    return 1U;
}

void ServoCmd_Poll(void)
{
    /* 0) 首次进入时对齐速度环档位（Flash 恢复的 kp 可能不是 1x） */
    sync_spd_gain_index();

    /* 1) deadman 触发：封锁输出 */
    if (s_kill != 0U) {
        s_kill = 0U;
        do_disable("deadman 2s");
        return;
    }

    /* 2) 故障封锁收尾（2026-11-21 补）
     *
     * ISR 在故障时只关 MOE（servo_isr.c 的给定来源分支），并不清 s_armed
     * 与 s_src。实际后果（实测）：超速置位后 30090 读回 en=1 / cw=0x01 /
     * fault=0x0020，iq 恒 0、电机自由滑行 —— 界面说"使能中"，命令也返回成功。
     * 更麻烦的是清故障后再发控制字，do_enable 会走 "already ON" 幂等分支
     * 直接 return，MOE 再也不会被重新放开，只能先显式 disarm 一次。
     *
     * 这里收尾一次，把 armed / 控制字 / 各辨识状态一起清干净。
     * 只在"已 armed 且有故障"时触发，所以正常路径零开销。 */
    if ((s_armed != 0U) && (BspServo_GetFault() != 0U)) {
        do_disable("fault");
        return;
    }

    /* 3) 取字节喂 MAVLink。
     *
     * 2026-11-21：ASCII 单字符、0x02 MIT 帧、0x04 PCMP 帧三条通道已全部删除，
     * USART1 现在**只跑 MAVLink v2**。帧头识别、CRC、丢字节全部由
     * MavHost_FeedByte() 自己处理；不属于帧的字节（噪声、残留引导码）由它丢弃。
     *
     * 为什么保留这个环形队列而不在中断里直接调 MavHost_FeedByte：
     * 解析要动 CRC 与 156 B 的缓冲，放进 20 kHz 同优先级会超时。队列把
     * "收"与"解析"解耦，1 ms 窗口（256 B ≈ 2.8 ms）足够吸收突发。 */
    while (s_qt != s_qh) {
        const uint8_t c = s_q[s_qt];
        s_qt = (uint8_t)((s_qt + 1U) % CMD_Q_LEN);
        (void)MavHost_FeedByte(c);
    }
}
uint8_t ServoCmd_IsArmed(void) { return s_armed; }

uint16_t ServoCmd_GetRxOverflow(void) { return s_q_ovf; }


/* ---- 供 MAVLink 协议层调用的受控入口（复用 E/D/X 的联锁，见 servo_cmd.h）---- */
uint8_t ServoCmd_Arm(void)
{
    return do_enable();
}

uint8_t ServoCmd_Disarm(void)
{
    do_disable("mavlink");
    return 1U;                 /* D 无拒绝路径 */
}

uint8_t ServoCmd_ClearFault(void)
{
    if (BspServo_IsPwmEnabled() != 0U) { return 0U; }   /* X 联锁：须先 D */
    BspServo_ClearFault();
    BspServo_ClearOvcInfo();
    PosLoop_ClearFault();
    /* 位置环被重置成"停在当前位置"，去重锁存必须一并复位 ——
     * 否则随后重发同一个目标会被静默吞掉（见 Servo_ResetPosCmdLatch 的说明）。 */
    Servo_ResetPosCmdLatch();
    return 1U;
}

/* ---- 4 控制模式给定入口（MAVLink → app_param → 这里）----
 * 全部复用 do_* 的装配联锁与顺序，不另起一套。互斥切换策略见 servo_cmd.h。
 * 公共前置：若已使能且不在目标环，先 do_disable 自动封锁（用户选定策略），
 * 之后 do_* 会在 s_armed==0 分支走完整装配；若已在目标环则 do_* 自己走"只更新给定"。 */

/* 切环前置：返回 1 可继续（未使能或已在目标环），0 = 装配失败（联锁拒绝） */
static uint8_t prepare_switch(uint8_t target_ok)
{
    if (s_armed == 0U) { return 1U; }        /* 未使能：do_* 走完整装配 */
    if (target_ok != 0U) { return 1U; }      /* 已在目标环：do_* 走"只更新给定" */
    do_disable("mav mode switch");           /* 自动先封锁，再装配新环 */
    return 1U;
}

uint8_t ServoCmd_CtrlTorque(float iq_a)
{
    /* 力矩环 = 电流环在、且 MIT/速度/位置三环都不在。
     * 不能只看 IsCurrentMode：速度环和位置环内部也会起电流环（它是共同的最后一级），
     * 只看它会导致"从速度切力矩"被误判成已在力矩环，do_current 随后被其速度联锁
     * 拒绝，结果是环没切成、却回 SUCCESS —— 这正是 MAVLink 切换实测踩到的坑。 */
    uint8_t in_ring = (Servo_IsCurrentMode() != 0U && Servo_IsMit() == 0U &&
                       Servo_IsSpeedMode() == 0U && Servo_IsPosMode() == 0U) ? 1U : 0U;
    (void)prepare_switch(in_ring);
    do_current(iq_a);                        /* 联锁全在这里；失败时 s_armed 保持 0 */
    return (s_armed != 0U && Servo_IsCurrentMode() != 0U &&
            Servo_IsMit() == 0U && Servo_IsSpeedMode() == 0U &&
            Servo_IsPosMode() == 0U) ? 1U : 0U;
}

uint8_t ServoCmd_CtrlSpeed(float w_rad_s)
{
    uint8_t in_ring = (Servo_IsSpeedMode() != 0U) ? 1U : 0U;
    (void)prepare_switch(in_ring);
    do_speed(w_rad_s / CMD_RAD_PER_RPM);     /* rad/s → rpm */
    return (s_armed != 0U && Servo_IsSpeedMode() != 0U) ? 1U : 0U;
}

uint8_t ServoCmd_CtrlPos(float p_rad)
{
    uint8_t in_ring = (Servo_IsPosMode() != 0U) ? 1U : 0U;
    (void)prepare_switch(in_ring);
    do_pos(0U, p_rad);
    return (s_armed != 0U && Servo_IsPosMode() != 0U) ? 1U : 0U;
}

uint8_t ServoCmd_CtrlMit(float p, float v, float kp, float kd, float tau)
{
    uint8_t in_ring = (Servo_IsMit() != 0U) ? 1U : 0U;
    if (s_armed != 0U && in_ring == 0U) { do_disable("mav mode switch"); }
    return mit_engage(p, v, kp, kd, tau);    /* MIT 环自身联锁在 mit_engage 内 */
}

int8_t ServoCmd_ActiveMode(void)
{
    if (s_armed == 0U) { return 0; }

    /* 直接用给定源真值 s_src 判定，不要用 Servo_IsXxxMode()。
     * 那些接口返回的是派生量，POS 会派生出 spd_en（位置环本身串了速度环），
     * 于是"先查 IsSpeedMode 再查 IsPosMode"会把 POS 报成 3(VEL) ——
     * 2026-11-20 实测确认过这个顺序陷阱。switch 只有一个真值来源，
     * 以后再加模式也不会被派生关系误导。 */
    switch (Servo_GetSrc()) {
    case SERVO_SRC_MIT:    return 12;
    case SERVO_SRC_POS:    return 1;
    case SERVO_SRC_SPEED:  return 3;
    case SERVO_SRC_TORQUE: return 4;
    default:               return 0;
    }
}

/* ---------------- 统一动作入口（飞控协议 30069 -> 现有实现）----------------
 *
 * 2026-11-21 从"注入 ASCII 字符"改为**直接调用**底层启动函数。
 *
 * 为什么必须改：原先的做法是把动作映射成等价字符喂进 ASCII 队列，靠解析器
 * 再走一遍。一旦 ASCII 接收路径被 X72_ASCII_DEBUG 关掉，注入的字符就没人解析，
 * 动作会静默失效 —— 而且是"帧发出去了、ACK 也回 SUCCESS、电机却不动"
 * 这种最难查的形态。直调之后，动作是否可用与 ASCII 开关无关。
 *
 * 联锁为什么放在这里统一做：ASCII 侧每条 case 里都有一句
 * "still ON (send D first)"，9 条路径各写一遍必然漏一条。这里写一次，
 * 覆盖全部校准/辨识动作。
 *
 * 回显不再由本函数产生：ASCII 关闭后没有回显通道，结果一律靠
 * 30090（状态字）/30092（辨识结果）读取。 */
/* 启动被拒时上报一次原因（2026-11-21）。
 *
 * 为什么需要：AppParam_ExecSysCmd 把 SC_EXEC_REJECT 统一映射成
 * APP_PARAM_OUT_OF_RANGE(2)，上位机只看到一个 2 —— 而 RlIdent_Start 有
 * 7 种前置失败（busy / 没做 C / 没做 Z / 有故障 / MOE 没关 / 电流起不来 /
 * 拟合失败），全压成同一个数，现场只能靠猜。这里把子模块的真实错误码
 * 推给 30092（ok=0, v[0]=err），与"运行期失败"共用同一套读取方式。
 *
 * 只在**没启动成功**时调用，所以不会和运行期的结果帧重复。 */
static void push_reject(uint8_t kind, uint8_t err)
{
    float   v[8];
    uint8_t k;

    for (k = 0u; k < 8u; k++) { v[k] = 0.0f; }
    v[0] = (float)err;
    Telemetry_PushIdentResult(kind, 0u, 0u, v, 0.0f);
}

/* 机械四项共用：mode 1..4 对应 kind 6/4/3/5，启动失败时带上真实错误码 */
static uint8_t mech_start(uint8_t mode, uint8_t kind)
{
    const MechIdentErr e = MechIdent_Start(mode);

    if (e != MI2_OK) {
        push_reject(kind, (uint8_t)e);
        return SC_EXEC_REJECT;
    }
    return SC_EXEC_ACCEPT;
}

uint8_t ServoCmd_ExecAction(uint8_t act)
{
    if ((act >= SC_ACT_CAL_CURRENT) && (act <= SC_ACT_MECH_COG) && (s_armed != 0U)) {
        return SC_EXEC_NEED_DISARM;      /* 仍在闭环里：先封锁再辨识 */
    }

    switch (act) {
    case SC_ACT_CAL_CURRENT:
        do_cal();
        return SC_EXEC_ACCEPT;
    case SC_ACT_ALIGN_ZERO:
        do_align();
        return SC_EXEC_ACCEPT;
    case SC_ACT_IDENT_FLUX: {
        const FluxIdentErr e = FluxIdent_Start();
        if (e != FI_OK) { push_reject(MAV_ID_FLUX, (uint8_t)e); return SC_EXEC_REJECT; }
        return SC_EXEC_ACCEPT;
    }
    case SC_ACT_IDENT_RL: {
        const RlIdentErr e = RlIdent_Start(CMD_RL_TEST_A);
        if (e != RL_IDENT_OK) { push_reject(MAV_ID_RL, (uint8_t)e); return SC_EXEC_REJECT; }
        return SC_EXEC_ACCEPT;
    }
    case SC_ACT_ENC_CAL: {
        const EncCalErr e = EncCal_Start(0.0f);
        if (e != EC_OK) { push_reject(MAV_ID_ENC_CAL, (uint8_t)e); return SC_EXEC_REJECT; }
        return SC_EXEC_ACCEPT;
    }
    case SC_ACT_FRIC_2PT: {
        const FricCalErr e = FricCal_Start();
        if (e != FC_OK) { push_reject(MAV_ID_FRIC_2PT, (uint8_t)e); return SC_EXEC_REJECT; }
        return SC_EXEC_ACCEPT;
    }
    case SC_ACT_MECH_SWEEP:   return mech_start(1U, MAV_ID_SWEEP);
    case SC_ACT_MECH_FRIC:    return mech_start(2U, MAV_ID_FRIC_CURVE);
    case SC_ACT_MECH_INERTIA: return mech_start(3U, MAV_ID_INERTIA);
    case SC_ACT_MECH_COG:     return mech_start(4U, MAV_ID_COG);
    case SC_ACT_ABORT_IDENT:
        /* 电角度对齐有显式 Abort；机械/编码器辨识没有对外 Abort 接口，
         * 靠封锁输出让它们各自在 1 ms 任务里检测到 MOE=0 后自行收尾。 */
        MotorIdent_Abort();
        do_disable("ident abort");
        return SC_EXEC_ACCEPT;
    case SC_ACT_CAN_SELFTEST:
        /* 自检会 Stop/DeInit/Init FDCAN 两次，期间总线不响应；
         * 必须在 disarmed 下做，否则刚好有 30065 流在跑会丢帧。
         * 返回值只表示"已执行"，通过与否看 30092 kind=10 的 ok 位。 */
        if (s_armed != 0U) { return SC_EXEC_NEED_DISARM; }
        (void)CanProto_SelfTest();
        return SC_EXEC_ACCEPT;
    case SC_ACT_CAN_TEST_TX:
    case SC_ACT_CAN_TEST_RX:
        if (s_armed != 0U) { return SC_EXEC_NEED_DISARM; }
        if (CanProto_TestStart((act == SC_ACT_CAN_TEST_TX) ? 1u : 2u) == 0u) {
            return SC_EXEC_REJECT;      /* FDCAN 没起来 */
        }
        return SC_EXEC_ACCEPT;
    case SC_ACT_CAN_TEST_STOP:
        CanProto_TestStop();
        return SC_EXEC_ACCEPT;
    default:
        return SC_EXEC_REJECT;
    }
}
