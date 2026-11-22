/*
 * actuator.c —— 执行器占用仲裁
 *
 * 为什么需要这一层
 * ----------------
 * 本板有 6 个会"动电机 / 动编码器寄存器"的模块：
 *   motor_ident(Z 极对数/方向/零点)、motor_rl_ident(@ R/L)、
 *   motor_flux_ident(+ ψ/R/L)、mech_ident(扫频/摩擦/惯量/齿槽)、
 *   fric_cal($ 摩擦两点)、encoder_cal(^ MT6835 自校准)
 *
 * 它们各自都能直接 BspServo_PwmEnable()、Servo_SetOpenLoop()、
 * BspServo_SetEncoderZero()，但**没有任何互斥**。实测后果：
 *   · ^ 跑到一半被别的动作打断 -> MT6835 的 EEPROM 补偿表写坏，
 *     角度输出失效（2026-11-19 实际发生）
 *   · @ 之后紧跟 +            -> FI FAIL fault（前一个还占着执行器）
 *   · C 之后紧跟 Z            -> 极对数识别成 13（转子被前一个动作扰动）
 *   · Z 用 ratio=15 算零点写进 Flash，S 行仍显示 z=1（错零点被静默接受）
 *
 * 设计要点
 * --------
 *   1. 要动执行器先 Act_Acquire(owner, busy_fn)。占不到返回 0，
 *      调用方原样返回自己的 BUSY 码，上层能明确看到"被谁占着"。
 *   2. 释放不靠调用方记得写：Act_Task1ms() 每 1 ms 问一次 busy_fn，
 *      模块自报"不忙了"就自动 Release。少一处遗漏就少一个死锁。
 *   3. 刚 Acquire 的 500 ms 内不做自动释放（grace），
 *      避免某些模块在 Start 里还没置状态、被误判成"已结束"。
 *   4. D（do_disable）会 abort 所有辨识，所以它也 Act_ForceRelease()。
 *
 * 这一层不改变任何控制数值，只加互斥，属于零控制风险改动。
 */
#include "actuator.h"

#define ACT_GRACE_MS   (500U)     /* Acquire 后的保护窗口 */

/* 顺序必须与 ActOwner 枚举一致 */
static const char *const s_name[ACT_NUM] = {
    "none", "IDENT", "RL", "FLUX", "MECH", "FRIC", "ENCCAL"
};

static volatile ActOwner s_owner;
static uint8_t (*volatile s_busy)(void);
static volatile uint16_t s_grace;

void Act_Init(void)
{
    s_owner = ACT_NONE;
    s_busy  = 0;
    s_grace = 0;
}

uint8_t Act_Acquire(ActOwner o, uint8_t (*busy_fn)(void))
{
    if (((uint32_t)o == 0U) || ((uint32_t)o >= (uint32_t)ACT_NUM)) { return 0U; }
    if (s_owner != ACT_NONE) { return 0U; }   /* 已被占用：不抢占、不排队 */

    s_busy  = busy_fn;
    s_grace = ACT_GRACE_MS;
    s_owner = o;      /* 最后赋值：Task1ms 看到 owner 时 busy_fn 已就绪 */
    return 1U;
}

void Act_Release(ActOwner o)
{
    if ((o != ACT_NONE) && (s_owner != o)) { return; }   /* 只允许自己释放自己 */
    s_owner = ACT_NONE;
    s_busy  = 0;
    s_grace = 0;
}

void Act_ForceRelease(void) { Act_Release(ACT_NONE); }

ActOwner    Act_Owner(void)    { return s_owner; }
uint8_t     Act_IsBusy(void)   { return (s_owner != ACT_NONE) ? 1U : 0U; }

const char *Act_OwnerStr(void)
{
    const uint32_t i = (uint32_t)s_owner;
    return (i < (uint32_t)ACT_NUM) ? s_name[i] : "?";
}

/* 1 ms 调用：模块自报空闲即自动释放，避免每条退出路径都要记得 Release */
void Act_Task1ms(void)
{
    if (s_owner == ACT_NONE) { return; }

    if (s_grace != 0U) { s_grace--; return; }

    if (s_busy == 0)    { Act_ForceRelease(); return; }
    if (s_busy() == 0U) { Act_ForceRelease(); }
}
