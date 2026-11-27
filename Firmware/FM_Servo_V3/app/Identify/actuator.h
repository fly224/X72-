/*
 * actuator.h —— 执行器占用仲裁（见 actuator.c 的设计说明）
 *
 * 用法（模块的 Start() 里，所有前置检查通过之后、改任何状态之前）：
 *
 *     if (Act_Acquire(ACT_XXX, Xxx_IsBusy) == 0U) { return XXX_ERR_BUSY; }
 *
 * 不需要在退出路径手动 Release：Act_Task1ms() 会在模块自报空闲时自动释放。
 * 例外：D（do_disable）会 abort 所有辨识，那里显式 Act_ForceRelease()。
 */
#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <stdint.h>

typedef enum {
    ACT_NONE = 0,
    ACT_MOTOR_IDENT,     /* Z：极对数 / 方向 / 电角度零点 */
    ACT_RL_IDENT,        /* @：R / L 注入辨识 */
    ACT_FLUX_IDENT,      /* +：ψ / R / L 联合辨识 */
    ACT_MECH_IDENT,      /* ( ) < }：扫频 / 摩擦 / 惯量 / 齿槽 */
    ACT_FRIC_CAL,        /* $：摩擦前馈两点标定 */
    ACT_ENC_CAL,         /* ^：MT6835 芯片自校准 */
    ACT_NUM
} ActOwner;

void        Act_Init(void);

/* 返回 1 = 占用成功；0 = 已被别的模块占用（调用方应返回自己的 BUSY 码） */
uint8_t     Act_Acquire(ActOwner o, uint8_t (*busy_fn)(void));

void        Act_Release(ActOwner o);      /* 只释放自己；传 ACT_NONE 等价强制释放 */
void        Act_ForceRelease(void);

ActOwner    Act_Owner(void);
uint8_t     Act_IsBusy(void);
const char *Act_OwnerStr(void);           /* "IDENT" / "RL" / "ENCCAL" ... */

void        Act_Task1ms(void);            /* 1 ms 调用：空闲自动释放 */

#endif /* ACTUATOR_H */
