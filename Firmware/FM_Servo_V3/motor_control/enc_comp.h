/*
 * enc_comp.h —— 编码器电角度误差的**软件补偿**（随机械角变化的周期误差）
 *
 * 与 `^`（MT6835 芯片自校准，写 EEPROM）的区别：
 *   · `^` 改的是芯片内部插值，补偿数据只有一份、不可读、不可回滚；
 *   · 这里改的是 MCU 侧 Park 变换用的电角度，系数存在 RAM（可选落盘），
 *     默认全 0、随时可关，出错最多是"没补上"，不会把驱动打瘫。
 *
 * 误差模型（电气角，单位 rad，自变量是机械角 theta_m）：
 *
 *     d_e(theta_m) = SUM_{k=1..N} [ a_k * sin(k*theta_m) + b_k * cos(k*theta_m) ]
 *
 * 为什么用谐波而不是查表：编码器 INL 是光滑的周期函数，8 阶谐波（16 个系数）
 * 已能覆盖偏心(1 阶)、安装倾斜(2 阶)、磁钢不均(低阶) 这些主要成分，
 * 存 64 B 而不是 1 KB，且不会出现"表里某点坏了产生台阶"的问题。
 *
 * 运行时为了省 20 kHz 的时间，系数在**写入时**展开成 256 点表（1 KB RAM），
 * 求值只做一次乘 + 两次查表 + 一次插值。
 */
#ifndef ENC_COMP_H
#define ENC_COMP_H

#include <stdint.h>

#define ENC_COMP_MAX_H   (16U)     /* 最多 16 阶（32 系数）——实测 14 阶为主成分 */

void    EncComp_Init(void);

/* n = 0 表示关闭（等价于 SetEnable(0)）。a/b 长度至少 n（n ≤ 16）。
 * 系数超范围（|a|>0.1 rad ≈ 5.7°，或 NaN）整组拒绝，返回 0。 */
uint8_t EncComp_SetHarm(uint8_t n, const float *a, const float *b);
void    EncComp_GetHarm(float *a, float *b, uint8_t *n);

void    EncComp_SetEnable(uint8_t en);
uint8_t EncComp_GetEnable(void);

/* 返回电气角补偿量（rad）。theta_m 为机械角（rad，任意范围）。
 * 关闭时返回 0，调用方不需要判断。 */
float   EncComp_Eval(float theta_m);

/* 掉电保存：系数与开关落盘到独立 Flash 页（param_store 的 0x0807D000）。
 * Load 在初始化时调用，CRC 通过才应用（系数不依赖编码器零点，故不比对零点）。
 * Save 在"存参"（action=1）时调用，内部检查 MOE=0，返回 1=成功。 */
uint8_t EncComp_LoadFromFlash(void);
uint8_t EncComp_SaveToFlash(void);

#endif /* ENC_COMP_H */
