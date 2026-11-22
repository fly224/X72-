/*
 * deadband_comp.c —— 逆变桥死区补偿（移植自 FM_XC）
 *
 * 20 kHz 控制节拍里每相调用一次，单次约 15~20 周期。
 *
 * 状态：2026-11-17 起**默认开启**，带转速门控（≤100 rpm 全量、≥200 rpm 关闭）。
 *
 * 之前一轮（2026-11-15）的结论是"默认关闭"，理由是开启后 3 rpm 跑成 0.10 rpm、
 * clamp 从 0 涨到 748（严重过补偿）。那个判断本身没错，但**根因算错了**：
 *
 *   当时按 `(64+50)×2×tDTS = 670 ns` 算死区，补偿量 1.34%。
 *   实际上 DTG=50 的 bit7=0，属于 **0xx 段**，公式是 `DTG[7:0]×tDTS`：
 *       DT = 50 × 5.88 ns = 294 ns  →  Δd = 0.59%
 *   补偿量被放大了 2.28 倍 —— 过补偿是"量给多了"，不是"补偿没用"。
 *
 * 修正后重测（du=0.59%，I_LIN 从 0.6 A 收到 0.15 A）：
 *   30 rpm   σ +7.6%（三轮全正）
 *   100 rpm  σ +6.5%
 *   260 rpm  σ −25.5%（两轮都恶化）
 *   500/1000 在噪声内
 *   → 260 rpm 正好是速度环的高纹波带，补偿在那里等于往已经在振的环里注入扰动；
 *     高速端惯性本来就把畸变滤掉了。所以加了转速门控，只在低速段生效。
 *
 * 标定方式：`\` 设 du（单位 0.01%，59 = 0.59%），`B` 开关，`\` 空参数查询。
 */
#include "deadband_comp.h"

static uint8_t s_enable = 1U;     /* 2026-11-17 起默认开启（见文件头与 DBC_DU_MAX 注释） */

/* 实际补偿量（占空比）。默认取修正后的理论值 0.0059（294 ns），
 * 但理论值只提供了起点：闭环积分已经补掉了死区的**平均**电压损失，
 * 补偿真正要抵消的是过零附近的瞬时非线性，最优量必然小于等于理论值。
 * 用串口 `\` 扫它，拿 iq 的 6 次脉动做判据。 */
static float s_du = DBC_DU_MAX;

/* 转速门控系数，1 = 全量、0 = 关闭。由 SetSpeed 每拍更新。 */
static float s_gate = 1.0f;

/* 电流异常门限：超出视为采样故障/干扰，不做补偿，交给过流保护处理。
 * 不做这个保护的话，一个坏采样会让 duty 被补偿到边界，反而制造冲击。 */
#define DBC_I_ABS_MAX   (100.0f)

void DeadbandComp_SetSpeed(float w_mech)
{
    const float a = (w_mech < 0.0f) ? -w_mech : w_mech;

    if (a <= DBC_W_FULL)      { s_gate = 1.0f; }
    else if (a >= DBC_W_ZERO) { s_gate = 0.0f; }
    else                      { s_gate = (DBC_W_ZERO - a) / (DBC_W_ZERO - DBC_W_FULL); }
}

float DeadbandComp_Apply(float duty, float i_phase)
{
    float du;

    if ((s_enable == 0U) || (s_gate <= 0.0f)) {
        return duty;
    }

    /* NaN 与非有限值一律跳过（!(x > a && x < b) 对 NaN 为真） */
    if (!(i_phase > -DBC_I_ABS_MAX) || !(i_phase < DBC_I_ABS_MAX)) {
        return duty;
    }

    if (i_phase > DBC_I_LIN_A) {
        du = s_du;                             /* 正向饱和 */
    } else if (i_phase < -DBC_I_LIN_A) {
        du = -s_du;                            /* 反向饱和 */
    } else {
        /* 线性过渡：过零点附近按比例给，避免 ±du 硬跳变 */
        du = s_du * (i_phase / DBC_I_LIN_A);
    }

    duty += du * s_gate;

    if (duty < 0.0f) { duty = 0.0f; }
    if (duty > 1.0f) { duty = 1.0f; }
    return duty;
}

void    DeadbandComp_SetEnable(uint8_t en) { s_enable = (en != 0U) ? 1U : 0U; }
uint8_t DeadbandComp_IsEnabled(void)      { return s_enable; }
float   DeadbandComp_GetDuMax(void)       { return s_du; }

void DeadbandComp_SetDu(float du)
{
    if (du < 0.0f)   { du = 0.0f; }
    if (du > 0.05f)  { du = 0.05f; }
    s_du = du;
}

float DeadbandComp_GetDu(void) { return s_du; }
