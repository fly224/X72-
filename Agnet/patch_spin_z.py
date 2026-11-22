# -*- coding: utf-8 -*-
"""把 motor_ident 的 120° 阶跃法改成连续拖动法（保留 CRLF）。"""
import io
import re

P = "motor_ident.c"
s = io.open(P, encoding="utf-8", newline="").read()
s = s.replace("\r\n", "\n")          # 统一处理，最后再还原

# ---------- 1) 新增常量 ----------
anchor = "#define MI_RATIO_TOL       (0.25f)"
consts = """#define MI_RATIO_TOL       (0.25f)            /* |ratio - round(ratio)| 超过即判不可信 */

/* ---- 连续拖动法（2026-11-20 替换 120° 阶跃）----
 * 阶跃法的根本问题是恒定电流矢量 = 无阻尼单摆：4 A 下同步刚度约 5 N·m/rad、
 * J=7.67e-5，固有频率 41 Hz，150 ms 稳定窗里摆 6 个周期，采样点相位随机，
 * 实测同一步位移 244~>10000 mdeg（40 倍离散）。连续拖动无阶跃，转子准静态
 * 跟随，摩擦滞后在正反两遍里符号相反、平均抵消。 */
#define MI_SPIN_W_E        (15.708f)   /* 拖动电角速度 rad/s = 2π·2.5 Hz
                                        * 机械 2.5/14 = 0.179 rev/s ≈ 10.7 rpm */
#define MI_SPIN_RAMP_MS    (600U)      /* 0 → ωe 的速度斜坡，避免起拖失步 */
#define MI_SPIN1_MS        (4200U)     /* ① 比例段：3.6 s 匀速 ≈ 0.64 机械圈 */
#define MI_SPIN2_MS        (7000U)     /* ② 零点段：单方向 ≈ 1.07 机械圈，正反各一遍 */
#define MI_SPIN_SMP_MS     (5U)        /* 零点段采样间隔 */
#define MI_SPIN_MIN_RAD    (0.35f)     /* 3.6 s 连 20° 都走不满 -> 判卡死 */"""
assert anchor in s
s = s.replace(anchor, consts, 1)

# ---------- 2) 新增 s_th_e ----------
old_v = "static float              s_theta_v;    /* 虚拟电角度，始终 wrap 在 [-π, π] */"
new_v = old_v + "\nstatic float              s_th_e;       /* 连续拖动：命令电角累计（rad，单调） */"
assert old_v in s
s = s.replace(old_v, new_v, 1)

# ---------- 3) 替换 step_finish_ratio + step_step + step_sweep ----------
start = s.index("static uint8_t step_finish_ratio(void)")
end   = s.index("/* ---------------- 公开接口 ---------------- */")

new_block = r'''static uint8_t spin_finish_ratio(float th_e, float th_m)
{
    float am = fabsf(th_m);

    s_res.dir = (th_m >= 0.0f) ? 1 : -1;

    if (am < MI_SPIN_MIN_RAD) {
        /* 转了 3.6 s 连 20° 都不到 —— 卡死 / 功率线 / 抱闸 */
        s_res.ratio     = 0.0f;
        s_res.ratio_int = 0U;
        s_res.dmech_avg = am;
        s_res.dmech_min = am;
        s_res.dmech_max = am;
        finish_fail(MOTOR_IDENT_ERR_STUCK);
        return 1U;
    }

    s_res.ratio     = th_e / am;
    s_res.ratio_int = (uint16_t)(s_res.ratio + 0.5f);
    s_res.resid     = fabsf(s_res.ratio - (float)s_res.ratio_int);

    /* 回显统一成"每 120° 电角对应的机械角"，与旧步进法同量纲（正常 8571 mdeg），
     * 方便与历史数据横向对比。 */
    s_res.dmech_avg = am * (MI_STEP_ELEC / th_e);
    s_res.dmech_min = s_res.dmech_avg;
    s_res.dmech_max = s_res.dmech_avg;

    if ((s_res.ratio_int != (uint16_t)BSP_POLE_PAIRS) || (s_res.resid > MI_RATIO_TOL)) {
        finish_fail(MOTOR_IDENT_ERR_RATIO);
        return 1U;
    }

    s_osin = 0.0f; s_ocos = 0.0f;
    s_res.n_sweep = 0U;
    s_t = 0U;
    s_state = MOTOR_IDENT_SWEEP_FWD;
    return 0U;
}

/* ① 连续正转，测极对数：ratio = 命令电角累计 ÷ 编码器机械角累计 */
static uint8_t step_step(void)
{
    s_t++;

    if (s_t == 1U) {
        s_th_e       = 0.0f;
        s_res.n_step = 0U;
        s_res.n_fail = 0U;
        Servo_SetOLSpin(1U, 0.0f);        /* 角由 20 kHz ISR 累加，这里只给速度 */
        return 0U;
    }

    {
        float k = (float)(s_t - 1U) / (float)MI_SPIN_RAMP_MS;
        float w;

        if (k > 1.0f) { k = 1.0f; }
        w = MI_SPIN_W_E * k;
        Servo_SetOLSpin(1U, w);
        s_th_e += w * 0.001f;             /* 命令电角精确积分，不依赖编码器 */
        s_res.n_step++;
    }

    if (s_t < MI_SPIN1_MS) { return 0U; }

    Servo_SetOLSpin(0U, 0.0f);
    {
        const int64_t p_now = BspServo_GetPosSum();
        const float   th_m  = (float)(p_now - s_p_start) * MI_ENC_RAD_PER_CNT;

        return spin_finish_ratio(s_th_e, th_m);
    }
}

/* ② 连续扫整圈，累加相位误差 -> 零点。正反各一遍。
 * 注意：**不重置** s_p_start / s_theta_start（在 step_ramp 里与 s_raw_start
 * 同一时刻记录），否则 θm 变成相对量，φ 会带一个常量偏移，零点直接错。 */
static uint8_t step_sweep(void)
{
    float dir_w;

    s_t++;

    if (s_t == 1U) {
        Servo_SetOLSpin(1U, 0.0f);        /* 反向那一遍先停一拍再换向 */
        return 0U;
    }

    dir_w = (s_state == MOTOR_IDENT_SWEEP_FWD) ? MI_SPIN_W_E : -MI_SPIN_W_E;

    {
        float k = (float)(s_t - 1U) / (float)MI_SPIN_RAMP_MS;

        if (k > 1.0f) { k = 1.0f; }
        Servo_SetOLSpin(1U, dir_w * k);
    }

    if (s_t < MI_SPIN2_MS) {
        if ((s_t % MI_SPIN_SMP_MS) == 0U) {
            const int64_t p_now  = BspServo_GetPosSum();
            const float   th_m   = s_theta_start +
                                   ((float)(p_now - s_p_start) * MI_ENC_RAD_PER_CNT);
            const float   e_elec = (float)s_res.dir * (float)s_res.ratio_int * th_m;
            const float   phi    = wrap_pi(Servo_GetOLTheta() - e_elec);

            s_osin += sinf(phi);
            s_ocos += cosf(phi);
            s_res.n_sweep++;
        }
        return 0U;
    }

    Servo_SetOLSpin(0U, 0.0f);

    if (s_state == MOTOR_IDENT_SWEEP_FWD) {
        s_t = 0U;
        s_state = MOTOR_IDENT_SWEEP_REV;
        return 0U;
    }

    finish_ok();
    return 1U;
}

'''

s = s[:start] + new_block + s[end:]
s = s.replace("\n", "\r\n")
io.open(P, "w", encoding="utf-8", newline="").write(s)
print("patched, bytes =", len(s))
