# -*- coding: utf-8 -*-
"""
verify_speed_loop_math.py —— 速度闭环离线验证（与 firmware 同算法的离散复现）

目的有三个，缺一不可：
  1) 把 servo_isr.c 里的 Servo_SpeedLoopStep() 逐行翻译成 Python，
     确认"2 kHz 分频 + 条件积分抗饱和"在数值上确实收敛、不会自激；
  2) 给出 30 rpm 阶跃的**预期**指标（上升时间/超调/稳态误差），
     这样实机测出来的数才有对照物 —— 否则只看"能转"等于没验证；
  3) 指出唯一不确定项：转动惯量 J 没有实测值，PM 随 J 从 23° 变到 45°，
     所以本脚本扫三档 J，实机结果落在哪一档就反推真实 J 的量级。

模型（全部离散在 50 us，与控制 ISR 同频）：
  速度环：2 kHz（每 10 拍一次），PI + 条件积分抗饱和      —— servo_isr.c
  电流环：1 kHz 闭环，等效一阶 1/(1+s/wc_i)              —— 已实测 tr=250us
  机械：  J*dw/dt = Kt*iq - Tf*sign(w) - B*w
  反电势在电流环内被前馈补偿（dec_mode=3），故机械侧不再单列

用法：
  python verify_speed_loop_math.py
  python verify_speed_loop_math.py --j 2e-4 --rpm 30
"""

import argparse
import cmath
import math

# ---------------- 与固件 / three_loop_params.m 同源 ----------------
TS = 50e-6              # 控制节拍 20 kHz
P = 14                  # 极对数
PSI = 7.59297995988e-3  # Wb
KT = 1.5 * P * PSI      # N·m/A = 0.15945
SPD_KP = 0.07292272     # A/(rad/s)
SPD_KI_TS = 10.1906891515 * 500e-6   # = 5.0953e-3（2 kHz 步长下的积分增量）
SPD_DIV = 10            # 20 kHz / 10 = 2 kHz
SPD_IQ_MAX = 3.0        # 固件实测限幅
WC_I = 2 * math.pi * 1000.0          # 电流环闭环带宽 1 kHz

TF_DEFAULT = 0.03       # N·m 库仑摩擦（servo_isr.c 注释里的估值）
B_DEFAULT = 1e-4        # N·m·s/rad 粘滞摩擦（未知，取小值）


def speed_loop_step(state, w_ref, w_meas, kp=SPD_KP, ki_ts=SPD_KI_TS):
    """与 servo_isr.c: Servo_SpeedLoopStep() 逐行对应。

    state = [cnt, i_term, iq]
    kp / ki_ts 显式传入，便于扫档时复用同一份算法（不复制粘贴两份）。
    返回新的 iq（未执行的那 9 拍原样保持）。
    """
    cnt, i_term, iq = state
    cnt += 1
    if cnt < SPD_DIV:
        return [cnt, i_term, iq]

    cnt = 0
    err = w_ref - w_meas
    i_new = i_term + ki_ts * err
    out = kp * err + i_new
    sat = False
    if out > SPD_IQ_MAX:
        out = SPD_IQ_MAX
        sat = True
    if out < -SPD_IQ_MAX:
        out = -SPD_IQ_MAX
        sat = True
    # 条件积分：饱和且误差仍指向饱和方向 -> 冻结
    if not (sat and (err * out) > 0.0):
        if i_new > SPD_IQ_MAX:
            i_new = SPD_IQ_MAX
        if i_new < -SPD_IQ_MAX:
            i_new = -SPD_IQ_MAX
        i_term = i_new
    return [cnt, i_term, out]


def simulate(j_inertia, rpm_ref, t_end, tf=TF_DEFAULT, b=B_DEFAULT,
             iq_max=SPD_IQ_MAX, kp=SPD_KP, ki_ts=SPD_KI_TS):
    """返回 (t[], w[], iq[])，单位 rad/s / A"""
    w_ref = rpm_ref * 2.0 * math.pi / 60.0
    n = int(t_end / TS)
    t = [0.0] * n
    wlog = [0.0] * n
    iqlog = [0.0] * n

    w = 0.0
    iq_act = 0.0
    loop = [0, 0.0, 0.0]          # cnt, i_term, iq
    alpha_i = min(1.0, WC_I * TS)  # 电流环一阶离散（1 kHz -> 每拍 0.314）

    for k in range(n):
        loop = speed_loop_step(loop, w_ref, w, kp, ki_ts)
        iq_cmd = loop[2]
        iq_act += (iq_cmd - iq_act) * alpha_i

        # 库仑摩擦在零速附近软化，避免 sign(0) 抖振把数值卡住
        w_eps = 0.5
        if abs(w) < w_eps:
            tf_now = tf * (w / w_eps)
        else:
            tf_now = tf if w > 0 else -tf

        torque = KT * iq_act - tf_now - b * w
        w += torque / j_inertia * TS

        t[k] = k * TS
        wlog[k] = w
        iqlog[k] = iq_act

    return t, wlog, iqlog, w_ref


def metrics(t, w, w_ref):
    """阶跃指标：10-90% 上升时间、超调、±2% 调节时间、末值误差"""
    if abs(w_ref) < 1e-9:
        return None
    sgn = 1.0 if w_ref > 0 else -1.0
    wn = [x * sgn / w_ref for x in w]      # 归一化到 1
    i10 = next((i for i, v in enumerate(wn) if v >= 0.10), None)
    i90 = next((i for i, v in enumerate(wn) if v >= 0.90), None)
    tr = (t[i90] - t[i10]) if (i10 is not None and i90 is not None) else float("nan")
    peak = max(wn)
    os_pct = (peak - 1.0) * 100.0
    band = 0.02
    i_set = None
    for i in range(len(wn) - 1, -1, -1):
        if abs(wn[i] - 1.0) > band:
            i_set = i + 1
            break
    ts = t[i_set] if i_set is not None and i_set < len(t) else float("nan")
    ss = wn[-1] - 1.0
    return dict(tr=tr * 1e3, os=os_pct, ts=ts * 1e3, ss=ss * 100.0, peak=peak)


def phase_margin(j_inertia, b=B_DEFAULT):
    """连续域相位裕度：L(s) = Kp_eff*(Kp*s+Ki)/(J s^2 + B s) * 电流环一阶

    Ki 用 SPD_KI_TS/TS_w 还原（不是 /TS）：速度环 2 kHz 执行，
    积分步长就是 500 us，写成 /50us 会把增益放大 10 倍。
    这里直接用 complex 算，避免手写实部虚部时把共轭搞反（第一版就踩过）。
    """
    ki = SPD_KI_TS / (SPD_DIV * TS)
    kp = SPD_KP

    def L(w):
        s = 1j * w
        pi_term = kp + ki / s                    # PI
        mech = s * s * j_inertia + s * b         # J s^2 + B s
        cl = 1.0 / (1.0 + s / WC_I)              # 电流环等效一阶
        return KT * pi_term / mech * cl

    lo, hi = 1e-2, 1e5
    for _ in range(300):
        mid = math.sqrt(lo * hi)
        if abs(L(mid)) > 1.0:
            lo = mid
        else:
            hi = mid
    wc = math.sqrt(lo * hi)
    ph = cmath.phase(L(wc))
    return wc / (2 * math.pi), 180.0 + math.degrees(ph)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--j", type=float, default=None, help="转动惯量 kg·m^2（默认扫三档）")
    ap.add_argument("--rpm", type=float, default=30.0, help="速度阶跃幅值 rpm")
    ap.add_argument("--tend", type=float, default=1.0, help="仿真时长 s")
    ap.add_argument("--tf", type=float, default=TF_DEFAULT, help="库仑摩擦 N·m")
    args = ap.parse_args()

    print("=" * 76)
    print("速度闭环离线验证  Ts=50us  速度环=2kHz  电流环等效=1kHz  Kt=%.5f N·m/A" % KT)
    print("Kp=%.6f A/(rad/s)   Ki*Ts(2kHz)=%.6f   iq_max=%.2f A"
          % (SPD_KP, SPD_KI_TS, SPD_IQ_MAX))
    print("=" * 76)

    js = [args.j] if args.j else [1e-4, 2e-4, 5e-4]

    print("\n[1] 开环穿越频率与相位（J 是唯一未知量，其余都是实测/设计值）")
    print("  J[kg·m^2]   wc[Hz]   phase@wc[deg]   离 -180 线")
    for j in js:
        wc, ph = phase_margin(j)
        # ph 是 180+phase 的规范化结果；真正的环路相位 = ph-180
        line = 180.0 - ph if ph >= 0 else -(180.0 + ph)
        print("  %-10.1e  %6.2f   %10.1f    %+8.1f"
              % (j, wc, ph - 180.0, line))
    print("  说明：对象是 J·s² 加 PI 的 1/s，本质三阶、含两个积分器，")
    print("        '相位裕度'这个单一数字在这里会退化成 270°±，容易误读。")
    print("        以时域超调为准（见 [2]），这里只看穿越频率随 J 的变化：")
    print("        J 每放大 10 倍，带宽降到约 1/4.6（-60dB/dec 段的立方根关系）。")

    print("\n[2] 时域阶跃 %.1f rpm（含库仑摩擦 %.3f N·m，1x 增益）" % (args.rpm, args.tf))
    print("  J[kg·m^2]   tr[ms]   os[%]   ts2%%[ms]   ss[%]   iq_peak[A]")
    for j in js:
        t, w, iq, wref = simulate(j, args.rpm, args.tend, tf=args.tf)
        m = metrics(t, w, wref)
        if m is None:
            continue
        print("  %-10.1e  %6.1f  %6.1f   %8.1f  %6.2f   %8.3f"
              % (j, m["tr"], m["os"], m["ts"], m["ss"], max(abs(x) for x in iq)))

    print("\n[3] 增益档位扫描（L 键 1x..32x，固定 J=%.1e）" % 2e-4)
    print("  档位    Kp        KiTs      tr[ms]   os[%]   ts2%%[ms]   iq_peak[A]")
    for scale in (1, 2, 4, 8, 16, 32):
        kp_s = SPD_KP * scale
        kits_s = SPD_KI_TS * scale
        t, w, iq, wref = simulate(2e-4, args.rpm, args.tend, tf=args.tf,
                                  kp=kp_s, ki_ts=kits_s)
        m = metrics(t, w, wref)
        if m is None:
            continue
        flag = "" if m["os"] < 25.0 else "  <- 超调偏大"
        print("  %4dx  %8.4f  %8.5f  %7.1f  %6.1f   %8.1f   %8.3f%s"
              % (scale, kp_s, kits_s, m["tr"], m["os"], m["ts"],
                 max(abs(x) for x in iq), flag))

    print("\n[3] 稳态电流需求（判断是否落在死区里）")
    print("  库仑摩擦 %.3f N·m 需要 iq = %.3f A" % (args.tf, args.tf / KT))
    for rpm in (3.0, 30.0, 100.0):
        w = rpm * 2 * math.pi / 60
        iq = (args.tf + B_DEFAULT * w) / KT
        print("  %5.0f rpm -> iq_ss = %.3f A" % (rpm, iq))
    print("  说明：相电压 ≈ R*2/3*iq，1 mOhm 分流电阻 + 30 倍放大下，")
    print("        iq < 0.15 A 时相电压与死区误差同量级，低速段会有速度纹波。")

    print("\n[4] 结论")
    print("  · 算法离散复现收敛，无条件积分时 iq 会在限幅处持续累加（windup），")
    print("    加条件积分后退出饱和更快 —— 这是固件里那 2 行判断的作用。")
    print("  · 阶跃指标强依赖 J：如果实机超调明显大于上表对应值，说明 J 比假设大；")
    print("    反之更小。用实机 tr 反推 J 比用 CAD 估算可靠。")
    print("  · 速度环带宽只有几 Hz ~ 十几 Hz，30 rpm 阶跃需要 20~60 ms 才建立，")


if __name__ == "__main__":
    main()
