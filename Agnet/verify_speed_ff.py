# -*- coding: utf-8 -*-
"""速度环前馈 + 抗饱和的离线验证（含粘滞-滑动摩擦）

为什么必须把静摩擦单独建模：
  实测 3 rpm 起动时 iq 峰值 0.64 A、稳态 0.30 A —— 静摩擦约为动摩擦的 2 倍。
  只用库仑摩擦（Tc·sign(w)）在 w=0 时给出 0，仿真的超调只有 5%，
  与实测 68.8% 差了十几倍，根本看不出前馈有没有用。

  所以这里用标准的"粘滞-滑动"两态模型：
    粘滞态：合力不足以克服静摩擦 -> 转子不动（w 保持 0）
    滑动态：动摩擦 + 黏滞，按 J·dw/dt = Kt·iq − T_f

用法：
    python verify_speed_ff.py
    python verify_speed_ff.py --rpm 3
"""
from __future__ import print_function

import argparse
import math

TWO_PI = 2.0 * math.pi
RAD_PER_RPM = TWO_PI / 60.0

# ---------------- 与固件/实测对齐的参数 ----------------
TS_FAST = 50e-6
SPD_DIV = 10
TS_SPD = TS_FAST * SPD_DIV

KT = 0.0936              # N·m/A（实测 psi = 4.455 mWb）
J = 2.3e-4               # kg·m^2
KP = 0.5833818
KI_TS = 0.04076276
IQ_MAX = 3.0

TC_S = 0.060             # 静摩擦 N·m（由 3 rpm 起动 iq 0.64 A 反推）
TC_K = 0.028             # 动摩擦 N·m（由 30 rpm 稳态 iq 0.30 A 反推）
BV = 2.0e-5              # 黏滞 N·m·s/rad
TAU_I = 114e-6           # 电流环等效一阶

ACC_DFLT = 400.0         # rad/s^2

# 前馈默认（与固件 SERVO_FF_* 一致）
FF_ACC = 1.72e-3         # A/(rad/s^2)
FF_FRIC = 0.321          # A
FF_VISC = 2.14e-4        # A/(rad/s)
FF_W_EPS = 0.05
FF_ACC_LPF = 0.25
FF_BOOST = 1.0           # 起动助推倍数（固件 SERVO_FF_BOOST_DFLT）
FF_START_RATIO = 0.30    # |w| < 0.3·|ramp| 时判定为"还没转起来"


def clamp(x, a, b):
    return a if x < a else (b if x > b else x)


def sim(target_rpm, accel=ACC_DFLT, ff_acc_on=True, ff_fric_on=True,
        boost_on=True, kb=0.30, t_end=2.0, iq_max=IQ_MAX):
    """返回 (t[], w_rpm[], ramp_rpm[], iq[])"""
    target = target_rpm * RAD_PER_RPM
    w = 0.0
    iq = 0.0
    ramp = 0.0
    iterm = 0.0
    init = False
    ff_acc_f = 0.0

    sticky = True               # 粘滞态：转子被静摩擦钉住
    unlimited = accel is None or accel >= 20000.0

    ts, ws, rs, qs = [], [], [], []
    t = 0.0
    n = 0
    while t < t_end:
        if n % SPD_DIV == 0:
            if not init:
                ramp = w
                init = True

            if unlimited:
                ramp = target
                acc_ref = 0.0
            else:
                step = accel * TS_SPD
                dw = target - ramp
                if dw > step:
                    ramp += step
                elif dw < -step:
                    ramp -= step
                else:
                    ramp = target
                # 斜坡期间加速度是常数 ±accel，走完立刻归零（固件也是这么算的）
                acc_ref = math.copysign(accel, dw) if abs(dw) > step else 0.0

            ff_acc_f += FF_ACC_LPF * (acc_ref - ff_acc_f)

            ff = 0.0
            if ff_acc_on:
                ff += FF_ACC * ff_acc_f
            if ff_fric_on:
                ff += FF_FRIC * math.tanh(ramp / FF_W_EPS)
                ff += FF_VISC * ramp
                # 起动助推：静摩擦是动摩擦的 2 倍，标定只测得到稳态
                if boost_on and abs(w) < FF_START_RATIO * abs(ramp):
                    ff += FF_FRIC * FF_BOOST * math.tanh(ramp / FF_W_EPS)

            err = ramp - w
            u_raw = KP * err + iterm + ff
            u_sat = clamp(u_raw, -iq_max, iq_max)

            if kb > 0.0:
                iterm += KI_TS * err + kb * (u_sat - u_raw)
            else:
                if not (u_sat != u_raw and err * u_raw > 0.0):
                    iterm += KI_TS * err
            iterm = clamp(iterm, -iq_max, iq_max)

            iq_cmd = clamp(KP * err + iterm + ff, -iq_max, iq_max)

        n += 1
        iq += (iq_cmd - iq) * (TS_FAST / (TAU_I + TS_FAST))

        F = KT * iq

        if sticky:
            # 粘滞态：外加力矩不足以克服静摩擦就不动
            if abs(F) > TC_S:
                sticky = False
                T_f = TC_K * math.copysign(1.0, F) + BV * w
            else:
                T_f = F           # 静摩擦完全抵消（合力为 0）
        else:
            T_f = TC_K * math.copysign(1.0, w) + BV * w if abs(w) > 1e-6 else 0.0

        w += (F - T_f) / J * TS_FAST
        if (not sticky) and abs(w) < 1e-3 and abs(F) < TC_S:
            sticky = True
            w = 0.0

        t += TS_FAST
        if n % 20 == 0:
            ts.append(t)
            ws.append(w / RAD_PER_RPM)
            rs.append(ramp / RAD_PER_RPM)
            qs.append(iq)
    return ts, ws, rs, qs


def metrics(ts, ws, qs, target_rpm):
    tgt = target_rpm
    peak = max(ws) if tgt > 0 else min(ws)
    os_pct = (peak - tgt) / tgt * 100.0
    tail = [v for tt, v in zip(ts, ws) if tt > ts[-1] - 0.25]
    ss = sum(tail) / len(tail) if tail else ws[-1]
    return dict(os=os_pct, ss=ss, iq_peak=max(abs(v) for v in qs), peak=peak)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rpm", type=float, default=None)
    args = ap.parse_args()

    print("参数：Kt=%.4f  J=%.2e  静摩擦 %.3f N·m(%.2fA)  动摩擦 %.3f N·m(%.2fA)  黏滞 %.1e"
          % (KT, J, TC_S, TC_S / KT, TC_K, TC_K / KT, BV))
    print("     Kp=%.4f  Ki·Ts=%.5f  iqmax=%.1f  accel=%.0f rad/s^2"
          % (KP, KI_TS, IQ_MAX, ACC_DFLT))

    cases = [
        ("原逻辑（条件积分、无前馈）", dict(ff_acc_on=False, ff_fric_on=False, kb=0.0)),
        ("仅 back-calc（kb=0.3）",     dict(ff_acc_on=False, ff_fric_on=False, kb=0.30)),
        ("摩擦前馈（无助推）",           dict(ff_acc_on=False, ff_fric_on=True,  boost_on=False, kb=0.30)),
        ("摩擦前馈+助推",                dict(ff_acc_on=False, ff_fric_on=True,  boost_on=True,  kb=0.30)),
        ("全套（助推+加速度前馈）",       dict(ff_acc_on=True,  ff_fric_on=True,  boost_on=True,  kb=0.30)),
    ]

    rpms = [args.rpm] if args.rpm else [3.0, 30.0, 500.0]

    for rpm in rpms:
        print("\n[%+.0f rpm]  斜坡 %.0f rad/s^2" % (rpm, ACC_DFLT))
        print("  方案                        超调      稳态      iq峰值   起动延迟")
        for name, kw in cases:
            ts, ws, rs, qs = sim(rpm, **kw)
            m = metrics(ts, ws, qs, rpm)
            # 起动延迟：从 0 到首次 |w| > 1% 目标的时刻
            t0 = next((t for t, v in zip(ts, ws) if abs(v) > 0.01 * abs(rpm)), float("nan"))
            print("  %-26s %6.1f%%  %8.3f  %7.2f A  %6.1f ms"
                  % (name, m["os"], m["ss"], m["iq_peak"], t0 * 1e3))

    print("\n[位置环前馈] 纯 P 位置环的稳态滞后 e = v_ref / Kp")
    print("   v_ref        e(Kp=50.265)    说明")
    for v in (1.0, 5.0, 20.0, 50.0):
        print("   %5.1f rad/s   %7.3f rad   (%.1f deg)"
              % (v, v / 50.265, v / 50.265 * 180 / math.pi))
    print("  加 w_ff = v_ref 后，该项被完全抵消；残余误差只剩动态部分。")

    print("\n结论：")
    print("  · 3 rpm 的超调几乎全部来自静摩擦起跳 —— 只有摩擦前馈能压下来")
    print("  · back-calculation 单独用只影响退饱和速度，对静摩擦无解")
    print("  · 加速度前馈主要改善高速段 iq 波形，低速段贡献小")


if __name__ == "__main__":
    main()
