# -*- coding: utf-8 -*-
"""速度给定斜坡（加减速限制）的离线验证

目的：在烧板子之前先算清三件事
  1) 斜坡到底能把超调压掉多少（是不是值得加）
  2) 默认 accel 取多少（400 是否会太慢/太快）
  3) 斜坡对低速（3/30 rpm）和高速（3000 rpm）的收益差别

模型与实测对齐过的参数（2026-11-16 实测反推）：
  Kt = 1.5 * 14 * 4.455e-3 = 0.0936 N·m/A   （psi 由 2496 rpm 电压饱和点反算）
  J  = 2.3e-4 kg·m^2                        （由"8x 增益最优"反推）
  Kp = 0.5833818 A/(rad/s)、Ki·Ts = 0.04076276（实测最优档）
  库仑摩擦 Tc = 0.028 N·m                   （30 rpm 稳态 iq≈0.35 A × Kt）
  电流环一阶 tau = 114 us                   （实测 tr=250 us → 带宽约 1.4 kHz）

用法：
    python verify_speed_ramp.py
    python verify_speed_ramp.py --rpm 30 --acc 400
"""
from __future__ import print_function

import argparse
import math

TWO_PI = 2.0 * math.pi
RAD_PER_RPM = TWO_PI / 60.0

# ---------------- 与固件一致的参数 ----------------
TS_FAST = 50e-6          # 20 kHz 控制节拍
SPD_DIV = 10             # 速度环 2 kHz
TS_SPD = TS_FAST * SPD_DIV

KT = 0.0936              # N·m/A
J = 2.3e-4               # kg·m^2
KP = 0.5833818
KI_TS = 0.04076276
IQ_MAX = 3.0
TAU_I = 114e-6           # 电流环等效一阶
TC_FRIC = 0.028          # 库仑摩擦 N·m
BV_FRIC = 2.0e-5         # 黏滞 N·m·s/rad


def clamp(x, a, b):
    return a if x < a else (b if x > b else x)


def sim(target_rpm, accel, t_end=2.0, iq_max=IQ_MAX, kp=KP, kits=KI_TS,
        j=J, tc=TC_FRIC, bv=BV_FRIC, tau_i=TAU_I):
    """返回 (t[], w_rpm[], ramp_rpm[], iq[])"""
    target = target_rpm * RAD_PER_RPM
    w = 0.0          # 机械角速度 rad/s
    iq = 0.0         # 实际电流（一阶跟随给定）
    ramp = 0.0
    iterm = 0.0
    init = False
    # accel 为 None 表示不限制（纯阶跃）
    unlimited = accel is None or accel >= 20000.0

    ts = []
    ws = []
    rs = []
    qs = []
    t = 0.0
    n = 0
    while t < t_end:
        if n % SPD_DIV == 0:
            if not init:
                ramp = w
                init = True
            if unlimited:
                ramp = target
            else:
                step = accel * TS_SPD
                dw = target - ramp
                if dw > step:
                    ramp += step
                elif dw < -step:
                    ramp -= step
                else:
                    ramp = target

            err = ramp - w
            i_new = iterm + kits * err
            u = kp * err + i_new
            sat = 0
            if u > iq_max:
                u = iq_max
                sat = 1
            if u < -iq_max:
                u = -iq_max
                sat = 1
            # 条件积分（与固件 Servo_SpeedLoopStep 同逻辑）
            if not (sat and err * u > 0.0):
                iterm = clamp(i_new, -iq_max, iq_max)
            iq_cmd = u
        n += 1

        # 电流环一阶
        iq += (iq_cmd - iq) * (TS_FAST / (tau_i + TS_FAST))
        # 摩擦（tanh 平滑零速，避免 sign 在 0 附近抖）
        t_load = tc * math.tanh(w / 0.02) + bv * w
        # 机械方程
        w += (KT * iq - t_load) / j * TS_FAST
        t += TS_FAST

        if n % 20 == 0:      # 1 kHz 采样输出，够画图
            ts.append(t)
            ws.append(w / RAD_PER_RPM)
            rs.append(ramp / RAD_PER_RPM)
            qs.append(iq)
    return ts, ws, rs, qs


def metrics(ts, ws, rs, qs, target_rpm):
    """超调 / 稳态 / 峰值电流。给定取 ramp 的终点（= target）。"""
    tgt = target_rpm
    if abs(tgt) < 1e-9:
        return None
    peak = max(ws) if tgt > 0 else min(ws)
    os_pct = (peak - tgt) / tgt * 100.0
    tail = [v for tt, v in zip(ts, ws) if tt > ts[-1] - 0.2]
    ss = sum(tail) / len(tail) if tail else ws[-1]
    iq_peak = max(abs(v) for v in qs)
    return dict(os=os_pct, ss=ss, iq_peak=iq_peak, peak=peak)


def rise_time(ts, ws, target_rpm):
    """10%→90% 上升时间（ms）"""
    lo = 0.1 * target_rpm
    hi = 0.9 * target_rpm
    t10 = t90 = None
    for t, v in zip(ts, ws):
        if t10 is None and v >= lo:
            t10 = t
        if t90 is None and v >= hi:
            t90 = t
            break
    if t10 is None or t90 is None:
        return float("nan")
    return (t90 - t10) * 1e3


def ramp_time_ms(target_rpm, accel):
    if accel is None or accel >= 20000.0:
        return 0.0
    return abs(target_rpm) * RAD_PER_RPM / accel * 1e3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rpm", type=float, default=None)
    ap.add_argument("--acc", type=float, default=None)
    args = ap.parse_args()

    print("模型参数：Kt=%.4f N·m/A  J=%.2e kg·m^2  Kp=%.4f  Ki·Ts=%.5f  iqmax=%.1f A"
          % (KT, J, KP, KI_TS, IQ_MAX))
    print("         库仑摩擦 %.3f N·m（对应起动电流 %.2f A）  电流环 tau=%.0f us"
          % (TC_FRIC, TC_FRIC / KT, TAU_I * 1e6))

    if args.rpm is not None:
        accs = [None, args.acc if args.acc else 400.0, 200.0, 100.0]
        print("\n[单点] 给定 %+.1f rpm" % args.rpm)
        print("  accel       斜坡时间   上升时间   超调     稳态      峰值iq")
        for a in accs:
            ts, ws, rs, qs = sim(args.rpm, a)
            m = metrics(ts, ws, rs, qs, args.rpm)
            tr = rise_time(ts, ws, args.rpm)
            name = "不限(阶跃)" if a is None else "%.0f rad/s2" % a
            print("  %-11s %7.1fms %8.1fms %7.1f%% %8.3f  %6.2f A"
                  % (name, ramp_time_ms(args.rpm, a), tr, m["os"], m["ss"], m["iq_peak"]))
        return

    print("\n[1] 低速段：斜坡的收益有限（超调主要来自库仑摩擦起跳，不是加速度）")
    for rpm in (3.0, 30.0):
        print("\n  给定 %+.0f rpm" % rpm)
        print("    accel        斜坡时间   上升时间   超调      稳态      峰值iq")
        for a in (None, 800.0, 400.0, 200.0, 100.0):
            ts, ws, rs, qs = sim(rpm, a)
            m = metrics(ts, ws, rs, qs, rpm)
            tr = rise_time(ts, ws, rpm)
            name = "不限(阶跃)" if a is None else "%.0f" % a
            print("    %-12s %7.1fms %8.1fms %7.1f%% %8.3f  %6.2f A"
                  % (name, ramp_time_ms(rpm, a), tr, m["os"], m["ss"], m["iq_peak"]))

    print("\n[2] 高速段：斜坡的价值在这里 —— 阶跃会把 iq 顶到限幅并持续很久")
    for rpm in (500.0, 1500.0, 3000.0):
        print("\n  给定 %+.0f rpm（注意：仿真不含电压限幅，3000 实际到不了，只看加速段）" % rpm)
        print("    accel        斜坡时间   到 90%% 时间  峰值iq   iq 饱和时长")
        for a in (None, 800.0, 400.0, 200.0):
            ts, ws, rs, qs = sim(rpm, a, t_end=6.0)
            tr = rise_time(ts, ws, rpm)
            iq_peak = max(abs(v) for v in qs)
            sat_n = sum(1 for v in qs if abs(v) > IQ_MAX * 0.98)
            sat_ms = sat_n / 1000.0 * 1e3 * (ts[1] - ts[0]) if len(ts) > 1 else 0.0
            sat_ms = sat_n * (ts[1] - ts[0]) * 1e3 if len(ts) > 1 else 0.0
            name = "不限(阶跃)" if a is None else "%.0f" % a
            print("    %-12s %7.1fms %10.1fms %7.2f A %8.1f ms"
                  % (name, ramp_time_ms(rpm, a), tr, iq_peak, sat_ms))

    print("\n[3] 结论")
    print("  · 3 rpm 的 68.8%% 超调基本不受斜坡影响 —— 那是静摩擦起跳，需要摩擦补偿")
    print("  · 30 rpm 斜坡时间 <16 ms，相对 ~100 ms 的响应占比小，超调改善有限")
    print("  · 500 rpm 以上阶跃会让 iq 长时间顶在 3 A 限幅，斜坡能把它摊平")


if __name__ == "__main__":
    main()
