# -*- coding: utf-8 -*-
"""X72 电流采样链路核查：前端增益、RC 极点、ADC 触发时刻与采样窗口。

用法：
    python adc_chain_check.py            # 只算理论
    python adc_chain_check.py --live COM27   # 加上高速段占空比/电流实测
"""
import argparse
import math
import struct
import time

# ---------------- 硬件常量（来自网表 Hardware/Netlist_PG_X72_CAN_2026-11-07.enet） ----------------
R_SHUNT = 1.0e-3          # R3/R4 HoBB1216-3W-1mR
G_AMP = 30.0              # R30/R28 = 30k/1k（差放）
R_IN = 1.0e3              # R28/R29
R_FB = 30.0e3             # R30
C_FB = 100.0e-12          # C5
C_DIFF = 10.0e-12         # C3
VREF = 3.3
BITS = 4095.0

# ---------------- 定时器 ----------------
F_CPU = 170.0e6
ARR = 4250.0
T_CNT = 1.0 / F_CPU                # 5.882 ns/拍
T_PWM = 2.0 * ARR * T_CNT          # 50 µs
DT_CNT = 50.0                      # TIM1 DTG=50 → 0xx 段 → 50*tDTS
DT_NS = DT_CNT * T_CNT * 1e9

# FD6288Q 数据手册（Fortior，250V 三相驱动）
T_ON_TYP = 150e-9
T_OFF_TYP = 120e-9
T_ON_MAX = 250e-9
T_OFF_MAX = 250e-9
FD_DT = 200e-9                     # 内置死区 typ

ADC_CK = F_CPU / 4.0               # PCLK_DIV4
SMP_CYC = 24.5
CONV_CYC = 12.5


def front_end():
    mv_per_a = R_SHUNT * G_AMP
    lsb_v = VREF / BITS
    a_per_lsb = lsb_v / mv_per_a
    print("=" * 74)
    print("1) 采样前端（网表实证）")
    print("-" * 74)
    print("  R3/R4 = 1 mΩ 4 线 Kelvin，U_GND/W_GND <-> PGND  → 低边采样")
    print("  R28/R29=1k 输入，R30/R31=30k，C5=100pF 反馈，C3=10pF 差分")
    print("  差放增益 = R30/R28 = %.0f  → %.1f mV/A" % (G_AMP, mv_per_a * 1e3))
    print("  ADC LSB = %.4f mV  → %.3f mA/LSB" % (lsb_v * 1e3, a_per_lsb * 1e3))
    for cur, tag in ((5.0, "额定 5 A"), (9.0, "软件限幅 9 A"), (12.0, "硬件跳闸 12 A")):
        v = cur * mv_per_a
        print("  %-12s → %6.1f mV → %5.0f LSB  (%.1f%% 满量程)"
              % (tag, v * 1e3, v / lsb_v, v / VREF * 100))
    print("  实测噪声 0.048 A RMS = %.1f LSB" % (0.048 / a_per_lsb))
    print()
    fc_fb = 1.0 / (2 * math.pi * R_FB * C_FB)
    fc_diff = 1.0 / (2 * math.pi * 2 * R_IN * C_DIFF)
    print("  反馈极点 1/(2π·30k·100p) = %.2f kHz   ← 决定整个前端带宽" % (fc_fb / 1e3))
    print("  差分极点 1/(2π·2k·10p)  = %.2f MHz    ← 可忽略" % (fc_diff / 1e6))
    for f in (20e3, 100e3, 1e6, 5e6):
        att = -10 * math.log10(1 + (f / fc_fb) ** 2)
        print("     %6.0f kHz 处衰减 %6.1f dB" % (f / 1e3, att))
    print("  结论：20 kHz PWM 纹波在通带内（-0.6 dB），靠同步采样抑制，不靠滤波")
    print()


def trigger():
    print("=" * 74)
    print("2) ADC 注入触发链路")
    print("-" * 74)
    print("  TIM1 TRGO = OC4REF（sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF）")
    print("  ADC1/2 注入组 = ADC_EXTERNALTRIGINJEC_T1_TRGO + EDGE_RISING")
    print("  → 硬件同步，不是软件触发，也不是 UPDATE 事件  ✓")
    print("  CH1~3 = PWM2 + OCPolarity=HIGH，CCR=(1-d)·ARR")
    print("  → 上管导通以 CNT=ARR（峰值）为中心，下管导通以 CNT=0（谷值）为中心")
    print()
    print("  OC4 同为 PWM2，CCR4=150 → OC4REF 上升沿出现在上计数 CNT=150")
    print("  → 距谷值 %.0f ns（= 150 × %.2f ns）" % (150 * T_CNT * 1e9, T_CNT * 1e9))
    print("  死区 %.0f ns（DTG=50）插在 CNT=CCR 的开关沿处，不在谷值 —— 谷值是下管")
    print("  导通区间的正中，离两侧开关沿最远，本来就不在死区里。")
    print()


def window_table():
    print("=" * 74)
    print("3) 采样窗口 vs 占空比（低边采样，窗口在谷值附近）")
    print("-" * 74)
    print("  窗口（谷值→上计数沿）= CCR·tcnt = (1-d)·T/2 = (1-d)·25 µs")
    print("  真实导通 = 窗口起点 + t_on，终点 + t_off（FD6288Q 150/120 ns typ）")
    print()
    hdr = "  %5s %7s %10s %9s %9s %9s" % ("duty", "CCR", "窗口半宽", "C4=1", "C4=56", "C4=150")
    print(hdr)
    print("  " + "-" * 60)

    def margin(d, c4):
        ccr = (1.0 - d) * ARR
        # 采样点距窗口终点（含关断延时）还剩多少 ns
        return (ccr * T_CNT + T_OFF_TYP - c4 * T_CNT) * 1e9

    for d in (0.50, 0.70, 0.80, 0.90, 0.94, 0.95, 0.96, 0.97, 0.98):
        ccr = (1.0 - d) * ARR
        half = ccr * T_CNT * 1e6
        print("  %5.2f %7.0f %8.2f µs %7.0f ns %7.0f ns %7.0f ns"
              % (d, ccr, half, margin(d, 1), margin(d, 56), margin(d, 150)))

    print()
    print("  「还剩多少 ns」= 采样时刻距离低边关断的时间余量，负值表示已采在关断之后")
    print()
    # 各 CCR4 取值下能保证采样的最大占空比（要求余量 ≥ 300 ns 抗振铃）
    print("  保证 300 ns 余量的最大占空比：")
    for c4 in (1, 25, 56, 100, 150, 200):
        t_trig = c4 * T_CNT
        # CCR > (t_trig + 300ns - t_off)/tcnt
        ccr_need = (t_trig + 300e-9 - T_OFF_TYP) / T_CNT
        dmax = 1.0 - ccr_need / ARR
        print("    CCR4=%3d (采样点 %5.0f ns) → d_max = %.4f"
              % (c4, t_trig * 1e9, dmax))
    print()
    # 理想采样点：窗口正中心（含驱动延时）
    for d in (0.5, 0.95):
        ccr = (1.0 - d) * ARR
        start = -(ccr - DT_CNT) * T_CNT + T_ON_TYP
        end = ccr * T_CNT + T_OFF_TYP
        c = 0.5 * (start + end)
        print("  d=%.2f 时窗口 [%.0f, %.0f] ns，中心 %.0f ns → CCR4≈%.0f"
              % (d, start * 1e9, end * 1e9, c * 1e9, c / T_CNT))
    print()


def speed_limit(vbus=30.7, psi=4.333e-3, pp=14.0, r=0.4, iq=0.5):
    print("=" * 74)
    print("4) 占空比随转速（ψ=4.333 mWb，p=14，R=%.1f Ω，iq=%.1f A，母线 %.1f V）"
          % (r, iq, vbus))
    print("-" * 74)
    print("  %7s %9s %9s %9s %9s" % ("rpm", "BEMF(V)", "duty_max", "窗口(µs)", "可测?"))
    d95 = d100 = None
    for rpm in (500, 1000, 1500, 1800, 2000, 2140, 2200, 2300, 2400, 2500):
        wm = rpm * 2 * math.pi / 60.0
        vemf = psi * pp * wm
        vmag = math.hypot(vemf + r * iq, pp * wm * 0.0)  # 简化：只算幅值
        dmax = 0.5 + vmag / vbus
        win = (1.0 - min(dmax, 1.0)) * 25.0
        ok = "是" if dmax < 0.96 else ("临界" if dmax < 1.0 else "否")
        print("  %7d %9.2f %9.3f %9.2f %9s" % (rpm, vemf, dmax, win, ok))
        if d95 is None and dmax >= 0.95:
            d95 = rpm
        if d100 is None and dmax >= 1.0:
            d100 = rpm
    print()
    print("  → 占空比在 %s rpm 附近越过 0.95，%s rpm 附近顶到 1.0" % (d95, d100))
    print("  → 越过 0.95 后，该相低边窗口 < 1.25 µs；顶到 1.0 时该相低边永不导通，")
    print("    分流器上只剩死区期间的体二极管电流，该相电流**不可测**")
    print()


def live(port, rpms, secs):
    from cog_ff_test import Rig
    from sweep_1_1000 import set_rpm, LEN_IN, MSG_IN, x25

    LEN_OUT, MSG_OUT, CRC_OUT = 32, 30027, 58

    def cap(rig, secs):
        rows = []
        t0 = time.time()
        nxt = t0 + 0.8
        rig.s.reset_input_buffer()
        while time.time() - t0 < secs:
            d = rig.s.read(4096)
            if d:
                rig.raw += d
            if time.time() > nxt:
                rig.s.write(b"S")
                rig.s.flush()
                nxt += 0.8
            i = 0
            n = len(rig.raw)
            while i + 12 <= n:
                if rig.raw[i] != 0xFD:
                    i += 1
                    continue
                ln = rig.raw[i + 1]
                mid = rig.raw[i + 7] | (rig.raw[i + 8] << 8) | (rig.raw[i + 9] << 16)
                b = i + 10
                if b + ln + 2 > n:
                    break
                crc = rig.raw[b + ln] | (rig.raw[b + ln + 1] << 8)
                ex = 191 if mid == MSG_IN else CRC_OUT
                if mid in (MSG_IN, MSG_OUT) and x25(rig.raw[i + 1:b + ln], ex) == crc:
                    f = lambda o, bb=b: struct.unpack_from("<f", rig.raw, bb + o)[0]
                    if mid == MSG_IN and ln == LEN_IN:
                        rows.append(("in", f(24), f(28), f(32), f(36)))
                    elif mid == MSG_OUT and ln == LEN_OUT:
                        rows.append(("out", f(12), f(16)))
                    i = b + ln + 2
                else:
                    i += 1
            del rig.raw[:i]
        return rows

    print("=" * 74)
    print("5) 实测：高速段占空比与电流环表现")
    print("-" * 74)
    print("  %6s %8s %8s %8s %8s %8s %8s" %
          ("rpm", "vbus", "dmax", "窗口µs", "id均值", "idσ", "iqσ"))
    rig = Rig(port)
    try:
        time.sleep(1.0)
        rig.cmd("X", 0.5)
        for want in rpms:
            rig.cmd("X", 0.3)      # 清超速锁存（上一点可能顶到门限）
            set_rpm(rig, want)
            rig.keep(2.5)
            rows = cap(rig, 3.0)
            ins = [r for r in rows if r[0] == "in"]
            outs = [r for r in rows if r[0] == "out"]
            if not ins or not outs:
                print("  %6d  无数据" % want)
                rig.cmd("D", 0.3)
                continue
            vb = sum(r[1] for r in ins) / len(ins)
            dmax = 0.0
            for r in ins:
                for u in (r[2], r[3], r[4]):
                    dv = 0.5 + u / vb
                    dmax = max(dmax, dv)
            dclamp = min(1.0, dmax)
            win = (1.0 - dclamp) * 25.0
            ids = [r[1] for r in outs]
            iqs = [r[2] for r in outs]
            mi = sum(ids) / len(ids)
            si = math.sqrt(sum((x - mi) ** 2 for x in ids) / len(ids))
            qi = sum(iqs) / len(iqs)
            sq = math.sqrt(sum((x - qi) ** 2 for x in iqs) / len(iqs))
            print("  %6d %8.2f %8.3f %8.2f %8.3f %8.3f %8.3f"
                  % (want, vb, dmax, win, mi, si, sq))
            rig.cmd("D", 0.3)
    finally:
        rig.cmd("D", 0.3)
        rig.cmd("X", 0.3)
        rig.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--rpms", default="1500,2000,2200,2350,2450")
    ap.add_argument("--secs", type=float, default=3.0)
    a = ap.parse_args()

    front_end()
    trigger()
    window_table()
    speed_limit()
    if a.live:
        live(a.port, [int(x) for x in a.rpms.split(",")], a.secs)
