# -*- coding: utf-8 -*-
"""spd_spec.py —— 速度纹波的全谱峰值扫描（定位"为什么这个转速抖"）

判据：
  主峰落在 f = rpm/60 的整数倍 -> 转速相关的机械/齿槽成分
  主峰落在固定频率（不随 rpm 变） -> 控制振荡或结构共振
  主峰落在 24·rpm/60        -> 齿槽

用法: python spd_spec.py COM27 --rpms 150,260,300 --secs 6
"""
import argparse
import math

from cog_ff_test import Rig
from sweep_1_1000 import collect_rich


def hann(n):
    if n < 2:
        return [1.0]
    return [0.5 - 0.5 * math.cos(2.0 * math.pi * i / (n - 1)) for i in range(n)]


def amp_at(xm, w, fs, f, wsum):
    c = 2.0 * math.pi * f / fs
    re = 0.0
    im = 0.0
    for i, x in enumerate(xm):
        a = c * i
        re += x * w[i] * math.cos(a)
        im += x * w[i] * math.sin(a)
    return 2.0 * math.hypot(re, im) / wsum


def top_peaks(v, fs, lo, hi, step, keep=3):
    m = sum(v) / len(v)
    xm = [x - m for x in v]
    w = hann(len(v))
    wsum = sum(w)
    out = []
    f = lo
    while f <= hi:
        out.append((amp_at(xm, w, fs, f, wsum), f))
        f += step
    out.sort(reverse=True)
    # 去邻近（±0.5 Hz 内只留最大）
    picked = []
    for a, f in out:
        if all(abs(f - g) > 0.6 for _, g in picked):
            picked.append((a, f))
        if len(picked) >= keep:
            break
    return picked, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--rpms", default="150,260,300")
    ap.add_argument("--secs", type=float, default=6.0)
    a = ap.parse_args()

    rpms = [float(x) for x in a.rpms.split(",")]
    rig = Rig(a.port)
    try:
        print("S:", rig.cmd("S", 0.5))
        for want in rpms:
            rig.cmd("D", 0.3)
            # 复用 sweep 的稳健发送
            from sweep_1_1000 import set_rpm
            echo = set_rpm(rig, want)
            print("\n=== %.0f rpm ===  %s" % (want, echo))
            rig.keep(1.5)
            fr = collect_rich(rig, a.secs)
            rig.cmd("D", 0.3)
            if len(fr) < 64:
                print("  帧太少", len(fr))
                continue
            t = [x[0] for x in fr]
            fs = (len(t) - 1) / (t[-1] - t[0])
            v = [x[1] for x in fr]
            sd = math.sqrt(sum((x - sum(v) / len(v)) ** 2 for x in v) / len(v))
            peaks, m = top_peaks(v, fs, 1.0, min(45.0, fs * 0.45), 0.1)
            print("  fs=%.1f Hz n=%d 实测=%.2f rpm σ=%.3f (%.2f%%)"
                  % (fs, len(v), m, sd, sd / want * 100))
            print("  基频 %.2f Hz  24阶 %.2f Hz" % (want / 60.0, 24 * want / 60.0))
            for amp, f in peaks:
                tag = ""
                if abs(f % (want / 60.0)) < 0.35:
                    tag = " <- %d×基频" % round(f / (want / 60.0))
                elif abs(f - 24 * want / 60.0) < 0.6:
                    tag = " <- 24阶齿槽"
                print("    峰 %5.2f Hz  幅值 %.4f rpm%s" % (f, amp, tag))
        return 0
    finally:
        rig.cmd("D", 0.3)
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
