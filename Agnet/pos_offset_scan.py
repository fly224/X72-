# -*- coding: utf-8 -*-
"""pos_offset_scan.py —— 定位偏差 vs 行程 扫描

目的：目标补偿不能只用一个常数——实测 90° 摆动只偏 2"，20 圈偏 4120"。
先量出"偏差(d)"的关系，再决定补偿模型（常数 / 位置查表 / 行程查表）。

方法：固定起点 A，对每个行程 d 做"正向到达 + 反向到达"各 n 次，
取稳态偏差均值。每次到达后回 A，避免多圈累积。

用法：
    python pos_offset_scan.py COM27
    python pos_offset_scan.py COM27 --n 2 --d 1.57,6.28,25.13,125.66
"""
import argparse
import math
import os
import re
import statistics
import sys
import time

from cog_ff_test import Rig
from pos_test import ask, collect

AS = 206265.0
TWO_PI = 2.0 * math.pi


def goto(rig, tgt, settle=6.0, tail=0.32):
    cmd = b'P' + ('%.4f' % tgt).encode() + b'\r'
    rig.s.write(cmd)
    rig.s.flush()
    t0 = time.time()
    nxt = t0 + 1.35
    while time.time() - t0 < settle - tail:
        if time.time() >= nxt:
            rig.s.write(cmd)          # 重发同一目标：只续 deadman，不扰动规划器
            rig.s.flush()
            nxt += 1.35
        time.sleep(0.02)
    p, _ = collect(rig, tail)
    return (sum(p[-5:]) / 5) if p else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--n", type=int, default=2)
    ap.add_argument("--d", default="1.5708,3.1416,6.2832,12.566,25.133,50.265,125.664")
    a = ap.parse_args()

    ds = [float(x) for x in a.d.split(",")]
    rig = Rig(a.port)
    print("=" * 76)
    print("定位偏差 vs 行程   port=%s  n=%d" % (a.port, a.n))
    print("=" * 76)

    line = ask(rig, b"P\r", 1.0)
    print("[1] " + line)
    m = re.search(r"tgt=(-?\d+\.\d+)", line)
    if not m:
        line = ask(rig, b"P\r", 1.0)
        print("    retry: " + line)
        m = re.search(r"tgt=(-?\d+\.\d+)", line)
    if not m:
        print("    [ABORT] 读不到 tgt")
        rig.close()
        return 3
    A = float(m.group(1))
    p, _ = collect(rig, 0.4)
    enc0 = sum(p[-5:]) / 5
    off = A - enc0
    print("    A=%.4f  off=%.4f" % (A, off))

    rows = []
    print("\n[2] 扫描")
    print("   d[rad]   d[圈]   fwd偏差      rev偏差     |偏差|均值")
    for d in ds:
        fw, rv = [], []
        for _ in range(a.n):
            e = goto(rig, A + d, 6.0)
            if e is not None:
                fw.append((e + off - (A + d)) * AS)
            goto(rig, A, 6.0)
            e = goto(rig, A - d, 6.0)
            if e is not None:
                rv.append((e + off - (A - d)) * AS)
            goto(rig, A, 6.0)
        mf = statistics.mean(fw) if fw else float("nan")
        mr = statistics.mean(rv) if rv else float("nan")
        # 两个方向都"少走"时 fwd 为负、rev 为正，取绝对值均值衡量"少走量"
        mag = (abs(mf) + abs(mr)) / 2
        print("  %8.4f  %5.2f  %+9.1f  %+11.1f   %8.1f"
              % (d, d / TWO_PI, mf, mr, mag))
        rows.append((d, mf, mr, mag))

    rig.s.write(b"D")
    rig.s.flush()
    time.sleep(0.2)
    rig.close()

    if len(rows) >= 3:
        print("\n[3] 规律")
        d0, _, _, m0 = rows[0]
        d1, _, _, m1 = rows[-1]
        print("  最小行程 %.2f 圈 -> %.1f\"   最大行程 %.2f 圈 -> %.1f\""
              % (d0 / TWO_PI, m0, d1 / TWO_PI, m1))
        ratio_d = d1 / d0
        ratio_e = (m1 / m0) if m0 > 1 else float("inf")
        print("  行程比 %.1f×  ->  偏差比 %.1f×" % (ratio_d, ratio_e))
        if ratio_e < ratio_d * 0.3:
            print("  => 偏差不是线性于行程，更像'超过某个速度/动能门槛后被齿槽锁住'")
        print("  建议：若要补偿，按行程分档（短行程不补，长行程补）")
    print("=" * 76)
    return 0


if __name__ == "__main__":
    sys.exit(main())
