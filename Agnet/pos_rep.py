#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""位置环精度与重复定位精度测试（2026-11-17）。

测什么：
  用编码器自身读数评估"指令位置 → 实际停位"的一致性，分三项：
    1. 定位精度（accuracy）      ：多次到达的均值与目标之差
    2. 单向重复定位精度           ：每次从**同一方向**接近目标，到达点的散布
    3. 双向重复定位精度           ：正/反两个方向交替接近，除散布外还包含
                                   "正反向到达点之差"（反映传动链空程）

不测什么（重要）：
  编码器装在**电机侧**，减速器背隙在输出端，这里量不到。所以本结果反映的是
  "控制环把电机停准的能力"，不等于"关节输出端停在哪儿"。输出端精度需要
  输出端编码器或外部测量（激光干涉仪/千分表）。

用法：
  python pos_rep.py COM27 --delta 1.0 --n 10
  python pos_rep.py COM27 --delta 0.3 --n 8 --approach 0.5
"""
import argparse
import math
import re
import statistics
import time

from cog_ff_test import Rig
from pos_test import ask, read_cmd, collect, stats


def goto(rig, target, dwell=1.2, tail=0.35, off=0.0):
    """定位到 target 并返回稳定后的位置（取末尾若干帧均值）。

    off 把 30040 的 enc_sum 坐标换算到控制器坐标：
    enc_sum 是"相对上电点"的累积量，控制器用 BspServo_GetThetaMulti()（带绝对零点），
    两者差一个常量。不换算的话"相对目标偏差"会差出整圈数。"""
    rig.s.write(b"P")
    rig.s.flush()
    time.sleep(0.04)
    for ch in ("%.4f" % target):
        rig.s.write(ch.encode())
        rig.s.flush()
        time.sleep(0.009)
    rig.s.write(b"\r")
    rig.s.flush()

    # 驻留 + 保活（deadman 2 s，必须持续喂 S）
    t0 = time.time()
    while time.time() - t0 < dwell:
        time.sleep(0.45)
        rig.s.write(b"S")
        rig.s.flush()

    pts, _ = collect(rig, tail)
    if not pts:
        return None
    n = max(5, len(pts) // 3)
    return sum(pts[-n:]) / n + off


def pose(rig):
    """读一次当前位置（控制器坐标系，用于取基准）"""
    r = ask(rig, b"P\r", 0.8)

    m = re.search(r"tgt=(-?\d+\.\d+)", r)
    return float(m.group(1)) if m else None


def summarize(name, vals, target=None):
    if not vals:
        print("  %s: 无数据" % name)
        return None
    m = statistics.mean(vals)
    sd = statistics.pstdev(vals) if len(vals) > 1 else 0.0
    rng = max(vals) - min(vals)
    out = dict(mean=m, sd=sd, rng=rng, n=len(vals))
    print("  %-10s n=%2d  mean=%.6f  sd=%.6f  range=%.6f  ±3σ=%.6f"
          % (name, len(vals), m, sd, rng, 3 * sd))
    print("             %s" % (" ".join("%+.1f" % ((v - m) * 1e6) for v in vals) + "  (µrad，相对均值)"))
    if target is not None:
        dev = m - target
        out["dev"] = dev
        print("             相对目标 %+.6f rad = %+.4f deg = %+.1f µrad"
              % (dev, dev * 57.2958, dev * 1e6))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--delta", type=float, default=1.0, help="目标相对基准的位移 rad")
    ap.add_argument("--approach", type=float, default=0.5, help="接近前先退开的距离 rad")
    ap.add_argument("--n", type=int, default=10, help="每个方向的次数")
    ap.add_argument("--dwell", type=float, default=1.2, help="每点驻留 s")
    a = ap.parse_args()

    rig = Rig(a.port)
    try:
        time.sleep(2.0)
        st = ""
        for _ in range(5):
            st = ask(rig, "S", 0.8)
            if st != "(no echo)":
                break
            time.sleep(0.3)
        print("S:", st[:120])
        if "cal=1" not in st or "z=1" not in st:
            print("!! 需要先 C 和 Z")
            return

        print("\n[0] 进入手写位置环保持")
        print("   ", ask(rig, "E", 0.9))
        p_base = pose(rig)
        if p_base is None:
            print("!! 拿不到基准位置")
            return
        tgt = p_base + a.delta

        # enc_sum（相对上电点）→ 控制器坐标 的常量偏移
        pts0, _ = collect(rig, 0.35)
        if not pts0:
            print("!! 采不到位置帧")
            return
        p0_enc = sum(pts0[-10:]) / len(pts0[-10:])
        off = p_base - p0_enc
        print("    基准 p=%.6f  目标=%.6f rad (+%.4f)  坐标偏移=%+.4f"
              % (p_base, tgt, a.delta, off))

        print("\n[1] 单向重复定位（每次从 %+.3f 侧接近）" % (-a.approach))
        uni = []
        for i in range(a.n):
            if goto(rig, tgt - a.approach, a.dwell * 0.7, off=off) is None:
                print("    第 %d 次预置失败" % (i + 1))
                continue
            p = goto(rig, tgt, a.dwell, off=off)
            if p is not None:
                uni.append(p)
                print("    %2d: %.6f  (偏差 %+.1f µrad)" % (i + 1, p, (p - tgt) * 1e6))
        print()
        s_uni = summarize("单向", uni, tgt)

        print("\n[2] 双向重复定位（正/反交替接近）")
        pos, neg = [], []
        for i in range(a.n):
            goto(rig, tgt - a.approach, a.dwell * 0.7, off=off)
            p = goto(rig, tgt, a.dwell, off=off)
            if p is not None:
                pos.append(p)
                print("    %2d 正向: %.6f  (偏差 %+.1f µrad)" % (i + 1, p, (p - tgt) * 1e6))

            goto(rig, tgt + a.approach, a.dwell * 0.7, off=off)
            p = goto(rig, tgt, a.dwell, off=off)
            if p is not None:
                neg.append(p)
                print("    %2d 反向: %.6f  (偏差 %+.1f µrad)" % (i + 1, p, (p - tgt) * 1e6))
        print()
        s_pos = summarize("正向", pos, tgt)
        s_neg = summarize("反向", neg, tgt)

        allbi = pos + neg
        print()
        s_all = summarize("双向合并", allbi, tgt)
        if s_pos and s_neg:
            gap = s_pos["mean"] - s_neg["mean"]
            print("\n  双向差（正向均值 − 反向均值）= %+.6f rad = %+.2f µrad = %.4f deg"
                  % (gap, gap * 1e6, gap * 57.2958))
            print("  该值主要反映传动链空程与摩擦滞环；编码器在电机侧，"
                  "不含减速器输出端背隙。")

        print("\n--- 退出 ---")
        ask(rig, "D", 0.6)
    finally:
        try:
            ask(rig, "D", 0.4)
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    main()
