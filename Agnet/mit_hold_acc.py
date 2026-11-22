#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""MIT 保持精度：多点目标下的稳态误差（齿槽 / 摩擦的位置相关性）。

为什么这么测：
  齿槽是**位置的函数**，零速保持时不会形成纹波，而是让"停位"在一个齿槽周期内
  随角度起伏。用固定目标测不到它 —— 必须在多个目标角上各停一次，看误差随
  角度变化。齿槽周期 = 2π/24 ≈ 15°(0.262 rad)，取 0.083 rad 步长 ≈ 每周期 3 点。

用法：
  python mit_hold_acc.py COM27 --span 1.0 --steps 12
  python mit_hold_acc.py COM27 --span 1.0 --steps 12 --cog off
"""
import argparse
import math
import statistics
import time

from cog_ff_test import Rig
from mit_test import pack
from pos_test import ask, collect


def send_hold(rig, kp, kd, p_des, tau=0.0, dur=0.8):
    """持续发同一帧（1 kHz 上限），期间保活"""
    t0 = time.time()
    nxt = t0 + 0.5
    f = pack(p_des, 0.0, kp, kd, tau)
    while time.time() - t0 < dur:
        rig.s.write(f)
        rig.s.flush()
        time.sleep(0.0015)
        if time.time() > nxt:
            rig.s.write(b"S")
            rig.s.flush()
            nxt += 0.5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--span", type=float, default=0.8, help="扫过的角度范围 rad")
    ap.add_argument("--steps", type=int, default=16)
    ap.add_argument("--kp", type=float, default=3.0)
    ap.add_argument("--kd", type=float, default=0.05)
    ap.add_argument("--cog", choices=["on", "off", "as-is"], default="as-is")
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
        print("S:", st[:100])
        if "cal=1" not in st or "z=1" not in st:
            print("!! 需要先 C 和 Z")
            return

        if a.cog != "as-is":
            r = ask(rig, ";", 0.6)
            print("   cog toggle:", r)
            # 确认状态
            c = ask(rig, ":", 0.6)
            print("   cog state:", c[:60])

        # 进入 MIT：**发帧即可**（mit_apply 的第一帧负责起环 + 放 MOE），
        # 不需要 > / M 之类。先给 kp=0 的一帧，让环路起来但不发力。
        print("[0] 进入 MIT（发帧）")
        print("   X:", ask(rig, "X", 0.6)[:50])
        send_hold(rig, 0.0, 0.0, 0.0, dur=0.4)
        r = ask(rig, "S", 0.6)
        if "f=0x00" not in r:
            print("!! 故障未清：", r[r.find("f="):][:20])
            return

        # 用 S 行读当前 p（f3，1 mrad；只作基准，误差是相对量所以够）
        import re
        p0 = None
        for _ in range(5):
            r = ask(rig, "S", 0.6)
            m = re.search(r"mit T=\S+ p=(-?\d+\.\d+)", r)
            if m:
                p0 = float(m.group(1))
                break
            time.sleep(0.2)
        if p0 is None:
            print("!! 读不到 MIT 位置")
            return
        print("    基准 p0=%.3f rad  kp=%.1f kd=%.3f  cog=%s"
              % (p0, a.kp, a.kd, a.cog))

        # 先稳定住
        send_hold(rig, a.kp, a.kd, p0, dur=1.2)

        step = a.span / a.steps
        errs = []
        print("   目标偏移  实测偏移   误差(µrad)")
        for i in range(a.steps + 1):
            d = step * i
            send_hold(rig, a.kp, a.kd, p0 + d, dur=0.85)
            pts, _ = collect(rig, 0.25)
            if not pts:
                print("    %.3f  采样失败" % d)
                continue
            n = max(5, len(pts) // 3)
            act = sum(pts[-n:]) / n
            errs.append((d, act))
            print("   %+.4f   %+.4f   (待算)" % (d, act))

        # 用第一个点作参考，算相对误差
        if len(errs) < 3:
            print("!! 数据不足")
            return
        d0, a0 = errs[0]
        rel = [(d, (av - a0) - (d - d0)) for d, av in errs]
        print("\n   目标偏移   相对误差(µrad)")
        for d, e in rel:
            print("   %+.3f     %+8.1f" % (d, e * 1e6))
        vals = [e for _, e in rel]
        pp = max(vals) - min(vals)
        sd = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        print("\n   相对误差：mean=%+.1f µrad  sd=%.1f µrad  **峰峰=%.1f µrad (%.4f deg)**"
              % (statistics.mean(vals) * 1e6, sd * 1e6, pp * 1e6, pp * 57.2958))
        print("   齿槽周期 2π/24 = 0.2618 rad，跨度 %.2f rad ≈ %.1f 个周期"
              % (a.span, a.span / 0.2618))

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
