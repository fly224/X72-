# -*- coding: utf-8 -*-
"""cur_eval.py —— 电流环动态性能评估（用片内 CurStep，不依赖 100 Hz 遥测）

为什么用 id 阶跃而不是 iq：
  id 沿转子磁链方向，表贴式（Ld=Lq）下不产生力矩，转子不动。
  用 iq 打 2 A 阶跃，30 ms 内转速会冲到 900+ rpm，直接撞 300 rpm 超速保护。
  id 与 iq 共用同一套 PI 和限幅，动态指标等价。

指标来自 cur_step.c 的片内计算（20 kHz）：
  tr  10%→90% 上升时间      理论值 ≈ 0.35/ωc，wc=1 kHz 时 ≈ 350 µs
  os  超调                 目标 < 10%（一阶+PI 零极点对消应无超调）
  ts  进入 ±5% 的调节时间    目标 ≈ 3~5 倍 tr
  ss  稳态误差               目标 < 1%（PI 有积分，应该接近 0）
  vp  峰值电压              与 0.5375·vbus 比较，判断是否触限

用法:
    python cur_eval.py COM27
    python cur_eval.py COM27 --rep 5
"""
import argparse
import re
import time

from cog_ff_test import Rig

RE_STEP = re.compile(
    r"\[STEP\] (\w+) amp=([\d.]+) tr=(\d+)us os=([-\d.]+)% ts=(\d+)us "
    r"ss=([-\d.]+)% pk=([-\d.]+) vp=([-\d.]+)")


def read_cmd(rig, secs=1.5):
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < secs:
        d = rig.s.read(1024)
        if d:
            buf += d
    txt = "".join(ch if 32 <= ord(ch) < 127 else "\n"
                  for ch in buf.decode("ascii", "replace"))
    return [ln.strip() for ln in txt.split("\n")
            if ln.strip().startswith(("[STEP]", "[TRC]", "[CMD]"))]


def run_step(rig, ch, wait=1.6):
    rig.s.reset_input_buffer()
    rig.s.write(ch.encode())
    rig.s.flush()
    lines = read_cmd(rig, wait)
    step = None
    trc = []
    for ln in lines:
        m = RE_STEP.search(ln)
        if m:
            step = dict(axis=m.group(1), amp=float(m.group(2)),
                        tr=int(m.group(3)), os=float(m.group(4)),
                        ts=int(m.group(5)), ss=float(m.group(6)),
                        pk=float(m.group(7)), vp=float(m.group(8)))
        elif ln.startswith("[TRC]"):
            p = ln.split()
            if len(p) >= 6:
                trc.append((float(p[1]), float(p[2]), float(p[3]),
                            float(p[4]), float(p[5])))
    return step, trc, [ln for ln in lines if ln.startswith("[CMD]")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--rep", type=int, default=4)
    ap.add_argument("--axis", default="u", help="u=id 阶跃（安全）, t=iq 阶跃（会加速）")
    a = ap.parse_args()

    rig = Rig(a.port)
    rows = []
    try:
        print("S:", rig.cmd("S", 0.6))
        print("X:", rig.cmd("X", 0.5))
        k = rig.cmd("K", 0.6)
        print("spd:", k)

        for i in range(a.rep):
            st, trc, cmds = run_step(rig, a.axis)
            for c in cmds:
                if "STEP" in c or "busy" in c or "refused" in c:
                    print("   ", c)
            if st is None:
                print("  rep%d 无结果" % (i + 1))
                continue
            rows.append(st)
            print("  rep%d %s amp=%.2f  tr=%4dus  os=%6.2f%%  ts=%4dus  ss=%6.2f%%  "
                  "pk=%.2f  vp=%.2f"
                  % (i + 1, st["axis"], st["amp"], st["tr"], st["os"],
                     st["ts"], st["ss"], st["pk"], st["vp"]))
            if trc and i == 0:
                print("     波形(t ms, id_ref, id_now, iq_ref, iq_now):")
                for p in trc[:16]:
                    print("      %.2f  %+.3f  %+.3f  %+.3f  %+.3f" % p)
            time.sleep(0.3)

        rig.cmd("D", 0.3)

        if rows:
            mean = lambda xs: sum(xs) / len(xs)
            print("\n=== 汇总（%d 次）===" % len(rows))
            print("  上升时间 tr = %d µs  （1 kHz 带宽理论 ≈ 350 µs）"
                  % mean([r["tr"] for r in rows]))
            print("  超调     os = %.2f %%   （一阶+极点对消应 <10%%）"
                  % mean([r["os"] for r in rows]))
            print("  调节时间 ts = %d µs" % mean([r["ts"] for r in rows]))
            print("  稳态误差 ss = %.2f %%" % mean([r["ss"] for r in rows]))
            print("  峰值电压 vp = %.2f V（限幅 0.5375×%.2f = %.2f V）"
                  % (mean([r["vp"] for r in rows]), 30.77, 0.5375 * 30.77))
        return 0
    finally:
        try:
            rig.cmd("D", 0.3)
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
