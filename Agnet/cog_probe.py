# -*- coding: utf-8 -*-
"""cog_probe.py —— 运行时直接看齿槽前馈的本拍输出（now=）

目的：A/B 的 σ 差异被噪声盖住了，先确认"前馈到底有没有在动"。
若 ON 时 now 峰峰约等于表的 pp（1.0 A 左右）且随角度周期变化，
说明通路正常，差异不显著就是物理事实；若 now 恒 0，那是有 bug。

用法: python cog_probe.py COM27 --rpm 100
"""
import argparse
import re
import time

from cog_ff_test import Rig, run_point

RE_NOW = re.compile(r"now=(-?\d+\.\d+)A")
RE_EN = re.compile(r"en=(\d)")


def busy_keep(rig):
    rig.s.write(b"S")
    rig.s.flush()


def probe(rig, rpm, secs):
    """跑起来后连续读 `:` 的 now=，同时保活"""
    rig.cmd("D", 0.3)
    rig.set_rpm(rpm)
    echo = rig.cmd("W", 0.45)
    if "refused" in echo or "failed" in echo:
        return None, echo
    rig.keep(1.0)

    vals = []
    en = None
    t0 = time.time()
    nxt_keep = t0 + 0.6
    while time.time() - t0 < secs:
        rig.s.reset_input_buffer()
        rig.s.write(b":")
        rig.s.flush()
        buf = bytearray()
        t1 = time.time()
        while time.time() - t1 < 0.045:
            d = rig.s.read(256)
            if d:
                buf += d
        txt = "".join(ch if 32 <= ord(ch) < 127 else "\n"
                      for ch in buf.decode("ascii", "replace"))
        m = RE_NOW.search(txt)
        if m:
            vals.append(float(m.group(1)))
        me = RE_EN.search(txt)
        if me:
            en = int(me.group(1))
        if time.time() > nxt_keep:
            busy_keep(rig)
            nxt_keep = time.time() + 0.6
        time.sleep(0.02)
    rig.cmd("D", 0.3)

    if len(vals) < 5:
        return None, "只读到 %d 个 now 样本" % len(vals)
    return dict(en=en, n=len(vals), mn=min(vals), mx=max(vals),
                pp=max(vals) - min(vals),
                mean=sum(vals) / len(vals), vals=vals), echo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--rpm", type=float, default=100.0)
    ap.add_argument("--secs", type=float, default=3.0)
    a = ap.parse_args()

    rig = Rig(a.port)
    orig_on = False
    try:
        st = rig.cmd(":", 0.5)
        print("[状态] %s" % st)
        if " v=1" not in st:
            print("!! 表无效")
            return 2
        orig_on = " en=1" in st

        for want in (False, True):
            st = rig.cmd(":", 0.3)
            if (" en=1" in st) != want:
                rig.cmd(";", 0.3)
            r, echo = probe(rig, a.rpm, a.secs)
            print("\n=== 前馈 %s @ %.0f rpm ===" % ("ON" if want else "OFF", a.rpm))
            if r is None:
                print("  失败:", echo)
                continue
            print("  now: n=%d en=%s min=%+.3f max=%+.3f pp=%.3f mean=%+.3f A"
                  % (r["n"], r["en"], r["mn"], r["mx"], r["pp"], r["mean"]))
            print("  样本:", " ".join("%+.2f" % v for v in r["vals"][:40]))
        return 0
    finally:
        rig.cmd("D", 0.3)
        st = rig.cmd(":", 0.3)
        if (" en=1" in st) != orig_on:
            rig.cmd(";", 0.3)
        print("\n[收尾] %s" % rig.cmd(":", 0.4))
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
