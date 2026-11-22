# -*- coding: utf-8 -*-
"""电流环 Kp 档位 A/B：阶跃响应 + 零电流噪声。

目的：直接回答"Kp 用 0.534（手写/实测 L=85u）还是 0.713（模型/假定 L=113.5u）更好"。
1.25× 档对应 0.6677，最接近模型的 0.71314（档位步进只有 0.25，无法精确到 1.335×）。

用法: python cur_ab.py COM27
"""
import argparse
import math
import re
import time

from cog_ff_test import Rig
from cur_eval import RE_STEP, read_cmd

SCALES = ["x1.00", "x1.25", "x1.50"]


def goto_scale(rig, tag):
    for _ in range(7):
        r = rig.cmd("G", 0.45)
        if tag in r:
            return r
    return r


def step_once(rig):
    rig.s.reset_input_buffer()
    rig.s.write(b"U")
    rig.s.flush()
    out = {}
    for ln in read_cmd(rig, 1.8):
        m = RE_STEP.search(ln)
        if m:
            out = dict(tr=float(m.group(3)), os=float(m.group(4)),
                       ts=float(m.group(5)), ss=float(m.group(6)))
    return out


def step_retry(rig, tries=3):
    """STEP 行会被上一条回显顶掉（s_out 单槽），失败就重试"""
    for _ in range(tries):
        r = step_once(rig)
        if r:
            return r
        time.sleep(0.4)
    return None


def noise_sd(rig, secs=0.4):
    """`&` 录 1024 点 iq @20 kHz，返回 sd（A）。用 ISR 内当拍值，避开遥测偏差。"""
    rig.s.reset_input_buffer()
    rig.s.write(b"&")
    rig.s.flush()
    hexs = []
    t0 = time.time()
    while time.time() - t0 < 2.5:
        d = rig.s.read(4096)
        if not d:
            continue
        txt = "".join(c if 32 <= ord(c) < 127 else "\n" for c in d.decode("ascii", "replace"))
        for ln in txt.split("\n"):
            ln = ln.strip()
            if ln.startswith("[REC]"):
                h = ln[5:]
                for i in range(0, len(h) - 3, 4):
                    v = int(h[i:i + 4], 16)
                    if v >= 0x8000:
                        v -= 0x10000
                    hexs.append(v / 131.0)
        if len(hexs) >= 1024:
            break
    if len(hexs) < 100:
        return float("nan")
    xs = hexs[:1024]
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--rep", type=int, default=3)
    a = ap.parse_args()

    rig = Rig(a.port)
    try:
        time.sleep(2.0)
        rig.cmd("D", 0.4)
        rig.cmd("X", 0.4)
        print("  %-8s %8s %8s %9s %8s %9s" %
              ("Kp档", "Kp", "tr(us)", "os(%)", "ts(us)", "零流sd(A)"))
        for tag in SCALES:
            goto_scale(rig, tag)
            time.sleep(0.7)                 # 等 G 的回显落地，否则会顶掉 STEP 行
            trs, oss, tss, sss = [], [], [], []
            for _ in range(a.rep):
                r = step_retry(rig)
                if r:
                    trs.append(r["tr"]); oss.append(r["os"])
                    tss.append(r["ts"]); sss.append(r["ss"])
                time.sleep(0.3)
            # 零电流噪声：力矩模式给 0，录 iq
            rig.cmd("D", 0.3)
            rig.cmd(">0\r", 0.4)
            rig.keep(0.8)
            sd = noise_sd(rig)
            rig.cmd("D", 0.3)
            f = lambda v: (sum(v) / len(v)) if v else float("nan")
            print("  %-8s %8.4f %8.0f %8.1f %9.0f %9.4f" %
                  (tag, 0.5341 * float(tag[1:]), f(trs), f(oss), f(tss), sd))
        goto_scale(rig, "x1.00")
        time.sleep(0.5)
        st = rig.cmd("S", 0.8)
        if "tx=" in st or "ue=" in st:
            print("  通道健康:", st[st.find("tx=") if "tx=" in st else st.find("ue="):])
    finally:
        rig.cmd("D", 0.3)
        rig.close()


if __name__ == "__main__":
    main()
