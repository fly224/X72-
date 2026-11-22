# -*- coding: utf-8 -*-
"""cur_sweep.py —— 电流环 Kp 档位扫描（用 id 阶跃，转子不动）

背景：实测 tr=150 µs（设计 350 µs）、os=6.6%、**ts=29.4 ms**，
波形显示 id 在整个 30 ms 阶跃期间持续在 1.74~2.28 之间摆动（≈4 kHz）。
这不是"响应快"，是相位裕度不足 —— 实际带宽约为设计值的 2.3 倍。

扫描目的：找出"既不振荡、又足够快"的 Kp 档位。
判据优先级：ts（长期振荡会把它拖到几十 ms）> os > tr。

用法:
    python cur_sweep.py COM27
    python cur_sweep.py COM27 --rep 3
"""
import argparse
import re
import time

from cog_ff_test import Rig
from cur_eval import RE_STEP, read_cmd

SCALES = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0]


def goto_scale(rig, target):
    """G 是循环键，转一圈最多 6 次；回显里带 'x0.50' 之类的标记"""
    tag = "x%.2f" % target
    for _ in range(7):
        r = rig.cmd("G", 0.45)
        if tag in r:
            return r
    return r


def step_once(rig):
    rig.s.reset_input_buffer()
    rig.s.write(b"U")
    rig.s.flush()
    lines = read_cmd(rig, 1.6)
    for ln in lines:
        m = RE_STEP.search(ln)
        if m:
            return dict(tr=int(m.group(3)), os=float(m.group(4)),
                        ts=int(m.group(5)), ss=float(m.group(6)),
                        pk=float(m.group(7)), vp=float(m.group(8)))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--rep", type=int, default=2)
    a = ap.parse_args()

    rig = Rig(a.port)
    rows = {}
    try:
        print("S:", rig.cmd("S", 0.6))
        rig.cmd("X", 0.5)
        # Ki 归到 1.0（H 同样是循环键）
        for _ in range(7):
            if "x1.00" in rig.cmd("H", 0.45):
                break

        print("\n%8s %8s %8s %9s %8s %8s" %
              ("Kp x", "tr(us)", "os(%)", "ts(us)", "ss(%)", "vp(V)"))
        print("-" * 56)
        for sc in SCALES:
            echo = goto_scale(rig, sc)
            got = []
            for _ in range(a.rep):
                st = step_once(rig)
                if st:
                    got.append(st)
                time.sleep(0.25)
            if not got:
                print("%8.2f   失败" % sc)
                continue
            mean = lambda k: sum(g[k] for g in got) / len(got)
            rows[sc] = dict(tr=mean("tr"), os=mean("os"), ts=mean("ts"),
                            ss=mean("ss"), vp=mean("vp"))
            print("%8.2f %8.0f %8.2f %9.0f %8.2f %8.2f"
                  % (sc, rows[sc]["tr"], rows[sc]["os"], rows[sc]["ts"],
                     rows[sc]["ss"], rows[sc]["vp"]))

        rig.cmd("D", 0.3)

        if rows:
            print("\n=== 判读 ===")
            ok = {k: v for k, v in rows.items() if v["ts"] < 3000}
            if ok:
                best = min(ok.items(), key=lambda kv: kv[1]["tr"])
                print("  ts < 3ms（无持续振荡）的档位里，最快的是 %.2fx："
                      "tr=%.0fus os=%.1f%% ts=%.0fus"
                      % (best[0], best[1]["tr"], best[1]["os"], best[1]["ts"]))
            else:
                print("  !! 所有档位 ts 都 > 3 ms，说明振荡不是单纯增益过高，"
                      "要查采样/计算延迟或离散化")
            print("  参考：设计 Kp 对应 wc=2π·1000，二阶系统 tr(10-90%)≈350 µs")
        return 0
    finally:
        try:
            rig.cmd("D", 0.3)
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
