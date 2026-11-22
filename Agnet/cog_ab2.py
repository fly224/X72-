# -*- coding: utf-8 -*-
"""cog_ab2.py —— 齿槽前馈 A/B 交错重复测量（用现有表，不重测、不写 Flash）

为什么交错：温漂、母线电压、机械预紧都会随几分钟漂移。若先测完所有 OFF
再测所有 ON，漂移会整段算到前馈头上。交错（OFF/ON/OFF/ON...）让漂移
平摊到两侧，只留下真正的差异。

输出：每点均值 ± 标准差，以及配对改善率（同一轮 OFF→ON 的比值均值）。

用法: python cog_ab2.py COM27 --rpms 3,100 --repeat 3
"""
import argparse
import math

from cog_ff_test import Rig, run_point


def mean(xs):
    return sum(xs) / len(xs)


def sd(xs):
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--rpms", default="3,100")
    ap.add_argument("--repeat", type=int, default=3)
    a = ap.parse_args()

    rig = Rig(a.port)
    rpms = [float(x) for x in a.rpms.split(",")]
    pairs = {r: [] for r in rpms}       # rpm -> [(off_sd, on_sd, off_ss, on_ss)]
    orig_on = False
    try:
        st = rig.cmd(":", 0.5)
        print("[状态] %s" % st)
        if " v=1" not in st:
            print("!! 表无效，需先 `}`")
            return 2
        orig_on = " en=1" in st
        print("[原始开关] %s，重复 %d 轮\n" % ("ON" if orig_on else "OFF", a.repeat))

        for rep in range(a.repeat):
            for rpm in rpms:
                row = {}
                for want in (False, True):
                    st = rig.cmd(":", 0.3)
                    if (" en=1" in st) != want:
                        rig.cmd(";", 0.3)
                    r, echo = run_point(rig, rpm, a.secs)
                    if r is None:
                        print("  rep%d %5.0f rpm %s 失败: %s"
                              % (rep + 1, rpm, "ON" if want else "OFF", echo))
                        continue
                    row[want] = r
                if len(row) == 2:
                    o, n = row[False], row[True]
                    pairs[rpm].append((o["sd"], n["sd"], o["ss"], n["ss"]))
                    print("  rep%d %5.0f rpm  OFF sd=%6.3f ss=%8.3f | "
                          "ON sd=%6.3f ss=%8.3f | %+6.1f%%"
                          % (rep + 1, rpm, o["sd"], o["ss"], n["sd"], n["ss"],
                             (o["sd"] - n["sd"]) / o["sd"] * 100 if o["sd"] else 0.0))
            print("")

        print("=== 汇总（%d 轮交错）===" % a.repeat)
        print("    rpm   OFF_sd(mean±sd)      ON_sd(mean±sd)      改善(配对均值)")
        for rpm in rpms:
            ps = pairs[rpm]
            if not ps:
                continue
            offs = [p[0] for p in ps]
            ons = [p[1] for p in ps]
            imps = [(p[0] - p[1]) / p[0] * 100 for p in ps if p[0]]
            print("  %5.0f   %6.3f ± %5.3f      %6.3f ± %5.3f     %+6.1f%% "
                  "(各轮 %s)" % (rpm, mean(offs), sd(offs), mean(ons), sd(ons),
                                 mean(imps),
                                 " ".join("%+.1f" % x for x in imps)))
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
