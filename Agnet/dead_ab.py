# -*- coding: utf-8 -*-
"""dead_ab.py —— 死区补偿 A/B：du=0 vs du=理论值，看低速平稳性

为什么用速度纹波而不是 iq 谐波：
  iq 的 6 次脉动需要 ≥100 Hz 采样才看得清，而 1024 点 @20 kHz 只有 51 ms
  （100 rpm 才 1.2 个电周期），幅值估计的置信度太差。
  速度纹波用 100 Hz 遥测，采集窗口 3 s，同一工况重复 3 轮 —— 前面验证过
  长驻留时 σ 的重复性很好（300 rpm 六窗 7.19~7.53）。

交错测量：OFF/ON/OFF/ON，让温漂平摊到两侧。

用法:
    python dead_ab.py COM27 --rpms 3,10,30 --du 59 --repeat 3
"""
import argparse
import math

from cog_ff_test import Rig
from dead_sweep import query_dead
from sweep_1_1000 import collect_rich, set_rpm
from torque_test import send_num


def run_point(rig, rpm, settle, secs):
    set_rpm(rig, rpm)
    rig.keep(settle)
    fr = collect_rich(rig, secs)
    rig.cmd("D", 0.3)
    if len(fr) < 64:
        return None
    n = max(6, int(len(fr) * 0.5))
    v = [x[1] for x in fr[-n:]]
    iq = [x[2] for x in fr[-n:]]
    m = sum(v) / len(v)
    s = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
    return dict(mean=m, sd=s, iq=sum(iq) / len(iq), n=len(fr))


def set_du(rig, du):
    """du 单位 0.01%；0 表示"补偿开着但量为 0" = 等价关闭"""
    send_num(rig, "\\", du, dec=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--rpms", default="3,10,30")
    ap.add_argument("--du", type=float, default=59.0, help="ON 侧的补偿量，0.01%")
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--settle", type=float, default=2.5)
    ap.add_argument("--secs", type=float, default=3.0)
    a = ap.parse_args()

    rpms = [float(x) for x in a.rpms.split(",")]
    rig = Rig(a.port)
    pairs = {r: [] for r in rpms}
    try:
        print("S:", rig.cmd("S", 0.6))
        en, du, line = query_dead(rig)
        print("dead 初始:", line)
        for _ in range(3):
            if en == 1:
                break
            rig.cmd("B", 0.4)
            en, du, line = query_dead(rig)
        if en != 1:
            print("!! 死区补偿打不开")
            return 2
        print("dead 已开:", line)
        print("ON 侧 du = %.0f (%.2f%%)\n" % (a.du, a.du * 0.01))

        for rep in range(a.repeat):
            for rpm in rpms:
                got = {}
                for want in (False, True):
                    set_du(rig, a.du if want else 0.0)
                    r = run_point(rig, rpm, a.settle, a.secs)
                    if r is None:
                        print("  rep%d %5.0f rpm %s 数据不足" % (rep + 1, rpm, want))
                        continue
                    got[want] = r
                if len(got) == 2:
                    o, n = got[False], got[True]
                    pairs[rpm].append((o["sd"], n["sd"], o["iq"], n["iq"]))
                    print("  rep%d %5.0f rpm  OFF σ=%6.3f (iq %.3f) | ON σ=%6.3f (iq %.3f) | %+6.1f%%"
                          % (rep + 1, rpm, o["sd"], o["iq"], n["sd"], n["iq"],
                             (o["sd"] - n["sd"]) / o["sd"] * 100 if o["sd"] else 0))
            print("")
        set_du(rig, 59.0)

        print("=== 汇总（%d 轮交错）===" % a.repeat)
        print("    rpm   OFF σ        ON σ         改善(配对均值)   各轮")
        for rpm in rpms:
            ps = pairs[rpm]
            if not ps:
                continue
            offs = [p[0] for p in ps]
            ons = [p[1] for p in ps]
            imps = [(p[0] - p[1]) / p[0] * 100 for p in ps if p[0]]
            mean = lambda x: sum(x) / len(x)
            print("  %5.0f   %.3f        %.3f        %+6.1f%%         %s"
                  % (rpm, mean(offs), mean(ons), mean(imps),
                     " ".join("%+.0f" % x for x in imps)))
        return 0
    finally:
        rig.cmd("D", 0.3)
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
