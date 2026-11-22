# -*- coding: utf-8 -*-
"""cog_ab.py —— 用现有齿槽表做 A/B 开关对比（不重测 `}`、不写 Flash）

用法: python cog_ab.py COM27
      python cog_ab.py COM27 --rpms 3,30,100 --secs 4
"""
import argparse

from cog_ff_test import Rig, run_point


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--rpms", default="3,30,100")
    a = ap.parse_args()

    rig = Rig(a.port)
    rpms = [float(x) for x in a.rpms.split(",")]
    res = {}
    orig_on = False
    ok = True
    try:
        st = rig.cmd(":", 0.5)
        print("[状态] %s" % st)
        if " v=1" not in st:
            print("!! 表无效或零点已变，需先测 `}` 重建表")
            ok = False
            return 2
        orig_on = " en=1" in st
        print("[原始开关] %s" % ("ON" if orig_on else "OFF"))

        for want in (False, True):
            st = rig.cmd(":", 0.3)
            if (" en=1" in st) != want:
                print("   %s" % rig.cmd(";", 0.3))
            print("\n=== 前馈 %s ===" % ("ON" if want else "OFF"))
            for rpm in rpms:
                r, echo = run_point(rig, rpm, a.secs)
                if r is None:
                    print("  %5.0f rpm 失败: %s" % (rpm, echo))
                    continue
                res[(want, rpm)] = r
                print("  %5.0f rpm ss=%8.3f sd=%6.3f peak=%8.3f iq=%6.3f"
                      % (rpm, r["ss"], r["sd"], r["peak"], r["iq_mean"]))

        print("\n=== 对比（纹波 sigma）===")
        print("    rpm     OFF_sd   ON_sd     改善     OFF_iq   ON_iq")
        for rpm in rpms:
            off = res.get((False, rpm))
            on = res.get((True, rpm))
            if not off or not on:
                continue
            imp = (off["sd"] - on["sd"]) / off["sd"] * 100.0 if off["sd"] else 0.0
            print("  %5.0f   %7.3f  %7.3f  %+7.1f%%  %7.3f  %7.3f"
                  % (rpm, off["sd"], on["sd"], imp, off["iq_mean"], on["iq_mean"]))
        return 0
    finally:
        rig.cmd("D", 0.3)
        try:
            st = rig.cmd(":", 0.3)
            if (" en=1" in st) != orig_on:
                rig.cmd(";", 0.3)
            print("\n[收尾] %s" % rig.cmd(":", 0.4))
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
