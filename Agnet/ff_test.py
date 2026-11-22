# -*- coding: utf-8 -*-
"""摩擦前馈 / 起动助推的实机对比（3 rpm，空载）

这是第 1 项的核心判据：
  关前馈时 3 rpm 超调实测 68.8%（2026-11-16 基准），
  离线仿真预测加"摩擦前馈 + 起动助推"后降到 ~21%。

前提：COM27 未被上位机占用（Web Serial 会独占句柄）。
      报 PermissionError(13) 就先在页面点「断开串口」。

用法：
    python ff_test.py                          # 默认 3 rpm，全流程
    python ff_test.py --rpm 3 --skip-cal       # 跳过 C/Z
    python ff_test.py --fric-only              # 只测开/关前馈，不做标定
"""
from __future__ import print_function

import argparse
import re
import sys
import time

import ramp_test as rt


def take(rig, rpm, secs=2.5):
    rig.cmd("D", 0.25)
    rig.drain(0.3)
    r = rig.num("=", rpm, 0.5)
    if "refused" in r or "failed" in r:
        print("      [跳过] " + r[:70])
        return None
    rows = rig.capture(secs)
    rig.cmd("D", 0.3)
    m = rt.metrics(rows, rpm)
    if m:
        m["echo"] = r
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--rpm", type=float, default=3.0)
    ap.add_argument("--secs", type=float, default=2.5)
    ap.add_argument("--skip-cal", action="store_true")
    ap.add_argument("--fric-only", action="store_true", help="不做 $ 标定")
    args = ap.parse_args()

    try:
        rig = rt.Rig(args.port)
    except Exception as e:
        print("打不开 %s：%s" % (args.port, e))
        print("→ 先在上位机点「断开串口」")
        return 2

    try:
        print("[0] 预置")
        print("    " + rig.cmd("D"))
        print("    " + rig.cmd("X"))
        st = rig.cmd("S", 0.4)
        print("    " + st)

        if not args.skip_cal:
            if "cal=1" not in st:
                print("    " + rig.cmd("C", 0.5))
                time.sleep(0.9)
            st = rig.cmd("S", 0.4)
            if "z=1" not in st:
                print("    " + rig.cmd("Z", 0.4))
                for _ in range(15):
                    time.sleep(1.0)
                    st = rig.cmd("S", 0.3)
                    if "z=1" in st:
                        break

        # ---- 1) 关前馈：基准 ----
        print("\n[1] 关前馈，给定 %+.1f rpm" % args.rpm)
        print("    " + rig.cmd("*", 0.35))         # 若本来是开->关；下面再确认一次
        e = rig.cmd("*", 0.35)
        if "ON" in e:                              # 第一次是开->关，第二次又打开了
            print("    (回显为 ON，再按一次关掉)")
            e = rig.cmd("*", 0.35)
        print("    " + e)
        off = take(rig, args.rpm, args.secs)

        # ---- 2) 摩擦标定（自动开启前馈） ----
        if not args.fric_only:
            print("\n[2] 摩擦两点标定 $（约 4 s，期间电机会转）")
            r = rig.cmd("$", 0.4)
            print("    " + r)
            res = None
            t0 = time.time()
            buf = bytearray()
            while time.time() - t0 < 10.0:
                d = rig.s.read(512)
                if d:
                    buf += d
                if b"FC ok" in buf or b"FC FAIL" in buf:
                    break
            for ln in bytes(buf).decode("ascii", "replace").splitlines():
                if "FC " in ln:
                    res = ln.strip()
                    print("    " + res)
                    break
            if res is None:
                print("    (未捕获结果行，继续用当前前馈参数)")
        else:
            print("\n[2] 跳过标定，手动设 fric=0.32 boost=1.0")
            print("    " + rig.num("[", 0.32))
            print("    " + rig.num("{", 1.0))
            print("    " + rig.cmd("*", 0.35))

        # ---- 3) 开前馈 ----
        print("\n[3] 开前馈，给定 %+.1f rpm" % args.rpm)
        e = rig.cmd("*", 0.35)
        if "OFF" in e:
            print("    (回显为 OFF，再按一次打开)")
            e = rig.cmd("*", 0.35)
        print("    " + e)
        on = take(rig, args.rpm, args.secs)

        # ---- 4) 对比 ----
        print("\n[4] 对比")
        if off and on:
            print("    %-10s %8s %8s %10s %9s" % ("状态", "超调", "稳态", "iq峰值", "上升时间"))
            print("    %-10s %7.1f%% %8.3f %8.2f A %7.0f ms"
                  % ("关前馈", off["os"], off["ss"], off["iq_peak"], off["tr"]))
            print("    %-10s %7.1f%% %8.3f %8.2f A %7.0f ms"
                  % ("开前馈", on["os"], on["ss"], on["iq_peak"], on["tr"]))
            d = off["os"] - on["os"]
            print("\n    超调改善 %.1f 个百分点（%.0f%%）" % (d, d / max(off["os"], 1e-6) * 100))
            if d < 5.0:
                print("    ⚠ 改善不明显，检查：")
                print("      · * 回显里 fric 是否为 0（标定没成，用 [0.32 手动设）")
                print("      · boost 是否为 1.0（用 {1 重设）")
                print("      · S 的 f=0x00（有没有跳闸）")
        elif off is None or on is None:
            print("    数据不足，无法对比")

        print("\n[5] 收尾")
        print("    " + rig.cmd("D", 0.3))
        print("    " + rig.cmd("S", 0.4))
    finally:
        rig.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
