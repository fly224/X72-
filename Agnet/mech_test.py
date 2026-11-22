# -*- coding: utf-8 -*-
"""机械环节辨识：跑固件并解析/分析结果

四种模式对应固件 `_1`..`_4`：
  1  谐振扫频（20..1000 Hz，iq 正弦，同步解调）
  2  摩擦曲线（8 个转速点的稳态 iq）
  3  折算惯量（两个转矩幅值的加速度差分）
  4  齿槽转矩（2 rpm 转 1 圈，iq 按机械角分 128 箱）

用法：
    python mech_test.py COM27 --mode 1
    python mech_test.py COM27 --mode all
    python mech_test.py COM27 --mode 1 --skip-cal     # Flash 参数还在时省掉 C/Z

前提：COM27 未被上位机占用。报 PermissionError(13) 就先在页面点「断开串口」。
"""
from __future__ import print_function

import argparse
import math
import re
import sys
import time

import ramp_test as rt

MODE_NAME = {1: "谐振扫频", 2: "摩擦曲线", 3: "折算惯量", 4: "齿槽转矩"}
# 固件用 4 个独立字符启动（避开 `_<n>` 数值命令的字节竞态）
MODE_CH = {1: "(", 2: ")", 3: "<", 4: "}"}


def collect(rig, timeout=20.0):
    """发 _<mode> 并收集 [MI] 行，返回 (rows, end_line)"""
    rows = []
    end = None
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = rig.s.read(4096)
        if d:
            buf += d
            # 注意：**不要**在这里发 S 保活。
            # 辨识期间 s_armed=0，deadman 根本没启动；而 S 的回显会占用 s_out
            # 单缓冲，把正在排队的 [MI] 行顶掉（第一次实跑就是这么丢数据的）。
        txt = bytes(buf).decode("ascii", "replace")
        while "\n" in txt:
            line, txt = txt.split("\n", 1)
            # 遥测是二进制，切行后会在同一行里混进乱码，[MI] 不一定在行首。
            # 所以找 [MI] 的位置再截断，而不是 startswith。
            k = line.find("[MI]")
            if k >= 0:
                rows.append(line[k:].strip())
            elif "MI ok" in line or "MI FAIL" in line or "MI end" in line:
                if "MI end" in line:
                    end = line
                    buf = bytearray(txt.encode("ascii", "replace"))
                    return rows, end
                if "MI FAIL" in line:
                    end = line
                    return rows, end
        buf = bytearray(txt.encode("ascii", "replace"))
    return rows, end


def show_sweep(rows):
    pts = []
    for r in rows:
        m = re.match(r"\[MI\] SW ([\-\d.]+) ([\-\d.]+) ([\-\d.]+)", r)
        if m:
            pts.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    if not pts:
        print("    (无扫频数据)")
        return

    print("    频率[Hz]   增益[rad/s per A]   相位[deg]")
    for f, g, ph in pts:
        bar = "#" * int(min(40, g * 200))
        print("    %8.2f   %12.4f      %8.2f  %s" % (f, g, ph, bar))

    # 找增益峰（排除第一个点，它可能是积分段的主导）
    peak = None
    for i in range(1, len(pts) - 1):
        if pts[i][1] > pts[i - 1][1] and pts[i][1] >= pts[i + 1][1]:
            if peak is None or pts[i][1] > peak[1]:
                peak = pts[i]
    if peak:
        print("\n    ▸ 谐振峰约 %.0f Hz（增益 %.4f rad/s per A）" % (peak[0], peak[1]))
        print("      速度环带宽上限的工程经验值 ≈ 谐振频率 / 5 ~ / 3")
        print("      → 当前 3 Hz 若远低于此，说明带宽还有很大余量")
    else:
        print("\n    ▸ 未找到明显峰（全频段单调）—— 可能是刚性耦合，或激励幅值太小")

    # 低频段增益应满足 |w/i| = Kt/(J·2πf)，用来反推 J 做交叉验证
    if pts and pts[0][0] > 5:
        f0, g0, _ = pts[0]
        j_est = 0.0936 / (g0 * 2 * math.pi * f0)
        print("    由最低频点反推 J ≈ %.2e kg·m^2（与 _3 的结果应同量级）" % j_est)


def show_fric(rows, end):
    pts = []
    for r in rows:
        m = re.match(r"\[MI\] FR ([\-\d.]+) ([\-\d.]+) ([\-\d.]+)", r)
        if m:
            pts.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    if not pts:
        print("    (无摩擦数据)")
        return

    print("    给定[rpm]  实测[rpm]   稳态iq[A]")
    for a, b, c in pts:
        print("    %9.2f  %9.2f   %9.4f" % (a, b, c))

    if end:
        m = re.search(r"fric_a=([\-\d.]+)A visc=([\-\d.]+)e-3", end)
        if m:
            print("\n    ▸ 线性回归：fric_a = %s A、visc = %s e-3 A/(rad/s)" % (m.group(1), m.group(2)))

    # Stribeck 视角：低速段（<10 rpm）的 iq 若明显高于线性外推，说明有静摩擦/Stribeck 项
    low = [p for p in pts if p[1] < 10.0]
    high = [p for p in pts if p[1] > 20.0]
    if low and high:
        iq_low = sum(p[2] for p in low) / len(low)
        iq_high = sum(p[2] for p in high) / len(high)
        print("    低速均值 %.4f A / 高速均值 %.4f A，比值 %.2f" % (iq_low, iq_high, iq_low / iq_high))
        if iq_low / iq_high > 1.3:
            print("    ▸ 低速显著偏大 → 存在明显静摩擦（这正是起动助推要补的部分）")


def show_inertia(rows):
    for r in rows:
        m = re.match(r"\[MI\] IG J=([\-\d.]+)e-4 acc=([\-\d.]+)/([\-\d.]+) tq=([\-\d.]+)/([\-\d.]+)", r)
        if m:
            j = float(m.group(1)) * 1e-4
            print("    J_eq   = %.3e kg·m^2" % j)
            print("    acc    = %.2f / %.2f rad/s^2（1 A / 2 A）" % (float(m.group(2)), float(m.group(3))))
            print("    转矩   = %.4f / %.4f N·m" % (float(m.group(4)), float(m.group(5))))
            print("    ⚠ J 与 psi 成正比：psi 若有 41% 偏差，J 也偏 41%")
            print("      当前用实测 psi=4.455 mWb（Kt=0.0936）")
            return
    print("    (无惯量数据)")


def show_cog(rows, end):
    vals = []
    for r in rows:
        m = re.match(r"\[MI\] CG (.*)", r)
        if m:
            for tok in m.group(1).split():
                try:
                    vals.append(float(tok))
                except ValueError:
                    pass
    if not vals:
        print("    (无齿槽数据)")
        return

    n = len(vals)
    lo, hi = min(vals), max(vals)
    print("    箱数 %d，iq 范围 %.4f … %.4f A" % (n, lo, hi))
    if end:
        m = re.search(r"pp=([\-\d.]+)A", end)
        if m:
            print("    峰峰值 %s A" % m.group(1))

    # 128 点对应 1 机械圈 → bin k 就是第 k 阶
    def dft_mag(k):
        re_ = sum(v * math.cos(2 * math.pi * k * i / n) for i, v in enumerate(vals))
        im_ = sum(v * math.sin(2 * math.pi * k * i / n) for i, v in enumerate(vals))
        return 2.0 * math.hypot(re_, im_) / n

    tops = sorted(((dft_mag(k), k) for k in range(1, n // 2)), reverse=True)[:5]
    print("\n    主要阶次（1 阶 = 每机械圈 1 次）：")
    for mag, k in tops:
        print("      %3d 阶  幅值 %.4f A" % (k, mag))
    print("    ▸ 14 极对 + 常见槽数下，齿槽主阶次通常是 LCM(槽,极) 或其约数；")
    print("      若最大阶次是 2 阶，更可能是装配偏心而不是齿槽。")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--mode", default="1", help="1..4 或 all")
    ap.add_argument("--skip-cal", action="store_true")
    args = ap.parse_args()

    modes = [1, 2, 3, 4] if args.mode == "all" else [int(args.mode)]

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
            print("    " + rig.cmd("S", 0.4))

        for md in modes:
            print("\n" + "=" * 58)
            print("[%d] %s" % (md, MODE_NAME.get(md, "?")))
            echo = rig.cmd(MODE_CH[md], 0.5)
            print("    " + echo)
            if "refused" in echo:
                continue

            rows, end = collect(rig, timeout=45.0)
            print("    收到 %d 行" % len(rows))
            if end:
                print("    " + end)
            else:
                print("    ⚠ 未收到结束行（可能超时）")

            if md == 1:
                show_sweep(rows)
            elif md == 2:
                show_fric(rows, end)
            elif md == 3:
                show_inertia(rows)
            elif md == 4:
                show_cog(rows, end)
            time.sleep(0.4)

        print("\n[9] 收尾")
        print("    " + rig.cmd("D", 0.3))
        print("    " + rig.cmd("S", 0.4))
    finally:
        rig.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
