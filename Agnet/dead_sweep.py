# -*- coding: utf-8 -*-
"""dead_sweep.py —— 死区补偿量标定：扫 du，用 iq 的 6 次脉动找最优点

为什么用 6 次脉动做判据：
  死区误差的符号跟着相电流方向走，在 abc 域是"每相每周期两次台阶"，
  折到 dq 域就是 **6 次脉动**（6×rpm/60×14 Hz）。
  补偿量对不上时，这个分量会变大；对上了才最小。
  它比"转速纹波"干净得多——转速里混着齿槽、摩擦、黏滑。

为什么用 20 kHz 片内录制而不是 100 Hz 遥测：
  100 rpm 时 6 次是 140 Hz，超过 100 Hz 遥测的 Nyquist（58 Hz），
  遥测根本看不到。`&` 命令 20 kHz 采 1024 点，140 Hz 轻松可测。

判据的局限：
  1024 点 @20 kHz = 51 ms，100 rpm 的一个电周期是 43 ms，
  只有 1.2 个周期 —— 幅值估计的置信度有限。所以每个 du 重复 3 次取中位数，
  且只看**相对趋势**（哪个 du 让 6 次最小），不看绝对值。

用法:
    python dead_sweep.py COM27 --rpm 100
    python dead_sweep.py COM27 --rpm 100 --dus 0,20,30,40,50,59,80
"""
import argparse
import math
import re
import time

from cog_ff_test import Rig
from sweep_1_1000 import set_rpm
from torque_test import send_num

POLE = 14.0
REC_SCALE = 131.0     # 与 servo_isr.c 录制时的 *131 对应


def grab_record(ser, timeout=6.0):
    """发 `&` 收 [REC] 十六进制行，返回 iq[A]

    不能直接 import harm_fft 的同名函数 —— 那个模块在 import 时就
    解析 sys.argv（RPM = float(sys.argv[2])），入口参数一冲突就抛异常。
    另外发之前必须 reset_input_buffer：上一轮残留的 "REC end" 会让
    本次循环立刻退出，采到 0 点。
    """
    ser.reset_input_buffer()
    ser.write(b"&")
    ser.flush()
    raw = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = ser.read(4096)
        if d:
            raw += d
        if b"REC end" in raw:
            break
    vals = []
    for m in re.finditer(rb"\[REC\]((?:[0-9a-f]{4})+)", bytes(raw)):
        body = m.group(1)
        for k in range(0, len(body), 4):
            u = int(body[k:k + 4], 16)
            v = u - 0x10000 if u >= 0x8000 else u
            vals.append(v / REC_SCALE)
    return vals


def dft_amp(x, fs, f):
    """Hann 窗单点 DFT，返回正弦幅值"""
    n = len(x)
    if n < 64:
        return 0.0
    m = sum(x) / n
    ws = 0.0
    re = 0.0
    im = 0.0
    for i, v in enumerate(x):
        w = 0.5 - 0.5 * math.cos(2.0 * math.pi * i / (n - 1))
        a = 2.0 * math.pi * f * i / fs
        re += (v - m) * w * math.cos(a)
        im += (v - m) * w * math.sin(a)
        ws += w
    return 2.0 * math.hypot(re, im) / ws


def sd(x):
    m = sum(x) / len(x)
    return math.sqrt(sum((v - m) ** 2 for v in x) / len(x))


def query_dead(rig, tries=3):
    """`\\` 空参数查询：返回 (en, du, raw)

    必须内部重试：ASCII 回显是单槽（s_out_pending），碰上遥测 DMA 在飞
    整行会被丢掉。只查一次就判"打不开补偿"会误判。
    """
    for _ in range(tries):
        rig.s.reset_input_buffer()
        rig.s.write(b"\\")
        rig.s.flush()
        time.sleep(0.06)
        rig.s.write(b"\r")
        rig.s.flush()
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < 0.6:
            d = rig.s.read(512)
            if d:
                buf += d
        txt = "".join(ch if 32 <= ord(ch) < 127 else "\n"
                      for ch in buf.decode("ascii", "replace"))
        for ln in txt.split("\n"):
            ln = ln.strip()
            if ln.startswith("[CMD] dead"):
                m = re.search(r"en=(\d) du=([\d.]+)%", ln)
                if m:
                    return int(m.group(1)), float(m.group(2)), ln
        time.sleep(0.15)
    return None, None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--rpm", type=float, default=100.0)
    ap.add_argument("--dus", default="0,20,30,40,50,59,80")
    ap.add_argument("--rep", type=int, default=4)
    ap.add_argument("--div", type=int, default=5,
                    help="录制分频。5=4kHz/256ms，能在 100rpm 下分辨 140Hz；1=20kHz/51ms")
    a = ap.parse_args()

    dus = [float(x) for x in a.dus.split(",")]
    fs = 20000.0 / float(a.div)
    win_ms = 51.2 * float(a.div)
    rig = Rig(a.port)
    f_elec = a.rpm * POLE / 60.0
    print("转速 %.0f rpm -> 电频率 %.2f Hz，6 次 %.1f Hz，5 次 %.1f，7 次 %.1f"
          % (a.rpm, f_elec, 6 * f_elec, 5 * f_elec, 7 * f_elec))
    print("录制: div=%d  fs=%.0f Hz  窗口 %.0f ms（%.1f 个电周期）分辨率 %.2f Hz"
          % (a.div, fs, win_ms, win_ms / 1000.0 * f_elec, fs / 1024.0))

    rows = []
    try:
        print("S:", rig.cmd("S", 0.6))

        # 明确把补偿打开：`B` 是切换不是置位，连按两次会绕回原状态。
        # 必须先用 `\` 查询读到 en，再决定要不要按。
        en, du, line = query_dead(rig)
        print("dead 初始:", line or "(query 失败)")
        for _ in range(3):
            if en == 1:
                break
            rig.cmd("B", 0.4)
            en, du, line = query_dead(rig)
        if en != 1:
            print("!! 无法打开死区补偿，中止")
            return 2
        print("dead 已开:", line)

        print(set_rpm(rig, a.rpm))
        rig.keep(1.5)
        print("  ", send_num(rig, "'", a.div, dec=0))   # 设录制分频

        print("\n%6s %8s %8s %8s %8s %8s" %
              ("du%", "6th(A)", "5th(A)", "7th(A)", "σ(A)", "iq均值"))
        print("-" * 56)
        for du in dus:
            print("  ", send_num(rig, "\\", du, dec=0))
            time.sleep(0.6)
            a6, a5, a7, sds, iqm = [], [], [], [], []
            for _ in range(a.rep):
                v = grab_record(rig.s)
                if len(v) < 256:
                    continue
                m = sum(v) / len(v)
                a6.append(dft_amp(v, fs, 6 * f_elec))
                a5.append(dft_amp(v, fs, 5 * f_elec))
                a7.append(dft_amp(v, fs, 7 * f_elec))
                sds.append(sd(v))
                iqm.append(m)
                time.sleep(0.15)
            if not a6:
                print("%6.0f   录制失败" % du)
                continue
            med = lambda xs: sorted(xs)[len(xs) // 2]
            rows.append((du, med(a6), med(a5), med(a7), med(sds), med(iqm)))
            print("%6.0f %8.4f %8.4f %8.4f %8.4f %8.3f"
                  % (du, med(a6), med(a5), med(a7), med(sds), med(iqm)))

        rig.cmd("D", 0.3)

        if rows:
            best = min(rows, key=lambda r: r[1])
            print("\n=== 结论 ===")
            print("  6 次最小出现在 du = %.0f (0.01%%) = %.3f%% 占空比，幅值 %.4f A"
                  % (best[0], best[0] * 0.01, best[1]))
            base = [r for r in rows if r[0] == 0.0]
            if base:
                print("  相对 du=0（不补偿）: %+.1f%%"
                      % ((base[0][1] - best[1]) / base[0][1] * 100 if base[0][1] else 0))
            print("  理论值 294 ns × 20 kHz = 0.59%%（即 du=59）")
        return 0
    finally:
        try:
            rig.cmd("D", 0.3)
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
