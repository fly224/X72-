# -*- coding: utf-8 -*-
"""noise_spec.py —— 电流采样噪声的频谱定性

目的：决定该用哪种手段优化采样。
  · 若能量集中在低频（<1 kHz）      -> 是真实的电流波动，采样本身没问题，别去滤
  · 若集中在 20 kHz / 10 kHz 附近   -> 是 PWM 纹波或触发点拍频 -> 做多点采样平均
  · 若全频带平坦                     -> 白噪声 -> 硬件过采样（Oversampling）最有效

方法：`&` 录制 iq（div=1，20 kHz，1024 点），radix-2 FFT 后按频段积分能量。

用法:
    python noise_spec.py COM27
    python noise_spec.py COM27 --div 1
"""
import argparse
import cmath
import math
import re
import time

from cog_ff_test import Rig
from torque_test import send_num

REC_SCALE = 131.0


def grab(ser, timeout=8.0):
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


def fft(x):
    n = len(x)
    a = [complex(v) for v in x]
    j = 0
    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j |= bit
        if i < j:
            a[i], a[j] = a[j], a[i]
    ln = 2
    while ln <= n:
        ang = -2.0 * math.pi / ln
        wl = cmath.exp(1j * ang)
        for i in range(0, n, ln):
            w = 1 + 0j
            half = ln >> 1
            for k in range(half):
                u = a[i + k]
                v = a[i + k + half] * w
                a[i + k] = u + v
                a[i + k + half] = u - v
                w *= wl
        ln <<= 1
    return a


def bands(vals, fs):
    n = 1
    while n * 2 <= len(vals):
        n *= 2
    seg = vals[:n]
    mean = sum(seg) / n
    seg = [v - mean for v in seg]
    # Hann 窗，减少泄漏
    seg = [v * (0.5 - 0.5 * math.cos(2 * math.pi * i / (n - 1))) for i, v in enumerate(seg)]
    sp = fft(seg)
    amp = [abs(sp[k]) * 2.0 / (n * 0.5) for k in range(n // 2)]
    df = fs / n
    edges = [(0, 100), (100, 1000), (1000, 5000), (5000, 9000), (9000, fs / 2)]
    out = []
    for lo, hi in edges:
        k0 = int(lo / df)
        k1 = min(int(hi / df), len(amp) - 1)
        e = math.sqrt(sum(a * a for a in amp[k0:k1 + 1]) / max(1, k1 - k0 + 1))
        pk = max(amp[k0:k1 + 1]) if k1 >= k0 else 0.0
        out.append((lo, hi, e, pk))
    return out, df, amp, mean


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--div", type=int, default=1)
    a = ap.parse_args()

    fs = 20000.0 / a.div
    rig = Rig(a.port)
    try:
        print("S:", rig.cmd("S", 0.6))
        rig.cmd("X", 0.4)
        print(send_num(rig, ">", 0.0))          # 进力矩模式，iq=0（MOE 开、有开关动作）
        print("  ", send_num(rig, "'", a.div, dec=0))
        time.sleep(0.8)

        vals = []
        for _ in range(6):
            rig.s.write(b"S")                   # 保活，防 deadman
            rig.s.flush()
            vals.extend(grab(rig.s))
            time.sleep(0.1)
        rig.cmd("D", 0.3)

        if len(vals) < 256:
            print("采集不足", len(vals))
            return 1
        n = len(vals)
        m = sum(vals) / n
        sd = math.sqrt(sum((v - m) ** 2 for v in vals) / n)
        print("\nn=%d  fs=%.0f Hz  均值=%+.4f A  总 sd=%.4f A  峰峰=%.3f"
              % (n, fs, m, sd, max(vals) - min(vals)))

        bs, df, amp, _ = bands(vals, fs)
        print("\n%-16s %-12s %-12s" % ("频段 Hz", "RMS (A)", "峰值 (A)"))
        for lo, hi, e, pk in bs:
            print("%6.0f - %-7.0f %-12.4f %-12.4f" % (lo, hi, e, pk))

        # 找全谱最大峰
        k = max(range(1, len(amp)), key=lambda i: amp[i])
        print("\n最大谱峰: %.1f Hz  幅值 %.4f A（分辨率 %.1f Hz）"
              % (k * df, amp[k], df))
        return 0
    finally:
        try:
            rig.cmd("D", 0.3)
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
