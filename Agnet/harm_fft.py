# -*- coding: utf-8 -*-
"""harm_fft.py —— 用片内 20 kHz 记录验证 5/7 次谐波抑制

为什么不用 100 Hz 遥测：
  6 次脉动频率 = 6 × rpm/60 × 14 = 1.4·rpm [Hz]。30 rpm → 42 Hz 已经贴着
  50 Hz 的 Nyquist，实测两次 OFF 之间就差了 2 倍（0.0209 vs 0.0434 A），
  完全不可判读。所以改用固件里的 `&` 命令：20 kHz 采 1024 点 iq（51 ms），
  分辨率 19.5 Hz、覆盖 0~10 kHz，1000 rpm 时的 1400 Hz 谐波也在范围内。

判读：
  · 关闭时在 6 次频率处有峰，打开后峰下降 → 抑制生效
  · 打开后反而变大 → 5/7 次坐标系符号错了（5 次是负序）或 Kp 太大
  · 两边都没有峰 → 该工况谐波本身就小

用法：
  python harm_fft.py                # COM27，1000 rpm
  python harm_fft.py COM5 600       # 指定口与转速
"""
import cmath
import math
import re
import struct
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
RPM = float(sys.argv[2]) if len(sys.argv) > 2 else 1000.0
BAUD = 921600
POLE = 14.0
FS = 20000.0          # 记录采样率
SCALE = 131.0         # int16 -> A


def cmd(c, ser, wait=0.3):
    ser.reset_input_buffer()
    ser.write(c.encode())
    ser.flush()
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < wait:
        d = ser.read(256)
        if d:
            buf += d
            i = buf.rfind(b"[CMD]")
            if i >= 0 and b"\r\n" in buf[i:]:
                return bytes(buf[i:]).split(b"\r\n")[0].decode("ascii", "replace")
    i = buf.rfind(b"[CMD]")
    return bytes(buf[i:]).split(b"\r\n")[0].decode("ascii", "replace") if i >= 0 else ""


def grab_record(ser, timeout=6.0):
    """发 & 并收集 [REC] 行，返回 iq[A] 列表"""
    # 必须先清缓冲：否则上一轮残留的 "REC end" 会让本次循环立刻 break，
    # 结果是"0 点"——实测连续三次里后两次全空，就是这个原因。
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
            vals.append(v / SCALE)
    return vals


def fft(x):
    """迭代 radix-2 FFT，长度必须是 2 的幂"""
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


def spectrum(vals):
    """返回 [(freq, amp)]，amp 已按长度归一（正弦幅值）"""
    n = len(vals)
    if n < 64:
        return []
    m = 1
    while m * 2 <= n:
        m *= 2
    seg = vals[:m]
    mean = sum(seg) / m
    seg = [x - mean for x in seg]        # 去直流，否则 DC 泄漏淹没低频
    sp = fft(seg)
    out = []
    for k in range(1, m // 2):
        out.append((k * FS / m, 2.0 * abs(sp[k]) / m))
    return out


def peak_near(spec, f0, tol=60.0):
    best = (0.0, 0.0)
    for f, a in spec:
        if abs(f - f0) <= tol and a > best[1]:
            best = (f, a)
    return best


def measure(ser, tag, rpm_now):
    time.sleep(1.0)
    vals = grab_record(ser)
    if len(vals) < 256:
        print("    %-6s 采样不足 (%d 点)" % (tag, len(vals)))
        return None
    spec = spectrum(vals)
    f6 = 6.0 * rpm_now / 60.0 * POLE
    f_pk, a_pk = peak_near(spec, f6)
    dc = sum(vals) / len(vals)
    sd = math.sqrt(sum((x - dc) ** 2 for x in vals) / len(vals))
    print("    %-6s n=%4d  iq均值=%+.4fA  std=%.4fA  6次(%.0fHz)峰: %.5fA @ %.0fHz"
          % (tag, len(vals), dc, sd, f6, a_pk, f_pk))
    return dict(n=len(vals), dc=dc, sd=sd, f6=f6, fpk=f_pk, a6=a_pk, spec=spec)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    print("=" * 74)
    print("5/7 次谐波抑制验证（片内 20 kHz 记录）  %s  目标 %.0f rpm" % (PORT, RPM))
    print("=" * 74)

    print("\n[1] 复位并检查联锁")
    print("    " + cmd("D", ser))
    print("    " + cmd("X", ser))
    st = cmd("S", ser)
    print("    " + st)
    if "cal=1" not in st or "z=1" not in st:
        print("    [FAIL] 需要先 C + Z")
        ser.close()
        return 3

    print("\n[2] 选转速档位并进环")
    tab = [3.0, 30.0, 100.0, 500.0, 1500.0, 3000.0]
    idx = min(range(len(tab)), key=lambda i: abs(tab[i] - RPM))
    print("    实际用档位 %.0f rpm" % tab[idx])
    print("    " + cmd("Y", ser))
    for _ in range((idx - 1) % 6):
        cmd("O", ser)
    print("    " + cmd("W", ser, 0.5))

    print("\n[3] 关闭谐波抑制")
    st = cmd("S", ser)
    if "h=1" in st:
        print("    " + cmd("%", ser))
    rpm_now = tab[idx]
    r_off = measure(ser, "OFF", rpm_now)

    print("\n[4] 打开谐波抑制")
    print("    " + cmd("%", ser))
    r_on = measure(ser, "ON", rpm_now)

    print("\n[5] 再关一次确认可逆")
    print("    " + cmd("%", ser))
    r_off2 = measure(ser, "OFF2", rpm_now)

    print("\n" + "=" * 74)
    if r_off and r_on:
        print("  6 次脉动 @ %.0f Hz：" % r_off["f6"])
        print("    OFF  : %.5f A" % r_off["a6"])
        print("    ON   : %.5f A" % r_on["a6"])
        if r_off2:
            print("    OFF2 : %.5f A" % r_off2["a6"])
        base = min(x["a6"] for x in (r_off, r_off2) if x)
        if base > 1e-5:
            print("    抑制比（ON vs OFF 最小）：%+.1f%%"
                  % ((r_on["a6"] / base - 1.0) * 100.0))
        # 前 8 个谱峰，便于看整体有没有别的问题
        if r_off and r_on:
            print("\n  频谱 Top10（OFF / ON，A）：")
            top = sorted(r_off["spec"], key=lambda x: -x[1])[:10]
            for f, a in top:
                aon = peak_near(r_on["spec"], f, tol=25.0)[1]
                print("    %7.0f Hz   %.5f  /  %.5f" % (f, a, aon))
    print("=" * 74)

    cmd("%", ser)
    cmd("D", ser)
    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
