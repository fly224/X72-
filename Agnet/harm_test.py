# -*- coding: utf-8 -*-
"""harm_test.py —— 验证 5/7 次谐波电流抑制（MSRF）是否真的把 6 次脉动压下去了

原理：
  5 次（负序）与 7 次（正序）相电流谐波，在基波 dq 坐标系里都折叠成 **6ωe** 的脉动。
  所以只要看 iq（或 id）在 6 倍电频率处的幅度，就能判断抑制有没有效果。

为什么必须在 30 rpm 附近测：
  遥测只有 100 Hz（Nyquist 50 Hz）。电频率 = rpm/60 × 14，
  6 次 = 6 × rpm/60 × 14 = 1.4 × rpm [Hz]。要落在 50 Hz 以下，rpm < 35.7。
  30 rpm → 电频率 7 Hz、6 次 42 Hz，刚好在可用范围内。

判读：
  · 关掉时 42 Hz 有明显峰值、开起来后峰值下降 → 抑制生效
  · 两个状态都看不到峰值 → 该转速下谐波本身就很小（反电势只有 1.9 V），
    需要换更高转速 + 更高采样率才能评估（见文档"局限"）
"""
import math
import re
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
BAUD = 921600

MSG_IN, LEN_IN, EXTRA_IN = 30025, 57, 191
POLE = 14.0


def x25(data, extra):
    crc = 0xFFFF

    def acc(c, b):
        t = (b ^ (c & 0xFF)) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        return ((c >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF

    for b in data:
        crc = acc(crc, b)
    return acc(crc, extra)


def cmd(ser, c, wait=0.3):
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


def capture(ser, secs=3.0, keepalive_cmd="W"):
    """采 30025 帧，返回 (iq_tar[], rpm[])，10 ms 一帧"""
    buf = bytearray()
    iq, rpm = [], []
    t0 = time.time()
    nxt = t0 + 1.5
    while time.time() - t0 < secs:
        d = ser.read(1024)
        if d:
            buf += d
        if time.time() > nxt:
            ser.write(keepalive_cmd.encode())
            ser.flush()
            nxt += 1.5
        i = 0
        while i + 24 <= len(buf):
            if buf[i] != 0xFD:
                i += 1
                continue
            ln = buf[i + 1]
            mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            if (ln, mid) != (LEN_IN, MSG_IN):
                i += 1
                continue
            b = i + 10
            if b + ln + 2 > len(buf):
                break
            if x25(buf[i + 1:b + ln], EXTRA_IN) != (buf[b + ln] | (buf[b + ln + 1] << 8)):
                i += 1
                continue
            iq.append(struct_f(buf, b + 4))
            rpm.append(struct_f(buf, b + 44) / POLE * 60.0 / (2 * math.pi))
            i = b + ln + 2
        del buf[:i]
    return iq, rpm


def struct_f(b, off):
    import struct
    return struct.unpack_from("<f", b, off)[0]


def amp_at(v, f, fs=100.0):
    """单频点 DFT 幅度（Goertzel 等价写法）。v 均匀采样，采样率 fs"""
    n = len(v)
    if n < 8:
        return 0.0
    s = c = 0.0
    for k, x in enumerate(v):
        ph = 2.0 * math.pi * f * k / fs
        s += x * math.sin(ph)
        c += x * math.cos(ph)
    return 2.0 / n * math.hypot(s, c)


def scan(v, f_lo, f_hi, step=0.5):
    """扫频找峰，避免转速漂移导致定点 DFT 采不到峰"""
    best_f, best_a = 0.0, 0.0
    f = f_lo
    while f <= f_hi:
        a = amp_at(v, f)
        if a > best_a:
            best_a, best_f = a, f
        f += step
    return best_f, best_a


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    print("=" * 70)
    print("5/7 次谐波抑制验证   port=%s" % PORT)
    print("=" * 70)

    print("\n[1] 复位")
    print("    " + cmd(ser, "D"))
    print("    " + cmd(ser, "X"))
    st = cmd(ser, "S")
    print("    " + st)
    if "cal=1" not in st or "z=1" not in st:
        print("    [FAIL] 需要先 C + Z")
        ser.close()
        return 3

    print("\n[2] 速度环复位到实测最优（Y），进 30 rpm")
    print("    " + cmd(ser, "Y"))
    print("    " + cmd(ser, "W", 0.4))

    def measure(tag):
        time.sleep(1.2)                      # 等环路收敛
        iq, rpm = capture(ser, 3.0)
        if len(iq) < 50:
            print("    %s: 帧太少 (%d)" % (tag, len(iq)))
            return None
        rpm_avg = sum(rpm) / len(rpm)
        f6 = 6.0 * rpm_avg / 60.0 * POLE
        f_pk, a_pk = scan(iq, max(2.0, f6 - 8.0), f6 + 8.0, 0.5)
        dc = sum(iq) / len(iq)
        sd = math.sqrt(sum((x - dc) ** 2 for x in iq) / len(iq))
        print("    %-8s n=%3d  rpm=%6.2f  6次理论=%.1fHz  峰值@%.1fHz  A6=%.4fA  iq均值=%.3fA  std=%.4f"
              % (tag, len(iq), rpm_avg, f6, f_pk, a_pk, dc, sd))
        return dict(f6=f6, fpk=f_pk, a6=a_pk, dc=dc, sd=sd)

    print("\n[3] 关闭谐波抑制（h=0）")
    if "h=1" in (cmd(ser, "S") or ""):
        print("    " + cmd(ser, "%"))
    r_off = measure("OFF")

    print("\n[4] 打开谐波抑制（h=1）")
    echo = cmd(ser, "%")
    print("    " + echo)
    r_on = measure("ON")

    print("\n[5] 再关一次，确认可逆")
    print("    " + cmd(ser, "%"))
    r_off2 = measure("OFF-2")

    print("\n" + "=" * 70)
    if r_off and r_on:
        print("  6 次脉动幅度（iq 谱峰）：")
        print("    关闭 %s : %.4f A  @ %.1f Hz" % ("OFF  ", r_off["a6"], r_off["fpk"]))
        print("    开启 %s : %.4f A  @ %.1f Hz" % ("ON   ", r_on["a6"], r_on["fpk"]))
        if r_off2:
            print("    关闭 %s : %.4f A  @ %.1f Hz" % ("OFF-2", r_off2["a6"], r_off2["fpk"]))
        if r_off["a6"] > 1e-4:
            print("    抑制比：%+.1f%%" % ((r_on["a6"] / r_off["a6"] - 1.0) * 100.0))
            if r_on["a6"] < r_off["a6"] * 0.7:
                print("    -> 有效（幅度降到 70% 以下）")
            elif r_on["a6"] > r_off["a6"]:
                print("    -> 反而变大：检查 5/7 次坐标系符号（负序 vs 正序）或降低 Kp")
            else:
                print("    -> 改善不明显：该转速下谐波本身就小，需更高转速/更高采样率评估")
        else:
            print("    -> OFF 时也看不到 6 次峰：30 rpm 下反电势只有约 1.9 V，谐波太小")
    print("=" * 70)

    cmd(ser, "%")            # 收尾：恢复关闭
    cmd(ser, "D")
    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
