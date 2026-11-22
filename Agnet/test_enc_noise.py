# -*- coding: utf-8 -*-
"""对照实验：PWM 关 / PWM 开(零电流) 两段各 3 s，看编码器读数是否被开关噪声污染。

判据：
  - PWM 关：enc 稳定，标准差 ~0
  - PWM 开：enc 出现 ±数百~数千 counts 的跳变 → 编码器被干扰（SPI/供电/地）

用法：python test_enc_noise.py COM27
"""
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\X72_Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"

rig = Rig(PORT)
time.sleep(1.0)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)


def acmd(c, wait=1.0, tries=5):
    for _ in range(tries):
        rig.rx.pause_evt.set()
        time.sleep(0.04)
        try:
            rig.s.reset_input_buffer()
            rig.s.write(c.encode())
            rig.s.flush()
            raw = b""
            t0 = time.time()
            while time.time() - t0 < wait:
                d = rig.s.read(8192)
                if d:
                    raw += d
        finally:
            rig.rx.buf = bytearray()
            rig.rx.pause_evt.clear()
        txt = strip_frames(raw).decode("ascii", "replace")
        if "[CMD]" in txt:
            return txt
        time.sleep(0.25)
    return ""


def sample(sec):
    """自己解 30040，返回 enc 列表与 (en, moe)。"""
    rig.rx.pause_evt.set()
    time.sleep(0.04)
    rig.s.reset_input_buffer()
    buf = bytearray()
    encs = []
    t0 = time.time()
    while time.time() - t0 < sec:
        d = rig.s.read(8192)
        if d:
            buf += d
        while len(buf) >= 24:
            if buf[0] == 0xFD:
                ln = buf[1]
                if len(buf) < 12 + ln:
                    break
                mid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
                if mid == 30040 and ln >= 8:
                    encs.append(struct.unpack_from("<q", bytes(buf), 10)[0])
                del buf[:12 + ln]
            else:
                del buf[:1]
    rig.rx.buf = bytearray()
    rig.rx.pause_evt.clear()
    return encs


def stat(name, encs):
    if not encs:
        print("  %s: 无数据" % name)
        return
    n = len(encs)
    mn, mx = min(encs), max(encs)
    avg = sum(encs) / n
    # 相邻差分的最大跳变最能反映污染
    dmax = max(abs(encs[i] - encs[i - 1]) for i in range(1, n))
    print("  %-10s n=%4d  范围[%d, %d] 摆幅 %d counts (%.3f deg)  最大相邻跳变 %d"
          % (name, n, mn, mx, mx - mn, (mx - mn) / 2097152.0 * 360.0, dmax))


print("S:", (acmd("S", 1.0).strip() or "(空)")[:170])

print("\n[1] PWM 关（moe=0）")
e_off = sample(3.0)
stat("PWM-OFF", e_off)

print("\n[2] 使能 PWM，零给定（moe=1, 无转矩）")
print("  E ->", (acmd("E", 1.2).strip() or "(空)")[:80])
time.sleep(0.8)
print("  S ->", (acmd("S", 1.0).strip() or "(空)")[:170])
e_on = sample(3.0)
stat("PWM-ON", e_on)

print("\n[3] 封锁")
print("  D ->", (acmd("D", 1.0).strip() or "(空)")[:60])
time.sleep(0.6)
e_off2 = sample(2.0)
stat("PWM-OFF2", e_off2)
print("  S ->", (acmd("S", 1.0).strip() or "(空)")[:170])

rig.close()
