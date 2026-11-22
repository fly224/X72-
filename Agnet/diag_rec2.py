# -*- coding: utf-8 -*-
"""确认录波第三通道（vq 积分）可用：'205 -> ch=vqint -> & 读回 1024 点。"""
import re
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402


def cmd(rig, c, wait=0.5):
    rig.rx.pause_evt.set()
    time.sleep(0.03)
    try:
        rig.s.reset_input_buffer()
        rig.s.write(c.encode() if isinstance(c, str) else c)
        rig.s.flush()
        raw = b""
        t0 = time.time()
        while time.time() - t0 < wait:
            d = rig.s.read(8192)
            if d:
                raw += d
        return strip_frames(raw).decode("ascii", "replace")
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


rig = Rig("COM27")
time.sleep(1.4)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.4)

print("set:", cmd(rig, "'205\r", 0.6).strip())
print("query:", cmd(rig, "'", 0.6).strip())

# 录波采样只在"使能 + 电流环在跑"的分支里执行，静止未使能时 s_rec_n 一直是 0。
# 所以这里先 E 使能、给定 0（PWM 开但不出力），再看有没有 1024 点。
print("enable:", cmd(rig, "E", 0.6).strip()[:60])
for _ in range(4):
    rig.s.write(b"=0\r")
    rig.s.flush()
    time.sleep(0.3)

rig.rx.pause_evt.set()
time.sleep(0.03)
rig.s.reset_input_buffer()
rig.s.write(b"&")
rig.s.flush()
raw = bytearray()
t0 = time.time()
while time.time() - t0 < 2.0:
    d = rig.s.read(4096)
    if d:
        raw += d
    if b"REC end" in raw:
        break
rig.rx.buf = bytearray()
rig.rx.pause_evt.clear()

vals = []
for m in re.finditer(rb"\[REC\]((?:[0-9a-f]{4})+)", bytes(raw)):
    body = m.group(1)
    for k in range(0, len(body), 4):
        u = int(body[k:k + 4], 16)
        vals.append(u - 0x10000 if u >= 0x8000 else u)

print("点数 %d" % len(vals))
if vals:
    print("前 8 点(mV): %s" % vals[:8])
    print("min/max/mean = %d / %d / %.1f mV"
          % (min(vals), max(vals), sum(vals) / len(vals)))
rig.close()
