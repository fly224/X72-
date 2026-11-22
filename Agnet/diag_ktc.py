# -*- coding: utf-8 -*-
"""诊断 2：使能后连续采 ktc 与温度，核对两者是否同源、是否有启动瞬态。

不转电机（给定 =0，PWM 开但 iq=0），只观察 NTC 与补偿系数。
"""
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


def one(rig):
    txt = cmd(rig, "S", 0.5)
    ktc = None
    t_str = None
    for line in txt.splitlines():
        if "[CMD] st" in line:
            i = line.find("ktc=")
            if i >= 0:
                ktc = line[i + 4:].split()[0]
            break
    snap = rig.rx.get_snap()[0] or {}
    return ktc, snap.get("temp_mot"), snap.get("temp_drv"), snap.get("vbus")


rig = Rig("COM27")
time.sleep(1.4)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.4)

print("--- 未使能 ---")
for i in range(3):
    print("  ktc=%-8s T_mot=%.2f  T_drv=%.2f  v=%.2f" % one(rig))
    time.sleep(0.4)

print("--- E 使能，=0 保活 ---")
cmd(rig, "E", 0.6)
t0 = time.time()
while time.time() - t0 < 8.0:
    rig.s.write(b"=0\r")
    rig.s.flush()
    ktc, tm, td, vb = one(rig)
    print("  t=%4.1fs ktc=%-8s T_mot=%.2f  T_drv=%.2f  v=%.2f"
          % (time.time() - t0, ktc, tm, td, vb))
    time.sleep(0.4)

print("--- D 停机 ---")
cmd(rig, "D", 0.6)
for i in range(3):
    print("  ktc=%-8s T_mot=%.2f  T_drv=%.2f  v=%.2f" % one(rig))
    time.sleep(0.4)
rig.close()
