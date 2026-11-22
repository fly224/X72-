# -*- coding: utf-8 -*-
"""一次性诊断：读 S/K 行原文 + 遥测快照，核对 ktc 与温度是否同源。"""
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402


def cmd(rig, c, wait=0.6):
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

print("=== S ===")
print(cmd(rig, "S", 0.7))
print("=== K ===")
print(cmd(rig, "K", 0.7))
snap = rig.rx.get_snap()[0]
print("=== 遥测 ===")
for k in ("temp_mot", "temp_drv", "vbus", "iq_ref", "w_elec", "spd_ref_rpm"):
    print("  %-12s %s" % (k, snap.get(k)))
print("=== '  录波设置回显 ===")
print(cmd(rig, "'", 0.7))
rig.close()
