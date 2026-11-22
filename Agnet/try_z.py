# -*- coding: utf-8 -*-
"""一次会话内：C 电流零点 → 设 align_id → Z 对齐 → 复核。

用法：python try_z.py COM27 [电流A]
"""
import re
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
sys.path.insert(0, r"G:\FOC\X72_Agnet")
import mavlink_proto as MV  # noqa: E402
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
AMP = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
MID = 30082
NAMES = ["align_id", "ol_vlim", "cur_kp", "cur_ki_ts", "spd_kp", "spd_ki_ts",
         "spd_iqmax", "pos_kp", "ff_acc", "ff_fric", "ff_visc", "dead_du",
         "cur_iqmax", "psi", "rsv14", "rsv15"]

rig = Rig(PORT)
time.sleep(1.1)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)


def xfer(data, wait=1.0):
    rig.rx.pause_evt.set()
    time.sleep(0.04)
    try:
        rig.s.reset_input_buffer()
        rig.s.write(data)
        rig.s.flush()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < wait:
            d = rig.s.read(8192)
            if d:
                buf += d
        return bytes(buf)
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def pk_write(seq, **kv):
    p = bytearray(192)
    mask = 0
    for k, v in kv.items():
        idx = NAMES.index(k)
        struct.pack_into("<f", p, idx * 4, float(v))
        mask |= 1 << idx
    struct.pack_into("<I", p, 64, mask)
    xfer(MV.encode(MID, bytes(p), seq=seq), 0.35)


def pk_read(seq):
    raw = xfer(MV.encode(30000, struct.pack("<I", MID), seq=seq), 1.2)
    i = 0
    while i + 204 <= len(raw):
        if raw[i] == 0xFD:
            ln = raw[i + 1]
            if ln == 192:
                mid = raw[i + 7] | (raw[i + 8] << 8) | (raw[i + 9] << 16)
                if mid == MID:
                    return struct.unpack_from("<16f", raw, i + 10)
            i += 12 + ln if ln else 1
        else:
            i += 1
    return None


def acmd(c, wait=1.0, tries=4):
    for _ in range(tries):
        txt = strip_frames(xfer(c.encode(), wait)).decode("ascii", "replace")
        if "[CMD]" in txt:
            return txt
        time.sleep(0.25)
    return ""


def passive(stops, timeout):
    rig.rx.pause_evt.set()
    time.sleep(0.04)
    try:
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < timeout:
            d = rig.s.read(4096)
            if d:
                buf += d
                txt = strip_frames(bytes(buf)).decode("ascii", "replace")
                for w in stops:
                    if w in txt:
                        return txt
            else:
                time.sleep(0.01)
        return strip_frames(bytes(buf)).decode("ascii", "replace")
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def state():
    for _ in range(5):
        s = acmd("S", 1.0)
        m = re.search(r"cal=(\d) z=(\d).*?f=(0x[0-9a-fA-F]+)", s)
        if m:
            return m.groups(), s
        time.sleep(0.25)
    return ("?", "?", "?"), ""


print("[1] C 电流零点")
print("  " + acmd("C", 0.9).strip()[:70])
time.sleep(3.2)
(c, z, f), s = state()
print("  -> cal=%s z=%s f=%s" % (c, z, f))

print("[2] 设 align_id=%.2f" % AMP)
pk_write(31, align_id=AMP, ol_vlim=0.10)
v = pk_read(32)
print("  回读 align_id=%.2f ol_vlim=%.3f" % (v[0], v[1]) if v else "  无应答")

print("[3] X 清故障")
acmd("X", 0.9)
time.sleep(0.3)

print("[4] Z（约 22 s）")
txt = ""
rig.rx.pause_evt.set()
time.sleep(0.04)
try:
    rig.s.reset_input_buffer()
    buf = bytearray()
    t0 = time.time()
    rig.s.write(b"Z")
    rig.s.flush()
    while time.time() - t0 < 60.0:
        d = rig.s.read(4096)
        if d:
            buf += d
            t = strip_frames(bytes(buf)).decode("ascii", "replace")
            if "[CMD] IDENT ok" in t or "[CMD] IDENT FAIL" in t:
                txt = t
                break
        else:
            time.sleep(0.01)
    else:
        txt = strip_frames(bytes(buf)).decode("ascii", "replace")
finally:
    rig.rx.buf = bytearray()
    rig.rx.pause_evt.clear()

for l in txt.splitlines():
    if "IDENT" in l:
        print("  " + l.strip())

time.sleep(1.0)
(c, z, f), s = state()
print("[5] 结果 cal=%s z=%s f=%s" % (c, z, f))
if s:
    print("  " + s.strip()[:190])
print("[%s]" % ("OK z=1，速度模式解锁" if z == "1" else "NG z 仍为 0"))

rig.close()
