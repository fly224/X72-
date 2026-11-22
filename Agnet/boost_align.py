# -*- coding: utf-8 -*-
"""在线加大 Z 对齐电流/开环限幅，然后重试电角度对齐（不烧固件）。

背景：Z 报 ratio_bad，60 步只挪 0.05deg。开环限幅 0.06*vbus=1.85V，
除以相电阻 0.4ohm 就只给 4.6A 上限；2A 注入若被摩擦吃掉，转子不跟。
先试 align_id=4.0 / ol_vlim=0.10（上限 7.7A，仍低于 12A 跳闸阈值）。

用法：python boost_align.py COM27
"""
import re
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
sys.path.insert(0, r"G:\FOC\X72\Agnet")
import mavlink_proto as MV  # noqa: E402
import serial  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
MID = 30082
NAMES = ["align_id", "ol_vlim", "cur_kp", "cur_ki_ts", "spd_kp", "spd_ki_ts",
         "spd_iqmax", "pos_kp", "ff_acc", "ff_fric", "ff_visc", "dead_du",
         "cur_iqmax", "psi", "rsv14", "rsv15"]


def recv_30082(s, timeout=1.2):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = s.read(4096)
        if d:
            buf += d
        i = 0
        while i + 204 <= len(buf):
            if buf[i] == 0xFD:
                ln = buf[i + 1]
                mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
                if mid == MID and ln == 192:
                    return bytes(buf[i + 10:i + 10 + 192])
                i += 12 + ln
            else:
                i += 1
    return None


def read_params(s, seq):
    s.reset_input_buffer()
    s.write(MV.encode(30000, struct.pack("<I", MID), seq=seq))
    s.flush()
    p = recv_30082(s)
    if p is None:
        return None
    return list(struct.unpack_from("<16f", p, 0))


def write_params(s, seq, **kv):
    p = bytearray(192)
    mask = 0
    for name, val in kv.items():
        idx = NAMES.index(name)
        struct.pack_into("<f", p, idx * 4, float(val))
        mask |= (1 << idx)
    struct.pack_into("<I", p, 64, mask)      # Debug_uint32[0] = 写掩码
    s.reset_input_buffer()
    s.write(MV.encode(MID, bytes(p), seq=seq))
    s.flush()
    time.sleep(0.15)
    s.reset_input_buffer()


# ---------------- 1) 用 MAVLink 调参 ----------------
s = serial.Serial(PORT, 921600, timeout=0.05)
time.sleep(0.8)

v0 = read_params(s, 1)
if v0 is None:
    print("[!] 读不到 30082，放弃")
    s.close()
    sys.exit(1)
print("当前: align_id=%.3f  ol_vlim=%.3f" % (v0[0], v0[1]))

write_params(s, 2, align_id=4.0, ol_vlim=0.10)
time.sleep(0.2)
v1 = read_params(s, 3)
if v1 is None:
    print("[!] 写入后读不回")
    s.close()
    sys.exit(1)
print("写入后: align_id=%.3f  ol_vlim=%.3f" % (v1[0], v1[1]))
if abs(v1[0] - 4.0) > 1e-3 or abs(v1[1] - 0.10) > 1e-3:
    print("[!] 写入未生效，放弃")
    s.close()
    sys.exit(1)
s.close()

# ---------------- 2) 用 ASCII 重试 Z ----------------
time.sleep(0.4)
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

rig = Rig(PORT)
time.sleep(1.0)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)


def acmd(c, wait=0.9):
    rig.rx.pause_evt.set()
    time.sleep(0.03)
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
        return strip_frames(raw).decode("ascii", "replace")
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def passive(stops, timeout):
    rig.rx.pause_evt.set()
    time.sleep(0.03)
    try:
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < timeout:
            d = rig.s.read(4096)
            if d:
                buf += d
                txt = bytes(buf).decode("ascii", "replace")
                for w in stops:
                    if w in txt:
                        return txt, time.time() - t0
            else:
                time.sleep(0.01)
        return bytes(buf).decode("ascii", "replace"), time.time() - t0
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


st = acmd("S", 1.0)
m = re.findall(r"cal=(\d) z=(\d) f=(0x[0-9a-f]+)", st)
print("对齐前: cal/z/f =", m)

print("发 Z（4 A 注入，等最多 45 s）...")
txt, el = passive(["[CMD] IDENT ok", "[CMD] IDENT FAIL"], 45.0)
for line in txt.splitlines():
    if "IDENT" in line:
        print("  %.1fs  %s" % (el, line.strip()))

time.sleep(0.8)
st = acmd("S", 1.0)
m2 = re.findall(r"cal=(\d) z=(\d) f=(0x[0-9a-f]+)", st)
print("对齐后: cal/z/f =", m2)
if m2 and m2[0][1] == "1":
    print("[OK] z=1，速度模式解锁")
else:
    print("[NG] z 仍为 0")

rig.close()
