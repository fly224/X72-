# -*- coding: utf-8 -*-
"""确认遥测帧集合 + 读编码器累积值 + 恢复 30082 默认值。

用法：python chk_enc.py COM27
"""
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
import mavlink_proto as MV  # noqa: E402
import serial  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
MID = 30082
NAMES = ["align_id", "ol_vlim", "cur_kp", "cur_ki_ts", "spd_kp", "spd_ki_ts",
         "spd_iqmax", "pos_kp", "ff_acc", "ff_fric", "ff_visc", "dead_du",
         "cur_iqmax", "psi", "rsv14", "rsv15"]

s = serial.Serial(PORT, 921600, timeout=0.05)
time.sleep(0.6)

# ---- 1) 帧统计 ----
s.reset_input_buffer()
buf = bytearray()
t0 = time.time()
while time.time() - t0 < 1.5:
    d = s.read(16384)
    if d:
        buf += d

stat = {}
i = 0
while i + 12 <= len(buf):
    if buf[i] == 0xFD:
        ln = buf[i + 1]
        mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        stat[mid] = stat.get(mid, 0) + 1
        i += 12 + ln
    else:
        i += 1
print("遥测帧统计 (%d 字节):" % len(buf))
for k in sorted(stat, key=lambda x: -stat[x])[:12]:
    print("  %d  x%d" % (k, stat[k]))

# ---- 2) 编码器 ----
def enc_once(timeout=1.0):
    s.reset_input_buffer()
    b = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = s.read(8192)
        if d:
            b += d
    i = 0
    while i + 24 <= len(b):
        if b[i] == 0xFD:
            ln = b[i + 1]
            mid = b[i + 7] | (b[i + 8] << 8) | (b[i + 9] << 16)
            if mid == 30040 and ln >= 8:
                e = struct.unpack_from("<q", b, i + 10)[0]
                ang = struct.unpack_from("<f", b, i + 18)[0] if ln >= 12 else float("nan")
                return e, ang
            i += 12 + ln
        else:
            i += 1
    return None, None


e1, a1 = enc_once()
time.sleep(1.5)
e2, a2 = enc_once()
print("编码器: enc_sum %s -> %s ; 电角度 %.3f -> %.3f" % (e1, e2, a1 or 0, a2 or 0))

# ---- 3) 恢复默认 ----
def recv(timeout=1.2):
    b = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = s.read(4096)
        if d:
            b += d
        i = 0
        while i + 204 <= len(b):
            if b[i] == 0xFD:
                ln = b[i + 1]
                mid = b[i + 7] | (b[i + 8] << 8) | (b[i + 9] << 16)
                if mid == MID and ln == 192:
                    return bytes(b[i + 10:i + 202])
                i += 12 + ln
            else:
                i += 1
    return None


s.reset_input_buffer()
s.write(MV.encode(30000, struct.pack("<I", MID), seq=21))
s.flush()
p = recv()
if p:
    v = struct.unpack_from("<16f", p, 0)
    print("当前 align_id=%.3f ol_vlim=%.3f" % (v[0], v[1]))

    q = bytearray(192)
    struct.pack_into("<f", q, 0, 2.0)      # align_id 默认
    struct.pack_into("<f", q, 4, 0.06)     # ol_vlim 默认
    struct.pack_into("<I", q, 64, (1 << 0) | (1 << 1))
    s.reset_input_buffer()
    s.write(MV.encode(MID, bytes(q), seq=22))
    s.flush()
    time.sleep(0.2)
    s.reset_input_buffer()
    s.write(MV.encode(30000, struct.pack("<I", MID), seq=23))
    s.flush()
    p2 = recv()
    if p2:
        v2 = struct.unpack_from("<16f", p2, 0)
        print("恢复后 align_id=%.3f ol_vlim=%.3f" % (v2[0], v2[1]))
else:
    print("[!] 30082 无应答")

s.close()
