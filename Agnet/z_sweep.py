# -*- coding: utf-8 -*-
"""Z 的注入电流扫描：看每步机械角（mdeg）是否随电流上升。

判据（正常 = 8571 mdeg = 8.571°）：
  - 随电流近似线性上升 → 粘滑/摩擦限制，需更大转矩 → 机械侧
  - 恒定 ~615 不随电流变 → 几何/齿槽锁定，不是转矩问题
  - 到某电流饱和      → 硬限位或抱闸，到位即停

单连接复用：Rig 独占 COM 口，30082 与 ASCII 都走 rig.s 直接收发。

用法：python z_sweep.py COM27 [电流列表]   默认 2.0,3.0,4.0
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
CUR = [float(x) for x in (sys.argv[2].split(",") if len(sys.argv) > 2 else ["2.0", "3.0", "4.0"])]
MID = 30082
NAMES = ["align_id", "ol_vlim", "cur_kp", "cur_ki_ts", "spd_kp", "spd_ki_ts",
         "spd_iqmax", "pos_kp", "ff_acc", "ff_fric", "ff_visc", "dead_du",
         "cur_iqmax", "psi", "rsv14", "rsv15"]

rig = Rig(PORT)
time.sleep(1.0)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)


def xfer(data, wait=1.0):
    """独占端口收发一拍，返回原始字节（含 ASCII 与二进制）。"""
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


def pk_write(seq, **kv):
    p = bytearray(192)
    mask = 0
    for k, v in kv.items():
        idx = NAMES.index(k)
        struct.pack_into("<f", p, idx * 4, float(v))
        mask |= 1 << idx
    struct.pack_into("<I", p, 64, mask)
    xfer(MV.encode(MID, bytes(p), seq=seq), 0.35)


def acmd(c, wait=1.0, tries=4):
    for _ in range(tries):
        txt = strip_frames(xfer(c.encode(), wait)).decode("ascii", "replace")
        if "[CMD]" in txt:
            return txt
        time.sleep(0.25)
    return ""


def run_z(stops, timeout):
    """发 Z 并被动收，直到出现结束标记。"""
    rig.rx.pause_evt.set()
    time.sleep(0.04)
    try:
        rig.s.reset_input_buffer()
        buf = bytearray()
        t0 = time.time()
        rig.s.write(b"Z")
        rig.s.flush()
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


v = pk_read(1)
print("初始: align_id=%.2f ol_vlim=%.3f" % (v[0], v[1]) if v else "30082 无应答")

rows = []
for k, a in enumerate(CUR):
    pk_write(10 + k, align_id=a, ol_vlim=0.10)
    v = pk_read(20 + k)
    real = v[0] if v else float("nan")

    acmd("X", 0.9)
    time.sleep(0.3)
    txt = run_z(["[CMD] IDENT ok", "[CMD] IDENT FAIL"], 55.0)

    print("\n--- 设定 %.2f A（回读 %.2f） ---" % (a, real))
    for l in txt.splitlines():
        if "IDENT" in l:
            print("  " + l.strip())
    m = re.search(r"ratio=(\d+) res=([\d.]+) mdeg=([\d.]+)/([\d.]+)/([\d.]+)", txt)
    if m:
        rows.append((real, int(m.group(1)), float(m.group(3)), float(m.group(4)), float(m.group(5))))
    else:
        rows.append((real, -1, float("nan"), float("nan"), float("nan")))
    time.sleep(0.6)

pk_write(90, align_id=2.0, ol_vlim=0.06)
v = pk_read(91)
if v:
    print("\n恢复默认: align_id=%.2f ol_vlim=%.3f" % (v[0], v[1]))

print("\n=== 注入电流 vs 每步机械角（正常 8571 mdeg） ===")
print("  I(A)  ratio   avg(mdeg)   min      max")
for r in rows:
    print("  %4.1f  %5d   %8.1f  %7.1f  %7.1f" % r)

rig.close()
