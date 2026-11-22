# -*- coding: utf-8 -*-
"""用 1000 rpm 的稳态积分项闭环标定黏滞前馈。

原理：稳态时 iq_cmd = ff + PI 输出；若 ff 恰好等于总摩擦，积分项 iterm → 0。
所以把 iterm 折算回电流再修正 visc：
    visc_new = visc_old - iterm / ω
fric 保持不动（它是常值项，修正它会污染低速）。

用法：
  python ff_tune_1000.py COM27 0.0889 2.5e-3      # 写入并测一轮
  python ff_tune_1000.py COM27                    # 只测（不改参数）
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
FRIC = float(sys.argv[2]) if len(sys.argv) > 2 else None
VISC = float(sys.argv[3]) if len(sys.argv) > 3 else None
RPM = 1000.0
TWO_PI = 6.283185307
MID = 30082
NAMES = ["align_id", "ol_vlim", "cur_kp", "cur_ki_ts", "spd_kp", "spd_ki_ts",
         "spd_iqmax", "pos_kp", "ff_acc", "ff_fric", "ff_visc", "dead_du",
         "cur_iqmax", "psi", "rsv14", "rsv15"]

rig = Rig(PORT)
time.sleep(1.0)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)


def xfer(data, wait=1.0):
    rig.rx.pause_evt.set()
    time.sleep(0.04)
    try:
        rig.s.reset_input_buffer()
        rig.s.write(data if isinstance(data, bytes) else data.encode())
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
    raw = xfer(MV.encode(30000, struct.pack("<I", MID), seq=seq), 1.1)
    i = 0
    while i + 204 <= len(raw):
        if raw[i] == 0xFD:
            ln = raw[i + 1]
            if ln == 192 and (raw[i + 7] | (raw[i + 8] << 8) | (raw[i + 9] << 16)) == MID:
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
    xfer(MV.encode(MID, bytes(p), seq=seq), 0.4)


def acmd(c, wait=1.0, tries=4):
    for _ in range(tries):
        t = strip_frames(xfer(c.encode() if isinstance(c, str) else c, wait)).decode("ascii", "replace")
        if "[CMD]" in t:
            return t
        time.sleep(0.2)
    return ""


v = None
for k in range(4):
    v = pk_read(50 + k)
    if v:
        break
    time.sleep(0.3)
if v:
    print("当前: ff_fric=%.4f  ff_visc=%.5f  ff_acc=%.5f" % (v[9], v[10], v[8]))

if FRIC is not None and VISC is not None:
    pk_write(60, ff_fric=FRIC, ff_visc=VISC)
    v2 = None
    for k in range(3):
        v2 = pk_read(61 + k)
        if v2:
            break
        time.sleep(0.3)
    if v2:
        print("写入后: ff_fric=%.4f  ff_visc=%.5f" % (v2[9], v2[10]))

# ---- 跑 1000 rpm ----
acmd("D", 1.0)
time.sleep(0.5)
acmd("E", 1.3)
time.sleep(0.6)

rig.rx.pause_evt.set()
time.sleep(0.04)
rig.s.reset_input_buffer()
t0 = time.time()
last = 0.0
while True:
    now = time.time() - t0
    if now >= 12.0:
        break
    if now - last > 0.4:
        rig.s.write(("=%g\r" % RPM).encode())
        rig.s.flush()
        last = now
    rig.s.read(4096)          # 清空流，避免缓冲堆积
    time.sleep(0.01)
rig.rx.buf = bytearray()
rig.rx.pause_evt.clear()

k_txt = acmd("K", 1.1)
acmd("D", 1.2)


def g(pat):
    m = re.search(pat, k_txt)
    return float(m.group(1)) if m else float("nan")


w = g(r"wmeas=(-?[\d.]+)rpm") * TWO_PI / 60.0
iterm = g(r"iterm=(-?[\d.]+)")
iqc = g(r"iqc=(-?[\d.]+)")
print("\n1000 rpm 稳态: wmeas=%.2f rpm  iqc=%+.4f A  iterm=%+.4f A  累计 iq=%+.4f A"
      % (w * 60 / TWO_PI, iqc, iterm, iqc + iterm))
if v:
    ff = v[9] + v[10] * w
    print("前馈提供: %.4f A  (fric %.4f + visc %.4f)" % (ff, v[9], v[10] * w))
    print("若 iterm 应归零，则 visc 修正为: %.5f  (iterm>0 = 补得不够, 要加)" % (v[10] + iterm / w))

rig.close()
