# -*- coding: utf-8 -*-
"""Z 全过程连续监视 enc_sum（100 Hz），判断转子是否真的在跟随。

输出：每 0.2 s 一个 enc 采样，最后给净位移与台阶数。
若 enc 呈阶梯上升 → 转子在跟，ratio 判据有问题；
若 enc 近似常数   → 转子没跟，属机械/接线/抱闸。

用法：python watch_z.py COM27
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


print("S:", (acmd("S", 1.0).strip() or "(空)")[:170])

# 直接抓流：暂停 rx 线程，自己解 30040
rig.rx.pause_evt.set()
time.sleep(0.05)
rig.s.reset_input_buffer()

buf = bytearray()
t0 = time.time()
rig.s.write(b"Z")
rig.s.flush()

samples = []          # (t, enc)
ascii_txt = bytearray()
while time.time() - t0 < 16.0:
    d = rig.s.read(8192)
    if d:
        buf += d
    # 解帧
    while len(buf) >= 24:
        if buf[0] == 0xFD:
            ln = buf[1]
            if len(buf) < 12 + ln:
                break
            mid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
            if mid == 30040 and ln >= 8:
                e = struct.unpack_from("<q", bytes(buf), 10)[0]
                samples.append((time.time() - t0, e))
            del buf[:12 + ln]
        else:
            # 非帧字节：当作 ASCII 收集
            ascii_txt.append(buf[0])
            del buf[:1]

rig.rx.buf = bytearray()
rig.rx.pause_evt.clear()

print("enc 采样 %d 个（%.1f Hz）" % (len(samples), len(samples) / 16.0))
if samples:
    e0 = samples[0][1]
    emin = min(s[1] for s in samples)
    emax = max(s[1] for s in samples)
    print("enc 起 %d  最小 %d  最大 %d  终 %d" % (e0, emin, emax, samples[-1][1]))
    print("  → 摆幅 %d counts = %.3f deg（2^21/rev）"
          % (emax - emin, (emax - emin) / 2097152.0 * 360.0))
    print("  → 净位移 %d counts = %.3f deg"
          % (samples[-1][1] - e0, (samples[-1][1] - e0) / 2097152.0 * 360.0))
    step = max(1, len(samples) // 24)
    print("enc 抽样: " + " ".join(str(samples[k][1]) for k in range(0, len(samples), step)))

txt = ascii_txt.decode("ascii", "replace")
hits = [l.strip() for l in txt.splitlines() if "[CMD]" in l]
print("ASCII 回显:")
for h in hits[-6:]:
    print("  " + h)
if not hits:
    print("  (无)")

time.sleep(0.4)
print("S:", (acmd("S", 1.0).strip() or "(空)")[:170])
rig.close()
