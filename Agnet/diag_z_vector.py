# -*- coding: utf-8 -*-
"""Z 期间抓 iabc + 编码器，判断"电流矢量是否在转"。

判据：
  - 电流矢量角 αβ 随时间递增 → 磁场在转 → 转子不跟 = 机械/接线
  - 电流矢量角近似恒定      → 磁场静止 = 虚拟角没被应用（软件）
  - 同时看 enc_sum 的净位移，确认转子到底动没动

用法：python diag_z_vector.py COM27
"""
import math
import struct
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"

s = serial.Serial(PORT, 921600, timeout=0.02)


def parse(buf):
    """返回 [(t, iu, iv, iw, enc), ...]，时间用帧序号近似（10 Hz 统一节拍）。"""
    out = []
    i = 0
    n = len(buf)
    t = 0.0
    while i < n - 12:
        if buf[i] == 0xFD:
            ln = buf[i + 1]
            mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            pay = i + 10
            if mid == 30025 and ln >= 28 and pay + 20 <= n:
                iu, iv, iw = struct.unpack_from("<3f", buf, pay + 8)
                out.append(("i", iu, iv, iw))
            elif mid == 30040 and ln >= 16 and pay + 18 <= n:
                enc = struct.unpack_from("<q", buf, pay)[0]
                out.append(("e", enc))
            i += 12 + ln
        else:
            i += 1
    return out


def capture(sec):
    s.reset_input_buffer()
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        d = s.read(8192)
        if d:
            buf += d
    return parse(buf)


# ---- 静止基线 ----
base = capture(0.8)
enc_b = [x[1] for x in base if x[0] == "e"]
e0 = enc_b[-1] if enc_b else None
print("静止基线: enc_sum=%s" % e0)

# ---- 发 Z 并抓全过程 ----
s.reset_input_buffer()
s.write(b"Z")
s.flush()
rec = capture(14.0)

cur = [x for x in rec if x[0] == "i"]
enc = [x for x in rec if x[0] == "e"]
print("采到 iabc %d 帧, enc %d 帧" % (len(cur), len(enc)))

if enc:
    e1 = enc[0][1]
    e2 = enc[-1][1]
    print("enc_sum: %d -> %d  净位移 %.5f 圈 (%.2f deg)"
          % (e0 if e0 is not None else 0, e2,
             (e2 - (e0 or 0)) / 2097152.0,
             (e2 - (e0 or 0)) / 2097152.0 * 360.0))

if len(cur) >= 10:
    # Clarke: ialpha = iu, ibeta = (iu + 2*iv)/sqrt(3)
    ang = []
    amp = []
    for _, iu, iv, iw in cur:
        ia = iu
        ib = (iu + 2.0 * iv) / math.sqrt(3.0)
        ang.append(math.atan2(ib, ia))
        amp.append(math.hypot(ia, ib))

    # 展开
    un = [ang[0]]
    for k in range(1, len(ang)):
        d = ang[k] - ang[k - 1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        un.append(un[-1] + d)

    span = un[-1] - un[0]
    print("电流矢量角: 起 %.3f rad 终 %.3f rad  净转 %.3f rad (%.1f deg)"
          % (un[0], un[-1], span, math.degrees(span)))
    print("幅值: 均值 %.3f A  峰值 %.3f A" % (sum(amp) / len(amp), max(amp)))

    # 打印每 0.5 s 的抽样角，看是否分段台阶
    step = max(1, len(un) // 20)
    print("矢量角抽样(rad): " + " ".join("%.2f" % un[k] for k in range(0, len(un), step)))
else:
    print("iabc 帧太少，无法算矢量角")

s.close()
