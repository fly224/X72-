# -*- coding: utf-8 -*-
"""Z 期间用 CRC 校验过的帧看编码器：判断 PWM 开通时读数是否被打坏。

只保留 CRC 正确的 30040/30025，避免 ASCII 交织导致的解析失步被误读成
"编码器跳变"（2026-11-20 曾因此误判）。

用法：python enc_crc.py COM27
"""
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
sys.path.insert(0, r"G:\FOC\X72_Agnet")
import mavlink_proto as MV  # noqa: E402
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"

CRC_EXTRA = {30040: 160, 30025: 191}   # 取自 mavlink_msgs.csv


def crc_extra_of(mid):
    return CRC_EXTRA.get(mid)


rig = Rig(PORT)
time.sleep(1.0)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)


def parse_valid(buf):
    """返回 [(mid, payload)]，只保留 CRC 正确的帧。"""
    out = []
    i = 0
    n = len(buf)
    while i + 12 <= n:
        if buf[i] != 0xFD:
            i += 1
            continue
        ln = buf[i + 1]
        total = 12 + ln + 2
        if i + total > n:
            break                      # 半帧，等下一次
        mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        pay = bytes(buf[i + 10:i + 10 + ln])
        got = buf[i + 10 + ln] | (buf[i + 11 + ln] << 8)
        ce = crc_extra_of(mid)
        ok = False
        if ce is not None:
            try:
                ok = (MV.crc_calculate(bytes(buf[i + 1:i + 10]) + pay, ce) == got)
            except Exception:
                ok = False
        if ok:
            out.append((mid, pay))
        i += total
    return out


def capture(sec, send_z=False):
    rig.rx.pause_evt.set()
    time.sleep(0.04)
    try:
        rig.s.reset_input_buffer()
        buf = bytearray()
        if send_z:
            rig.s.write(b"Z")
            rig.s.flush()
        t0 = time.time()
        while time.time() - t0 < sec:
            d = rig.s.read(8192)
            if d:
                buf += d
            else:
                time.sleep(0.005)
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()
    return parse_valid(bytes(buf))


print("== 静止基线 2 s ==")
a = capture(2.0)
enc_a = [struct.unpack_from("<q", p, 0)[0] for m, p in a if m == 30040 and len(p) >= 8]
cur_a = [(struct.unpack_from("<3f", p, 8)) for m, p in a if m == 30025 and len(p) >= 20]
print("  有效帧: 30040 x%d, 30025 x%d" % (len(enc_a), len(cur_a)))
if enc_a:
    print("  enc 范围 [%d, %d] 摆幅 %d counts (%.4f deg)"
          % (min(enc_a), max(enc_a), max(enc_a) - min(enc_a),
             (max(enc_a) - min(enc_a)) / 2097152.0 * 360.0))

print("== 发 Z，抓 6 s（PWM 已开通）==")
b = capture(6.0, send_z=True)
enc_b = [struct.unpack_from("<q", p, 0)[0] for m, p in b if m == 30040 and len(p) >= 8]
cur_b = [(struct.unpack_from("<3f", p, 8)) for m, p in b if m == 30025 and len(p) >= 20]
print("  有效帧: 30040 x%d, 30025 x%d" % (len(enc_b), len(cur_b)))
if enc_b:
    dif = [abs(enc_b[i] - enc_b[i - 1]) for i in range(1, len(enc_b))]
    print("  enc 范围 [%d, %d] 摆幅 %d counts (%.3f deg)"
          % (min(enc_b), max(enc_b), max(enc_b) - min(enc_b),
             (max(enc_b) - min(enc_b)) / 2097152.0 * 360.0))
    print("  最大相邻跳变 %d counts (%.4f deg)"
          % (max(dif) if dif else 0, (max(dif) if dif else 0) / 2097152.0 * 360.0))
    print("  enc 抽样: " + " ".join(str(enc_b[k]) for k in range(0, len(enc_b), max(1, len(enc_b) // 20))))

if cur_b:
    mu = sum(c[0] for c in cur_b) / len(cur_b)
    mv = sum(c[1] for c in cur_b) / len(cur_b)
    mw = sum(c[2] for c in cur_b) / len(cur_b)
    print("  三相均值 iu=%.3f iv=%.3f iw=%.3f" % (mu, mv, mw))
    # 用实测 iu / iw 幅值判断是否有一相开路
    au = max(abs(c[0]) for c in cur_b)
    av = max(abs(c[1]) for c in cur_b)
    aw = max(abs(c[2]) for c in cur_b)
    print("  三相峰值 |iu|=%.2f |iv|=%.2f |iw|=%.2f" % (au, av, aw))
    # iv 是否恒等于 -(iu+iw)（说明是算出来的，不是测的）
    err = max(abs(c[1] + c[0] + c[2]) for c in cur_b)
    print("  |iv + iu + iw| 最大 %.4f A  -> %s" % (err, "iv 为计算值" if err < 1e-3 else "iv 为实测值"))

rig.close()
