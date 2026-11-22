# -*- coding: utf-8 -*-
"""量化验证"V 相无电流"：统计 iu、iw、iu+iw 的 RMS 与相关系数。

平衡三相下，若三相都有电流：
    rms(iu) ≈ rms(iw) ≈ rms(iu+iw)        （iu、iw 相差 120°，和仍是一个正弦）
若 V 相开路（电流只在 U-W 之间流动）：
    iu ≈ -iw  ->  rms(iu+iw) << rms(iu)，相关系数 ≈ -1

用法：python vphase_check.py COM27
"""
import math
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
sys.path.insert(0, r"G:\FOC\X72_Agnet")
import mavlink_proto as MV  # noqa: E402
from thermal_soak import Rig, MSG_PERMISSION  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
CE = {30040: 160, 30025: 191}

rig = Rig(PORT)
time.sleep(1.0)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)


def capture(sec, send=None):
    rig.rx.pause_evt.set()
    time.sleep(0.04)
    try:
        rig.s.reset_input_buffer()
        buf = bytearray()
        if send:
            rig.s.write(send)
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

    out = []
    i = 0
    n = len(buf)
    while i + 12 <= n:
        if buf[i] != 0xFD:
            i += 1
            continue
        ln = buf[i + 1]
        if i + 14 + ln > n:
            break
        mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        pay = bytes(buf[i + 10:i + 10 + ln])
        got = buf[i + 10 + ln] | (buf[i + 11 + ln] << 8)
        ce = CE.get(mid)
        if ce is not None and MV.crc_calculate(bytes(buf[i + 1:i + 10]) + pay, ce) == got:
            if mid == 30025 and ln >= 20:
                out.append(struct.unpack_from("<3f", pay, 8))
        i += 14 + ln
    return out


print("抓 Z 期间 6 s 的三相电流 ...")
# 2026-11-21：ASCII 已关，Z 改走 30069 action=5。原来发 b"Z" 在 ASCII 关闭后
# 什么都不会发生 —— 脚本会安静地抓 6 s 静止数据，然后得出"三相平衡"的假结论。
cur = capture(6.0, send=MV.encode(30069, bytes([5]), seq=77))
print("有效 30025 帧: %d" % len(cur))

if len(cur) < 50:
    print("[!] 帧太少，结论不可靠")
else:
    iu = [c[0] for c in cur]
    iw = [c[2] for c in cur]
    sm = [iu[k] + iw[k] for k in range(len(cur))]

    def rms(v):
        return math.sqrt(sum(x * x for x in v) / len(v))

    ru, rw, rs = rms(iu), rms(iw), rms(sm)
    mu = sum(iu) / len(iu)
    mw = sum(iw) / len(iw)
    su = math.sqrt(sum((x - mu) ** 2 for x in iu) / len(iu))
    sw = math.sqrt(sum((x - mw) ** 2 for x in iw) / len(iw))
    cov = sum((iu[k] - mu) * (iw[k] - mw) for k in range(len(iu))) / len(iu)
    r = cov / (su * sw) if su > 1e-9 and sw > 1e-9 else float("nan")

    print("  rms(iu)   = %.3f A   max|iu| = %.3f" % (ru, max(abs(x) for x in iu)))
    print("  rms(iw)   = %.3f A   max|iw| = %.3f" % (rw, max(abs(x) for x in iw)))
    print("  rms(iu+iw)= %.3f A   max|iu+iw| = %.3f" % (rs, max(abs(x) for x in sm)))
    print("  比值 rms(iu+iw)/rms(iu) = %.3f   (平衡三相应 ≈1.0；V 相开路应 ≈0)" % (rs / ru if ru > 1e-9 else float("nan")))
    print("  相关系数 corr(iu, iw)   = %+.3f  (V 相开路应 ≈ -1.0)" % r)

    if rs / ru < 0.3 if ru > 1e-9 else False:
        print("\n[结论] V 相无电流 —— 电流只在 U/W 之间流动，磁场是脉动的，不产生平均转矩")
    else:
        print("\n[结论] 三相都有电流，V 相正常")

rig.close()
