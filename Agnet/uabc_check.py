# -*- coding: utf-8 -*-
"""看三相电压指令 ua/ub/uc：判断固件到底有没有驱动第三相。

健康三相调制：ua/ub/uc 都是正弦、幅值相近、互差 120°，三路 RMS 接近。
若某一相固件没在调制（占空比恒 0.5）：该路 u = 0 恒定，RMS ≈ 0。

用法：python uabc_check.py COM27
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
CE = {30025: 191}


def frames(buf):
    out = []
    i = 0
    n = len(buf)
    while i + 14 <= n:
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
            if ln >= 40:
                iu, iv, iw = struct.unpack_from("<3f", pay, 8)
                ua, ub, uc = struct.unpack_from("<3f", pay, 28)
                out.append((iu, iv, iw, ua, ub, uc))
        i += 14 + ln
    return out


rig = Rig(PORT)
time.sleep(1.0)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)

rig.rx.pause_evt.set()
time.sleep(0.04)
rig.s.reset_input_buffer()
rig.s.write(b"X")
rig.s.flush()
time.sleep(0.8)
rig.rx.buf = bytearray()
rig.rx.pause_evt.clear()
time.sleep(0.4)

rig.rx.pause_evt.set()
time.sleep(0.04)
rig.s.reset_input_buffer()
buf = bytearray()
t0 = time.time()
rig.s.write(b"Z")
rig.s.flush()
while time.time() - t0 < 6.0:
    d = rig.s.read(8192)
    if d:
        buf += d
    else:
        time.sleep(0.004)
rig.rx.buf = bytearray()
rig.rx.pause_evt.clear()

fr = frames(bytes(buf))
print("有效帧: %d" % len(fr))
if len(fr) < 30:
    print("[!] 帧太少")
    rig.close()
    sys.exit(1)

# 只取有明显电流的帧
act = [f for f in fr if max(abs(f[0]), abs(f[2])) > 1.0]
print("有电流的帧: %d" % len(act))
if len(act) < 20:
    act = fr


def stat(name, vals):
    n = len(vals)
    rms = math.sqrt(sum(v * v for v in vals) / n)
    mn, mx = min(vals), max(vals)
    span = mx - mn
    # 不同取值个数（粗量化到 10 mV），用来识别"恒定"
    lv = len(set(round(v, 2) for v in vals))
    print("  %s: rms=%6.2f V  范围[%7.2f, %7.2f]  摆幅 %6.2f V  不同值 %d/%d"
          % (name, rms, mn, mx, span, lv, n))
    return rms, span, lv


print("\n三相电压指令（duty-0.5)*vbus：")
ru, su, lu = stat("ua(U)", [f[3] for f in act])
rv, sv, lv2 = stat("ub(V)", [f[4] for f in act])
rw, sw, lw = stat("uc(W)", [f[5] for f in act])

print("\n三相电流：")
stat("iu", [f[0] for f in act])
stat("iv", [f[1] for f in act])
stat("iw", [f[2] for f in act])

print("\n判定：")
if min(ru, rv, rw) > 0.3 * max(ru, rv, rw) and min(su, sv, sw) > 0.5:
    print("  [固件正常] 三路占空比都在变化、幅值相近 —— 固件确实驱动了三相")
    print("  -> 问题在功率级之后（栅极驱动 / 相线 / 连接器 / 电机）")
else:
    bad = []
    if rv < 0.3 * max(ru, rw) or sv < 0.5:
        bad.append("V")
    if ru < 0.3 * max(rv, rw) or su < 0.5:
        bad.append("U")
    if rw < 0.3 * max(ru, rv) or sw < 0.5:
        bad.append("W")
    print("  [固件异常] 这些相的占空比几乎不变：%s" % (",".join(bad) if bad else "?"))
    print("  -> 是软件/时序问题，不是硬件")

rig.close()
