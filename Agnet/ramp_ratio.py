# -*- coding: utf-8 -*-
"""静止升流窗口的相电流比：区分"第三相开路"与"驱动/采样异常"。

Z 的 RAMP 段虚拟电角固定为 0，id 从 0 线性升到 id_ref，此时对健康三相：
    iu = +I,  iv = iw = -I/2   ->   iw/iu = -0.5
若第三相无电流（KCL: iu+iv+iw=0 且 iv=0）：
    iw/iu = -1.0
再往后进入 STEP（虚拟角开始步进/拖动），比例会旋转，不再有固定值，
所以只取"电流刚开始上升的那 ~150 ms"。

用法：python ramp_ratio.py COM27
"""
import statistics
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
sys.path.insert(0, r"G:\FOC\X72_Agnet")
import mavlink_proto as MV  # noqa: E402
from thermal_soak import Rig, MSG_PERMISSION  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
CE = {30025: 191}


def valid_30025(buf):
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
            if ln >= 20:
                out.append(struct.unpack_from("<3f", pay, 8))
        i += 14 + ln
    return out


rig = Rig(PORT)
time.sleep(1.0)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)

# 清故障，保证 Z 能启动
rig.rx.pause_evt.set()
time.sleep(0.04)
rig.s.reset_input_buffer()
rig.s.write(b"X")
rig.s.flush()
time.sleep(0.8)
rig.rx.buf = bytearray()
rig.rx.pause_evt.clear()
time.sleep(0.4)

# 触发 Z 并抓 1.6 s
rig.rx.pause_evt.set()
time.sleep(0.04)
rig.s.reset_input_buffer()
buf = bytearray()
t0 = time.time()
rig.s.write(b"Z")
rig.s.flush()
while time.time() - t0 < 1.6:
    d = rig.s.read(8192)
    if d:
        buf += d
    else:
        time.sleep(0.004)
rig.rx.buf = bytearray()
rig.rx.pause_evt.clear()

cur = valid_30025(bytes(buf))
print("有效 30025 帧: %d" % len(cur))

# 找到电流开始上升的位置
start = None
for k, c in enumerate(cur):
    if abs(c[0]) > 0.25:
        start = k
        break

if start is None:
    print("[!] 1.6 s 内没看到电流上升（Z 可能没启动或电流太小）")
    rig.close()
    sys.exit(1)

print("电流起始于第 %d 帧（之后 %d 帧）" % (start, len(cur) - start))

# 取起始后 3~13 帧（约 30~130 ms，仍在 RAMP 内、角度=0）
win = cur[start + 3:start + 13]
if len(win) < 4:
    win = cur[start + 1:start + 6]

print("\n  帧   iu(A)    iw(A)    iw/iu")
ratios = []
for k, c in enumerate(win):
    iu, iv, iw = c
    r = iw / iu if abs(iu) > 0.15 else float("nan")
    if r == r:
        ratios.append(r)
    print("  %2d  %7.3f  %7.3f   %+.3f" % (k, iu, iw, r))

if ratios:
    med = statistics.median(ratios)
    print("\n  中位数 iw/iu = %+.3f" % med)
    print("  健康三相应为 -0.500 ；第三相无电流应为 -1.000")
    if med < -0.80:
        print("  [结论] 第三相无电流（与 corr=-0.997 一致）")
    elif -0.65 < med < -0.35:
        print("  [结论] 三相都在导通 —— 之前的判断需要修正！")
    else:
        print("  [结论] 介于两者之间，数据不足以下结论，需更多样本")

rig.close()
