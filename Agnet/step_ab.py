# -*- coding: utf-8 -*-
"""0→1000 rpm 阶跃：抓上升时间与过冲，可选先设加速度前馈。

采样用 30025 的 `elec_angle_speed_rad_s`（+44，100 Hz），除以 14 得机械角速度。
为什么不用 `K` 行：K 查询一次阻塞 0.3 s，会把这期间的帧全挤到同一个时间戳上，
差分还原转速直接失真（上次被算成 379 rpm）。
为什么不用编码器差分：同样是多帧共用时间戳的问题。
为什么不让 `=` 重复保活：deadman 2 s，阶跃后 1.9 s 内不发第二条，避免
`=` 处理器重置斜坡污染阶跃响应。

用法：
  python step_ab.py COM27                 # 不改参数，抓基线
  python step_ab.py COM27 --accff 55      # 先发 ]55（单位 1e-4）
"""
import argparse
import math
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\X72_Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

POLE = 14.0
TWO_PI = 6.283185307

ap = argparse.ArgumentParser()
ap.add_argument("port", nargs="?", default="COM27")
ap.add_argument("--accff", type=float, default=None, help="先发 ]<v>，单位 1e-4")
ap.add_argument("--rpm", type=float, default=1000.0)
ap.add_argument("--win", type=float, default=1.9, help="阶跃后采样时长 s")
ap.add_argument("--tag", default="")
a = ap.parse_args()

rig = Rig(a.port)
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
        return strip_frames(bytes(buf)).decode("ascii", "replace")
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def acmd(c, wait=1.0, tries=4):
    for _ in range(tries):
        t = xfer(c, wait)
        if "[CMD]" in t:
            return t
        time.sleep(0.2)
    return ""


if a.accff is not None:
    print("设加速度前馈:", [l.strip() for l in acmd("]%g\r" % a.accff, 1.0).splitlines()
                           if "[CMD]" in l][:1])

print("D :", [l.strip() for l in acmd("D", 1.0).splitlines() if "[CMD]" in l][:1])
time.sleep(0.8)
print("E :", [l.strip() for l in acmd("E", 1.3).splitlines() if "[CMD]" in l][:1])
time.sleep(1.0)

# --- 阶跃并流式采样 ---
rig.rx.pause_evt.set()
time.sleep(0.04)
rig.s.reset_input_buffer()
buf = bytearray()
series = []          # (t, elec_rad_s)
t0 = time.time()
rig.s.write(("=%g\r" % a.rpm).encode())
rig.s.flush()
while True:
    now = time.time() - t0
    if now >= a.win:
        break
    d = rig.s.read(4096)
    if d:
        buf += d
    while len(buf) >= 14:
        if buf[0] == 0xFD:
            ln = buf[1]
            if len(buf) < 14 + ln:
                break
            mid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
            if mid == 30025 and ln >= 48:
                es = struct.unpack_from("<f", bytes(buf), 10 + 44)[0]
                iqr = struct.unpack_from("<f", bytes(buf), 10 + 4)[0]
                series.append((time.time() - t0, es, iqr))
            del buf[:14 + ln]
        else:
            del buf[:1]

rig.rx.buf = bytearray()
rig.rx.pause_evt.clear()

print("D :", [l.strip() for l in acmd("D", 1.2).splitlines() if "[CMD]" in l][:1])

if len(series) < 50:
    print("[!] 样本只有 %d，无法评估" % len(series))
    rig.close()
    sys.exit(1)

rpm = [(t, abs(es) / POLE * 60.0 / TWO_PI, iqr) for t, es, iqr in series]
vals = [v for _, v, _ in rpm]
peak = max(vals)
t_peak = [t for t, v, _ in rpm if v == peak][0]
tgt = a.rpm


def first_ge(frac):
    for t, v, _ in rpm:
        if v >= frac * tgt:
            return t
    return None


t10 = first_ge(0.10)
t90 = first_ge(0.90)
t100 = first_ge(1.00)
overshoot = (peak - tgt) / tgt * 100.0

print("\n=== 结果 %s ===" % (a.tag or ""))
print("  采样 %d 点 / %.2f s（%.0f Hz）" % (len(series), series[-1][0], len(series) / series[-1][0]))
print("  峰值 %.1f rpm @ %.2f s   过冲 %+.2f %%" % (peak, t_peak, overshoot))
print("  上升 10%%→90%% : %s" % ("%.3f s" % (t90 - t10) if (t10 and t90) else "未完成"))
print("  到达 90%%/100%%: %s / %s" % (("%.3f s" % t90) if t90 else "-", ("%.3f s" % t100) if t100 else "-"))
print("  判据 过冲≤5%% : %s" % ("PASS" if abs(overshoot) <= 5.0 else "FAIL"))
# 斜坡段（10%~90% 上升区间）的 iq_ref 均值：前馈是否生效的判据
w = [(t, iq) for t, v, iq in rpm if t10 is not None and t90 is not None and t10 <= t <= t90]
if w:
    print("  斜坡段 iq_ref 均值 = %+.4f A（%d 点）" % (sum(x[1] for x in w) / len(w), len(w)))
print("  序列(rpm): " + " ".join("%.0f" % v for v in vals[::max(1, len(vals) // 24)]))

rig.close()
