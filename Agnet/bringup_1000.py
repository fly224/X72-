# -*- coding: utf-8 -*-
"""一键验证：C（按需）→ Z（按需）→ 速度模式 → 出指标 → 停机。

采样用固件自己的 `K` 行 `wmeas`（不是编码器差分）：
遥测流里只有 30025/30027/30040，没有 30019；而 30040 的多帧常共用同一个
读时间戳，差分还原转速会失真（实测被算成 379 rpm，而 K 报 1007.64）。
ASCII 的 `=` 有 2 s deadman，所以每 0.4 s 重发一次给定。

用法：
  python bringup_1000.py COM27            # 1000 rpm / 20 s
  python bringup_1000.py COM27 500 30
"""
import math
import re
import sys
import time

sys.path.insert(0, r"G:\FOC\X72_Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
RPM = float(sys.argv[2]) if len(sys.argv) > 2 else 1000.0
SEC = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

rig = Rig(PORT)
time.sleep(1.1)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)


def xfer(data, wait=1.0):
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
        return strip_frames(bytes(buf)).decode("ascii", "replace")
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def acmd(c, wait=1.0, tries=4):
    for _ in range(tries):
        txt = xfer(c.encode(), wait)
        if "[CMD]" in txt:
            return txt
        time.sleep(0.2)
    return ""


def state():
    for _ in range(5):
        s = acmd("S", 1.0)
        m = re.search(r"cal=(\d) z=(\d).*?f=(0x[0-9a-fA-F]+)", s)
        if m:
            return m.groups(), s
        time.sleep(0.2)
    return ("?", "?", "?"), ""


def run_z(timeout=70.0):
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
                t = strip_frames(bytes(buf)).decode("ascii", "replace")
                if "[CMD] IDENT ok" in t or "[CMD] IDENT FAIL" in t:
                    return t
            else:
                time.sleep(0.01)
        return strip_frames(bytes(buf)).decode("ascii", "replace")
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


# ---------------- 1) 前置 ----------------
(c, z, f), s = state()
print("[1] 初始 cal=%s z=%s f=%s" % (c, z, f))
if f not in ("0x00", "?"):
    acmd("X", 1.0)
    time.sleep(0.4)
    (c, z, f), s = state()
    print("    清故障后 cal=%s z=%s f=%s" % (c, z, f))

if c != "1":
    print("    C 电流零点 ...")
    acmd("C", 1.0)
    time.sleep(3.2)
    (c, z, f), s = state()
    print("    -> cal=%s z=%s f=%s" % (c, z, f))

if c != "1":
    print("[!] 电流零点未通过，停止")
    rig.close()
    sys.exit(1)

# ---------------- 2) Z ----------------
if z != "1":
    print("[2] Z 电角度对齐（约 22 s）...")
    txt = run_z()
    for l in txt.splitlines():
        if "IDENT" in l:
            print("    " + l.strip())
    time.sleep(0.8)
    (c, z, f), s = state()
    print("    -> cal=%s z=%s f=%s" % (c, z, f))

if z != "1":
    print("[!] Z 未通过，速度模式会被联锁拒绝")
    rig.close()
    sys.exit(1)

# ---------------- 3) 速度模式 ----------------
print("[3] 速度模式 %.0f rpm，%.0f s" % (RPM, SEC))
acmd("D", 1.0)
time.sleep(0.3)
e = acmd("E", 1.3)
print("    E :", [l.strip() for l in e.splitlines() if "[CMD]" in l][:1])
time.sleep(0.4)

spd = []        # (t, rpm)
t_start = time.time()
tick = 0
first = True
while True:
    now = time.time() - t_start
    if now >= SEC:
        break
    txt = xfer(("=%g\r" % RPM).encode(), 0.22)
    if first:
        for l in txt.splitlines():
            if "[CMD]" in l and "SPD" in l:
                print("    " + l.strip()[:80])
        first = False
    tick += 1
    if tick % 2 == 0:
        k = xfer(b"K", 0.30)
        m = re.search(r"wmeas=(-?[\d.]+)rpm", k)
        if m:
            spd.append((time.time() - t_start, float(m.group(1))))

print("    D :", [l.strip() for l in acmd("D", 1.2).splitlines() if "[CMD]" in l][:1])
time.sleep(0.4)
(c2, z2, f2), s2 = state()
print("    停机后 cal=%s z=%s f=%s" % (c2, z2, f2))
for l in s2.splitlines():
    if "[CMD] st" in l:
        print("    " + l.strip()[:190])

# ---------------- 4) 指标 ----------------
if len(spd) < 8:
    print("[!] 转速样本只有 %d 个，无法评估" % len(spd))
    rig.close()
    sys.exit(1)

vals = [x[1] for x in spd]
n = len(vals)
tail = vals[int(n * 0.4):]              # 稳态取后 60%
mean_v = sum(tail) / len(tail)
var = sum((v - mean_v) ** 2 for v in tail) / len(tail)
sd = math.sqrt(var)
cv = sd / abs(mean_v) * 100.0 if abs(mean_v) > 1e-9 else float("nan")
err = (mean_v - RPM) / RPM * 100.0

t_rise = None
for t, v in spd:
    if abs(v - RPM) <= 0.02 * abs(RPM):
        t_rise = t
        break

print("\n=== 结果 ===")
print("  样本 %d 个，时长 %.1f s" % (n, spd[-1][0]))
print("  稳态平均 %.2f rpm   目标 %.0f   误差 %+.3f%%" % (mean_v, RPM, err))
print("  标准差 %.2f rpm   CV %.2f%%   范围 [%.1f, %.1f]"
      % (sd, cv, min(tail), max(tail)))
print("  上升时间(进 ±2%%): %s" % (("%.2f s" % t_rise) if t_rise is not None else "整段未进入"))
print("  判据 |误差|≤0.2%% : %s   CV≤2%% : %s"
      % ("PASS" if abs(err) <= 0.2 else "FAIL", "PASS" if cv <= 2.0 else "FAIL"))
print("\n  转速序列(rpm): " + " ".join("%.0f" % v for v in vals[::max(1, n // 20)]))

rig.close()
