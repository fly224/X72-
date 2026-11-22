# -*- coding: utf-8 -*-
"""同步诊断：p_now(30100, 已归一化) vs 30090.pos(单圈角, 含零点) vs 内部量。

目的：
  1. 进位置模式后不下发目标，逐拍看 p_now / pos / err / st / wmeas，
     判断"漂移 6 rad"是真转动还是跨 2π 的显示假象；
  2. 下发 0 后逐拍看，确认起点为何与 enter_pos 返回值不一致。
"""
import sys
import time
import math

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink

TWO = 6.283185307179586
RPM2RAD = 2 * math.pi / 60.0

lk = SerialLink()
lk.connect("COM27", 921600)
time.sleep(4.0)


def wrap(d):
    while d > math.pi:
        d -= TWO
    while d < -math.pi:
        d += TWO
    return d


def wait_status(tmo=8.0):
    t0 = time.time()
    while time.time() - t0 < tmo:
        s = lk.ctrl_status()
        if s:
            return s
        time.sleep(0.2)
    return None


def row(tag, t):
    q = lk.pos_status()
    s = lk.ctrl_status()
    lp = lk.loop_status()
    if not q or not s:
        return None
    print("  %-8s %5.2fs  p_now=%7.4f  pos=%7.4f  d=%+7.4f  err=%+8.4f  st=%-5s  wmeas=%+7.2f" % (
        tag, t, q["p_now"], s["pos"], wrap(q["p_now"] - s["pos"]),
        q["err"], q["st_name"], lp["wmeas"] if lp else 0.0))
    return q, s, lp


s = wait_status()
if s is None:
    print("链路异常")
    raise SystemExit(1)
lk.set_permission(0, 0)
time.sleep(0.3)
if s["fault"]:
    lk.action(3)
    time.sleep(0.8)

try:
    # 速度模式转起来
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.8)
    lk.set_mode(3)
    time.sleep(0.4)
    lk.set_speed_accel(500 * RPM2RAD)
    lk.set_speed_traj(0.0)
    lk.set_speed_profile(1, 0)
    lk.set_speed_rpm(300)
    time.sleep(0.2)
    lk.set_controlword(0x0F, 0)
    time.sleep(0.4)
    lk.set_keepalive(True, 3, 10.0)
    time.sleep(1.5)
    lk.set_keepalive(False)
    lk.set_speed_rpm(0)

    print("===== 速度模式停机过程（看余速） =====")
    t0 = time.time()
    for i in range(6):
        time.sleep(0.25)
        row("停机", time.time() - t0)
    lk.set_controlword(0x06, halt=1)
    print("===== halt 后（MOE=0，电机自由）=====")
    t0 = time.time()
    for i in range(6):
        time.sleep(0.25)
        row("halt后", time.time() - t0)

    print("\n===== 进位置模式（不下发目标，逐拍 15 次）=====")
    lk.set_mode(1)
    time.sleep(0.4)
    lk.set_controlword(0x0F, 0)
    t0 = time.time()
    first = None
    for i in range(15):
        time.sleep(0.2)
        r = row("进位置", time.time() - t0)
        if r and first is None:
            first = r[0]["p_now"]
    if first is not None:
        last = lk.pos_status()["p_now"]
        print("  区间: %.4f -> %.4f  环形变化 %+.2f mrad" % (
            first, last, wrap(last - first) * 1000))

    print("\n===== 下发 0.0000，逐拍 20 次 =====")
    lk.set_pos_speed(pos_rad=0.0)
    time.sleep(0.05)
    lk.set_keepalive(True, 1, 20.0)
    t0 = time.time()
    for i in range(20):
        time.sleep(0.08)
        row("->0", time.time() - t0)
        q = lk.pos_status()
        s2 = lk.ctrl_status()
        if s2 and s2["fault"]:
            print("  ★ fault=0x%04X" % s2["fault"])
            break
        if q and q["settled"]:
            print("  ✓ settled")
            break

finally:
    lk.set_keepalive(False)
    try:
        lk.set_controlword(0x00, 0)
    except Exception:
        pass
    time.sleep(0.6)
    s = lk.ctrl_status()
    if s:
        print("\n收尾 en=%d fault=0x%04X cal=%d z=%d" % (s["en"], s["fault"], s["cal"], s["z"]))
    lk.disconnect()
