# -*- coding: utf-8 -*-
"""长行程定位的逐拍诊断：起点约 2.7，下发 6.0（行程约 3 rad）。

输出 err / st / wref / wmeas / i_term，判断是规划问题还是环路振荡。
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


def wait_status(tmo=8.0):
    t0 = time.time()
    while time.time() - t0 < tmo:
        s = lk.ctrl_status()
        if s:
            return s
        time.sleep(0.2)
    return None


def wrap(d):
    while d > math.pi:
        d -= TWO
    while d < -math.pi:
        d += TWO
    return d


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
    # 停在 2.5~3.0 附近：转一小段即可
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.8)
    lk.set_mode(3)
    time.sleep(0.4)
    lk.set_speed_accel(300 * RPM2RAD)
    lk.set_speed_traj(0.0)
    lk.set_speed_profile(1, 0)
    lk.set_speed_rpm(150)
    time.sleep(0.2)
    lk.set_controlword(0x0F, 0)
    time.sleep(0.4)
    lk.set_keepalive(True, 3, 10.0)
    time.sleep(1.1)
    lk.set_keepalive(False)
    lk.set_speed_rpm(0)
    time.sleep(1.0)
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.8)

    lk.set_mode(1)
    time.sleep(0.4)
    lk.set_controlword(0x0F, 0)
    time.sleep(1.0)
    p = lk.pos_status()
    print("进位置: p_now=%.4f st=%s err=%+.5f" % (p["p_now"], p["st_name"], p["err"]))

    print("\n下发 6.0（当前 %.4f，最近等效目标应约 %.4f）" % (
        p["p_now"], 6.0 + round((p["p_now"] - 6.0) / TWO) * TWO))
    lk.set_pos_speed(pos_rad=6.0)
    time.sleep(0.05)
    lk.set_keepalive(True, 1, 20.0)

    t0 = time.time()
    print("  t(s)    p_now     err     st     wref    wmeas   i_term  fault")
    prev_st = None
    for i in range(40):
        time.sleep(0.1)
        q = lk.pos_status()
        s2 = lk.ctrl_status()
        lp = lk.loop_status()
        if not (q and s2 and lp):
            continue
        mark = ""
        if prev_st is not None and q["st_name"] != prev_st:
            mark = "  <- " + prev_st + ">" + q["st_name"]
        prev_st = q["st_name"]
        print("  %5.2f  %8.4f  %+7.4f  %-6s %+7.1f %+7.1f  %+7.3f  0x%04X%s" % (
            time.time() - t0, q["p_now"], q["err"], q["st_name"],
            lp["wref"], lp["wmeas"], lp["iterm"], s2["fault"], mark))
        if s2["fault"]:
            break
        if q["settled"]:
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
