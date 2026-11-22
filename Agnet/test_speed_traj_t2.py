# -*- coding: utf-8 -*-
"""重测速度轨迹时间：使能后**立即**采样，用 wref（斜坡给定）判上升时间。

判据：wref 从 10% 到 90% 的时间，S 曲线理论值 = t * 0.8。
"""
import sys
import time
import math

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink

RPM2RAD = 2 * math.pi / 60.0
RAD2RPM = 60.0 / (2 * math.pi)
TARGET = 1500.0

lk = SerialLink()
lk.connect("COM27", 921600)
time.sleep(4.0)


def ws(tmo=8.0):
    t0 = time.time()
    while time.time() - t0 < tmo:
        s = lk.ctrl_status()
        if s:
            return s
        time.sleep(0.2)


def traj_read():
    r = lk.speed_traj_read(timeout=1.5)
    return r if r.get("ok") else None


def run_once(t, note=""):
    """0 -> TARGET，测 wref 上升时间"""
    lk.set_keepalive(False)
    lk.set_speed_rpm(0)
    time.sleep(1.8)
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.7)
    lk.set_mode(3)
    time.sleep(0.3)
    lk.set_speed_accel(500 * RPM2RAD)
    lk.set_speed_profile(1, 0)          # S 曲线
    lk.set_speed_traj(t)
    time.sleep(0.15)
    r0 = traj_read()

    lk.set_speed_rpm(TARGET)
    time.sleep(0.05)
    lk.set_controlword(0x0F, 0)
    lk.set_keepalive(True, 3, 20.0)

    t0 = time.time()
    lo = hi = None
    peak = 0.0
    while time.time() - t0 < 4.0:
        lp = lk.loop_status()
        if lp:
            w = lp["wref"]
            peak = max(peak, w)
            if lo is None and w >= TARGET * 0.1:
                lo = time.time() - t0
            if hi is None and w >= TARGET * 0.9:
                hi = time.time() - t0
            if lo and hi:
                break
        time.sleep(0.002)
    rt = (hi - lo) if (lo is not None and hi is not None) else None
    print("  t=%.2fs%s: 10%%@%s 90%%@%s  上升 %s  峰值 wref=%.0f  (理论 %.2fs)" % (
        t, note,
        "%.3fs" % lo if lo is not None else "—",
        "%.3fs" % hi if hi is not None else "—",
        "%.3fs" % rt if rt is not None else "—",
        peak, t * 0.8))
    if r0:
        print("     回读: a_eff=%.0f rpm/s  j_eff=%.0f rpm/s2  t_left=%.3fs  flags[%s%s%s]" % (
            r0["a_eff"] * RAD2RPM, r0["j_eff"] * RAD2RPM * RAD2RPM, r0["t_left_s"],
            "TIME " if r0["time_on"] else "", "CLAMP " if r0["clamped"] else "",
            "S" if r0["scurve"] else "TRAP"))
    return rt


s = ws()
if s is None:
    print("链路异常")
    raise SystemExit(1)
lk.set_permission(0, 0)
time.sleep(0.3)
if s["fault"]:
    lk.action(3)
    time.sleep(0.8)

try:
    print("===== 实测上升时间（采样无盲区）=====")
    for t in (0.4, 1.2, 0.8):
        run_once(t)

    print("\n===== 电机已稳在目标转速时改 t，看回读是否变 =====")
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.6)
    lk.set_mode(3)
    time.sleep(0.3)
    lk.set_speed_profile(1, 0)
    lk.set_speed_rpm(TARGET)
    time.sleep(0.1)
    lk.set_controlword(0x0F, 0)
    lk.set_keepalive(True, 3, 20.0)
    time.sleep(2.5)
    lp = lk.loop_status()
    print("  已稳定在 wref=%.0f wmeas=%.0f" % (lp["wref"], lp["wmeas"]))
    for t in (0.4, 1.5, 0.2):
        lk.set_speed_traj(t)
        time.sleep(0.3)
        r = traj_read()
        if r:
            print("  设 t=%.1f -> 回读 t=%.2fs a_eff=%.0f rpm/s j_eff=%.0f  %s"
                  % (t, r["t_ramp_s"], r["a_eff"] * RAD2RPM,
                     r["j_eff"] * RAD2RPM * RAD2RPM,
                     "★ 与 t 不匹配（回退到 cfg）" if abs(r["a_eff"] * RAD2RPM) < 600 else "OK"))
        else:
            print("  设 t=%.1f -> 读失败" % t)

finally:
    lk.set_keepalive(False)
    lk.set_speed_rpm(0)
    time.sleep(0.8)
    try:
        lk.set_controlword(0x00, 0)
    except Exception:
        pass
    time.sleep(0.6)
    s = lk.ctrl_status()
    if s:
        print("\n收尾 en=%d fault=0x%04X" % (s["en"], s["fault"]))
    lk.disconnect()
