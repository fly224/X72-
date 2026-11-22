# -*- coding: utf-8 -*-
"""诊断"速度轨迹时间修改不起作用"。

阶段 A（不使能）：给定 2100 rpm 但目标先给 0，让斜坡停在 0，
  此时 dw = |2100-0| > 0.5，改 t 应能看到 a_eff/j_eff 随之变化。
阶段 B（使能）：实测 0 -> 1500 rpm 的加速时间，t=0.4 与 t=1.2 对比。
"""
import sys
import time
import math

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink

RPM2RAD = 2 * math.pi / 60.0
RAD2RPM = 60.0 / (2 * math.pi)

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


def show_traj(tag):
    r = lk.speed_traj_read(timeout=1.5)
    if not r.get("ok"):
        print("  %-10s 读失败: %s" % (tag, r.get("error")))
        return None
    print("  %-10s t=%6.3fs  a_eff=%8.1f rpm/s  j_eff=%9.1f rpm/s2  "
          "t_left=%5.3fs  [%s%s%s%s]  cfg a=%.1f j=%.1f" % (
              tag, r["t_ramp_s"], r["a_eff"] * RAD2RPM, r["j_eff"] * RAD2RPM * RAD2RPM,
              r["t_left_s"],
              "TIME " if r["time_on"] else "",
              "CLAMP " if r["clamped"] else "",
              "S " if r["scurve"] else "TRAP ",
              "POSBLK" if r["pos_blocked"] else "",
              r["acc_cfg"] * RAD2RPM, r["j_cfg"] * RAD2RPM * RAD2RPM))
    return r


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
    # ---------- 阶段 A：不使能，只改 t ----------
    print("===== 阶段 A：不使能，改 t 看回读 =====")
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.8)
    lk.set_mode(3)
    time.sleep(0.4)
    lk.set_speed_accel(500 * RPM2RAD)
    lk.set_speed_profile(1, 0)          # S 曲线
    lk.set_speed_rpm(0)                 # 目标 0，斜坡停在 0
    time.sleep(0.3)
    show_traj("初始")

    for t in (0.4, 1.0, 0.2):
        r = lk.set_speed_traj(t)
        if not r.get("ok"):
            print("  设 t=%.1f 失败: %s" % (t, r.get("error")))
            continue
        time.sleep(0.3)
        print("  设 t=%.1f 后（电机静止，dw 应≈0，看是否回退到 cfg）:" % t)
        show_traj("  t=%.1f" % t)

    # 让给定变化使 dw 变大：给 2100 但不使能 -> 斜坡不推进?
    print("\n  -- 给定 2100，仍不使能（斜坡应停在 0，dw=219.9）--")
    lk.set_speed_rpm(2100)
    time.sleep(0.5)
    for t in (0.4, 1.0):
        lk.set_speed_traj(t)
        time.sleep(0.3)
        print("  设 t=%.1f:" % t)
        show_traj("  t=%.1f" % t)

    # ---------- 阶段 B：使能，实测加速时间 ----------
    print("\n===== 阶段 B：使能，实测 0 -> 1500 rpm 加速时间 =====")
    for t in (1.2, 0.4):
        lk.set_keepalive(False)
        lk.set_speed_rpm(0)
        time.sleep(1.5)
        lk.set_controlword(0x06, halt=1)
        time.sleep(0.6)
        lk.set_mode(3)
        time.sleep(0.3)
        lk.set_speed_accel(500 * RPM2RAD)
        lk.set_speed_profile(1, 0)
        lk.set_speed_traj(t)
        time.sleep(0.2)
        lk.set_speed_rpm(1500)
        time.sleep(0.1)
        lk.set_controlword(0x0F, 0)
        time.sleep(0.4)
        lk.set_keepalive(True, 3, 20.0)

        t0 = time.time()
        samples = []
        while time.time() - t0 < 3.0:
            lp = lk.loop_status()
            if lp:
                samples.append((time.time() - t0, lp["wmeas"]))
            time.sleep(0.005)
        # 10% -> 90%
        hi = 1500 * 0.9
        lo = 1500 * 0.1
        t_lo = next((x[0] for x in samples if x[1] >= lo), None)
        t_hi = next((x[0] for x in samples if x[1] >= hi), None)
        if t_lo and t_hi:
            print("  t=%.2fs: 10%%@%.3fs 90%%@%.3fs -> 上升时间 %.3fs (期望≈%.2f)"
                  % (t, t_lo, t_hi, t_hi - t_lo, t * 0.8))
        else:
            print("  t=%.2fs: 采样不足 lo=%s hi=%s" % (t, t_lo, t_hi))
        show_traj("  回读")

finally:
    lk.set_keepalive(False)
    lk.set_speed_rpm(0)
    time.sleep(0.5)
    try:
        lk.set_controlword(0x00, 0)
    except Exception:
        pass
    time.sleep(0.6)
    s = lk.ctrl_status()
    if s:
        print("\n收尾 en=%d fault=0x%04X cal=%d z=%d" % (s["en"], s["fault"], s["cal"], s["z"]))
    lk.disconnect()
