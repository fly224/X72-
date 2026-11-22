# -*- coding: utf-8 -*-
"""验证回读修复：稳态改 t，a_eff/j_eff 应立即随 t 变化。

理论（S 曲线，Δω = 名义目标转速，单位 rad/s）：
  a_eff = 2*Δω/t,  j_eff = 4*Δω/t²
"""
import sys
import time
import math

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink

RPM2RAD = 2 * math.pi / 60.0
RAD2RPM = 60.0 / (2 * math.pi)
TARGET = 1500.0
DW = TARGET * RPM2RAD

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


def read():
    r = lk.speed_traj_read(timeout=1.5)
    return r if r.get("ok") else None


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
    # 使能并稳在 1500 rpm
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.7)
    lk.set_mode(3)
    time.sleep(0.3)
    lk.set_speed_accel(500 * RPM2RAD)
    lk.set_speed_profile(1, 0)          # S 曲线
    lk.set_speed_traj(0.0)              # 先关时间模式
    time.sleep(0.1)
    lk.set_speed_rpm(TARGET)
    time.sleep(0.1)
    lk.set_controlword(0x0F, 0)
    lk.set_keepalive(True, 3, 20.0)
    time.sleep(2.5)
    lp = lk.loop_status()
    print("已稳在 wref=%.0f wmeas=%.0f\n" % (lp["wref"], lp["wmeas"]))
    print("%-8s %-12s %-14s %-8s %s" % ("设 t", "回读 a_eff", "回读 j_eff", "clamp", "与理论比对"))

    for t in (0.4, 1.5, 0.2, 1.2):
        lk.set_speed_traj(t)
        time.sleep(0.35)
        r = read()
        if not r:
            print("  t=%.1f 读失败" % t)
            continue
        a_the = 2 * DW / t * RAD2RPM          # rpm/s
        j_the = 4 * DW / (t * t) * RAD2RPM * RAD2RPM
        a_got = r["a_eff"] * RAD2RPM
        j_got = r["j_eff"] * RAD2RPM * RAD2RPM
        ok = abs(a_got - a_the) < max(50.0, a_the * 0.02)
        print("%-8.2f %-12.0f %-14.0f %-8s %s (理论 %.0f / %.0f)" % (
            t, a_got, j_got, "YES" if r["clamped"] else "no",
            "OK" if ok else "★偏差", a_the, j_the))

    # 停机（目标 0）时改 t：应回退配置值
    print("\n停机后改 t（目标 0，无名义行程，应回退 cfg）:")
    lk.set_keepalive(False)
    lk.set_speed_rpm(0)
    time.sleep(2.0)
    lk.set_keepalive(True, 3, 20.0)
    time.sleep(0.5)
    for t in (0.4, 1.5):
        lk.set_speed_traj(t)
        time.sleep(0.35)
        r = read()
        if r:
            print("  t=%.1f -> a_eff=%.0f rpm/s (cfg=%.0f)  %s" % (
                t, r["a_eff"] * RAD2RPM, r["acc_cfg"] * RAD2RPM,
                "OK 回退" if abs(r["a_eff"] - r["acc_cfg"]) < 1.0 else "未回退"))

    # 恢复时间模式关闭
    lk.set_speed_traj(0.0)

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
        print("\n收尾 en=%d fault=0x%04X" % (s["en"], s["fault"]))
    lk.disconnect()
