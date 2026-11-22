# -*- coding: utf-8 -*-
"""位置归一化的两项补充验证（2026-11-22）。

A. 静置稳定性：进位置模式后**不下发目标**，3 s 内 p_now 是否漂移。
B. 跨 0 近路：从 ~6.1 下发 0，轨迹应**正向递增**穿过 6.28 → 0.0x
   （走 ~0.2 rad），而不是反向走 ~6.1 rad。
C. 反向跨 0：从 ~0.1 下发 6.0，应**反向递减**穿过 0 → 6.28（走 ~0.4 rad）。
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


def halt():
    lk.set_keepalive(False)
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.8)


def spin(rpm, sec):
    halt()
    lk.set_mode(3)
    time.sleep(0.4)
    lk.set_speed_accel(500 * RPM2RAD)
    lk.set_speed_traj(0.0)
    lk.set_speed_profile(1, 0)
    lk.set_speed_rpm(rpm)
    time.sleep(0.2)
    lk.set_controlword(0x0F, 0)
    time.sleep(0.4)
    lk.set_keepalive(True, 3, 10.0)
    time.sleep(sec)
    lk.set_keepalive(False)
    lk.set_speed_rpm(0)
    time.sleep(1.0)
    halt()


def enter_pos():
    lk.set_mode(1)
    time.sleep(0.4)
    lk.set_controlword(0x0F, 0)
    time.sleep(1.2)
    return lk.pos_status()


def goto_traj(rad, tmo=8.0):
    """下发目标并返回轨迹 [(t, p_now), ...]"""
    lk.set_pos_speed(pos_rad=rad)
    time.sleep(0.1)
    lk.set_keepalive(True, 1, 20.0)
    traj = []
    t0 = time.time()
    flt = 0
    while time.time() - t0 < tmo:
        q = lk.pos_status()
        s = lk.ctrl_status()
        if s and s["fault"]:
            flt = s["fault"]
            break
        if q:
            traj.append((time.time() - t0, q["p_now"]))
            if q["settled"]:
                break
        time.sleep(0.02)
    fin = lk.pos_status()
    return traj, fin, flt


def total_arc(traj):
    """轨迹上相邻点的环形步长之和 = 实际走了多少弧度"""
    a = 0.0
    for i in range(len(traj) - 1):
        a += abs(wrap(traj[i + 1][1] - traj[i][1]))
    return a


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
    # ---------- A. 静置稳定性 ----------
    print("===== A. 静置稳定性（进位置后不下发任何目标）=====")
    spin(300, 1.5)
    p = enter_pos()
    print("  进位置: p_now=%.4f st=%s err=%+.5f" % (p["p_now"], p["st_name"], p["err"]))
    vals = []
    t0 = time.time()
    while time.time() - t0 < 3.0:
        q = lk.pos_status()
        if q:
            vals.append(q["p_now"])
        time.sleep(0.02)
    drift = max(vals) - min(vals)
    print("  3 s 内 %d 次采样: %.6f ~ %.6f  漂移 %.2f mrad" % (
        len(vals), min(vals), max(vals), drift * 1000))
    print("  判定: %s" % ("OK 稳定（< 5 mrad）" if drift < 0.005 else "★ 漂移偏大"))
    halt()

    # ---------- B. 跨 0 近路 ----------
    print("\n===== B. 跨 0 近路（6.x -> 0）=====")
    spin(300, 1.5)
    p = enter_pos()
    print("  起点 p_now=%.4f (%.1f°)" % (p["p_now"], p["p_now"] * 57.2958))
    traj, fin, flt = goto_traj(0.0)
    arc = total_arc(traj)
    # 判断方向：轨迹上 p_now 是否单调递增（正向跨 0）
    inc = sum(1 for i in range(len(traj) - 1) if traj[i + 1][1] > traj[i][1])
    dec = len(traj) - 1 - inc
    print("  轨迹 %d 点, 累计弧长 %.3f rad, 递增 %d 步 / 递减 %d 步" % (
        len(traj), arc, inc, dec))
    print("  首点 %.4f -> 末点 %.4f  环形误差 %+.2f mrad  fault=0x%04X" % (
        traj[0][1], fin["p_now"], wrap(fin["p_now"]) * 1000, flt))
    near = any(0.0 <= v < 0.3 or v > 6.0 for _, v in traj)
    print("  判定: 弧长 %.2f rad %s；%s" % (
        arc,
        "≈ 0.2（走近路）" if arc < 0.8 else "★ 走了远路（> 0.8 rad）",
        "OK" if abs(wrap(fin["p_now"])) < 0.02 and flt == 0 else "★ 未到位"))
    halt()

    # ---------- C. 反向跨 0 ----------
    print("\n===== C. 反向跨 0（0.x -> 6.0）=====")
    spin(120, 0.6)          # 转一小段，让单圈角落在 0 附近
    p = enter_pos()
    print("  起点 p_now=%.4f (%.1f°)" % (p["p_now"], p["p_now"] * 57.2958))
    tgt = 6.0
    traj, fin, flt = goto_traj(tgt)
    arc = total_arc(traj)
    print("  轨迹 %d 点, 累计弧长 %.3f rad" % (len(traj), arc))
    print("  首点 %.4f -> 末点 %.4f  环形误差 %+.2f mrad  fault=0x%04X" % (
        traj[0][1], fin["p_now"], wrap(fin["p_now"] - tgt) * 1000, flt))
    print("  判定: %s" % ("OK（误差 < 20 mrad）" if abs(wrap(fin["p_now"] - tgt)) < 0.02 and flt == 0
                        else "★ 未到位"))

finally:
    lk.set_keepalive(False)
    try:
        lk.set_controlword(0x00, 0)
    except Exception:
        pass
    time.sleep(0.6)
    s = lk.ctrl_status()
    if s:
        print("\n收尾 en=%d fault=0x%04X cal=%d z=%d cog=%d" % (
            s["en"], s["fault"], s["cal"], s["z"], s["cog"]))
    lk.disconnect()
