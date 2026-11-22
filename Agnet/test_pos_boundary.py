# -*- coding: utf-8 -*-
"""边界验证：p_now 在 6.x 附近输入 0 是否走正向近路；p_tgt 显示值。"""
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
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.8)
    lk.set_mode(1)
    time.sleep(0.4)
    lk.set_controlword(0x0F, 0)
    time.sleep(1.0)

    # 先定位到 6.20 附近
    p = None
    for _ in range(20):
        p = lk.pos_status()
        if p:
            break
        time.sleep(0.2)
    if p is None:
        print("★ 读不到 30100")
        raise SystemExit(1)
    print("当前 p_now=%.4f" % p["p_now"])
    lk.set_pos_speed(pos_rad=6.20)
    time.sleep(0.05)
    lk.set_keepalive(True, 1, 20.0)
    t0 = time.time()
    while time.time() - t0 < 5.0:
        q = lk.pos_status()
        if q and q["settled"]:
            break
        time.sleep(0.01)
    q = lk.pos_status()
    print("已停到 %.4f  (p_tgt 显示 %.4f, err %+.5f, travel %.4f)" % (
        q["p_now"], q["p_tgt"], q["err"], q["travel"]))
    halt_start = q["p_now"]

    # 输入 0：应正向小步跨越 0
    traj = []
    lk.set_pos_speed(pos_rad=0.0)
    time.sleep(0.05)
    lk.set_keepalive(True, 1, 20.0)
    t0 = time.time()
    while time.time() - t0 < 5.0:
        r = lk.pos_status()
        if r:
            traj.append(r["p_now"])
            if r["settled"]:
                break
        time.sleep(0.02)
    fin = lk.pos_status()
    arc = 0.0
    for i in range(len(traj) - 1):
        arc += abs(wrap(traj[i + 1] - traj[i]))
    inc = sum(1 for i in range(len(traj) - 1) if traj[i + 1] > traj[i])
    dec = len(traj) - 1 - inc
    print("\n从 %.4f 下发 0.0000：" % halt_start)
    print("  轨迹 %d 点, 弧长 %.4f rad, 递增 %d / 递减 %d" % (len(traj), arc, inc, dec))
    print("  首点 %.4f -> 末点 %.4f" % (traj[0], fin["p_now"]))
    print("  环形误差 %+.2f mrad, p_tgt 显示 %.4f, travel %.4f, st=%s" % (
        wrap(fin["p_now"]) * 1000, fin["p_tgt"], fin["travel"], fin["st_name"]))
    print("  判定: %s" % (
        "OK 正向小步（弧长 < 0.5 rad）" if arc < 0.5 else
        "★ 走了大行程 %.2f rad" % arc))

    # 再验一个：输入 6.0，看 p_tgt 显示
    lk.set_pos_speed(pos_rad=6.0)
    time.sleep(0.05)
    t0 = time.time()
    while time.time() - t0 < 5.0:
        r = lk.pos_status()
        if r and r["settled"]:
            break
        time.sleep(0.01)
    fin = lk.pos_status()
    print("\n下发 6.0000: p_now=%.4f  p_tgt=%.4f  travel=%.4f  环形误差 %+.2f mrad" % (
        fin["p_now"], fin["p_tgt"], fin["travel"], wrap(fin["p_now"] - 6.0) * 1000))

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
