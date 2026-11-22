# -*- coding: utf-8 -*-
"""验证 do_disable 漏关位置环的修复。

失败场景（修复前实测）：
  goto 3.60 -> 停 keepalive 让 deadman 触发 -> 下发 6.20
  -> 电机不动（p_now 仍 3.60，travel=0.0005）

修复后应：deadman 触发 -> do_disable 清 s_pos_prev -> 下次 30019 真正重起环
         -> 目标生效，走满 2.6 rad。
"""
import sys
import time
import math

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink

TWO = 6.283185307179586

lk = SerialLink()
lk.connect("COM27", 921600)
time.sleep(4.0)


def wrap(d):
    while d > math.pi:
        d -= TWO
    while d < -math.pi:
        d += TWO
    return d


def ws(tmo=8.0):
    t0 = time.time()
    while time.time() - t0 < tmo:
        s = lk.ctrl_status()
        if s:
            return s
        time.sleep(0.2)


def ps():
    for _ in range(20):
        q = lk.pos_status()
        if q:
            return q
        time.sleep(0.2)


def goto(rad, tmo=6.0, ka=True):
    lk.set_pos_speed(pos_rad=rad)
    time.sleep(0.05)
    if ka:
        lk.set_keepalive(True, 1, 20.0)
    t0 = time.time()
    while time.time() - t0 < tmo:
        q = ps()
        if q and q["settled"]:
            break
        time.sleep(0.01)
    return ps()


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
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.8)
    lk.set_mode(1)
    time.sleep(0.4)
    lk.set_controlword(0x0F, 0)
    time.sleep(1.0)
    q = ps()
    print("进位置 p_now=%.4f st=%s" % (q["p_now"], q["st_name"]))

    # --- 1) 先定位到 3.60 ---
    q = goto(3.60)
    print("1) 到 3.60: p_now=%.4f st=%s" % (q["p_now"], q["st_name"]))

    # --- 2) 停 keepalive，等 deadman（2 s）触发封锁 ---
    lk.set_keepalive(False)
    print("2) 停 keepalive，等 2.8 s 让 deadman 触发...")
    time.sleep(2.8)
    s2 = lk.ctrl_status()
    print("   deadman 后: en=%d fault=0x%04X  (en=0 说明输出已封锁)"
          % (s2["en"], s2["fault"]))

    # --- 3) 下发 6.20，应真正生效 ---
    q = goto(6.20)
    d = wrap(q["p_now"] - 6.20)
    print("3) 下发 6.20: p_now=%.4f  travel=%.4f  环形误差 %+.2f mrad  st=%s"
          % (q["p_now"], q["travel"], d * 1000, q["st_name"]))
    ok1 = abs(d) < 0.02 and q["travel"] > 2.0
    print("   判定: %s" % ("OK 目标生效（修复成功）" if ok1
                          else "★ 目标仍未生效 travel=%.4f" % q["travel"]))

    # --- 4) 再下发 0，应走近路 ---
    traj = []
    lk.set_pos_speed(pos_rad=0.0)
    time.sleep(0.05)
    lk.set_keepalive(True, 1, 20.0)
    t0 = time.time()
    while time.time() - t0 < 6.0:
        r = ps()
        if r:
            traj.append(r["p_now"])
            if r["settled"]:
                break
        time.sleep(0.02)
    fin = ps()
    arc = sum(abs(wrap(traj[i + 1] - traj[i])) for i in range(len(traj) - 1))
    d = wrap(fin["p_now"])
    print("4) 下发 0: 弧长 %.4f rad (应≈0.08), 环形误差 %+.2f mrad, travel=%.4f"
          % (arc, d * 1000, fin["travel"]))
    ok2 = abs(d) < 0.02 and arc < 0.5
    print("   判定: %s" % ("OK 走近路" if ok2 else "★ 走了远路 %.2f rad" % arc))

    print("\n总结: %s" % ("两项全过" if (ok1 and ok2) else "有失败项"))

finally:
    lk.set_keepalive(False)
    try:
        lk.set_controlword(0x00, 0)
    except Exception:
        pass
    time.sleep(0.6)
    s = lk.ctrl_status()
    if s:
        print("收尾 en=%d fault=0x%04X cal=%d z=%d" % (s["en"], s["fault"], s["cal"], s["z"]))
    lk.disconnect()
