# -*- coding: utf-8 -*-
"""位置控制归一化的多轮重复回归（2026-11-22）。

覆盖：
  每轮 = 速度模式转若干圈 -> 切位置模式 -> 检查 p_now ∈ [0,2π)
         -> 依次下发 0 / 0.5 / 3.1416 / 6.0 / 0 -> 检查环形误差与故障

判定：
  · p_now 必须落在 [0, 2π)；
  · 每个目标的环形误差 |d| < 20 mrad（POS_WIN=10 mrad，留 2 倍余量）；
  · 全程 fault == 0。

用法： python test_pos_norm_multi.py [轮数]
"""
import sys
import time
import math

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink

TWO = 6.283185307179586
RPM2RAD = 2 * math.pi / 60.0
ROUNDS = int(sys.argv[1]) if len(sys.argv) > 1 else 3

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
    """把差值折到 [-π, π]"""
    while d > math.pi:
        d -= TWO
    while d < -math.pi:
        d += TWO
    return d


def spin(rpm=300, sec=1.5):
    """速度模式转若干圈（制造累计圈数）"""
    lk.set_keepalive(False)
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.6)
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
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.8)


def enter_pos():
    lk.set_mode(1)
    time.sleep(0.4)
    lk.set_controlword(0x0F, 0)
    time.sleep(1.2)
    return lk.pos_status()


def goto(rad, tmo=6.0):
    """下发目标，返回 (实测, 环形误差, 故障, 耗时)"""
    lk.set_pos_speed(pos_rad=rad)
    time.sleep(0.1)
    lk.set_keepalive(True, 1, 20.0)
    t0 = time.time()
    fin = None
    flt = 0
    while time.time() - t0 < tmo:
        q = lk.pos_status()
        s = lk.ctrl_status()
        if s and s["fault"]:
            flt = s["fault"]
            break
        if q and q["settled"]:
            fin = q
            break
        time.sleep(0.01)
    if fin is None:
        fin = lk.pos_status()
    return fin, wrap(fin["p_now"] - rad), flt, time.time() - t0


s0 = wait_status()
if s0 is None:
    print("链路异常：读不到 30090")
    raise SystemExit(1)
lk.set_permission(0, 0)
time.sleep(0.3)

print("起始: fault=0x%04X cal=%d z=%d cog=%d" % (s0["fault"], s0["cal"], s0["z"], s0["cog"]))
if s0["fault"]:
    lk.action(3)
    time.sleep(0.8)

TARGETS = (0.0, 0.5, 3.1416, 6.0, 0.0)
rows = []
bad = []

try:
    for r in range(ROUNDS):
        print("\n===== 第 %d 轮 =====" % (r + 1))
        spin(300 if r % 2 == 0 else 250, 1.5 if r % 2 == 0 else 2.0)

        p = enter_pos()
        inside = 0.0 <= p["p_now"] < TWO
        print("  进位置: p_now=%.4f  st=%-5s err=%+.5f  %s" % (
            p["p_now"], p["st_name"], p["err"],
            "OK ∈[0,2π)" if inside else "★ 超出 2π"))
        if not inside:
            bad.append("R%d 归一化失败 p_now=%.4f" % (r + 1, p["p_now"]))

        for tgt in TARGETS:
            fin, d, flt, dt = goto(tgt)
            ok = (flt == 0) and (abs(d) < 0.020)
            if not ok:
                bad.append("R%d 目标%.4f 误差%+.1fmrad fault=0x%04X" % (
                    r + 1, tgt, d * 1000, flt))
            print("    %s -> %.4f  实到 %.4f  环形误差 %+7.2f mrad  %.1fs%s" % (
                "✓" if ok else "★", tgt, fin["p_now"], d * 1000, dt,
                ("  fault=0x%04X" % flt) if flt else ""))
            rows.append((r + 1, tgt, fin["p_now"], d * 1000, flt, dt))
            if flt:
                break

    print("\n===== 汇总 =====")
    print("共 %d 轮 × %d 目标 = %d 次定位" % (ROUNDS, len(TARGETS), len(rows)))
    if rows:
        errs = [abs(x[3]) for x in rows]
        print("环形误差: max %.2f mrad, 均值 %.2f mrad" % (max(errs), sum(errs) / len(errs)))
        ts = [x[5] for x in rows]
        print("定位耗时: max %.2fs, 均值 %.2fs" % (max(ts), sum(ts) / len(ts)))
        print("故障次数: %d" % sum(1 for x in rows if x[4]))
    if bad:
        print("\n★ 失败 %d 项:" % len(bad))
        for b in bad:
            print("   " + b)
    else:
        print("\n判定: 全部通过")
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
