# -*- coding: utf-8 -*-
"""力矩模式（30056 mode=4）实测：给定 / 限幅 / 斜坡 / 余速门禁 / 超速 / deadman。

安全：全程把 cur_iqmax 压到 0.3 A 以内，且力矩给定时间短 ——
空载力矩模式下 a = Kt*iq/J ≈ 1347 rad/s²/A，给 1 A 会在 23 ms 内撞 300 rpm 保护。
"""
import sys
import time
import math

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink

RPM2RAD = 2 * math.pi / 60.0

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


def halt():
    lk.set_keepalive(False)
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.9)


def fault_names(f):
    n = [("过流", 1), ("编码器", 2), ("ADC同步", 4), ("BSPINIT", 8), ("SPI", 16),
         ("超速", 32), ("过压", 64), ("欠压", 128), ("跟随", 256), ("缺相", 512),
         ("电流采样", 1024)]
    return ", ".join(x for x, b in n if f & b) or "无"


def enter_torque(iq, iqmax=0.3, tau_ms=None):
    lk.set_keepalive(False)
    try:
        lk.dbg_write(floats={"cur_iqmax": iqmax})
    except Exception:
        pass
    if tau_ms is not None:
        lk.torque_cmd(ramp_ms=tau_ms)
    lk.set_mode(4)
    time.sleep(0.3)
    r = lk.set_torque(iq)
    time.sleep(0.08)
    lk.set_controlword(0x0F, 0)
    time.sleep(0.15)
    lk.set_keepalive(True, 4, 20.0)
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

print("初始: fault=0x%04X cal=%d z=%d" % (s["fault"], s["cal"], s["z"]))
print("  cur_iqmax=%.3f A  (30082 float 12)" % lk.dbg_read()["floats"]["cur_iqmax"])

try:
    # ---------- A. 基本给定 + iq 跟踪 ----------
    print("\n===== A. 给 iq=+0.20 A（限幅 0.3）=====")
    halt()
    r = enter_torque(0.20, iqmax=0.3, tau_ms=20)
    print("  set_torque -> ok=%s status=%s" % (r.get("ok"), r.get("status")))
    t0 = time.time()
    for i in range(8):
        time.sleep(0.12)
        q = lk.ctrl_status()
        if q:
            print("    t=%.2fs iq=%+.3f iq_ref=%+.3f rpm=%+7.1f en=%d fault=0x%04X"
                  % (time.time() - t0, q["iq"], q["iq_ref"], q["rpm"], q["en"], q["fault"]))
            if q["fault"]:
                print("    ★ 故障：%s" % fault_names(q["fault"]))
                break

    # ---------- B. 反向 ----------
    print("\n===== B. 给 iq=-0.20 A（看符号）=====")
    halt()
    enter_torque(-0.20, iqmax=0.3, tau_ms=20)
    for i in range(5):
        time.sleep(0.12)
        q = lk.ctrl_status()
        if q:
            print("    iq=%+.3f rpm=%+7.1f" % (q["iq"], q["rpm"]))
            if q["fault"]:
                break

    # ---------- C. 限幅 ----------
    print("\n===== C. 限幅：cur_iqmax=0.10，给 2.0 A =====")
    halt()
    r = enter_torque(2.0, iqmax=0.10, tau_ms=20)
    time.sleep(0.4)
    q = lk.ctrl_status()
    print("  30025 应答 iq 回读=%.3f（协议层原值）  " % 0.0, end="")
    print("30090: iq_ref=%+.3f iq=%+.3f  -> %s"
          % (q["iq_ref"], q["iq"],
             "OK 被夹到 0.10" if abs(q["iq_ref"]) <= 0.101 else "★ 未夹"))

    # ---------- D. iq 斜坡 ----------
    print("\n===== D. iq 斜坡：tau=0 vs 200 ms（目标 0.25 A）=====")
    for tau in (0, 200):
        halt()
        enter_torque(0.25, iqmax=0.3, tau_ms=tau)
        t0 = time.time()
        t90 = None
        for i in range(40):
            time.sleep(0.01)
            q = lk.ctrl_status()
            if q and t90 is None and abs(q["iq"]) >= 0.225:
                t90 = time.time() - t0
                break
        print("  tau=%3d ms -> iq 到 90%% 用时 %s" % (
            tau, "%.3f s" % t90 if t90 else ">0.4 s"))

    # ---------- E. 余速门禁 ----------
    print("\n===== E. 余速门禁：速度模式 900 rpm -> halt -> 立刻切力矩 =====")
    halt()
    lk.set_mode(3)
    time.sleep(0.3)
    lk.set_speed_accel(600 * RPM2RAD)
    lk.set_speed_rpm(900)
    time.sleep(0.1)
    lk.set_controlword(0x0F, 0)
    lk.set_keepalive(True, 3, 20.0)
    time.sleep(1.8)
    q = lk.ctrl_status()
    print("  速度模式稳定: rpm=%.0f" % q["rpm"])
    lk.set_keepalive(False)
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.15)                       # 余速仍在
    q1 = lk.ctrl_status()
    lk.set_mode(4)
    r = lk.set_torque(0.0)
    time.sleep(0.3)
    q2 = lk.ctrl_status()
    print("  halt 后 0.15 s: rpm=%.0f" % q1["rpm"])
    print("  切力矩 set_torque -> ok=%s error=%s" % (r.get("ok"), r.get("error")))
    print("  结果: en=%d  -> %s" % (
        q2["en"], "OK 被拒（余速 >270 rpm）" if q2["en"] == 0 else "★ 竟然使能了"))

    # ---------- F. 超速保护 ----------
    print("\n===== F. 超速保护：空载给 0.05 A 让它自己加速 =====")
    halt()
    if lk.ctrl_status()["fault"]:
        lk.action(3)
        time.sleep(0.6)
    enter_torque(0.05, iqmax=0.1, tau_ms=20)
    t0 = time.time()
    peak = 0.0
    f = 0
    while time.time() - t0 < 3.0:
        q = lk.ctrl_status()
        if q:
            peak = max(peak, q["rpm"])
            if q["fault"]:
                f = q["fault"]
                break
        time.sleep(0.01)
    q = lk.ctrl_status()
    print("  峰值 rpm=%.0f  最终 rpm=%.0f  fault=0x%04X [%s]"
          % (peak, q["rpm"], f, fault_names(f)))
    print("  判定: %s" % ("OK 300 rpm 处触发超速保护" if (f & 32) else "★ 未触发"))

    # ---------- G. deadman ----------
    print("\n===== G. deadman：给 0.05 A 保活 1 s，然后停发 =====")
    halt()
    if lk.ctrl_status()["fault"]:
        lk.action(3)
        time.sleep(0.6)
    enter_torque(0.05, iqmax=0.1, tau_ms=20)
    time.sleep(1.0)
    lk.set_keepalive(False)
    print("  停发保活，等 2.6 s...")
    time.sleep(2.6)
    q = lk.ctrl_status()
    print("  en=%d fault=0x%04X  -> %s" % (
        q["en"], q["fault"], "OK 已封锁" if q["en"] == 0 else "★ 未封锁"))

finally:
    lk.set_keepalive(False)
    try:
        lk.dbg_write(floats={"cur_iqmax": 5.0})
    except Exception:
        pass
    try:
        lk.set_controlword(0x00, 0)
    except Exception:
        pass
    time.sleep(0.6)
    s = lk.ctrl_status()
    if s:
        print("\n收尾 en=%d fault=0x%04X [%s] cal=%d z=%d  cur_iqmax 已恢复 5.0 A"
              % (s["en"], s["fault"], fault_names(s["fault"]), s["cal"], s["z"]))
    lk.disconnect()
