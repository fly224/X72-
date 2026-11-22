# -*- coding: utf-8 -*-
"""decel_profile_test.py —— 降速到底有没有轨迹规划？两个对照

A. 受控减速：给 0（走 speed 环的 ramp_step，a_max 限幅）
B. 急停滑行：cw=0x06 halt（do_disable 直接关 MOE，无规划）

输出：到 5%（50 rpm）的时间、wref 的下降斜率（rad/s²）、母线峰值、是否报故障。
预期：A 的斜率 ≈ -a_max、时间 ≈ Δω/a；B 是指数自由滑行，没有斜率可言。

用法：python decel_profile_test.py COM27 1000 [acc]
"""
import math
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink   # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
RPM = float(sys.argv[2]) if len(sys.argv) > 2 else 1000.0
ACC = float(sys.argv[3]) if len(sys.argv) > 3 else 400.0
RPM_TO_RADS = 2.0 * math.pi / 60.0


def snap(lk, key, last):
    m = lk.last_msg.get(key)
    if m is None or m["ts"] == last:
        return None, last
    raw = lk._payload_of(m)
    if not raw:
        return None, last
    if key == 30090 and len(raw) >= 40:
        f = struct.unpack_from("<8f", raw, 4)
        return {"ts": m["ts"], "rpm": f[2], "iq": f[0], "vbus": f[4],
                "fault": raw[36] | (raw[37] << 8), "en": (raw[3] >> 2) & 1}, m["ts"]
    if key == 30091 and len(raw) >= 48:
        return {"ts": m["ts"],
                "wref": struct.unpack_from("<f", raw, 4)[0],
                "wmeas": struct.unpack_from("<f", raw, 8)[0]}, m["ts"]
    return None, last


def capture(lk, dur, trigger=None):
    rows, a, b = [], None, None
    t0 = time.time()
    if trigger is not None:
        trigger()
    while time.time() - t0 < dur:
        d, a = snap(lk, 30090, a)
        r, b = snap(lk, 30091, b)
        if d:
            d["wref"] = r["wref"] if r else None
            d["t"] = d["ts"] - t0
            rows.append(d)
        time.sleep(0.004)
    return rows


def analyze(name, rows):
    if not rows:
        print("    %s: 没有数据" % name)
        return
    r0, r1 = rows[0], rows[-1]
    tgt5 = RPM * 0.05
    t5 = next((r["t"] for r in rows if abs(r["rpm"]) <= tgt5), None)
    zero = next((r["t"] for r in rows if abs(r["rpm"]) <= RPM * 0.005), None)
    vmax = max(r["vbus"] for r in rows)
    fm = next((r for r in rows if r["fault"]), None)
    # wref 下降斜率：取 wref 单调下降那段的前 60% 做最小二乘
    pts = [(r["t"], r["wref"] * RPM_TO_RADS) for r in rows
           if r["wref"] is not None and r["wref"] > RPM * 0.02]
    slope = None
    if len(pts) >= 8:
        n = len(pts)
        mx = sum(p[0] for p in pts) / n
        my = sum(p[1] for p in pts) / n
        den = sum((p[0] - mx) ** 2 for p in pts)
        if den > 1e-9:
            slope = sum((p[0] - mx) * (p[1] - my) for p in pts) / den
    print("    %s" % name)
    print("      到 5%%（%.0f rpm）: %s" % (tgt5 if False else RPM * 0.05,
                                           ("%.3f s" % t5) if t5 else "未达到"))
    print("      到 0.5%%: %s" % (("%.3f s" % zero) if zero else "未达到"))
    print("      wref 斜率: %s rad/s²" % (("%.0f" % slope) if slope else "n/a"))
    print("      母线峰值 %.2f V（起点 %.2f）" % (vmax, r0["vbus"]))
    print("      故障: %s" % (("0x%04X @t=%.3f" % (fm["fault"], fm["t"])) if fm else "无"))
    print("      iq 峰值 %.2f A" % max(abs(r["iq"]) for r in rows))


def main():
    lk = SerialLink()
    c = lk.connect(PORT, 921600)
    print("连接:", c.get("ok"), c.get("error", ""))
    if not c.get("ok"):
        return 2
    time.sleep(1.2)
    lk.set_permission(0, 0)
    st = lk.ctrl_status()
    if st is None:
        print("30090 无应答")
        lk.disconnect()
        return 2
    if st["fault"]:
        lk.action(3)
        time.sleep(0.6)
    lk.set_speed_traj(0.0)
    lk.set_speed_accel(ACC)
    lk.set_speed_profile(0, 20000.0)
    lk.set_mode(3)

    def spin_up():
        lk.set_speed_rpm(RPM)
        lk.set_keepalive(True, 3, 5.0)
        time.sleep(0.2)
        lk.set_controlword(0x0F, 0)
        t = time.time()
        while time.time() - t < 5:
            s = lk.ctrl_status()
            if s and abs(s["rpm"] - RPM) < RPM * 0.02:
                return True
            time.sleep(0.05)
        return False

    print("\n[1] 起转到 %.0f rpm（acc=%.0f）" % (RPM, ACC))
    ok = spin_up()
    print("    到速:", ok, " fault=0x%04X" % lk.ctrl_status()["fault"])
    time.sleep(0.5)

    print("\n[2] A：给 0（受控减速，走 ramp）")
    rows = capture(lk, 1.0, trigger=lambda: lk.set_speed_rpm(0.0))
    analyze("A 受控减速（=0）", rows)
    time.sleep(0.8)

    print("\n[3] B：halt（cw=0x06，直接关 MOE 自由滑行）")
    if not spin_up():
        print("    起转失败，跳过 B")
    else:
        time.sleep(0.5)
        rows = capture(lk, 1.4, trigger=lambda: lk.set_controlword(0x06, halt=1))
        analyze("B 急停滑行（halt）", rows)

    print("\n[4] 收尾")
    lk.set_keepalive(False)
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.5)
    st = lk.ctrl_status()
    print("    en=%d fault=0x%04X rpm=%.1f" % (st["en"], st["fault"], st["rpm"]))
    lk.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
