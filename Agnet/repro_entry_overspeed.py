# -*- coding: utf-8 -*-
"""repro_entry_overspeed.py —— 复现"带余速重新进环给 0，误判超速"

路径（与界面操作一致）：
    1000 rpm 稳态 → 点停止（cw=0x06/halt，do_disable 把 s_spd_prev 清 0）
    → 电机自由滑行 0.25 s（余速仍 ~900 rpm）
    → 给 0 rpm 重新进环
    → Servo_SetSpeedMode() 里 s_spd_wpeak = |给定| = 0
    → 超速门限 = 1.5×0 + 10 = 10 rad/s（95 rpm）< 余速 94 rad/s
    → 8 拍（0.4 ms）后置 BSP_FAULT_OVERSPEED，随后封锁
    → 200 ms 后保活帧 30019 被拒（status=3）

预期：旧固件复现 0x0020；修好后应正常减速到 0、无故障。

用法：python repro_entry_overspeed.py COM27 1000
"""
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink   # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
RPM_HI = float(sys.argv[2]) if len(sys.argv) > 2 else 1000.0


def snap90(lk):
    m = lk.last_msg.get(30090)
    if m is None:
        return None
    raw = lk._payload_of(m)
    if not raw or len(raw) < 40:
        return None
    f = struct.unpack_from("<8f", raw, 4)
    return {"en": (raw[3] >> 2) & 1, "rpm": f[2], "iq": f[0], "vbus": f[4],
            "fault": raw[36] | (raw[37] << 8), "ts": m["ts"]}


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
        print("清故障 0x%04X" % st["fault"])
        lk.action(3)
        time.sleep(0.6)
    if not st["cal"] or not st["z"]:
        print("cal/z 未就绪，终止")
        lk.disconnect()
        return 2

    lk.set_speed_traj(0.0)
    lk.set_speed_accel(400.0)
    lk.set_speed_profile(0, 20000.0)
    lk.set_mode(3)
    lk.set_speed_rpm(RPM_HI)
    # 保活必须在**第一次给定之后**开：它要求 setpoints_valid[key] 为真，
    # 否则静默失败（返回 ok=False），2 s 后 deadman 自动封锁 —— 现象是
    # "起转正常但两秒后自己停、en=0"。第一版脚本就踩了这个。
    ka = lk.set_keepalive(True, 3, 5.0)
    print("保活:", ka.get("ok"), ka.get("error", ""))
    time.sleep(0.3)
    cw = lk.set_controlword(0x0F, 0)
    print("使能:", cw.get("ok"), {k: cw.get(k) for k in ("status", "status_text", "error")})

    t0 = time.time()
    while time.time() - t0 < 5.0:
        st = lk.ctrl_status()
        if st and abs(st["rpm"] - RPM_HI) < RPM_HI * 0.02:
            break
        time.sleep(0.05)
    print("\n[1] 已到速 %.1f rpm  en=%d fault=0x%04X" % (st["rpm"], st["en"], st["fault"]))
    time.sleep(0.6)

    print("\n[2] 停止（cw=0x06 halt=1）→ 自由滑行 0.25 s")
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.25)
    s = snap90(lk)
    print("    余速 %.1f rpm（%.1f rad/s），en=%d" % (s["rpm"], s["rpm"] * 2 * 3.14159265 / 60, s["en"]))

    print("\n[3] 给 0 rpm 重新进环（这一步就是界面上的\"切到零速\"）")
    t1 = time.time()
    r = lk.set_speed_rpm(0.0)
    print("    30019 -> status=%s ok=%s" % (r.get("status"), r.get("ok")))

    print("    跟踪 1.2 s：")
    hit = None
    rows = []
    last = None
    while time.time() - t1 < 1.2:
        d = snap90(lk)
        if d and d["ts"] != last:
            last = d["ts"]
            d["t"] = d["ts"] - t1
            rows.append(d)
            if hit is None and d["fault"] != 0:
                hit = d
        time.sleep(0.004)
    for d in rows[:40]:
        mark = "  <<< 首个故障" if hit is not None and d is hit else ""
        print("      t=%5.3f  rpm=%8.2f  iq=%6.3f  en=%d  fault=0x%04X%s"
              % (d["t"], d["rpm"], d["iq"], d["en"], d["fault"], mark))
    if hit is None:
        print("    无故障；末点 rpm=%.2f" % (rows[-1]["rpm"] if rows else -1))
    else:
        print("    故障出现 t=%.3f（%.0f ms）fault=0x%04X rpm=%.1f"
              % (hit["t"], hit["t"] * 1000, hit["fault"], hit["rpm"]))

    print("\n[4] 再发一帧 30019（等价保活），看是否 status=3")
    time.sleep(0.25)
    r2 = lk.set_speed_rpm(50.0)
    print("    status=%s ok=%s err=%s" % (r2.get("status"), r2.get("ok"), r2.get("error")))
    st = lk.ctrl_status()
    print("    en=%d cw=0x%02X fault=0x%04X" % (st["en"], st["cw"], st["fault"]))

    print("\n[5] 收尾")
    lk.set_keepalive(False)
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.5)
    st = lk.ctrl_status()
    print("    en=%d fault=0x%04X" % (st["en"], st["fault"]))
    lk.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
