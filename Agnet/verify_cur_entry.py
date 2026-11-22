# -*- coding: utf-8 -*-
"""verify_cur_entry.py —— 力矩模式余速门禁验证

用例（1000 rpm → halt → 带余速切力矩）：
  [1] set_torque(0.15) 默认带预检 → 应直接返回错误（说明 rpm 太高）
  [2] set_torque(0.15, precheck=False) 绕过预检发给固件
      → 固件拒绝（status=3），且**不得置 0x0020**
      修前：会把 src 切到 TORQUE，然后 0.4 ms 内 0x0020 整机封锁
  [3] 停稳后再 set_torque(0.15) → 成功，iq_ref=0.150

用法：python verify_cur_entry.py COM27 1000
"""
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink   # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
RPM = float(sys.argv[2]) if len(sys.argv) > 2 else 1000.0
ok_n = bad_n = 0


def chk(name, cond, info=""):
    global ok_n, bad_n
    if cond:
        ok_n += 1
        print("  [PASS] %s   %s" % (name, info))
    else:
        bad_n += 1
        print("  [FAIL] %s   %s" % (name, info))


def snap(lk):
    m = lk.last_msg.get(30090)
    if m is None:
        return None
    raw = lk._payload_of(m)
    if not raw or len(raw) < 40:
        return None
    f = struct.unpack_from("<8f", raw, 4)
    return {"ts": m["ts"], "en": (raw[3] >> 2) & 1, "rpm": f[2], "iq": f[0],
            "iq_ref": f[1], "fault": raw[36] | (raw[37] << 8)}


def main():
    lk = SerialLink()
    c = lk.connect(PORT, 921600)
    print("连接:", c.get("ok"), c.get("error", ""))
    if not c.get("ok"):
        return 2
    time.sleep(1.2)
    lk.set_permission(0, 0)
    st = lk.ctrl_status()
    if st["fault"]:
        lk.action(3)
        time.sleep(0.6)
    lk.set_speed_traj(0.0)
    lk.set_speed_accel(400.0)
    lk.set_mode(3)
    lk.set_speed_rpm(RPM)
    lk.set_keepalive(True, 3, 5.0)
    time.sleep(0.3)
    lk.set_controlword(0x0F, 0)
    t0 = time.time()
    while time.time() - t0 < 5:
        s = lk.ctrl_status()
        if abs(s["rpm"] - RPM) < RPM * 0.02:
            break
        time.sleep(0.05)
    print("\n[0] 起转 %.1f rpm" % s["rpm"])

    print("\n[1] 运行中直接切力矩（不 halt，余速仍 ~1000）")
    d = snap(lk)
    print("     当前 %.1f rpm" % d["rpm"])
    chk("仍在高速", abs(d["rpm"]) > 270.0, "%.1f rpm" % d["rpm"])
    lk.set_keepalive(False)
    lk.set_mode(4)
    time.sleep(0.2)

    print("\n[2] 带预检切力矩（应被上位机拦下）")
    r = lk.set_torque(0.15)
    print("     -> ok=%s  %s" % (r.get("ok"), r.get("error", "")))
    chk("上位机预检拦下", r.get("ok") is False and "rpm" in str(r.get("error", "")),
        str(r.get("error", ""))[:60])
    d = snap(lk)
    chk("未产生故障", d["fault"] == 0, "0x%04X" % d["fault"])

    print("\n[3] 绕过预检直接发（固件门禁应拒绝，且不置 0x0020）")
    r = lk.set_torque(0.15, precheck=False)
    time.sleep(0.4)
    d = snap(lk)
    print("     发送 ok=%s  en=%d  fault=0x%04X  rpm=%.1f" % (r.get("ok"), d["en"], d["fault"], d["rpm"]))
    chk("固件未置超速故障", d["fault"] == 0, "0x%04X（修前会是 0x0020）" % d["fault"])
    chk("未装环（en=0）", d["en"] == 0, "en=%d" % d["en"])

    print("\n[4] 等停稳再切力矩")
    for _ in range(80):
        d = snap(lk)
        if abs(d["rpm"]) < 5.0:
            break
        time.sleep(0.1)
    print("     停稳 %.1f rpm" % d["rpm"])
    r = lk.set_torque(0.15)
    print("     -> ok=%s %s" % (r.get("ok"), r.get("error", "")))
    time.sleep(0.4)
    d = snap(lk)
    print("     iq_ref=%.3f  iq=%.3f  en=%d  fault=0x%04X" % (d["iq_ref"], d["iq"], d["en"], d["fault"]))
    chk("装环成功", abs(d["iq_ref"] - 0.15) < 0.02, "iq_ref=%.3f" % d["iq_ref"])
    chk("无故障", d["fault"] == 0, "0x%04X" % d["fault"])

    print("\n[5] 收尾")
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.5)
    lk.set_mode(3)
    d = snap(lk)
    print("     en=%d fault=0x%04X rpm=%.1f" % (d["en"], d["fault"], d["rpm"]))
    lk.disconnect()
    print("\n===== %d/%d 通过 =====" % (ok_n, ok_n + bad_n))
    return 0 if bad_n == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
