# -*- coding: utf-8 -*-
"""verify_arm_semantics.py —— 验证"使能 ≠ 跑到上次目标"

用例：
  [1] 速度：1000 rpm → halt（电机滑行）→ cw 使能
      修前：立刻冲回 1000 rpm；修后：保持 0（CtrlSpeed(0) 拉停）
  [2] 再给 500：应正常到 500（给定自己起转，不依赖 cw）
  [3] 力矩：0.15 A → halt → cw 使能
      修前：iq ≈ 0.15；修后：iq ≈ 0

用法：python verify_arm_semantics.py COM27
"""
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink   # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
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


def watch(lk, dur):
    """采 dur 秒，返回 (rpm 列表, iq 列表, 末帧)。"""
    rpm, iq, last, end = [], [], None, None
    t0 = time.time()
    while time.time() - t0 < dur:
        d = snap(lk)
        if d and d["ts"] != last:
            last = d["ts"]
            rpm.append(d["rpm"])
            iq.append(d["iq"])
            end = d
        time.sleep(0.004)
    return rpm, iq, end


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
    r = lk.set_speed_rpm(1000.0)
    ka = lk.set_keepalive(True, 3, 5.0)
    print("     给定: ok=%s  保活: ok=%s %s" % (r.get("ok"), ka.get("ok"), ka.get("error", "")))
    time.sleep(0.3)
    lk.set_controlword(0x0F, 0)
    t0 = time.time()
    while time.time() - t0 < 5:
        s = lk.ctrl_status()
        if abs(s["rpm"] - 1000) < 20:
            break
        time.sleep(0.05)
    print("\n[0] 起转 %.1f rpm" % s["rpm"])

    print("\n[1] halt → 滑行 0.5 s → cw 使能（修前会冲回 1000）")
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.5)
    d = snap(lk)
    print("     滑行余速 %.1f rpm" % d["rpm"])
    r = lk.set_controlword(0x0F, 0)
    print("     cw 应答: ok=%s status=%s" % (r.get("ok"), r.get("status")))
    rpm, iq, end = watch(lk, 1.2)
    print("     cw 后 1.2 s: 末速 %.2f rpm  iq %.3f A  en=%d" % (end["rpm"], end["iq"], end["en"]))
    chk("使能后没有冲向旧目标", abs(end["rpm"]) < 10.0, "末速 %.2f rpm（旧行为会到 1000）" % end["rpm"])
    chk("全程未超 100 rpm", max(abs(x) for x in rpm) < 100.0, "峰值 %.1f" % max(abs(x) for x in rpm))
    chk("无故障", end["fault"] == 0, "0x%04X" % end["fault"])
    chk("仍处于使能", end["en"] == 1)

    print("\n[2] 直接给 500（给定自己起转）")
    r = lk.set_speed_rpm(500.0)
    print("     30019 -> ok=%s status=%s" % (r.get("ok"), r.get("status")))
    for i in range(10):
        lp = lk.loop_status()
        print("       t=%.2f wref=%7.1f wmeas=%7.1f" % (0.15 * (i + 1), lp["wref"], lp["wmeas"]))
        time.sleep(0.15)
    rpm, iq, end = watch(lk, 0.4)
    print("     末速 %.2f rpm" % end["rpm"])
    chk("到 500 rpm", abs(end["rpm"] - 500) < 10.0, "%.2f" % end["rpm"])
    chk("无故障", end["fault"] == 0)

    print("\n[3] 力矩模式：0.15 A → halt → cw 使能（修前会继续出力）")
    lk.set_controlword(0x06, halt=1)
    # 必须等电机真的停：力矩模式的超速门限是固定 300 rpm（SERVO_CUR_WMAX），
    # 带 1000 rpm 余速切过去会在 0.4 ms 内置 0x0020 —— 那是另一个待修缺口
    # （进环余速没有像速度环那样纳入门限），不属本次验证范围。
    end = None
    for _ in range(60):
        end = snap(lk)
        if end and abs(end["rpm"]) < 5.0:
            break
        time.sleep(0.1)
    print("     停稳 rpm=%.1f" % (end["rpm"] if end else -1))
    lk.set_keepalive(False)
    lk.set_mode(4)
    time.sleep(0.2)
    r = lk.set_torque(0.15)
    print("     给定 0.15 A: ok=%s" % r.get("ok"))
    rpm, iq, end = watch(lk, 0.5)
    print("     给定后 iq_ref=%.3f A  iq=%.3f  rpm=%.1f" % (end["iq_ref"], end["iq"], end["rpm"]))
    # 用 iq_ref（固件内部给定）判定，不用实测 iq：0.15 A 给定下实测只有 ~0.08
    # （死区 + 电流环静差），用它判别"装配成功"会被噪声和工况带偏。
    chk("力矩给定已装环", abs(end["iq_ref"] - 0.15) < 0.02, "iq_ref=%.3f" % end["iq_ref"])
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.4)
    r = lk.set_controlword(0x0F, 0)
    print("     cw 应答: ok=%s status=%s" % (r.get("ok"), r.get("status")))
    rpm, iq, end = watch(lk, 0.8)
    print("     cw 后 iq_ref=%.3f A  iq=%.3f  rpm=%.1f  en=%d"
          % (end["iq_ref"], end["iq"], end["rpm"], end["en"]))
    chk("使能后没有按旧 iq 出力", abs(end["iq_ref"]) < 0.01,
        "iq_ref=%.3f（旧行为 0.15）" % end["iq_ref"])

    print("\n[4] 收尾")
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.5)
    lk.set_mode(3)
    st = lk.ctrl_status()
    print("     en=%d fault=0x%04X rpm=%.1f" % (st["en"], st["fault"], st["rpm"]))
    lk.disconnect()
    print("\n===== %d/%d 通过 =====" % (ok_n, ok_n + bad_n))
    return 0 if bad_n == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
