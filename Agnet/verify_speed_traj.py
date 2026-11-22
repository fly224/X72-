# -*- coding: utf-8 -*-
"""verify_speed_traj.py —— 30097 速度轨迹"到达时间优先"验证

判据（100 → 300 rpm，Δω = 20.944 rad/s）：
  梯形 t=0.5 s   → a_eff ≈ 41.9 rad/s²，实测到达 0.5 s
  S 曲线 t=0.5 s → a_eff ≈ 83.8、j_eff ≈ 335.1，实测到达 0.5 s
  关闭 t=0 + acc=400 → a_eff=400，到达 ≈ 52 ms
  t=0.0005 s     → 反算 a≈41888 超上限，被夹到 20000 且 flags.clamped=1

用法：python verify_speed_traj.py COM27
"""
import math
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink   # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
RPM_LO, RPM_HI = 100.0, 300.0
DW = (RPM_HI - RPM_LO) * 2.0 * math.pi / 60.0      # 20.944 rad/s
ok_n = bad_n = 0


def chk(name, cond, info=""):
    global ok_n, bad_n
    if cond:
        ok_n += 1
        print("  [PASS] %s   %s" % (name, info))
    else:
        bad_n += 1
        print("  [FAIL] %s   %s" % (name, info))


def sample(lk, dur):
    """采 30091 的 wref/wmeas（100 Hz），按时间戳去重。"""
    ts, wr, wm, acc = [], [], [], []
    t0 = time.time()
    last = None
    while time.time() - t0 < dur:
        m = lk.last_msg.get(30091)
        if m is not None and m["ts"] >= t0 and m["ts"] != last:
            raw = lk._payload_of(m)
            if raw and len(raw) >= 48:
                last = m["ts"]
                ts.append(m["ts"] - t0)
                wr.append(struct.unpack_from("<f", raw, 4)[0])
                wm.append(struct.unpack_from("<f", raw, 8)[0])
                acc.append(struct.unpack_from("<f", raw, 24)[0])
        time.sleep(0.004)
    return ts, wr, wm, acc


def step(lk, target_rpm, dur=1.2):
    """下发目标转速，返回 (到达时间 s, 峰值, a_eff, 轨迹回读)。"""
    t0 = time.time()
    lk.set_speed_rpm(target_rpm)
    ts, wr, wm, acc = sample(lk, dur)
    tr = lk.speed_traj_read()
    tgt = target_rpm
    t_reach = None
    for t, r in zip(ts, wr):
        if abs(r - tgt) <= abs(tgt) * 0.01:
            t_reach = t
            break
    a_eff = acc[-1] if acc else float("nan")
    return t_reach, max(wm) if wm else 0.0, a_eff, tr


def main():
    lk = SerialLink()
    r = lk.connect(PORT, 921600)
    print("连接:", r.get("ok"), r.get("error", ""))
    time.sleep(1.2)
    lk.set_permission(0, 0)
    st = lk.ctrl_status()
    if st["fault"]:
        lk.action(3)
        time.sleep(0.5)
    lk.set_mode(3)
    lk.set_speed_accel(400.0)
    lk.set_speed_rpm(RPM_LO)
    lk.set_keepalive(True, 3, 5.0)
    time.sleep(0.3)
    lk.set_controlword(0x0F, 0)
    time.sleep(1.5)
    st = lk.ctrl_status()
    print("起转: en=%d mode=%d rpm=%.2f fault=0x%04X" % (st["en"], st["mode"], st["rpm"], st["fault"]))
    if st["en"] == 0:
        print("使能失败，终止")
        lk.disconnect()
        return 2

    print("\n[0] 初始回读")
    b = lk.speed_traj_read()
    print("     ", {k: b.get(k) for k in ("ok", "t_ramp_s", "a_eff", "j_eff", "time_on",
                                          "clamped", "scurve", "acc_cfg", "j_cfg")})
    chk("30097 可读", b.get("ok") is True, str(b.get("error", "")))
    chk("默认时间模式关", b.get("time_on") is False)
    chk("acc_cfg=400", abs(b.get("acc_cfg", 0) - 400.0) < 1.0, "%.1f" % b.get("acc_cfg", 0))

    print("\n[1] 梯形 t=0.5 s（100→300 rpm）")
    lk.set_speed_profile(0, 20000.0)
    r1 = lk.set_speed_traj(0.5)
    print("     写入:", {k: r1.get(k) for k in ("ok", "t_ramp_s", "time_on", "clamped")})
    chk("时间模式生效", r1.get("time_on") is True)
    # a_eff 是"本段正在用的值"，在目标变化那一拍才锁存 —— 所以要在改转速之后读
    tr, pk, a_eff, rr1 = step(lk, RPM_HI)
    print("     到达 %.3f s  峰值 %.2f rpm" % (tr or -1, pk))
    print("     改速后回读: a_eff=%.2f j_eff=%.0f flags(clamp)=%s"
          % (rr1.get("a_eff", 0), rr1.get("j_eff", 0), rr1.get("clamped")))
    chk("反算 a=Δω/t=41.9", abs(rr1.get("a_eff", 0) - DW / 0.5) < 1.5,
        "a_eff=%.2f" % rr1.get("a_eff", 0))
    chk("到达时间 0.5 s（±15%）", tr is not None and abs(tr - 0.5) <= 0.075, "实测 %.3f" % (tr or -1))
    chk("遥测 acc 报实际值", abs(a_eff - DW / 0.5) < 15.0, "%.1f" % a_eff)

    print("\n[2] S 曲线 t=0.5 s（100→300 rpm）")
    lk.set_speed_rpm(RPM_LO)
    time.sleep(0.9)
    lk.set_speed_profile(1, 20000.0)
    r2 = lk.set_speed_traj(0.5)
    tr2, pk2, a2, rr2 = step(lk, RPM_HI)
    print("     到达 %.3f s  峰值 %.2f rpm" % (tr2 or -1, pk2))
    print("     改速后回读: a_eff=%.2f j_eff=%.1f scurve=%s clamp=%s"
          % (rr2.get("a_eff", 0), rr2.get("j_eff", 0), rr2.get("scurve"), rr2.get("clamped")))
    chk("S 曲线 a=2Δω/t=83.8", abs(rr2.get("a_eff", 0) - 2 * DW / 0.5) < 3.0,
        "%.2f" % rr2.get("a_eff", 0))
    chk("S 曲线 j=4Δω/t²=335.1", abs(rr2.get("j_eff", 0) - 4 * DW / 0.25) < 12.0,
        "%.1f" % rr2.get("j_eff", 0))
    chk("S 曲线到达 0.5 s（±20%）", tr2 is not None and abs(tr2 - 0.5) <= 0.10, "实测 %.3f" % (tr2 or -1))

    print("\n[3] 关闭时间模式 + 梯形 + acc=400（应 ≈ 52 ms）")
    lk.set_speed_rpm(RPM_LO)
    # 必须切回梯形：[2] 把 profile 设成了 S 曲线，j=20000 下加速度从 0 爬到 400
    # 要多花 20 ms、少走 4 rad/s，实测会把 52 ms 拖到 85 ms 以上。
    lk.set_speed_profile(0, 20000.0)
    # 必须等起点真的稳在 100 rpm：否则 Δω 不是 20.944，时间自然偏长
    for _ in range(40):
        if abs(lk.loop_status().get("wref", 1e9) - RPM_LO) < 2.0:
            break
        time.sleep(0.1)
    time.sleep(0.3)
    r3 = lk.set_speed_traj(0.0)
    chk("时间模式已关", r3.get("time_on") is False)
    tr3, _, a3, rr3 = step(lk, RPM_HI, dur=0.8)
    print("     到达 %.3f s（理论 0.052）  改速后 a_eff=%.1f" % (tr3 or -1, rr3.get("a_eff", 0)))
    chk("回落到 a=400", abs(rr3.get("a_eff", 0) - 400.0) < 1.0, "%.1f" % rr3.get("a_eff", 0))
    # 30091 是 100 Hz：首点相位 0~10 ms + 末点量化 10 ms + 分发延迟 ~2 ms，
    # 52 ms 的斜坡最低只能测到 55~85 ms。下界防"根本没走斜坡"，上界含量化。
    chk("到达 30~95 ms（量化下限）", tr3 is not None and 0.030 <= tr3 <= 0.095,
        "实测 %.3f" % (tr3 or -1))

    print("\n[4] 超短时间触发夹取（t=0.0005 s → a 需 41888 > 20000）")
    lk.set_speed_rpm(RPM_LO)
    time.sleep(0.9)
    lk.set_speed_profile(0, 20000.0)
    lk.set_speed_traj(0.0005)
    tr4, _, _, rr4 = step(lk, RPM_HI, dur=0.6)
    print("     到达 %.3f s  回读 a_eff=%.0f clamp=%s" % (tr4 or -1, rr4.get("a_eff", 0), rr4.get("clamped")))
    chk("a 被夹到 20000", abs(rr4.get("a_eff", 0) - 20000.0) < 1.0, "%.0f" % rr4.get("a_eff", 0))
    chk("flags 标出被夹", rr4.get("clamped") is True)

    print("\n[5] t_left 合理性（t=2 s 时读到 ≈2 s）")
    lk.set_speed_rpm(RPM_LO)
    time.sleep(1.0)
    r5 = lk.set_speed_traj(2.0)
    lk.set_speed_rpm(RPM_HI)
    time.sleep(0.25)
    r5b = lk.speed_traj_read()
    print("     a_eff=%.2f t_left=%.2f" % (r5b.get("a_eff", 0), r5b.get("t_left_s", 0)))
    chk("t_left 在 (0.3, 2.0] 内", 0.3 < r5b.get("t_left_s", -1) <= 2.0, "%.2f" % r5b.get("t_left_s", -1))

    print("\n[6] 收尾")
    lk.set_speed_traj(0.0)
    lk.set_speed_rpm(0)
    time.sleep(0.8)
    lk.set_keepalive(False)
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.6)
    st = lk.ctrl_status()
    print("     en=%d rpm=%.2f fault=0x%04X" % (st["en"], st["rpm"], st["fault"]))
    chk("封锁成功", st["en"] == 0)
    lk.disconnect()
    print("\n===== %d/%d 通过 =====" % (ok_n, ok_n + bad_n))
    return 0 if bad_n == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
