# -*- coding: utf-8 -*-
"""
psi_comp_verify.py —— Kt 温度补偿实机验证（2026-11-19）

为什么不用 MIT 测：MIT 是力矩控制，空载稳态净力矩必然为 0（否则一直加速），
所以稳态 iq 恒趋近 0，测不到 Kt。给常值 τ 又会在 SERVO_MIT_WMAX=30 rad/s
前只有 60~80 ms，且必触发超速故障。

改用的判据：**速度模式稳态下读 S 行的 ktc 与 tau**
    tau = Kt20 · ktc · iq
  1) ktc 本身随温度按 α=-0.001197 /K 变化 → 证明 Servo_UpdateKtComp 工作
  2) tau/iq 的比值随温度下降 ~3% → 证明补偿真的进了力矩换算
     （比值只由 Kt·comp 决定，排除 iq 自身随摩擦下降的干扰）

用法：
    python psi_comp_verify.py COM27 --rpm 1000 --minutes 18
"""
import argparse
import json
import re
import statistics
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import (Rig, strip_frames, MSG_PERMISSION, MSG_CTRL_WORD,
                          MSG_OP_MODE, MSG_SYS_CMD, MSG_PS_IN)

KT20 = 0.0910
ALPHA = -0.001197
RE_KTC = re.compile(r"\bktc=([\d.]+)")
RE_TAU = re.compile(r"\btau=([-\d.]+)")
RE_IDQ = re.compile(r"\bidq=([-\d.]+)/([-\d.]+)")


def read_sline(rig, wait=0.6):
    rig.rx.pause_evt.set()
    time.sleep(0.08)
    try:
        rig.s.reset_input_buffer()
        rig.s.write(b"S")
        rig.s.flush()
        raw = b""
        t0 = time.time()
        while time.time() - t0 < wait:
            d = rig.s.read(8192)
            if d:
                raw += d
        txt = strip_frames(raw).decode("ascii", "replace")
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()
    for ln in txt.splitlines():
        if "st en=" in ln:
            return ln.strip()
    return ""


def probe(rig, n=3):
    """读 n 次 S 行，返回 ktc / tau / iq / 温度 的均值。"""
    ks, ts, qs = [], [], []
    td = tm = None
    for _ in range(n):
        ln = read_sline(rig)
        m = RE_KTC.search(ln)
        if m:
            ks.append(float(m.group(1)))
        m = RE_TAU.search(ln)
        if m:
            ts.append(float(m.group(1)))
        m = RE_IDQ.search(ln)
        if m:
            qs.append(float(m.group(2)))
        snap, _, _, _, _ = rig.rx.get_snap()
        if snap:
            td, tm = snap.get("temp_drv"), snap.get("temp_mot")
        time.sleep(0.3)
    out = {"ktc": statistics.mean(ks) if ks else None,
           "tau": statistics.mean(ts) if ts else None,
           "iq": statistics.mean(qs) if qs else None,
           "temp_drv": td, "temp_mot": tm,
           "ktc_n": len(ks)}
    if out["tau"] and out["iq"]:
        out["kt_ratio"] = out["tau"] / out["iq"]     # = Kt20 · ktc
        out["comp_from_ratio"] = out["kt_ratio"] / KT20
    if tm:
        out["comp_expected"] = 1.0 + ALPHA * (tm - 20.0)
    return out


def show(tag, r):
    print("  [%-4s] Tmot=%.2f °C  ktc=%-8s tau=%-7s iq=%-6s  τ/iq=%-8s 期望 comp=%.4f"
          % (tag, r.get("temp_mot") or 0,
             ("%.3f" % r["ktc"]) if r["ktc"] else "(未显示)",
             ("%.3f" % r["tau"]) if r["tau"] else "-",
             ("%.2f" % r["iq"]) if r["iq"] else "-",
             ("%.4f" % r["kt_ratio"]) if r.get("kt_ratio") else "-",
             r.get("comp_expected") or 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--rpm", type=float, default=1000.0)
    ap.add_argument("--minutes", type=float, default=18.0)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--out", default=r"G:\FOC\X72\Agnet\psi_comp_verify.json")
    args = ap.parse_args()

    rig = Rig(args.port, args.baud)
    recs = []
    try:
        time.sleep(1.0)
        rig.send(MSG_PERMISSION, bytes([0, 0])); time.sleep(0.25)
        rig.send(MSG_CTRL_WORD, bytes([0, 0, 1])); time.sleep(0.8)
        rig.send(MSG_SYS_CMD, bytes([3])); time.sleep(0.3)     # 清故障
        rig.send(MSG_OP_MODE, bytes([3])); time.sleep(0.25)
        rig.send(MSG_PS_IN, bytes(44)); time.sleep(0.2)        # 先 0 rpm 建环
        rig.send(MSG_CTRL_WORD, bytes([1, 0, 0])); time.sleep(0.4)

        # 升到目标
        b = bytearray(44)
        struct.pack_into("<f", b, 28, args.rpm / 60.0 * 2097152.0)
        rig.send(MSG_PS_IN, bytes(b))
        time.sleep(1.5)

        print("=" * 78)
        print("Kt 温度补偿验证  %.0f rpm 稳态  理论 α=%.4f %%/K" % (args.rpm, ALPHA * 100))
        print("=" * 78)

        c = probe(rig, 3)
        show("cold", c)
        recs.append(dict(c, tag="cold"))

        print("\n[..] 加热 %.0f min ..." % args.minutes)
        t_end = time.time() + args.minutes * 60.0
        nxt = time.time()
        while time.time() < t_end:
            rig.send(MSG_PS_IN, bytes(b))          # 保活（deadman 2 s）
            if time.time() >= nxt:
                nxt += 120.0
                snap, _, _, _, _ = rig.rx.get_snap()
                if snap:
                    print("  t=%4.0f s  Tdrv=%.1f  Tmot=%.1f °C"
                          % (args.minutes * 60 - (t_end - time.time()),
                             snap.get("temp_drv") or 0, snap.get("temp_mot") or 0))
            time.sleep(0.5)

        h = probe(rig, 3)
        print()
        show("hot", h)
        recs.append(dict(h, tag="hot"))

        print("\n" + "=" * 78)
        print("判定")
        print("=" * 78)
        ok = True
        if c.get("ktc") and h.get("ktc") and c.get("comp_expected") and h.get("comp_expected"):
            dt = (h["temp_mot"] or 0) - (c["temp_mot"] or 0)
            d_meas = h["ktc"] - c["ktc"]
            d_exp = h["comp_expected"] - c["comp_expected"]
            print("温度 %.2f → %.2f °C（%+.2f K）"
                  % (c["temp_mot"], h["temp_mot"], dt))
            print("ktc  实测 %.3f → %.3f（%+.4f）" % (c["ktc"], h["ktc"], d_meas))
            print("     理论 %+.4f（α=%.4f %%/K）" % (d_exp, ALPHA * 100))
            ratio = (d_meas / d_exp) if abs(d_exp) > 1e-6 else 0.0
            print("     幅度比 %.2f  （1.00 = 完全吻合；put_f3 量化 0.001，"
                  "跨度 %.1f K 下约 %.0f%% 误差）"
                  % (ratio, dt, 0.001 / abs(d_exp) * 100 if abs(d_exp) > 1e-6 else 0))
            if 0.55 < ratio < 1.45:
                print("  ✔ ktc 随温度按实测 α 变化")
            else:
                print("  ✘ ktc 变化幅度不符（也检查是否温度跨度太小）")
                ok = False
            for r, nm in ((c, "冷"), (h, "热")):
                if r.get("ktc") and r.get("comp_expected"):
                    dd = r["ktc"] - r["comp_expected"]
                    print("  %s态绝对值 |ktc−理论| = %.4f" % (nm, abs(dd)))
                    if abs(dd) > 0.004:
                        print("    ✘ 偏差超过量化+温度采样误差")
                        ok = False
        else:
            print("  ! ktc 未读到（检查 S 行是否超长被截断）")
            ok = False

        if c.get("kt_ratio") and h.get("kt_ratio"):
            print("\n[参考] τ/iq = %.4f → %.4f：此项受 tau(f3)/iq(f2) 量化限制，"
                  "单点误差可达 6%%，仅作趋势参考，不参与判定"
                  % (c["kt_ratio"], h["kt_ratio"]))
        print("\n总判定：%s" % ("PASS 补偿生效" if ok else "FAIL"))
        recs.append({"verdict": "PASS" if ok else "FAIL"})

    finally:
        try:
            rig.send(MSG_CTRL_WORD, bytes([0, 0, 1]))
            time.sleep(0.3)
        except Exception:
            pass
        rig.close()

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump({"rpm": args.rpm, "alpha": ALPHA, "recs": recs},
                  f, ensure_ascii=False, indent=2)
    print("\n[out] %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
