# -*- coding: utf-8 -*-
"""速度模式 1000 rpm 稳态测试（一键）。

流程：D（确保停）→ E（使能）→ 给定 1000 rpm → 保活采样 60 s → D → 出表。

为什么要保活：速度模式 deadman 2 s，2 s 内没有命令就自动封锁。
采样期间每 0.35 s 重发一次 `=1000`，同时用最小二乘斜率算真实转速
（不用首尾差：起止点落在加速段会把误差放大，11-18 踩过，标度偏差 3~5%）。

判据：
  · 稳态误差 = (斜率转速 − 1000) / 1000，|误差| ≤ 0.2% 为合格
  · 波动用 2 s 分箱的 CV（变异系数），≤ 2% 为平稳
  · iq 均值应与历史 2000 rpm 空载的 0.5 A 同量级（1000 rpm 通常略低）
  · 温度只记录，不作为判据（本测试是稳态短跑，温升 < 5 K）

用法：
  python speed_1000rpm.py COM27
  python speed_1000rpm.py COM27 --rpm 1000 --dur 60
"""
import argparse
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

ENC = 2097152.0
RAD_PER_RPM = 0.10471975512


def cmd(rig, c, wait=0.5):
    """发命令并收回显（暂停 rx 线程，避免遥测帧插入把回显顶掉）。"""
    rig.rx.pause_evt.set()
    time.sleep(0.03)
    try:
        rig.s.reset_input_buffer()
        rig.s.write(c.encode() if isinstance(c, str) else c)
        rig.s.flush()
        raw = b""
        t0 = time.time()
        while time.time() - t0 < wait:
            d = rig.s.read(8192)
            if d:
                raw += d
        return strip_frames(raw).decode("ascii", "replace")
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def one_line(txt, key):
    for l in txt.splitlines():
        if key in l:
            return l.strip()
    return ""


def slope_rpm(samples):
    """最小二乘斜率 → rpm。samples = [(t, enc_sum)]"""
    n = len(samples)
    if n < 3:
        return None
    mt = sum(s[0] for s in samples) / n
    me = sum(s[1] for s in samples) / n
    num = sum((s[0] - mt) * (s[1] - me) for s in samples)
    den = sum((s[0] - mt) ** 2 for s in samples)
    if den <= 0:
        return None
    counts_per_s = num / den
    return counts_per_s / ENC * 60.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--rpm", type=float, default=1000.0)
    ap.add_argument("--dur", type=float, default=60.0, help="采样时长 s")
    ap.add_argument("--settle", type=float, default=3.0, help="起转后稳定时间 s")
    args = ap.parse_args()

    rig = Rig(args.port)
    rows = []
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.3)

        print("[1] 前置检查", flush=True)
        print("    " + one_line(cmd(rig, "S", 0.7), "[CMD] st"))
        cmd(rig, "D", 0.6)                     # 确保从封锁态起步

        print("[2] 使能", flush=True)
        r = cmd(rig, "E", 0.7)
        print("    " + (one_line(r, "[CMD]") or "(无回显)"))
        if "ON" not in r:
            print("[!] 使能失败，终止")
            return 1

        print("[3] 给定 %.0f rpm，稳定 %.0f s" % (args.rpm, args.settle), flush=True)
        t0 = time.time()
        while time.time() - t0 < args.settle:
            with rig.wlock:
                rig.s.write(("=%g\r" % args.rpm).encode())
                rig.s.flush()
            time.sleep(0.35)

        print("[4] 采样 %.0f s" % args.dur, flush=True)
        t0 = time.time()
        while time.time() - t0 < args.dur:
            with rig.wlock:
                rig.s.write(("=%g\r" % args.rpm).encode())
                rig.s.flush()
            snap, ts, iq_now, enc, enc_t = rig.rx.get_snap()
            if enc is not None:
                rows.append((time.time(), float(enc), snap.get("temp_mot"),
                             snap.get("iq_ref"), snap.get("vbus")))
            time.sleep(0.35)

        st = cmd(rig, "S", 0.7)
        k = cmd(rig, "K", 0.7)
    finally:
        try:
            cmd(rig, "D", 0.6)
        except Exception:
            pass
        rig.close()

    if len(rows) < 10:
        print("[!] 采样点不足 %d，无法统计" % len(rows))
        return 1

    # ---- 转速：前 10% 视为残余加速段，丢掉 ----
    cut = max(3, len(rows) // 10)
    use = rows[cut:]
    sp = [(r[0] - use[0][0], r[1]) for r in use]
    rpm = slope_rpm(sp)

    # ---- 波动：2 s 分箱 CV ----
    bins = []
    cur = []
    bt0 = sp[0][0]
    for t, e in sp:
        if t - bt0 >= 2.0:
            if len(cur) >= 3:
                bins.append(slope_rpm([(x - cur[0][0], y) for x, y in cur]))
            cur = []
            bt0 = t
        cur.append((t, e))
    if len(cur) >= 3:
        bins.append(slope_rpm([(x - cur[0][0], y) for x, y in cur]))
    bins = [b for b in bins if b]
    cv = (statistics.pstdev(bins) / statistics.mean(bins) * 100.0) if len(bins) > 1 else float("nan")

    iqs = [r[3] for r in use if r[3] is not None]
    temps = [r[2] for r in use if r[2] is not None]
    err = (rpm - args.rpm) / args.rpm * 100.0 if rpm else float("nan")

    print("\n=== 结果 ===")
    print("给定        : %.0f rpm" % args.rpm)
    print("实测(斜率)  : %.2f rpm   误差 %+.3f %%" % (rpm, err))
    print("分箱 CV     : %.2f %%  (%d 个 2 s 箱)" % (cv, len(bins)))
    print("iq 均值     : %.3f A" % (statistics.mean(iqs) if iqs else float("nan")))
    print("温度        : %.1f → %.1f °C" % (temps[0], temps[-1]) if temps else "温度        : --")
    print("采样点      : %d（丢前 %d 点）" % (len(use), cut))
    print("\nS: " + one_line(st, "[CMD] st"))
    print("K: " + one_line(k, "[CMD] spd"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
