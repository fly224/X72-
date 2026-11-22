"""速度闭环恢复验收与特性测试（X72 FOC）

用途
----
编码器修好（换板 / MT6835 官方工具重写补偿表）之后，用它一次性完成：
  ① 联锁自检   f=0x00、cal=1、z=1
  ② Z 电角度对齐 并**硬校验** ratio≈14 / pn=14
  ③ 转矩能力验收  0° 与 +90° 电角度补偿对比（正确时应出现单峰、±90° 接近 0）
  ④ 速度闭环阶跃  60 / 300 / 1000 rpm：上升时间、超调、稳态误差、iq

前置条件（必须人工确认，本脚本不做）
------------------------------------
  * 已完整跑过 `^` 编码器自校准，且**断电 >6 s 再上电**
  * 已做过 C（电流零点）；未做过则先发 C 并等 3 s，否则 Z 可能 ratio_bad
  * 电机空载、可自由旋转；母线约 30 V

用法
----
    python speed_loop_verify.py COM27
    python speed_loop_verify.py COM27 --rpm 60 300 1000 --out verify.csv

判据（全部满足才 PASS）
----------------------
  * Z: 13.5 <= ratio <= 14.5 且 pn == 14 且 |zero| <= 20°
  * 转矩: peak(0°) > 3 * peak(+90°) 且 peak(0°) > 60 rpm（1 A / 0.25 s）
  * 速度: 每个目标 |稳态误差| <= 1% 且超调 <= 15% 且无故障
"""

import argparse
import csv
import math
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from thermal_soak import (Rig, strip_frames, MSG_PERMISSION,
                          POLE_PAIRS, RAD_S_TO_RPM)

RE_IDENT = re.compile(
    r"ratio=([\d.]+)\s+pn=(\d+)\s+dir=([-+]\d+)\s+off=([-\d.]+)deg"
    r"\s+zero=([-\d.]+)deg")
RE_FAULT = re.compile(r"f=0x([0-9a-fA-F]+)")


# ---------------------------------------------------------------- 基础工具
def cmd(rig, s, wait=0.55):
    """发一条 ASCII 命令并取回显（期间暂停 RX 线程，避免抢串口）。"""
    rig.rx.pause_evt.set()
    time.sleep(0.04)
    try:
        rig.s.reset_input_buffer()
        rig.s.write(s.encode())
        rig.s.flush()
        buf = b""
        t0 = time.time()
        while time.time() - t0 < wait:
            d = rig.s.read(8192)
            if d:
                buf += d
        return strip_frames(buf).decode("ascii", "replace")
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def pick(text, key):
    for line in text.splitlines():
        if key in line:
            return line.strip()
    return ""


def status(rig):
    """返回 (故障码, moe, 原始 S 行)。"""
    line = pick(cmd(rig, "S", 0.65), "st en=")
    mf = RE_FAULT.search(line)
    mm = re.search(r"moe=(\d)", line)
    return (int(mf.group(1), 16) if mf else None,
            int(mm.group(1)) if mm else None, line)


def clean(rig, tries=3):
    """封锁 + 清故障，直到 f=0x00。"""
    for _ in range(tries):
        cmd(rig, "D", 0.45)
        cmd(rig, "X", 0.5)
        time.sleep(0.15)
        f, moe, _ = status(rig)
        if f == 0 and moe == 0:
            return True
    return False


def rpm_of(rig):
    snap = rig.rx.get_snap()[0]
    w = snap.get("w_elec") if snap else None
    return 0.0 if w is None else w / POLE_PAIRS * RAD_S_TO_RPM


def iq_of(rig):
    snap = rig.rx.get_snap()[0]
    return (snap.get("iq_ref") or 0.0) if snap else 0.0


def sample(rig, dur, dt=0.02):
    """连续采样 (t, rpm, iq)。"""
    out = []
    t0 = time.time()
    while True:
        t = time.time() - t0
        if t >= dur:
            break
        out.append((t, rpm_of(rig), iq_of(rig)))
        time.sleep(dt)
    return out


# ---------------------------------------------------------------- 各阶段
def stage_lock(rig):
    ok = clean(rig)
    f, moe, line = status(rig)
    print("[1] 联锁自检   f=0x%s moe=%s" % (
        "?" if f is None else "%02x" % f, moe))
    if line:
        print("    " + line[:150])
    return ok


def stage_align(rig):
    txt = cmd(rig, "Z", 18.0)
    line = pick(txt, "IDENT ok") or pick(txt, "IDENT FAIL") or "(无回显)"
    print("[2] Z 对齐     " + line[:150])
    m = RE_IDENT.search(txt)
    if not m:
        return False, None
    ratio, pn, d, off, zero = (float(m.group(1)), int(m.group(2)),
                               int(m.group(3)), float(m.group(4)),
                               float(m.group(5)))
    ok = (13.5 <= ratio <= 14.5) and (pn == POLE_PAIRS) and (abs(zero) <= 20.0)
    print("    ratio=%.2f  pn=%d  dir=%+d  off=%.2f°  zero=%.2f°  -> %s"
          % (ratio, pn, d, off, zero, "PASS" if ok else "FAIL"))
    return ok, ratio


def stage_torque(rig):
    """0° 与 +90° 电角度补偿对比：正确时应为单峰。"""
    res = {}
    for eo in (0, 90):
        if not clean(rig):
            res[eo] = 0.0
            continue
        cmd(rig, "-%d\r" % eo, 0.4)
        cmd(rig, ">1.0\r", 0.15)
        vals = [abs(v) for _, v, _ in sample(rig, 0.25)]
        cmd(rig, "D", 0.35)
        res[eo] = max(vals) if vals else 0.0
        clean(rig)
    cmd(rig, "-0\r", 0.4)          # 复位补偿（掉电也会复位，这里显式做一次）
    ok = (res[90] < res[0] / 3.0) and (res[0] > 60.0)
    print("[3] 转矩能力   peak(0°)=%.1f rpm  peak(+90°)=%.1f rpm  -> %s"
          % (res[0], res[90], "PASS" if ok else "FAIL"))
    if not ok:
        print("    提示：单峰不成立说明编码器角度仍不可用（零点错或随位置失真）")
    return ok


def stage_speed(rig, target):
    if not clean(rig):
        return {"target": target, "ok": False, "note": "clear_fault_failed"}
    cmd(rig, "=%g\r" % target, 0.35)
    data = sample(rig, 2.5)
    f, moe, _ = status(rig)
    cmd(rig, "=0\r", 0.3)
    cmd(rig, "D", 0.35)

    rpms = [v for _, v, _ in data]
    if not rpms:
        return {"target": target, "ok": False, "note": "no_sample"}

    t90 = None
    for t, v, _ in data:
        if abs(v) >= 0.9 * abs(target):
            t90 = t
            break

    tail = [v for t, v, _ in data if t >= 1.9]      # 末段 0.6 s 作稳态
    ss = sum(tail) / len(tail) if tail else rpms[-1]
    peak = max(rpms, key=abs)
    overshoot = (abs(peak) - abs(target)) / abs(target) * 100.0 if target else 0.0
    err = ss - target
    iq = sum(i for _, _, i in data) / len(data)

    ok = (f == 0) and (abs(err) <= 0.01 * abs(target)) and (overshoot <= 15.0)
    print("[4] %6.0f rpm   t90=%s  超调=%+5.1f%%  稳态=%8.2f rpm（err %+6.2f）"
          "  iq=%.3f A  f=0x%s -> %s"
          % (target,
             "--" if t90 is None else "%.2fs" % t90,
             overshoot, ss, err, iq,
             "?" if f is None else "%02x" % f,
             "PASS" if ok else "FAIL"))

    return {"target": target, "t90": t90, "overshoot": overshoot,
            "ss": ss, "err": err, "iq": iq, "fault": f, "ok": ok}


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--rpm", type=float, nargs="+", default=[60, 300, 1000])
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    rig = Rig(args.port, args.baud)
    verdict = []
    rows = []
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.3)

        a = stage_lock(rig)
        verdict.append(("联锁", a))
        if not a:
            print("\n联锁未通过，先排查：f 清不掉 / moe 不为 0")
            return 2

        b, ratio = stage_align(rig)
        verdict.append(("Z 对齐", b))
        if not b:
            print("\nZ 未通过：不要继续。先确认 ^ 已完整跑完并断电 >6 s 再上电。")
            return 2

        c = stage_torque(rig)
        verdict.append(("转矩能力", c))

        ok_all = True
        for target in args.rpm:
            r = stage_speed(rig, target)
            rows.append(r)
            ok_all = ok_all and r["ok"]
        verdict.append(("速度闭环", ok_all))

        print("\n" + "=" * 64)
        for name, ok in verdict:
            print("  %-10s %s" % (name, "PASS" if ok else "FAIL"))
        print("=" * 64)

        if args.out:
            with open(args.out, "w", newline="", encoding="utf-8-sig") as fp:
                w = csv.writer(fp)
                w.writerow(["target_rpm", "t90_s", "overshoot_pct",
                            "ss_rpm", "err_rpm", "iq_a", "fault", "ok"])
                for r in rows:
                    w.writerow([r.get("target"), r.get("t90"),
                                r.get("overshoot"), r.get("ss"),
                                r.get("err"), r.get("iq"),
                                r.get("fault"), r.get("ok")])
            print("CSV: %s" % args.out)

        return 0 if all(ok for _, ok in verdict) else 2
    finally:
        try:
            rig.s.write(b"D")
            rig.s.flush()
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    sys.exit(main())
