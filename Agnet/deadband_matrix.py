# -*- coding: utf-8 -*-
"""死区补偿收益区间标定：1~2000 rpm × 4 档 du。

矩阵：11 转速 × 4 配置(OFF / 0.30% / 0.59% / 0.90%)
判据：
  1) 转速 CV（编码器位置差分分箱）—— 机械层面平稳度
  2) iq 峰峰值与 RMS（30025 iq_ref）—— 电气层面畸变
  3) 平均 iq —— 效率（受温度影响，仅同组内可比）

用法：python deadband_matrix.py COM27 [--out xxx.csv]
"""
import argparse
import csv
import statistics
import sys
import time
from datetime import datetime

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

ENC = 2097152.0
RPMS = [1, 3, 10, 30, 60, 100, 200, 300, 500, 1000, 2000]
DUS = [("OFF", None), ("0.30%", 30), ("0.59%", 59), ("0.90%", 90)]


def cmd(rig, c, wait=0.5):
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


def fault_of(rig):
    t = cmd(rig, "S", 0.55)
    for l in t.splitlines():
        if "st en=" in l:
            i = l.find("f=0x")
            if i > 0:
                try:
                    return int(l[i + 4:i + 6], 16)
                except ValueError:
                    return None
    return None


def get_enable(rig):
    """B 是切换键：发一次并读切换后的状态。"""
    t = cmd(rig, "B", 0.55)
    return ("ON" in t)


def set_enable(rig, want):
    cur = get_enable(rig)
    if cur != want:
        cur = get_enable(rig)
    return cur


def set_du(rig, n):
    cmd(rig, "\\%d\r" % n, 0.45)


def box_dur(rpm):
    if rpm <= 3:
        return 15.0, 45.0
    if rpm <= 10:
        return 6.0, 20.0
    if rpm <= 30:
        return 3.0, 15.0
    return 2.0, 14.0


def measure(rig, rpm_tgt, box, dur):
    e0 = rig.rx.enc
    if e0 is None:
        return None
    cmd(rig, "=%g\r" % rpm_tgt, 0.35)
    rows, iqref, temps = [], [], []
    t0 = time.time()
    while time.time() - t0 < dur:
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.35)
        e = rig.rx.enc
        s = rig.rx.get_snap()[0]
        if e is not None:
            rows.append((time.time() - t0, e))
        if s is not None:
            if s.get("iq_ref") is not None:
                iqref.append(s["iq_ref"])
            if s.get("temp_mot") is not None:
                temps.append(s["temp_mot"])
    e1 = rig.rx.enc
    if len(rows) < 4 or e1 is None:
        return None

    dur_eff = rows[-1][0] - rows[0][0]
    mean_rpm = (e1 - e0) / ENC / dur_eff * 60.0

    bins = {}
    for t, e in rows:
        bins.setdefault(int(t // box), []).append((t, e))
    segs = []
    for k in sorted(bins):
        v = bins[k]
        if len(v) < 2:
            continue
        dt = v[-1][0] - v[0][0]
        dc = v[-1][1] - v[0][1]
        if dt > box * 0.5:
            segs.append((dc / ENC) / dt * 60.0)
    mu_seg = statistics.mean(segs) if segs else 0.0
    cv_rpm = (statistics.pstdev(segs) / abs(mu_seg) * 100.0) if (len(segs) >= 2 and abs(mu_seg) > 1e-9) else float("nan")

    iq_mean = statistics.mean(iqref) if iqref else float("nan")
    iq_rms = (sum(v * v for v in iqref) / len(iqref)) ** 0.5 if iqref else float("nan")
    iq_pp = (max(iqref) - min(iqref)) if len(iqref) >= 2 else float("nan")
    return dict(mean=mean_rpm, cv=cv_rpm, nseg=len(segs),
                iq_mean=iq_mean, iq_rms=iq_rms, iq_pp=iq_pp,
                temp=(statistics.mean(temps) if temps else float("nan")))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    out = args.out or ("deadband_matrix_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S"))
    rig = Rig(args.port)
    results = []
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.25)
        cmd(rig, "D", 0.6)
        cmd(rig, "X", 0.7)
        cmd(rig, "X", 0.5)
        fa = fault_of(rig)
        print("起始状态 f=0x%s" % ("--" if fa is None else "%02x" % fa))

        # 确认基础档位（速度环 1x，避免之前 L 键漂档影响可比性）
        cmd(rig, "Y", 0.5)
        print("速度环档位已复位 1x")

        for du_name, du_val in DUS:
            if du_val is None:
                set_enable(rig, False)
                print("\n=== 配置 OFF（补偿关闭）===")
            else:
                set_enable(rig, True)
                set_du(rig, du_val)
                print("\n=== 配置 du=%s ===" % du_name)
            time.sleep(0.3)

            for rpm in RPMS:
                box, dur = box_dur(rpm)
                m = measure(rig, rpm, box, dur)
                cmd(rig, "=0\r", 0.3)
                time.sleep(0.25)
                if m is None:
                    print("  %5d rpm  无数据" % rpm)
                    continue
                err = (m["mean"] - rpm) / rpm * 100.0
                f = fault_of(rig)
                if f not in (0, None):
                    cmd(rig, "X", 0.5)
                print("  %5d rpm | %8.2f rpm (%+6.2f%%) | CV %5.1f%% (n=%d) | iq %.2fA rms %.3f pp %.3f | %.1fC | f=0x%s"
                      % (rpm, m["mean"], err, m["cv"], m["nseg"],
                         m["iq_mean"], m["iq_rms"], m["iq_pp"], m["temp"],
                         "--" if f is None else "%02x" % f))
                results.append(dict(config=du_name, rpm_tgt=rpm, rpm_meas=round(m["mean"], 4),
                                    err_pct=round(err, 3), cv_rpm=round(m["cv"], 3),
                                    nseg=m["nseg"], iq_mean=round(m["iq_mean"], 4),
                                    iq_rms=round(m["iq_rms"], 4), iq_pp=round(m["iq_pp"], 4),
                                    temp=round(m["temp"], 2), fault=f))
                # 每点落盘，避免中断丢数据
                with open(out, "w", newline="", encoding="utf-8-sig") as fp:
                    w = csv.DictWriter(fp, fieldnames=list(results[0].keys()))
                    w.writeheader()
                    w.writerows(results)

        # 恢复默认
        set_enable(rig, True)
        set_du(rig, 59)
        cmd(rig, "=0\r", 0.3)
        cmd(rig, "D", 0.8)

        # 汇总
        print("\n" + "=" * 96)
        print("汇总：各转速下 4 档配置的 CV / iq_pp，★=该转速最优")
        print("%6s | %-28s | %-28s" % ("rpm", "CV(%) OFF / 0.30 / 0.59 / 0.90", "iq_pp(A) OFF / 0.30 / 0.59 / 0.90"))
        for rpm in RPMS:
            row = {r["config"]: r for r in results if r["rpm_tgt"] == rpm}
            if len(row) < 2:
                continue
            cvs = [row[c]["cv_rpm"] for c, _ in DUS if c in row]
            pps = [row[c]["iq_pp"] for c, _ in DUS if c in row]
            names = [c for c, _ in DUS if c in row]
            best = min(range(len(cvs)), key=lambda i: (cvs[i] if cvs[i] == cvs[i] else 9e9))
            ct = " / ".join("%5.1f%s" % (v, "★" if i == best else " ") for i, v in enumerate(cvs))
            pt = " / ".join("%.3f " % v for v in pps)
            print("%6d | %-28s | %-28s  best=%s" % (rpm, ct, pt, names[best]))
        print("=" * 96)
        print("[out] %s" % out)
        return 0
    finally:
        try:
            rig.s.write(b"=0\r")
            time.sleep(0.15)
            rig.s.write(b"D")
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    sys.exit(main())
