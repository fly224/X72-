# -*- coding: utf-8 -*-
"""组合配置全速段实测：死区补偿 0.59% + 5/7 次谐波抑制 ON，1~2000 rpm。

测量方法（已修正）：
  1) 先给目标并稳定 2 s（期间保活）
  2) 连续采 (t, enc_sum) + 30025 的 iq_ref/温度
  3) 转速用最小二乘斜率（不受起止点边界影响）
  4) 波动用同批数据分箱（低速箱长，保证每箱至少约 1 转）

每点落盘 CSV，便于中断保存。

用法：python combo_sweep.py COM27 [--out xxx.csv]
"""
import argparse
import csv
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

ENC = 2097152.0
RPMS = [60, 100, 200, 300, 500, 1000, 2000]   # 1~30 rpm 已证实谐波抑制不适用，见报告


def cmd(rig, c, wait=0.55):
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


def fault(rig):
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


def toggle_expect(rig, key, want_on):
    t = cmd(rig, key, 0.55)
    cur = ("ON" in t)
    if cur != want_on:
        t = cmd(rig, key, 0.55)
        cur = ("ON" in t)
    return cur


def box_dur(rpm):
    if rpm <= 3:
        return 15.0, 45.0
    if rpm <= 10:
        return 6.0, 22.0
    if rpm <= 30:
        return 3.0, 16.0
    return 2.0, 14.0


def measure(rig, rpm_tgt, box, dur, settle=2.0, harm_settle=2.5):
    """先建环再开抑制（固件注释要求）；失稳则返回 unstable 标记而不是硬跑。"""
    toggle_expect(rig, "%", False)          # 关抑制建环
    cmd(rig, "=%g\r" % rpm_tgt, 0.35)
    t0 = time.time()
    while time.time() - t0 < settle:
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.4)

    # 闭环内开抑制，等积分建立
    toggle_expect(rig, "%", True)
    t0 = time.time()
    while time.time() - t0 < harm_settle:
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.4)

    f1 = fault(rig)                          # 只读一次（会暂停接收线程）
    if f1 not in (0, None):
        return dict(unstable=True, fault=f1)

    xs, ys, iqs, temps = [], [], [], []
    ts = time.time()
    iq_abort = False
    while time.time() - ts < dur:
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.4)
        e = rig.rx.enc
        s = rig.rx.get_snap()[0]
        if e is not None:
            xs.append(time.time() - ts)
            ys.append(float(e))
        if s is not None:
            if s.get("iq_ref") is not None:
                iqs.append(s["iq_ref"])
                if abs(s["iq_ref"]) > 2.85:
                    iq_abort = True
            if s.get("temp_mot") is not None:
                temps.append(s["temp_mot"])

    f = fault(rig)                           # 窗口末尾再确认一次
    if f not in (0, None):
        return dict(unstable=True, fault=f)

    if len(xs) < 8:
        return None
    n = len(xs)
    mx = statistics.mean(xs)
    my = statistics.mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    k = sxy / sxx if sxx > 0 else 0.0
    rpm_pos = k / ENC * 60.0

    bins = {}
    for x, y in zip(xs, ys):
        bins.setdefault(int(x // box), []).append((x, y))
    segs = []
    for key in sorted(bins):
        v = bins[key]
        if len(v) < 2:
            continue
        dt = v[-1][0] - v[0][0]
        dc = v[-1][1] - v[0][1]
        if dt > box * 0.5:
            segs.append((dc / ENC) / dt * 60.0)
    mu = statistics.mean(segs) if segs else 0.0
    cv = (statistics.pstdev(segs) / abs(mu) * 100.0) if (len(segs) >= 2 and abs(mu) > 1e-9) else float("nan")

    return dict(rpm=rpm_pos, cv=cv, nseg=len(segs),
                iq_mean=statistics.mean(iqs) if iqs else float("nan"),
                iq_rms=(sum(v * v for v in iqs) / len(iqs)) ** 0.5 if iqs else float("nan"),
                iq_pp=(max(iqs) - min(iqs)) if len(iqs) >= 2 else float("nan"),
                iq_abort=iq_abort,
                temp=statistics.mean(temps) if temps else float("nan"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--out", default="combo_dead_harm_1-2000rpm_20261120.csv")
    args = ap.parse_args()

    rig = Rig(args.port)
    out = args.out
    rows = []
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.25)
        cmd(rig, "D", 0.6)
        cmd(rig, "X", 0.7)
        cmd(rig, "Y", 0.4)

        # 固定配置：死区 0.59%；谐波抑制由每点内部按"先建环后开"控制
        print("配置: 死区 %s du=0.59%%" % toggle_expect(rig, "B", True))
        cmd(rig, "\\59\r", 0.4)
        toggle_expect(rig, "%", False)
        for l in cmd(rig, "\\\r", 0.5).splitlines():
            if "[CMD]" in l:
                print("  ", l.strip()[:100])
        time.sleep(0.4)

        print("\n=== 死区0.59%% + 5/7 谐波抑制 ON，1~2000 rpm ===")
        print("%6s | %9s | %7s | %7s | %8s | %8s | %8s | %s"
              % ("目标", "实测rpm", "误差%", "CV%", "iq均A", "iq_rms", "iq_pp", "温度/故障"))

        for rpm_tgt in RPMS:
            # 每点前强制复位，避免上一点失稳的状态残留
            toggle_expect(rig, "%", False)
            cmd(rig, "D", 0.5)
            cmd(rig, "X", 0.5)
            time.sleep(0.3)
            box, dur = box_dur(rpm_tgt)
            m = measure(rig, rpm_tgt, box, dur)
            cmd(rig, "=0\r", 0.3)
            time.sleep(0.3)
            if m is not None and m.get("unstable"):
                print("%6d | *** 谐波抑制失稳 f=0x%02x，该点无有效数据 ***"
                      % (rpm_tgt, m["fault"]))
                toggle_expect(rig, "%", False)
                cmd(rig, "X", 0.6)
                time.sleep(0.4)
                rows.append(dict(rpm_tgt=rpm_tgt, rpm_meas=0.0, err_pct=-100.0,
                                 cv=float("nan"), nseg=0, iq_mean=float("nan"),
                                 iq_rms=float("nan"), iq_pp=float("nan"),
                                 temp=float("nan"), fault=m["fault"]))
                with open(out, "w", newline="", encoding="utf-8-sig") as fp:
                    w = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
                    w.writeheader()
                    w.writerows(rows)
                continue
            if m is None:
                print("  %5d  数据不足" % rpm_tgt)
                continue
            err = (m["rpm"] - rpm_tgt) / rpm_tgt * 100.0
            f = 0                      # measure 内部已确认无故障
            print("%6d | %9.2f | %+6.2f | %7.2f | %8.3f | %8.3f | %8.3f | %.1fC f=0x%s%s"
                  % (rpm_tgt, m["rpm"], err, m["cv"], m["iq_mean"], m["iq_rms"], m["iq_pp"],
                     m["temp"], "--" if f is None else "%02x" % f,
                     " [iq顶限幅]" if m["iq_abort"] else ""))
            rows.append(dict(rpm_tgt=rpm_tgt, rpm_meas=round(m["rpm"], 3),
                             err_pct=round(err, 3), cv=round(m["cv"], 3), nseg=m["nseg"],
                             iq_mean=round(m["iq_mean"], 4), iq_rms=round(m["iq_rms"], 4),
                             iq_pp=round(m["iq_pp"], 4), temp=round(m["temp"], 2), fault=f))
            with open(out, "w", newline="", encoding="utf-8-sig") as fp:
                w = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)

        # 恢复默认：谐波关、死区恢复默认 ON（du 保持 0.59 理论值）
        toggle_expect(rig, "%", False)
        toggle_expect(rig, "B", True)
        cmd(rig, "=0\r", 0.3)
        cmd(rig, "D", 0.8)
        print("\n[out] %s（%d 点）" % (out, len(rows)))
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
