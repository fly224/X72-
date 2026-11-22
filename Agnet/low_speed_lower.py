# -*- coding: utf-8 -*-
"""继续下探最低转速：0.05 / 0.02 / 0.01 rpm。

判据仍是编码器位置差分。卡住判据按目标比例算（不固定阈值），
因为 0.01 rpm 在 40 s 内只转约 1400 count。
"""
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

ENC = 2097152.0


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


def measure(rig, rpm_tgt, dur):
    e0 = rig.rx.enc
    if e0 is None:
        return None
    cmd(rig, "=%g\r" % rpm_tgt, 0.35)
    rows, iqs = [], []
    t0 = time.time()
    while time.time() - t0 < dur:
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.5)
        e = rig.rx.enc
        s = rig.rx.get_snap()[0]
        if e is not None:
            rows.append((time.time() - t0, e))
        if s is not None and s.get("iq_ref") is not None:
            iqs.append(s["iq_ref"])
    e1 = rig.rx.enc
    if len(rows) < 3 or e1 is None:
        return None
    dur_eff = rows[-1][0] - rows[0][0]
    d = e1 - e0
    mean = (d / ENC) / dur_eff * 60.0
    expect = abs(rpm_tgt) / 60.0 * dur_eff * ENC

    # 10 s 分箱
    bins = {}
    for t, e in rows:
        bins.setdefault(int(t // 10), []).append((t, e))
    seg = []
    for k in sorted(bins):
        v = bins[k]
        if len(v) < 2:
            continue
        dt = v[-1][0] - v[0][0]
        dc = v[-1][1] - v[0][1]
        if dt > 1.0:
            seg.append((dc / ENC) / dt * 60.0)
    mu = statistics.mean(seg) if seg else 0.0
    cv = (statistics.pstdev(seg) / abs(mu) * 100.0) if (len(seg) >= 2 and abs(mu) > 1e-9) else 0.0
    return dict(mean=mean, d=d, dur=dur_eff, expect=expect, seg=seg, cv=cv,
                iq=(statistics.mean(iqs) if iqs else None),
                iqmax=(max(iqs, key=abs) if iqs else None),
                stuck=(abs(d) < 0.2 * expect))


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    rig = Rig(port)
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.25)
        cmd(rig, "D", 0.6)
        cmd(rig, "X", 0.7)
        cmd(rig, "X", 0.5)

        targets = ([float(x) for x in sys.argv[2:]]
                   if len(sys.argv) > 2 else [0.05, 0.02, 0.01])
        print("=== 更低速探测（目标 <0.01 时窗口 60 s，否则 40 s；10 s 分箱）===")
        print("  目标    平均          误差     CV     分段(10s)                         iq均/峰      判定")
        lowest_ok = None
        for tgt in targets:
            dur = 60.0 if tgt < 0.01 else 40.0
            m = measure(rig, tgt, dur)
            if m is None:
                print("  %.2f rpm  无数据" % tgt)
                cmd(rig, "=0\r", 0.3)
                continue
            err = (m["mean"] - tgt) / tgt * 100.0
            ok = (not m["stuck"]) and abs(err) < 15.0
            print("  %.4g rpm  %+9.5f rpm  %+6.1f%%  %5.1f%%  %-32s %.2f/%.2f A  %s"
                  % (tgt, m["mean"], err, m["cv"],
                     " ".join("%+.4f" % v for v in m["seg"]),
                     m["iq"] or 0, m["iqmax"] or 0,
                     "转动" if ok else "卡住/失准"))
            if ok and (lowest_ok is None or tgt < lowest_ok):
                lowest_ok = tgt
            cmd(rig, "=0\r", 0.35)
            time.sleep(0.4)

        print("\n本轮可达最低：%s rpm" % ("%.4g" % lowest_ok if lowest_ok else "均不合格"))
        cmd(rig, "=0\r", 0.3)
        cmd(rig, "D", 0.8)
        t = cmd(rig, "S", 0.7)
        for l in t.splitlines():
            if "st en=" in l:
                print("收尾: %s" % l.strip()[:140])
                break
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
