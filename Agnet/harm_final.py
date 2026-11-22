# -*- coding: utf-8 -*-
"""最终判定：5/7 次谐波抑制能否抵消死区补偿引入的转速波动。

方法（针对单次测量重复性差的问题）：
  每工况重复 N=4 次，每次 10 s 转速采样 + 1 段 iq 录波，
  输出 CV 与 6 次幅值的 均值 ± σ，用 σ 判断差异是否显著。

矩阵：{基线, 死区0.59, 死区0.59+谐波ON} × {100, 200, 300 rpm}

用法：python harm_final.py COM27 [N]
"""
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, MSG_PERMISSION  # noqa: E402
from harm_vs_deadband import cmd, record_iq, mag_at, toggle_expect, fault  # noqa: E402

ENC = 2097152.0


def cv_once(rig, rpm_tgt, dur=10.0, box=2.0):
    e0 = rig.rx.enc
    if e0 is None:
        return None
    rows = []
    t0 = time.time()
    while time.time() - t0 < dur:
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.4)
        e = rig.rx.enc
        if e is not None:
            rows.append((time.time() - t0, e))
    if len(rows) < 4:
        return None
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
    mu = statistics.mean(segs) if segs else 0.0
    if len(segs) < 2 or abs(mu) < 1e-9:
        return None
    return statistics.pstdev(segs) / abs(mu) * 100.0


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    n_rep = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    rig = Rig(port)
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.25)
        cmd(rig, "D", 0.6)
        cmd(rig, "X", 0.7)
        cmd(rig, "Y", 0.4)

        cases = [("基线", False, False),
                 ("死区0.59", True, False),
                 ("死区+谐波", True, True)]

        print("=== 最终判定（每点重复 %d 次）===" % n_rep)
        print("%-12s %5s | %-22s | %-24s | %s"
              % ("配置", "rpm", "转速CV % (mean±σ)", "iq 6次幅值 A (mean±σ)", "故障"))

        summary = {}
        for name, dead_on, harm_on in cases:
            toggle_expect(rig, "B", dead_on)
            if dead_on:
                cmd(rig, "\\59\r", 0.4)
            toggle_expect(rig, "%", harm_on)
            time.sleep(0.5)

            for rpm_tgt in (100.0, 200.0, 300.0):
                cmd(rig, "=%g\r" % rpm_tgt, 0.35)
                t0 = time.time()
                while time.time() - t0 < 2.5:
                    rig.s.write(("=%g\r" % rpm_tgt).encode())
                    rig.s.flush()
                    time.sleep(0.4)

                cvs, a6s = [], []
                for _ in range(n_rep):
                    c = cv_once(rig, rpm_tgt, 10.0)
                    if c is not None:
                        cvs.append(c)
                    w, fs = record_iq(rig)
                    if len(w) >= 1000:
                        a6s.append(mag_at(w, fs, 6.0 * rpm_tgt / 60.0 * 14.0))

                f = fault(rig)
                if f not in (0, None):
                    cmd(rig, "X", 0.5)

                def ms(v):
                    if not v:
                        return "      -      "
                    if len(v) == 1:
                        return "%8.3f (n=1) " % v[0]
                    return "%6.3f ± %-6.3f" % (statistics.mean(v), statistics.pstdev(v))

                print("%-12s %5.0f | %-22s | %-24s | f=0x%s"
                      % (name, rpm_tgt, ms(cvs), ms(a6s),
                         "--" if f is None else "%02x" % f))
                summary[(name, rpm_tgt)] = (cvs, a6s)
                cmd(rig, "=0\r", 0.3)
                time.sleep(0.4)

        # 显著性判断：|均值差| 是否超过合并 σ
        print("\n=== 相对基线的差异（超过合并 σ 记为显著）===")
        for rpm_tgt in (100.0, 200.0, 300.0):
            b_cv, b_a6 = summary.get(("基线", rpm_tgt), ([], []))
            if not b_cv or not b_a6:
                continue
            for name in ("死区0.59", "死区+谐波"):
                c_cv, c_a6 = summary.get((name, rpm_tgt), ([], []))
                if not c_cv or not c_a6:
                    continue
                for label, b, c in (("CV", b_cv, c_cv), ("a6", b_a6, c_a6)):
                    mb, mc = statistics.mean(b), statistics.mean(c)
                    sb = statistics.pstdev(b) if len(b) > 1 else 0.0
                    sc = statistics.pstdev(c) if len(c) > 1 else 0.0
                    pooled = ((sb * sb + sc * sc) / 2.0) ** 0.5 if (sb or sc) else 0.0
                    delta = (mc / mb - 1.0) * 100.0 if mb else float("nan")
                    sig = "显著" if (pooled > 0 and abs(mc - mb) > pooled) else "不显著"
                    print("  %3.0f rpm %-10s %-2s: 基线 %7.4f → %7.4f  (%+6.1f%%)  %s"
                          % (rpm_tgt, name, label, mb, mc, delta, sig))

        toggle_expect(rig, "%", False)
        toggle_expect(rig, "B", False)
        cmd(rig, "=0\r", 0.3)
        cmd(rig, "D", 0.8)
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
