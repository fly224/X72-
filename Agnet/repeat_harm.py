# -*- coding: utf-8 -*-
"""复测：谐波抑制在 100/200 rpm 的重复性（每次连录 3 段 iq）。"""
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, MSG_PERMISSION  # noqa: E402
from harm_vs_deadband import cmd, record_iq, mag_at, toggle_expect, fault  # noqa: E402


def run_point(rig, rpm_tgt, n=3):
    cmd(rig, "=%g\r" % rpm_tgt, 0.35)
    t0 = time.time()
    while time.time() - t0 < 2.5:
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.4)

    f6 = 6.0 * rpm_tgt / 60.0 * 14.0
    a6s, means, pps = [], [], []
    for _ in range(n):
        w, fs = record_iq(rig)
        if len(w) < 800:
            a6s.append(float("nan"))
            means.append(float("nan"))
            pps.append(float("nan"))
        else:
            a6s.append(mag_at(w, fs, f6))
            means.append(statistics.mean(w))
            pps.append(max(w) - min(w))
        time.sleep(0.5)
    return a6s, means, pps


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    rig = Rig(port)
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.25)
        cmd(rig, "D", 0.6)
        cmd(rig, "X", 0.7)
        cmd(rig, "Y", 0.4)

        cases = [
            ("100 死区OFF 谐波OFF", 100.0, False, False),
            ("100 死区OFF 谐波ON ", 100.0, False, True),
            ("100 死区0.59 谐波ON", 100.0, True, True),
            ("200 死区OFF 谐波OFF", 200.0, False, False),
            ("200 死区0.59 谐波ON", 200.0, True, True),
            ("300 死区OFF 谐波ON ", 300.0, False, True),
        ]

        print("=== 重复性复测（每点连录 3 段 iq，段间隔 0.5 s）===")
        print("%-20s | %-28s | %-24s | %s" % ("工况", "6次幅值 A (×3)", "iq 均值 A (×3)", "iq_pp A (×3)"))
        for name, rpm_tgt, dead_on, harm_on in cases:
            toggle_expect(rig, "B", dead_on)
            if dead_on:
                cmd(rig, "\\59\r", 0.4)
            toggle_expect(rig, "%", harm_on)
            time.sleep(0.5)

            a6s, means, pps = run_point(rig, rpm_tgt, 3)
            f = fault(rig)
            ok = [v for v in a6s if v == v]
            spread = ("%.1f%%" % ((max(ok) - min(ok)) / statistics.mean(ok) * 100)) if len(ok) >= 2 and statistics.mean(ok) else "-"
            print("%-20s | %-28s | %-24s | %-24s f=0x%s 极差 %s"
                  % (name,
                     " ".join("%.4f" % v for v in a6s),
                     " ".join("%+.3f" % v for v in means),
                     " ".join("%.2f" % v for v in pps),
                     "--" if f is None else "%02x" % f, spread))
            cmd(rig, "=0\r", 0.3)
            time.sleep(0.4)

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
