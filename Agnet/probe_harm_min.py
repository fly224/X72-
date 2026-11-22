# -*- coding: utf-8 -*-
"""探测 5/7 次谐波抑制的最低可用转速：先建环，再开抑制，监测是否失稳。"""
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, MSG_PERMISSION  # noqa: E402
from combo_sweep import cmd, fault, toggle_expect  # noqa: E402


def probe(rig, rpm_tgt, watch=5.0):
    toggle_expect(rig, "%", False)          # 先关，保证干净建环
    cmd(rig, "=%g\r" % rpm_tgt, 0.35)
    t0 = time.time()
    while time.time() - t0 < 2.5:
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.4)

    f0 = fault(rig)
    if f0 not in (0, None):
        cmd(rig, "X", 0.5)
        return "建环前有故障 f=0x%02x" % f0, None

    toggle_expect(rig, "%", True)          # 闭环内开抑制
    peak = 0.0
    bad = False
    t0 = time.time()
    while time.time() - t0 < watch:
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.3)
        s = rig.rx.get_snap()[0]
        if s is not None and s.get("iq_ref") is not None:
            v = abs(s["iq_ref"])
            peak = max(peak, v)
            if v > 2.6:
                bad = True
                break
    # 故障只在窗口末尾读一次：fault() 会暂停接收线程，循环内调用会让 iq 读不到
    f = fault(rig)
    toggle_expect(rig, "%", False)
    cmd(rig, "=0\r", 0.3)
    time.sleep(0.3)
    if f not in (0, None):
        cmd(rig, "X", 0.6)
        return "失稳：f=0x%02x peak=%.2fA" % (f, peak), peak
    if bad:
        return "失稳：iq 顶限幅 peak=%.2fA" % peak, peak
    return "OK peak=%.2fA" % peak, peak


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
        toggle_expect(rig, "%", False)
        toggle_expect(rig, "B", True)
        cmd(rig, "\\59\r", 0.4)

        print("=== 谐波抑制最低可用转速探测（闭环内开抑制，观察 5 s）===")
        ok_min = None
        for rpm_tgt in (1.0, 3.0, 10.0, 30.0, 60.0, 100.0, 150.0, 200.0, 300.0):
            res, pk = probe(rig, rpm_tgt, 5.0)
            print("  %5.0f rpm: %s" % (rpm_tgt, res))
            if res.startswith("OK") and ok_min is None:
                ok_min = rpm_tgt
            time.sleep(0.4)

        print("\n最低可用转速：%s rpm" % ("%.0f" % ok_min if ok_min else "无一可用"))
        toggle_expect(rig, "%", False)
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
