# -*- coding: utf-8 -*-
"""确认组合配置（死区0.59+谐波ON）在 60 rpm 与 2000 rpm 的边界行为是否可重复。"""
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, MSG_PERMISSION  # noqa: E402
from combo_sweep import cmd, measure, toggle_expect, fault  # noqa: E402


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
        toggle_expect(rig, "B", True)
        cmd(rig, "\\59\r", 0.4)

        print("=== 边界确认（每点 2 次，12 s 窗口）===")
        print("%6s | %9s | %7s | %7s | %8s | %s"
              % ("目标", "实测rpm", "误差%", "CV%", "iq均A", "iq_pp / 备注"))
        for rpm_tgt in (60.0, 80.0, 100.0, 1500.0, 2000.0):
            for k in range(2):
                toggle_expect(rig, "%", False)
                cmd(rig, "D", 0.4)
                cmd(rig, "X", 0.4)
                m = measure(rig, rpm_tgt, 2.0, 12.0)
                cmd(rig, "=0\r", 0.3)
                time.sleep(0.3)
                if m is None:
                    print("%6.0f | 数据不足" % rpm_tgt)
                    continue
                if m.get("unstable"):
                    print("%6.0f | *** 失稳 f=0x%02x ***" % (rpm_tgt, m["fault"]))
                    cmd(rig, "X", 0.5)
                    continue
                err = (m["rpm"] - rpm_tgt) / rpm_tgt * 100.0
                note = "iq顶限幅" if m["iq_abort"] else ""
                print("%6.0f | %9.2f | %+6.2f | %7.2f | %8.3f | %.3f %s"
                      % (rpm_tgt, m["rpm"], err, m["cv"], m["iq_mean"], m["iq_pp"], note))
            print("  ---")

        toggle_expect(rig, "%", False)
        toggle_expect(rig, "B", True)
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
