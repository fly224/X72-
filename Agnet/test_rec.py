# -*- coding: utf-8 -*-
"""验证 1024 点 iq 录波的触发与解析（跑完整矩阵前先确认）。"""
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, MSG_PERMISSION  # noqa: E402
from harm_vs_deadband import cmd, record_iq, mag_at  # noqa: E402

rig = Rig("COM27")
try:
    time.sleep(1.3)
    rig.send(MSG_PERMISSION, bytes([0, 0]))
    time.sleep(0.25)
    cmd(rig, "D", 0.6)
    cmd(rig, "X", 0.7)
    cmd(rig, "Y", 0.4)

    for rpm in (100.0, 200.0):
        cmd(rig, "=%g\r" % rpm, 0.35)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            rig.s.write(("=%g\r" % rpm).encode())
            rig.s.flush()
            time.sleep(0.4)

        w, fs = record_iq(rig)
        n = len(w)
        f6 = 6.0 * rpm / 60.0 * 14.0
        print("\n%3.0f rpm: 点数 %d / 1024, fs %.0f Hz, 6次频率 %.1f Hz"
              % (rpm, n, fs, f6))
        if n > 100:
            print("  前 12 值: %s" % " ".join("%+.3f" % v for v in w[:12]))
            print("  均值 %+.4f  min %+.4f  max %+.4f  pp %.4f"
                  % (sum(w) / n, min(w), max(w), max(w) - min(w)))
            print("  6 次幅值 = %.5f A" % mag_at(w, fs, f6))
            print("  基波(1次)幅值 = %.5f A" % mag_at(w, fs, rpm / 60.0 * 14.0))
        else:
            print("  [!] 录波解析失败，检查 [REC] 行格式")
        cmd(rig, "=0\r", 0.3)
        time.sleep(0.4)

    cmd(rig, "=0\r", 0.3)
    cmd(rig, "D", 0.8)
finally:
    try:
        rig.s.write(b"=0\r")
        time.sleep(0.15)
        rig.s.write(b"D")
    except Exception:
        pass
    rig.close()
