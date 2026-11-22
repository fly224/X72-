# -*- coding: utf-8 -*-
"""核实控制节拍与转速标度：
  1) 遥测帧率（设计 100 Hz）→ 反推实际控制节拍
  2) 同一窗口内 w_elec 与位置差分的同时对比 → 标度比
"""
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

ENC = 2097152.0


def cmd(rig, c, wait=0.6):
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


def count_frames(raw, want):
    n = 0
    i = 0
    L = len(raw)
    while i + 12 <= L:
        if raw[i] != 0xFD:
            i += 1
            continue
        ln = raw[i + 1]
        tot = 12 + ln
        if i + tot > L:
            break
        mid = raw[i + 7] | (raw[i + 8] << 8) | (raw[i + 9] << 16)
        if mid in want:
            n += 1
            i += tot
        else:
            i += 1
    return n


rig = Rig("COM27")
try:
    time.sleep(1.3)
    rig.send(MSG_PERMISSION, bytes([0, 0]))
    time.sleep(0.25)
    cmd(rig, "D", 0.6)
    cmd(rig, "X", 0.7)

    # ---- 1) 帧率 ----
    rig.rx.pause_evt.set()
    time.sleep(0.1)
    rig.s.reset_input_buffer()
    raw = b""
    t0 = time.time()
    while time.time() - t0 < 10.0:
        d = rig.s.read(8192)
        if d:
            raw += d
    el = time.time() - t0
    rig.rx.pause_evt.clear()
    rig.rx.buf = bytearray()
    n = count_frames(raw, (30025,))
    f25 = n / el
    print("=== 帧率（外部时钟判据）===")
    print("  30025: %d 帧 / %.3f s = %.2f Hz（设计 100 Hz）" % (n, el, f25))
    print("  推算节拍: 50us × (100/%.2f) = %.2f us → %.2f kHz"
          % (f25, 50.0 * 100.0 / f25, 1.0e6 / (50.0 * 100.0 / f25)))
    print("  位置/速度标度比预期: %.4f" % (100.0 / f25))

    # ---- 2) 同窗口 w_elec vs 位置差分 ----
    print("\n=== 1000 rpm 下双通道对比（30 s）===")
    for rpm_tgt in (300.0, 1000.0, 1500.0):
        e0 = rig.rx.enc
        cmd(rig, "=%g\r" % rpm_tgt, 0.35)
        ws, rows = [], []
        t0 = time.time()
        while time.time() - t0 < 18.0:
            rig.s.write(("=%g\r" % rpm_tgt).encode())
            rig.s.flush()
            time.sleep(0.35)
            e = rig.rx.enc
            s = rig.rx.get_snap()[0]
            if e is not None:
                rows.append((time.time() - t0, e))
            if s is not None and s.get("w_elec") is not None:
                ws.append(s["w_elec"] / 14.0 * 60.0 / (2 * 3.141592653589793))
        e1 = rig.rx.enc
        if len(rows) < 4 or e1 is None:
            print("  %5.0f rpm 无数据" % rpm_tgt)
            continue
        dur = rows[-1][0] - rows[0][0]
        rpm_pos = (e1 - e0) / ENC / dur * 60.0
        rpm_w = statistics.mean(ws) if ws else float("nan")
        print("  %5.0f rpm | 位置差分 %8.2f | w_elec %8.2f | 比值 %.4f | w/pos %.2f%%"
              % (rpm_tgt, rpm_pos, rpm_w, rpm_pos / rpm_w if rpm_w else 0,
                 (rpm_w / rpm_pos - 1.0) * 100.0 if rpm_pos else 0))
        cmd(rig, "=0\r", 0.3)
        time.sleep(0.3)

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
