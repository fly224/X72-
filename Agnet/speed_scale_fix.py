# -*- coding: utf-8 -*-
"""修正测量方法后重测转速标度。

前几次测量的缺陷：e0 在"给目标之前"取，而时长从第一个采样点算起，
中间约 0.4~0.5 s 的角度变化被计入分子却不计入分母 → 系统性高估，
且转速越高、高估越明显（与 60/100/150 准确、300/1000/1500 偏高 3% 的现象一致）。

本脚本改为：
  1) 先给目标并等 settle 秒真正稳定（期间保活）
  2) 连续采 (t, enc_sum)，用最小二乘拟合斜率得转速
  3) 同时读 w_elec 做对照

用法：python speed_scale_fix.py COM27
"""
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

ENC = 2097152.0
POLE_PAIRS = 14.0


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


def measure_fixed(rig, rpm_tgt, dur=18.0, settle=2.0):
    cmd(rig, "=%g\r" % rpm_tgt, 0.35)
    t0 = time.time()
    while time.time() - t0 < settle:          # 稳定期，同时保活
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.4)

    xs, ys, ws = [], [], []
    ts = time.time()
    while time.time() - ts < dur:
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.4)
        e = rig.rx.enc
        if e is not None:
            xs.append(time.time() - ts)
            ys.append(float(e))
        s = rig.rx.get_snap()[0]
        if s is not None and s.get("w_elec") is not None:
            ws.append(s["w_elec"])

    if len(xs) < 8:
        return None
    n = len(xs)
    mx = statistics.mean(xs)
    my = statistics.mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    k = sxy / sxx if sxx > 0 else 0.0
    rpm_pos = k / ENC * 60.0
    rpm_w = (statistics.mean(ws) / POLE_PAIRS * 60.0 / (2 * 3.141592653589793)) if ws else float("nan")

    # 拟合残差 → 位置通道的噪声水平（rpm）
    resid = [y - (my + k * (x - mx)) for x, y in zip(xs, ys)]
    sd = statistics.pstdev(resid) / ENC * 60.0
    return dict(pos=rpm_pos, w=rpm_w, n=n, span=xs[-1] - xs[0], sd=sd)


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

        print("=== 修正方法：稳定后斜率拟合（每点 settle 2 s + 18 s）===")
        print("%6s | %10s | %10s | %8s | %6s | %s"
              % ("目标", "位置(斜率)", "w_elec", "偏差", "采样数", "位置残差σ"))
        rows = []
        for rpm_tgt in (60.0, 100.0, 150.0, 300.0, 500.0, 1000.0, 1500.0, 2000.0):
            m = measure_fixed(rig, rpm_tgt)
            cmd(rig, "=0\r", 0.3)
            time.sleep(0.3)
            if m is None:
                print("  %5.0f rpm  数据不足" % rpm_tgt)
                continue
            dev = (m["pos"] - rpm_tgt) / rpm_tgt * 100.0
            print("%6.0f | %10.2f | %10.2f | %+7.2f%% | %6d | %.2f rpm"
                  % (rpm_tgt, m["pos"], m["w"], dev, m["n"], m["sd"]))
            rows.append((rpm_tgt, m["pos"], m["w"]))

        # 若位置与 w_elec 一致，给出配对差的统计
        if len(rows) >= 3:
            diffs = [(p - w) / w * 100.0 for _, p, w in rows if w == w]
            print("\n位置 vs w_elec 相对差：均值 %+.2f%%，范围 %+.2f ~ %+.2f%%"
                  % (statistics.mean(diffs), min(diffs), max(diffs)))

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
