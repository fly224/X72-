# -*- coding: utf-8 -*-
"""5/7 次谐波抑制能否抵消死区补偿引入的转速波动？

判据（两层）：
  1) 电气源头：1024 点 @20kHz 的 iq 录波，DFT 测 6 次脉动幅值
     （死区误差产生 5/7 次电流谐波 -> dq 域表现为 6 次转矩脉动）
  2) 机械结果：编码器位置差分分箱 CV

矩阵：{死区 OFF, 0.59%} × {谐波 OFF, ON} × {100, 200, 300 rpm}

用法：python harm_vs_deadband.py COM27
"""
import math
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

ENC = 2097152.0
POLE_PAIRS = 14.0
REC_DIV = 4          # 5 kHz 采样，窗口 204.8 ms


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
    """key='B' 死区 / '%' 谐波，均为切换键。返回实际状态。"""
    t = cmd(rig, key, 0.55)
    cur = ("ON" in t)
    if cur != want_on:
        t = cmd(rig, key, 0.55)
        cur = ("ON" in t)
    return cur


def record_iq(rig):
    """触发 1024 点 iq 录波并解析。返回 (序列 A, 采样率 Hz)。"""
    cmd(rig, "'%d\r" % REC_DIV, 0.35)
    rig.rx.pause_evt.set()
    time.sleep(0.1)
    rig.s.reset_input_buffer()
    rig.s.write(b"&")
    rig.s.flush()
    raw = b""
    t0 = time.time()
    while time.time() - t0 < 10.0:
        d = rig.s.read(8192)
        if d:
            raw += d
        if b"REC end" in raw:
            time.sleep(0.2)
            d = rig.s.read(8192)
            if d:
                raw += d
            break
    rig.rx.pause_evt.clear()
    rig.rx.buf = bytearray()

    txt = raw.decode("ascii", "replace")
    vals = []
    for line in txt.splitlines():
        # 行首可能被遥测帧/回显污染，用 find 定位标记而不是 startswith
        idx = line.find("[REC]")
        if idx < 0:
            continue
        h = line[idx + 5:]
        h = "".join(ch for ch in h if ch in "0123456789abcdefABCDEF")
        for i in range(0, len(h) - 3, 4):
            v = int(h[i:i + 4], 16)
            if v >= 32768:
                v -= 65536
            vals.append(v / 131.0)
    fs = 20000.0 / REC_DIV
    return vals, fs


def mag_at(sig, fs, f):
    """单频点 DFT 幅值（Hann 窗，幅值归一化为峰值）。"""
    n = len(sig)
    if n < 32:
        return float("nan")
    mean = sum(sig) / n
    re = im = 0.0
    wsum = 0.0
    for i, v in enumerate(sig):
        w = 0.5 - 0.5 * math.cos(2.0 * math.pi * i / (n - 1))
        a = 2.0 * math.pi * f * i / fs
        x = (v - mean) * w
        re += x * math.cos(a)
        im -= x * math.sin(a)
        wsum += w
    return 2.0 * math.hypot(re, im) / wsum if wsum > 0 else float("nan")


def cv_and_iq(rig, rpm_tgt, dur=10.0, box=2.0):
    e0 = rig.rx.enc
    if e0 is None:
        return None
    rows, iqs = [], []
    t0 = time.time()
    while time.time() - t0 < dur:
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.4)
        e = rig.rx.enc
        s = rig.rx.get_snap()[0]
        if e is not None:
            rows.append((time.time() - t0, e))
        if s is not None and s.get("iq_ref") is not None:
            iqs.append(s["iq_ref"])
    e1 = rig.rx.enc
    if len(rows) < 4 or e1 is None:
        return None
    dur_eff = rows[-1][0] - rows[0][0]
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
    cv = (statistics.pstdev(segs) / abs(mu) * 100.0) if (len(segs) >= 2 and abs(mu) > 1e-9) else float("nan")
    return dict(cv=cv, nseg=len(segs),
                iq_mean=statistics.mean(iqs) if iqs else float("nan"),
                iq_pp=(max(iqs) - min(iqs)) if len(iqs) >= 2 else float("nan"))


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    rig = Rig(port)
    rows_out = []
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.25)
        cmd(rig, "D", 0.6)
        cmd(rig, "X", 0.7)
        cmd(rig, "Y", 0.4)

        print("=== 死区补偿 × 5/7 次谐波抑制（判据：iq 的 6 次脉动 + 转速 CV）===")
        print("%-18s %5s | %8s | %7s | %8s | %8s | %s"
              % ("配置", "rpm", "6次幅值A", "CV%", "iq均A", "iq_ppA", "故障"))

        cfgs = [("死区OFF 谐波OFF", False, None),
                ("死区0.59 谐波OFF", True, 59),
                ("死区OFF 谐波ON", False, None),
                ("死区0.59 谐波ON", True, 59)]

        for name, dead_on, du in cfgs:
            # 设死区
            toggle_expect(rig, "B", dead_on)
            if dead_on and du is not None:
                cmd(rig, "\\%d\r" % du, 0.4)
            # 设谐波
            toggle_expect(rig, "%", ("谐波ON" in name))
            time.sleep(0.4)

            for rpm_tgt in (100.0, 200.0, 300.0):
                # 升速 + 稳定
                cmd(rig, "=%g\r" % rpm_tgt, 0.35)
                t0 = time.time()
                while time.time() - t0 < 2.0:
                    rig.s.write(("=%g\r" % rpm_tgt).encode())
                    rig.s.flush()
                    time.sleep(0.4)

                m = cv_and_iq(rig, rpm_tgt, 10.0)
                iq_wave, fs = record_iq(rig)
                f_e = rpm_tgt / 60.0 * POLE_PAIRS
                a6 = mag_at(iq_wave, fs, 6.0 * f_e) if len(iq_wave) > 900 else float("nan")
                f = fault(rig)
                if f not in (0, None):
                    cmd(rig, "X", 0.5)
                if m is None:
                    print("%-18s %5.0f | 数据不足" % (name, rpm_tgt))
                    continue
                print("%-18s %5.0f | %8.4f | %7.2f | %8.3f | %8.3f | f=0x%s n=%d"
                      % (name, rpm_tgt, a6, m["cv"], m["iq_mean"], m["iq_pp"],
                         "--" if f is None else "%02x" % f, len(iq_wave)))
                rows_out.append(dict(cfg=name, rpm=rpm_tgt, a6=round(a6, 4),
                                     cv=round(m["cv"], 3), iq_mean=round(m["iq_mean"], 4),
                                     iq_pp=round(m["iq_pp"], 4), fault=f))
                cmd(rig, "=0\r", 0.3)
                time.sleep(0.3)

            # 每配置结束复位：死区关、谐波关
            toggle_expect(rig, "%", False)
            toggle_expect(rig, "B", False)

        # 结论辅助：按转速比较"死区引入多少 6 次"与"谐波抑制压掉多少"
        print("\n=== 6 次脉动对比 ===")
        for rpm_tgt in (100.0, 200.0, 300.0):
            d = {r["cfg"]: r for r in rows_out if r["rpm"] == rpm_tgt}
            def g(k):
                return d.get(k, {}).get("a6", float("nan"))
            base = g("死区OFF 谐波OFF")
            dead = g("死区0.59 谐波OFF")
            harm = g("死区OFF 谐波ON")
            both = g("死区0.59 谐波ON")
            print("  %3.0f rpm: 基线 %.4f | 死区 %.4f (%+.1f%%) | 谐波 %.4f (%+.1f%%) | 两者 %.4f (%+.1f%%)"
                  % (rpm_tgt, base,
                     dead, (dead / base - 1) * 100 if base else float("nan"),
                     harm, (harm / base - 1) * 100 if base else float("nan"),
                     both, (both / base - 1) * 100 if base else float("nan")))

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
