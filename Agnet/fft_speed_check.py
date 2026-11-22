# -*- coding: utf-8 -*-
"""用电流基频做独立判据，裁决"位置差分 vs w_elec"的标度之争。

物理关系：电频率 f_e = 机械转速(rpm) / 60 × 极对数(14)
        => rpm = f_e × 60 / 14

采样：30025 (iu/iv/iw/w_elec) + 30040 (enc_sum)，100 Hz。
为避免 PWM 混叠污染，选 60/100/150 rpm（f_e = 14/23.3/35 Hz，均在奈奎斯特内）。

频率估计两条独立路径：
  1) 过零插值（对非均匀采样鲁棒，长窗精度高）
  2) 手写 DFT 峰值扫描（可交叉验证峰位）

用法：python fft_speed_check.py COM27
"""
import math
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

ENC = 2097152.0
POLE_PAIRS = 14.0
MSG_CUR_IN = 30025
MSG_POS = 30040


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


def acquire(rig, rpm_tgt, dur):
    """独占串口采 dur 秒，返回 (ialpha 序列, w_elec 序列, enc 起止, 时长)。"""
    rig.rx.pause_evt.set()
    time.sleep(0.1)
    rig.s.reset_input_buffer()
    raw = b""
    iq_ref = []
    t_start = time.time()
    next_ka = 0.0
    while time.time() - t_start < dur:
        now = time.time() - t_start
        if now >= next_ka:
            rig.s.write(("=%g\r" % rpm_tgt).encode())
            rig.s.flush()
            next_ka += 0.4
        d = rig.s.read(4096)
        if d:
            raw += d
    t_end = time.time()
    rig.rx.pause_evt.clear()
    rig.rx.buf = bytearray()

    ialpha, ualpha, ws = [], [], []
    enc0 = enc1 = None
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
        if mid == MSG_CUR_IN and ln >= 61:
            b = i + 10
            iu = _f32(raw, b + 8)
            iv = _f32(raw, b + 12)
            iw = _f32(raw, b + 16)
            uu = _f32(raw, b + 28)
            uv = _f32(raw, b + 32)
            uw = _f32(raw, b + 36)
            we = _f32(raw, b + 44)
            if _finite(iu) and _finite(iv) and _finite(iw):
                ialpha.append((2.0 * iu - iv - iw) / 3.0)
                ws.append(we if _finite(we) else float("nan"))
            if _finite(uu) and _finite(uv) and _finite(uw):
                ualpha.append((2.0 * uu - uv - uw) / 3.0)
        elif mid == MSG_POS and ln >= 12:
            v = _i64(raw, i + 10)
            if enc0 is None:
                enc0 = v
            enc1 = v
        i += tot
    return ialpha, ualpha, ws, enc0, enc1, (t_end - t_start)


def _f32(buf, off):
    import struct
    return struct.unpack_from("<f", buf, off)[0]


def _i64(buf, off):
    import struct
    return struct.unpack_from("<q", buf, off)[0]


def _finite(v):
    return v == v and abs(v) < 1e9


def zero_cross_freq(sig, dt):
    n = len(sig)
    if n < 50:
        return None, 0
    mean = sum(sig) / n
    s = [v - mean for v in sig]
    cr = []
    for i in range(1, n):
        if s[i - 1] <= 0.0 < s[i]:
            den = s[i] - s[i - 1]
            cr.append((i - 1 + (-s[i - 1] / den if den != 0 else 0.0)) * dt)
    if len(cr) < 3:
        return None, len(cr)
    T = cr[-1] - cr[0]
    if T <= 0:
        return None, len(cr)
    return (len(cr) - 1) / T, len(cr)


def dft_peak(sig, dt, fmin, fmax, step):
    n = len(sig)
    mean = sum(sig) / n
    s = [v - mean for v in sig]
    # Hann 窗
    w = [0.5 - 0.5 * math.cos(2.0 * math.pi * i / (n - 1)) for i in range(n)]
    sw = [a * b for a, b in zip(s, w)]
    best_f, best_m = fmin, -1.0
    f = fmin
    while f <= fmax:
        om = 2.0 * math.pi * f * dt
        re = im = 0.0
        for i, v in enumerate(sw):
            ang = om * i
            re += v * math.cos(ang)
            im -= v * math.sin(ang)
        m = re * re + im * im
        if m > best_m:
            best_m, best_f = m, f
        f += step
    return best_f


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    rig = Rig(port)
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.25)
        cmd(rig, "D", 0.6)
        cmd(rig, "X", 0.7)
        cmd(rig, "Y", 0.4)          # 速度环档位复位，保证可比
        print("起始: %s" % cmd(rig, "S", 0.6).split("[CMD]")[-1].strip()[:90])

        print("\n=== 电流基频判据（60/100/150 rpm，各 15 s）===")
        print("%6s | %9s | %9s | %9s | %8s | %6s | %s"
              % ("目标", "位置差分", "w_elec", "基频推算", "f_peak", "过零数", "一致性"))

        for rpm_tgt in (60.0, 100.0, 150.0):
            ia, ua, ws, e0, e1, dur = acquire(rig, rpm_tgt, 15.0)
            cmd(rig, "=0\r", 0.3)
            time.sleep(0.3)
            n = len(ua)
            if n < 200 or e0 is None or e1 is None:
                print("  %5.0f rpm  数据不足（%d 点）" % (rpm_tgt, n))
                continue
            dt = dur / (n - 1)

            ws_ok = [w for w in ws if _finite(w)]
            rpm_w = (statistics.mean(ws_ok) / POLE_PAIRS * 60.0 / (2 * math.pi)) if ws_ok else float("nan")
            rpm_pos = (e1 - e0) / ENC / dur * 60.0

            f_exp = rpm_tgt / 60.0 * POLE_PAIRS
            f_zc, ncr = zero_cross_freq(ua, dt)
            # 只在预期 ±25% 内找峰，避免锁到与转速无关的固定干扰
            f_coarse = dft_peak(ua, dt, max(4.0, f_exp * 0.75), min(48.0, f_exp * 1.25), 0.1)
            f_fine = dft_peak(ua, dt, max(4.0, f_coarse - 0.5), min(48.0, f_coarse + 0.5), 0.01)
            f_use = f_fine if f_fine == f_fine else f_zc
            rpm_f = (f_use * 60.0 / POLE_PAIRS) if f_use else float("nan")

            d_pos = abs(rpm_pos - rpm_f) / rpm_f * 100.0 if rpm_f else 9e9
            d_w = abs(rpm_w - rpm_f) / rpm_f * 100.0 if rpm_f else 9e9
            verdict = ("位置对" if d_pos < d_w else "w_elec 对")
            print("%6.0f | %9.2f | %9.2f | %9.2f | %8.3f | %6d | %s（位置差 %.1f%%，w 差 %.1f%%）"
                  % (rpm_tgt, rpm_pos, rpm_w, rpm_f, f_use, ncr, verdict, d_pos, d_w))

            # 打印首段波形，确认是否干净正弦（判断过零是否可信）
            if rpm_tgt == 100.0:
                print("         uα 前 40 点: %s" % " ".join("%+.3f" % v for v in ua[:40]))
                print("         iα 前 40 点: %s" % " ".join("%+.3f" % v for v in ia[:40]))

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
