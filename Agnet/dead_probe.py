# -*- coding: utf-8 -*-
"""dead_probe.py —— 采三相电流原始波形，看死区造成的过零畸变

为什么用 30025 的 iu/iv/iw 而不是 `&` 录制：
  `&` 录的是 iq（dq 轴），1024 点 @20 kHz = 51 ms。
  而我们要看的是"相电流过零附近的平台"，必须看 abc 原始波形。
  30025 每帧带 iu/iv/iw 三个瞬时值，帧率实测约 115 Hz。
  低速时电频率极低（3 rpm × 14 极对 / 60 = 0.7 Hz），
  每周期 160 点，足够看清过零段的形状。

判据：
  · 正常正弦：过零处斜率最大、连续
  · 死区畸变：过零附近出现"平台"（电流钉在 0 附近不动），
    或波形在两相切换处塌陷；症状是 THD 上升、5/7 次增大
  · 死区 + 小电流：可能出现断续（电流断流再恢复）

用法:
    python dead_probe.py COM27 --rpm 3 --secs 8
    python dead_probe.py COM27 --rpm 10 --secs 4 --csv wave.csv
"""
import argparse
import math
import struct
import time

from cog_ff_test import LEN_IN, MSG_IN, Rig, x25
from sweep_1_1000 import set_rpm

POLE = 14.0
TWO_PI = 2.0 * math.pi


def collect_abc(rig, secs):
    """采 iu/iv/iw + 电角度 + 机械角 + iq 给定"""
    out = []
    t0 = time.time()
    nxt = t0 + 0.8
    while time.time() - t0 < secs:
        d = rig.s.read(4096)
        if d:
            rig.raw += d
        if time.time() > nxt:
            rig.s.write(b"S")
            rig.s.flush()
            nxt += 0.8
        i = 0
        n = len(rig.raw)
        while i + 12 <= n:
            if rig.raw[i] != 0xFD:
                i += 1
                continue
            ln = rig.raw[i + 1]
            mid = rig.raw[i + 7] | (rig.raw[i + 8] << 8) | (rig.raw[i + 9] << 16)
            if ln != LEN_IN or mid != MSG_IN:
                i += 1
                continue
            b = i + 10
            if b + ln + 2 > n:
                break
            crc = rig.raw[b + ln] | (rig.raw[b + ln + 1] << 8)
            if x25(rig.raw[i + 1:b + ln], 191) != crc:
                i += 1
                continue
            f = lambda o: struct.unpack_from("<f", rig.raw, b + o)[0]
            out.append(dict(t=time.time(), iu=f(8), iv=f(12), iw=f(16),
                            the=f(20), iq_ref=f(4), vbus=f(24), we=f(44)))
            i = b + ln + 2
        del rig.raw[:i]
    return out


def park(fr):
    """把 abc 变到 dq（用遥测的 elec_theta，不含 π/2 补偿）"""
    for r in fr:
        a, b = r["iu"], (r["iu"] + 2.0 * r["iv"]) / math.sqrt(3.0)
        th = r["the"] + math.pi / 2.0
        c, s = math.cos(th), math.sin(th)
        r["id"] = c * a + s * b
        r["iq"] = -s * a + c * b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--rpm", type=float, default=3.0)
    ap.add_argument("--secs", type=float, default=8.0)
    ap.add_argument("--csv", default="")
    a = ap.parse_args()

    rig = Rig(a.port)
    try:
        print("S:", rig.cmd("S", 0.6))
        print(set_rpm(rig, a.rpm))
        rig.keep(3.0)

        fr = collect_abc(rig, a.secs)
        rig.cmd("D", 0.3)
        if len(fr) < 40:
            print("帧太少", len(fr))
            return 1

        ts = [r["t"] for r in fr]
        fs = (len(ts) - 1) / (ts[-1] - ts[0])
        f_elec = a.rpm * POLE / 60.0
        print("n=%d  fs=%.1f Hz  电频率=%.2f Hz  每周期 %.1f 点"
              % (len(fr), fs, f_elec, fs / f_elec if f_elec else 0))

        mean = lambda k: sum(r[k] for r in fr) / len(fr)
        print("均值: iu=%+.4f iv=%+.4f iw=%+.4f iq_ref=%+.4f vbus=%.2f"
              % (mean("iu"), mean("iv"), mean("iw"), mean("iq_ref"), mean("vbus")))

        amp = max(max(abs(r["iu"]) for r in fr), max(abs(r["iv"]) for r in fr))
        print("相电流峰值 %.4f A" % amp)

        park(fr)
        print("dq: id=%.4f iq=%.4f（Park 复算，应与 iq_ref 接近）"
              % (mean("id"), mean("iq")))

        # 过零段分析：统计 |iu| < 阈值的停留时间占比
        thr = max(0.02, amp * 0.15)
        near0 = [r for r in fr if abs(r["iu"]) < thr]
        print("A 相 |iu|<%.3f A 的点占比 %.1f%%（正弦理论值 %.1f%%）"
              % (thr, len(near0) * 100.0 / len(fr),
                 math.degrees(2.0 * math.asin(min(1.0, thr / amp))) / 180.0 * 100.0
                 if amp > 0 else 0.0))

        # 打印一个电周期的 A 相波形（等间隔取 40 点）
        n_per = max(1, int(round(fs / f_elec))) if f_elec else len(fr)
        seg = fr[:min(len(fr), n_per)]
        step = max(1, len(seg) // 40)
        print("\nA 相一个电周期（%d 点，每 %d 点取 1）：" % (len(seg), step))
        print("  " + " ".join("%+.3f" % seg[k]["iu"] for k in range(0, len(seg), step)))

        if a.csv:
            with open(a.csv, "w", encoding="utf-8-sig", newline="") as f:
                f.write("t,iu,iv,iw,id,iq,elec_theta,iq_ref,vbus\n")
                for r in fr:
                    f.write("%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.5f,%.2f\n"
                            % (r["t"] - fr[0]["t"], r["iu"], r["iv"], r["iw"],
                               r["id"], r["iq"], r["the"], r["iq_ref"], r["vbus"]))
            print("\nCSV -> %s" % a.csv)
        return 0
    finally:
        rig.cmd("D", 0.3)
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
