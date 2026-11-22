# -*- coding: utf-8 -*-
"""速度给定斜坡（加减速限制）的实机对比测试

对比不同 accel 下同一转速的响应：超调、上升时间、iq 峰值。
accel=20000 等效关闭斜坡（固件把 >=20000 当作不限制），作为基线。

前提：COM27 没被上位机页面占用（Web Serial 会独占句柄）。
      若报 PermissionError(13)，先在上位机点「断开串口」。

用法：
    python ramp_test.py                      # 默认 500 rpm，扫 4 档 accel
    python ramp_test.py --rpm 1500
    python ramp_test.py --rpm 500 --accs 20000,400,200
"""
from __future__ import print_function

import argparse
import math
import re
import struct
import sys
import time

import serial

TWO_PI = 2.0 * math.pi
POLE = 14.0
MSG_IN = 30025
LEN_IN = 61          # 2026-11-16: 57 -> 61（新增 +57 spd_ref_rpm）
CRC_EXTRA = 191


def x25(data, extra):
    crc = 0xFFFF

    def acc(c, b):
        t = (b ^ (c & 0xFF)) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        return ((c >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF

    for b in data:
        crc = acc(crc, b)
    return acc(crc, extra)


class Rig(object):
    def __init__(self, port, baud=921600):
        self.s = serial.Serial(port, baud, timeout=0.05)
        self.buf = bytearray()

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass

    def cmd(self, c, wait=0.35):
        self.s.reset_input_buffer()
        self.s.write(c.encode() if isinstance(c, str) else c)
        self.s.flush()
        t0 = time.time()
        b = bytearray()
        while time.time() - t0 < wait:
            d = self.s.read(256)
            if d:
                b += d
        i = b.rfind(b"[CMD]")
        if i < 0:
            return "(no echo)"
        return bytes(b[i:]).split(b"\r\n")[0].decode("ascii", "replace")

    def num(self, prefix, v, wait=0.45):
        """发带参数命令：先发引导字符，隔 30 ms 再发数值 + CR。

        为什么不一次写完：引导字符和数字挤在同一批 Poll 里时，
        偶尔会出现"数字在引导符之前被 handle"的竞态，表现为命令走了查询分支
        （回显 "mech state=0" 而不是 "MI start"）。分开发彻底避开。
        """
        import time as _t
        txt = "%g" % v
        self.s.reset_input_buffer()
        self.s.write(prefix.encode())
        self.s.flush()
        _t.sleep(0.03)
        self.s.write((txt + "\r").encode())
        self.s.flush()
        t0 = _t.time()
        b = bytearray()
        while _t.time() - t0 < wait:
            d = self.s.read(256)
            if d:
                b += d
        i = b.rfind(b"[CMD]")
        if i < 0:
            return "(no echo)"
        return bytes(b[i:]).split(b"\r\n")[0].decode("ascii", "replace")

    def drain(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            self.s.read(4096)

    def capture(self, sec, keepalive=True):
        """采 30025 帧，返回 [(t, rpm, spdRef, iq, vbus)]"""
        self.s.reset_input_buffer()
        self.buf = bytearray()
        rows = []
        t0 = time.time()
        nxt = t0 + 1.5
        while time.time() - t0 < sec:
            d = self.s.read(4096)
            if d:
                self.buf += d
            if keepalive and time.time() > nxt:
                # deadman 2 s：不保活会在采集途中被封锁，采到的是减速段
                self.s.write(b"S")
                self.s.flush()
                nxt += 1.5

            i = 0
            buf = self.buf
            while i + 24 <= len(buf):
                if buf[i] != 0xFD:
                    i += 1
                    continue
                ln = buf[i + 1]
                mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
                if (ln, mid) != (LEN_IN, MSG_IN):
                    i += 1
                    continue
                body = i + 10
                if body + ln + 2 > len(buf):
                    break
                got = buf[body + ln] | (buf[body + ln + 1] << 8)
                if x25(buf[i + 1:body + ln], CRC_EXTRA) != got:
                    i += 1
                    continue
                iq = struct.unpack_from("<f", buf, body + 4)[0]
                we = struct.unpack_from("<f", buf, body + 44)[0]
                vb = struct.unpack_from("<f", buf, body + 24)[0]
                ref = struct.unpack_from("<f", buf, body + 57)[0]
                rows.append((time.time() - t0, we / POLE * 60.0 / TWO_PI, ref, iq, vb))
                i = body + ln + 2
            self.buf = buf[i:]
        return rows


def metrics(rows, target):
    """超调 / 上升时间 / 稳态 / iq 峰值"""
    if len(rows) < 20:
        return None
    sg = 1.0 if target > 0 else -1.0
    tgt = abs(target)
    t = [r[0] for r in rows]
    v = [r[1] * sg for r in rows]
    iq = [r[3] for r in rows]
    ref = [r[2] * sg for r in rows]

    i10 = next((i for i, q in enumerate(v) if q >= 0.10 * tgt), None)
    i90 = next((i for i, q in enumerate(v) if q >= 0.90 * tgt), None)
    tr = (t[i90] - t[i10]) * 1e3 if (i10 is not None and i90 is not None) else float("nan")

    peak = max(v)
    os_pct = (peak - tgt) / tgt * 100.0
    tail = [q for tt, q in zip(t, v) if tt > t[-1] - 0.35]
    ss = sum(tail) / len(tail) if tail else v[-1]

    # 斜坡走完的时刻（ref 首次到达 99% 目标）
    i_ramp = next((i for i, q in enumerate(ref) if q >= 0.99 * tgt), None)
    ramp_ms = t[i_ramp] * 1e3 if i_ramp is not None else float("nan")
    # ref 真正接管的时刻（= 斜坡结束，之后误差由 PI 收）
    return dict(tr=tr, os=os_pct, ss=ss, iq_peak=max(abs(x) for x in iq),
                ramp=ramp_ms, n=len(rows), ref_end=ref[-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--rpm", type=float, default=500.0)
    ap.add_argument("--accs", default="20000,800,400,200")
    ap.add_argument("--secs", type=float, default=3.0)
    ap.add_argument("--skip-cal", action="store_true", help="跳过 C/Z（Flash 参数还在时用）")
    args = ap.parse_args()

    try:
        rig = Rig(args.port)
    except Exception as e:
        print("打不开 %s：%s" % (args.port, e))
        print("→ 先在上位机页面点「断开串口」（Web Serial 独占句柄）")
        return 2

    try:
        print("[0] 预置")
        print("    " + rig.cmd("D"))
        print("    " + rig.cmd("X"))
        st = rig.cmd("S", 0.4)
        print("    " + st)

        if not args.skip_cal:
            if "cal=1" not in st:
                print("    " + rig.cmd("C", 0.5))
                time.sleep(0.8)
            st = rig.cmd("S", 0.4)
            if "z=1" not in st:
                print("    " + rig.cmd("Z", 0.4))
                for _ in range(15):
                    time.sleep(1.0)
                    st = rig.cmd("S", 0.3)
                    if "z=1" in st:
                        break
                print("    等待辨识完成…")
            print("    " + rig.cmd("S", 0.4))

        accs = [float(x) for x in args.accs.split(",")]
        print("\n[1] 给定 %+.0f rpm，扫 %d 档 accel" % (args.rpm, len(accs)))
        print("    accel     回显 acc        上升时间   超调     稳态      iq峰值   斜坡到位")
        rows_out = []
        for a in accs:
            r1 = rig.num("~", a)
            print("    %-9.0f %s" % (a, r1[:60]))
            m = re.search(r"acc -> ([\d.]+)", r1)
            got = float(m.group(1)) if m else float("nan")

            rig.cmd("D", 0.25)
            rig.drain(0.3)
            r2 = rig.num("=", args.rpm, 0.5)
            print("              " + r2[:70])
            if "refused" in r2 or "failed" in r2:
                print("              [跳过] 进环被拒")
                continue

            rows = rig.capture(args.secs)
            rig.cmd("D", 0.3)
            m2 = metrics(rows, args.rpm)
            if m2 is None:
                print("              采样不足（%d 帧）" % len(rows))
                continue
            print("              %10.1f ms %6.1f%% %8.2f %7.2f A %7.0f ms  (n=%d)"
                  % (m2["tr"], m2["os"], m2["ss"], m2["iq_peak"], m2["ramp"], m2["n"]))
            rows_out.append((a, got, m2))
            time.sleep(0.4)

        if len(rows_out) >= 2:
            print("\n[2] 结论")
            base = rows_out[0][2]
            for a, got, m in rows_out:
                d = m["iq_peak"] - base["iq_peak"]
                print("    acc=%-7.0f iq峰值 %5.2f A (%+5.2f vs 阶跃)  超调 %5.1f%%  到速 %6.1f ms"
                      % (got, m["iq_peak"], d, m["os"], m["ramp"]))

        print("\n[3] 收尾")
        print("    " + rig.cmd("D", 0.3))
        print("    " + rig.cmd("S", 0.4))
    finally:
        rig.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
