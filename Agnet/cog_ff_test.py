# -*- coding: utf-8 -*-
"""
cog_ff_test.py —— 齿槽前馈 A/B 实测

流程：C → Z → `}` 测齿槽（自动装表）→ `;` 开关对比 3/30/100 rpm

判据：开前馈后纹波 σ 应明显下降（齿槽峰峰 1.17 A 是 30 rpm 运行电流的 9 倍）。

用法：
    python cog_ff_test.py COM27
    python cog_ff_test.py COM27 --secs 4 --skip-cal
"""

import argparse
import math
import struct
import time

import serial

MSG_IN = 30025
LEN_IN = 61
POLE = 14.0
TWO_PI = 2.0 * math.pi
RAD_PER_RPM = TWO_PI / 60.0
RPM_TAB = [3.0, 30.0, 100.0, 500.0, 1500.0, 3000.0]


def x25(data, extra):
    crc = 0xFFFF

    def acc(c, b):
        t = (b ^ (c & 0xFF)) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        return ((c >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF

    for b in data:
        crc = acc(crc, b)
    return acc(crc, extra)


class Rig:
    def __init__(self, port):
        self.s = serial.Serial(port, 921600, timeout=0.05)
        self.raw = bytearray()

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass

    def cmd(self, c, wait=0.4):
        self.s.reset_input_buffer()
        self.s.write(c.encode() if isinstance(c, str) else c)
        self.s.flush()
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < wait:
            d = self.s.read(512)
            if d:
                buf += d
        i = buf.rfind(b"[CMD]")
        if i < 0:
            return "(no echo)"
        return bytes(buf[i:]).split(b"\r\n")[0].decode("ascii", "replace")

    def keep(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            time.sleep(0.6)
            self.s.write(b"S")
            self.s.flush()

    def set_rpm(self, rpm):
        idx = RPM_TAB.index(abs(rpm))
        self.cmd("Y", 0.25)
        for _ in range((idx - 1) % 6):
            self.cmd("O", 0.26)

    def collect(self, secs):
        frames = []
        t0 = time.time()
        nxt = t0 + 0.8
        while time.time() - t0 < secs:
            d = self.s.read(4096)
            if d:
                self.raw += d
            if time.time() > nxt:
                self.s.write(b"S")
                self.s.flush()
                nxt += 0.8
            i = 0
            n = len(self.raw)
            while i + 12 <= n:
                if self.raw[i] != 0xFD:
                    i += 1
                    continue
                ln = self.raw[i + 1]
                mid = self.raw[i + 7] | (self.raw[i + 8] << 8) | (self.raw[i + 9] << 16)
                if ln != LEN_IN or mid != MSG_IN:
                    i += 1
                    continue
                b = i + 10
                if b + ln + 2 > n:
                    break
                crc = self.raw[b + ln] | (self.raw[b + ln + 1] << 8)
                if x25(self.raw[i + 1:b + ln], 191) != crc:
                    i += 1
                    continue
                iq = struct.unpack_from("<f", self.raw, b + 4)[0]
                we = struct.unpack_from("<f", self.raw, b + 44)[0]
                frames.append((time.time(), we / POLE / RAD_PER_RPM, iq))
                i = b + ln + 2
            del self.raw[:i]
        return frames


def stats(frames, tail_frac=0.3):
    if len(frames) < 20:
        return None
    # 剔除尾部封锁帧（rpm 与 iq 同时为 0）
    cut = len(frames)
    for i in range(len(frames) - 1, len(frames) // 2, -1):
        if abs(frames[i][1]) < 0.05 and abs(frames[i][2]) < 0.05:
            cut = i
        else:
            break
    frames = frames[:max(cut, 1)]
    if len(frames) < 20:
        return None
    v = [f[1] for f in frames]
    iq = [f[2] for f in frames]
    n = max(5, int(len(v) * tail_frac))
    tail = v[-n:]
    m = sum(tail) / len(tail)
    sd = math.sqrt(sum((x - m) ** 2 for x in tail) / len(tail))
    return dict(ss=m, sd=sd, peak=max(tail, key=abs),
                iq_mean=sum(iq[-n:]) / n, n=len(v))


def run_point(rig, rpm, secs):
    rig.cmd("D", 0.3)
    rig.set_rpm(rpm)
    echo = rig.cmd("W", 0.45)
    if "refused" in echo or "failed" in echo:
        return None, echo
    rig.keep(0.9)

    fr = rig.collect(secs)
    rig.cmd("D", 0.3)
    return stats(fr), echo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--skip-cal", action="store_true")
    ap.add_argument("--rpms", default="3,30,100")
    args = ap.parse_args()

    rig = Rig(args.port)
    try:
        print("[1] 状态")
        print("   ", rig.cmd("D", 0.3))
        print("   ", rig.cmd("X", 0.3))

        if not args.skip_cal:
            print("[2] C + Z")
            print("   ", rig.cmd("C", 0.5))
            time.sleep(1.0)
            rig.s.reset_input_buffer()
            rig.s.write(b"Z")
            rig.s.flush()
            t0 = time.time()
            buf = bytearray()
            while time.time() - t0 < 25.0:
                d = rig.s.read(512)
                if d:
                    buf += d
                if b"IDENT ok" in buf:
                    break
            for ln in bytes(buf).decode("ascii", "replace").splitlines():
                if "IDENT ok" in ln:
                    print("   ", ln.strip()[:105])
        else:
            print("[2] 跳过 C+Z")

        print("[3] 测齿槽（正反各一圈，约 14 s）")
        print("   ", rig.cmd("}", 0.5))
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < 45.0:
            d = rig.s.read(1024)
            if d:
                buf += d
            if b"MI end" in buf:
                break
        for ln in bytes(buf).decode("ascii", "replace").splitlines():
            j = ln.find("[CMD] MI")
            if j >= 0:
                print("   ", ln[j:j + 130].strip())
        print("   ", rig.cmd(":", 0.4))
        print("   ", rig.cmd(";", 0.4))       # 开
        print("   ", rig.cmd(":", 0.4))

        rpms = [float(x) for x in args.rpms.split(",")]
        res = {}
        for want in (True, False):
            st = rig.cmd(":", 0.3)
            cur = " en=1" in st
            if cur != want:
                rig.cmd(";", 0.3)
            print("\n----- 齿槽前馈 %s -----" % ("ON" if want else "OFF"))
            for rpm in rpms:
                r, echo = run_point(rig, rpm, args.secs)
                if r is None:
                    print("  %5.0f rpm: 失败 %s" % (rpm, echo))
                    continue
                res[(want, rpm)] = r
                print("  %5.0f rpm: ss=%8.3f sd=%6.3f peak=%8.3f iq=%6.3f/%6.3f"
                      % (rpm, r["ss"], r["sd"], r["peak"], r["iq_mean"],
                         max(abs(f) for f in [r["iq_mean"]])))

        print("\n=== 对比（纹波 σ）===")
        print("   rpm     OFF      ON      改善")
        for rpm in rpms:
            a = res.get((True, rpm))
            b = res.get((False, rpm))
            if not a or not b:
                continue
            imp = (a["sd"] - b["sd"]) / b["sd"] * 100 if b["sd"] else 0
            print("  %5.0f  %7.3f  %7.3f  %+6.1f%%" % (rpm, b["sd"], a["sd"], imp))

        print("\n[4] 保存到 Flash")
        print("   ", rig.cmd("A", 1.2))
        print("   ", rig.cmd(":", 0.4))
        return 0
    finally:
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
