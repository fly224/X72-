# -*- coding: utf-8 -*-
"""
sched_test.py —— 速度环增益调度 A/B 实测

对比同一给定序列下：
  sched ON  （多段查表，`, ` 命令开）
  sched OFF （单组增益 0.5834，Y 命令复位）

指标：稳态误差、纹波 σ、峰值、iq 均值/峰值。
零速单独测：给定 0，看 2 s 内的位移漂移与速度抖动。

用法：
    python sched_test.py COM27                 # 默认 5 点
    python sched_test.py COM27 --secs 3
    python sched_test.py COM27 --zero          # 只测零速保持
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

# 固件 O 键的档位表（顺序敏感）
RPM_TAB = [3.0, 30.0, 100.0, 500.0, 1500.0, 3000.0]
IDX_DEFAULT = 1      # Y 复位后是 30 rpm（索引 1）


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
        self.frames = []

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

    def set_rpm(self, rpm):
        """按 O 键差值切档。Y 复位后停在 30 rpm。"""
        target = abs(rpm)
        if target not in RPM_TAB:
            raise ValueError("rpm %g 不在档位表 %s" % (rpm, RPM_TAB))
        idx = RPM_TAB.index(target)
        # 注意用固件表长度 6，不是 RPM_TAB 长度 5：
        # RPM_TAB 只是"要测的点"，固件 O 键循环的是 3/30/100/500/1500/3000 六档。
        # 用 %5 会让 3 rpm 实际停到 3000（实测过：K 行 ramp=3000.00rpm）。
        presses = (idx - IDX_DEFAULT) % 6
        for _ in range(presses):
            self.cmd("O", 0.28)

    def keepalive_sleep(self, secs):
        """等待期间必须保活：deadman 是 2 s，而进环后到第一次采集
        之间要经历 W 回显(0.5s) + 稳定等待 + drain，累计会超过 2 s，
        电机被封锁后采到的全是 0（第一版就是这么得到 ss=0 的假数据）。"""
        t0 = time.time()
        while time.time() - t0 < secs:
            time.sleep(0.6)
            try:
                self.s.write(b"S")
                self.s.flush()
            except Exception:
                return

    def drain(self, secs=0.2):
        t0 = time.time()
        nxt = t0 + 0.6
        while time.time() - t0 < secs:
            self.s.read(4096)
            if time.time() > nxt:
                self.s.write(b"S")
                self.s.flush()
                nxt += 0.6
        self.raw = bytearray()

    def collect(self, secs):
        """采 30025，返回 [(t, rpm, iq_tar, vbus)]"""
        self.frames = []
        t0 = time.time()
        nxt = t0 + 0.8
        while time.time() - t0 < secs:
            d = self.s.read(4096)
            if d:
                self.raw += d
            now = time.time()
            if now > nxt:
                self.s.write(b"S")       # deadman 保活（2 s）
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
                vb = struct.unpack_from("<f", self.raw, b + 24)[0]
                self.frames.append((time.time(), we / POLE / RAD_PER_RPM, iq, vb))
                i = b + ln + 2
            del self.raw[:i]
        return self.frames


def stats(frames, target, tail_frac=0.25):
    if len(frames) < 20:
        return None
    # 剔除尾部"已封锁"的帧：封锁后 rpm 与 iq 同时为 0。
    # 不剔除的话稳态会被算成 0（表现为 ss=0、err=−给定）。
    cut = len(frames)
    run = 0
    for i in range(len(frames) - 1, max(len(frames) // 2, 0) - 1, -1):
        r, iq0 = frames[i][1], frames[i][2]
        if abs(r) < 0.05 and abs(iq0) < 0.05:
            run += 1
            cut = i
        else:
            run = 0
            if cut < len(frames):
                break
    frames = frames[:max(cut, 1)] if cut < len(frames) else frames
    if len(frames) < 20:
        return None
    v = [f[1] for f in frames]
    iq = [f[2] for f in frames]
    sgn = 1.0 if target >= 0 else -1.0
    vn = [x * sgn for x in v]

    n_tail = max(5, int(len(vn) * tail_frac))
    tail = vn[-n_tail:]
    tail_iq = iq[-n_tail:]
    ss = sum(tail) / len(tail)
    m = ss
    sd = math.sqrt(sum((x - m) ** 2 for x in tail) / len(tail))

    return dict(
        ss=ss, sd=sd, peak=max(vn, key=abs),
        iq_mean=sum(tail_iq) / len(tail_iq),
        iq_peak=max(abs(x) for x in iq),
        n=len(frames),
    )


def set_sched(rig, want_on):
    """用 '.'（只读）判断当前状态，不一致才用 ','（切换）翻一次。
    不能用 ',' 判状态——它按一次就翻转，会把自己的判断依据改掉。"""
    st = rig.cmd(".", 0.3)
    cur = "ON" in st
    if cur != want_on:
        st = rig.cmd(",", 0.3)
        cur = "ON" in st
    return cur, st


def one_point(rig, rpm, secs, sched_on):
    """测一个工作点：进环 → 稳 0.8 s → 采 secs → 封锁"""
    tag = "sched ON " if sched_on else "sched OFF"

    rig.cmd("D", 0.3)
    st = rig.cmd("S", 0.35)
    if "cal=1" not in st or "z=1" not in st:
        raise RuntimeError("需要先 C+Z：%s" % st)
    if "f=0x00" not in st:
        rig.cmd("X", 0.3)

    rig.cmd("Y", 0.3)                       # 增益 + 转速档位复位到 30 rpm
    rig.set_rpm(rpm)

    echo = rig.cmd("W" if rpm > 0 else "Q", 0.5)
    if "refused" in echo or "failed" in echo:
        raise RuntimeError("进环被拒：%s" % echo)

    rig.keepalive_sleep(0.9)                # 等斜坡走完、稳态建立（期间保活）
    rig.drain(0.12)
    frames = rig.collect(secs)
    rig.cmd("D", 0.3)

    st = stats(frames, rpm)
    if st is None:
        return None
    st["tag"] = tag
    st["rpm"] = rpm
    return st


def zero_hold(rig, secs, sched_on):
    """零速保持：给定 0，看漂移与抖动。
    速度环给 0 时斜坡从实测速度起步，所以必须先让它真停住再采。"""
    rig.cmd("D", 0.3)
    rig.cmd("Y", 0.3)
    # 2026-11-17：原来这里发 "P" 切位置模式（不使能）。P 已改成数值引导符
    # （空回车=保持并使能），发裸 "P" 会让数值解析器挂 300 ms 并吞掉后面的 "="。
    # 这个函数本来就在 D 之后、要的是"未使能"状态，直接去掉这行。
    rig.keepalive_sleep(0.5)

    echo = rig.cmd("=0\r" if False else "=", 0.3)   # 引导符
    time.sleep(0.05)
    rig.s.write(b"0\r")
    rig.s.flush()
    rig.keepalive_sleep(0.4)
    rig.drain(0.12)
    frames = rig.collect(secs)
    rig.cmd("D", 0.3)
    if len(frames) < 20:
        return None
    v = [f[1] for f in frames]
    m = sum(v) / len(v)
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
    return dict(mean=m, sd=sd, mx=max(abs(x) for x in v), n=len(v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--secs", type=float, default=3.0)
    ap.add_argument("--only", type=float, default=None, help="只测这一个 rpm")
    ap.add_argument("--zero", action="store_true", help="只测零速保持")
    args = ap.parse_args()

    rig = Rig(args.port)
    try:
        if args.zero:
            print("=== 零速保持（给定 0，%.1f s）===" % args.secs)
            for want in (True, False):
                ok, st = set_sched(rig, want)
                r = zero_hold(rig, args.secs, ok)
                if r:
                    print("  %-9s mean=%+.4f rpm  sd=%.4f  max|.|=%.4f  n=%d"
                          % ("sched " + ("ON" if ok else "OFF"),
                             r["mean"], r["sd"], r["mx"], r["n"]))
                else:
                    print("  sched %s: 数据不足" % ("ON" if ok else "OFF"))
            return 0

        rpms = [args.only] if args.only else [3.0, 30.0, 100.0, 500.0, 1500.0]

        results = {}
        for sched in (True, False):
            ok, st = set_sched(rig, sched)
            if ok != sched:
                print("!! 调度状态切换失败：%s" % st)
                return 2
            print("\n----- %s （回显：%s）-----" % ("sched ON" if sched else "sched OFF", st))
            for rpm in rpms:
                try:
                    r = one_point(rig, rpm, args.secs, sched)
                except Exception as e:
                    print("  %6.0f rpm: %s" % (rpm, e))
                    continue
                if r is None:
                    print("  %6.0f rpm: 数据不足" % rpm)
                    continue
                results[(sched, rpm)] = r
                print("  %6.0f rpm: ss=%8.3f  err=%+7.3f  sd=%6.3f  peak=%8.3f  iq=%5.3f/%5.3f"
                      % (rpm, r["ss"], r["ss"] - rpm, r["sd"], r["peak"],
                         r["iq_mean"], r["iq_peak"]))

        # 对比表
        print("\n=== 对比（|稳态误差| / 纹波 σ）===")
        print("  rpm     sched ON            sched OFF           改善")
        for rpm in rpms:
            a = results.get((True, rpm))
            b = results.get((False, rpm))
            if not a or not b:
                continue
            ea, eb = abs(a["ss"] - rpm), abs(b["ss"] - rpm)
            da = (a["sd"] / eb * 100) if eb else 0
            print("  %6.0f  %7.3f / %6.3f    %7.3f / %6.3f    sd %+6.1f%%"
                  % (rpm, ea, a["sd"], eb, b["sd"],
                     (a["sd"] - b["sd"]) / b["sd"] * 100 if b["sd"] else 0))
        return 0
    finally:
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
