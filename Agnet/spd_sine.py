# -*- coding: utf-8 -*-
"""spd_sine.py —— 速度环正弦轨迹 + 母线泵升监测

为什么用上位机而不是固件：固件里速度给定是常数（`=rpm`），没有正弦发生器，
而可打印字符已全部占满，加新命令要动协议。上位机以 50 ms 周期重发 `=rpm`
即可合成任意轨迹 —— `Servo_SetSpeedMode` 在已使能时只改给定、不重开 MOE，
且每次都会刷新 deadman（2 s），脚本崩溃时硬件自动封锁。

四象限注意：正弦必然包含"电机发电"的减速段，能量回灌母线。母线电容小或
电源单向时会顶到 OV(58 V) 跳闸。所以先跑 --probe 小幅度量泵升系数，再外推。

用法：
    python spd_sine.py --probe                      # ±300 rpm × 3 周期，量泵升
    python spd_sine.py --amp 2000 --freq 0.1 --secs 600 --tag run10m
    python spd_sine.py --amp 2000 --freq 0.1 --secs 30 --dry   # 不落盘

判据（probe 模式）：
    dV300 = 母线相对静止时的最大抬升。外推 dV2000 ≈ dV300 × (2000/300)²
    若 dV2000 + Vbus > 52 V（留 6 V 余量）则 2000 rpm 全幅不可行。
"""
import argparse
import csv
import math
import os
import re
import struct
import sys
import time

import serial

BAUD = 921600
MSG_IN, LEN_IN, EXTRA_IN = 30025, 61, 191
POLE = 14.0
TWO_PI = 2.0 * math.pi
RAD_PER_RPM = TWO_PI / 60.0
OUTDIR = os.path.dirname(os.path.abspath(__file__))


def x25(data, extra):
    crc = 0xFFFF

    def acc(c, b):
        t = (b ^ (c & 0xFF)) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        return ((c >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF

    for b in data:
        crc = acc(crc, b)
    return acc(crc, extra)


class Link:
    """不做 reset_input_buffer 的发送器：遥测流不能被打断。"""

    def __init__(self, port):
        self.s = serial.Serial(port, BAUD, timeout=0)
        self.raw = bytearray()
        self.frames = 0
        self.bad = 0
        self.last = {}
        self.recent = []

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass

    def write(self, b):
        self.s.write(b if isinstance(b, bytes) else b.encode())

    def set_rpm(self, rpm):
        self.write(b"=" + ("%.1f" % rpm).encode() + b"\r")

    def pump(self):
        d = self.s.read(8192)
        if d:
            self.raw += d
        self._parse()

    def _parse(self):
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
            if x25(self.raw[i + 1:b + ln], EXTRA_IN) != crc:
                self.bad += 1
                i += 1
                continue
            # 30025 布局：13×f32(0..51) + u8 mode(52) + f32 motor_temp(53) + f32 spd_ref(57)
            f = struct.unpack_from("<13fBff", self.raw, b)
            self.last = {
                "iq_tar": f[1],
                "iu": f[2], "iv": f[3], "iw": f[4],
                "vbus": f[6],
                "temp_gan": f[10],
                "w_elec": f[11],
                "temp_motor": f[13],
                "ref_rpm": f[14],
            }
            self.frames += 1
            self.recent.append((time.time(), self.last.copy()))
            if len(self.recent) > 200:
                self.recent.pop(0)
            i = b + ln + 2
        del self.raw[:i]

    def wait_frames(self, k=8, secs=1.5):
        t0 = time.time()
        while self.frames < k and time.time() - t0 < secs:
            self.pump()
            time.sleep(0.002)

    def status(self):
        """发 S 并抓完整回显。行不完整时返回 {}（由调用方重试），绝不误判 moe=0。"""
        self.write(b"S")
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < 0.30:
            d = self.s.read(2048)
            if d:
                buf += d
            else:
                time.sleep(0.002)
        i = buf.rfind(b"[CMD] st")
        if i < 0:
            return {}
        seg = bytes(buf[i:])
        j = seg.find(b"\r\n")
        if j <= 0:
            return {}                       # 还没收全
        line = seg[:j].decode("ascii", "replace")
        if "v=" not in line or "f=0x" not in line:
            return {}                       # 截断，丢弃
        d = {"line": line}
        for k in ("en", "moe", "cal", "z", "f", "dm", "v", "cur", "spd"):
            m = re.search(r"\b" + k + r"=([-\w.]+)", line)
            if m:
                d[k] = m.group(1)
        return d


def run(link, amp, freq, secs, offset, cmd_dt, log_dt, watch_fault, tag, dry, verbose):
    print("=" * 78)
    print("速度正弦  amp=±%.0f rpm  offset=%.0f  f=%.3f Hz  T=%.1f s  dur=%.0f s"
          % (amp, offset, freq, 1.0 / freq if freq > 0 else 0, secs))
    print("=" * 78)

    link.write(b"D")
    time.sleep(0.4)
    link.raw.clear()
    link.frames = 0
    st = {}
    for _ in range(6):
        st = link.status()
        if st.get("cal") is not None:
            break
        time.sleep(0.10)
    print("[1] 初始: " + st.get("line", "(no echo)"))
    if st.get("f") not in (None, "0x00"):
        print("    [ABORT] 有故障位 %s，先 X" % st.get("f"))
        return None
    if st.get("cal") != "1" or st.get("z") != "1":
        print("    [ABORT] cal/z 未完成 (cal=%s z=%s)" % (st.get("cal"), st.get("z")))
        return None

    link.wait_frames(8)
    v0 = link.last.get("vbus", float("nan"))
    print("[2] 静止母线 %.2f V" % v0)
    print("[3] 使能速度环（=0 rpm）")
    link.set_rpm(0.0)
    time.sleep(0.20)
    st = {}
    for _ in range(6):
        st = link.status()
        if st.get("moe") is not None:
            break
        time.sleep(0.10)
    if st.get("moe") == "0":
        print("    [ABORT] MOE 未开: " + st.get("line", "(no echo)"))
        return None
    if st.get("moe") is None:
        print("    [ABORT] 状态行读不全（回显被遥测顶掉）")
        return None
    print("    " + st.get("line", ""))

    rows = []
    t0 = time.time()
    nxt_cmd = t0
    nxt_log = t0 + log_dt
    nxt_chk = t0 + 2.0
    vmax = -1e9
    fault_hit = None
    aborted = False

    try:
        while True:
            now = time.time()
            el = now - t0
            if el >= secs:
                break

            if now >= nxt_cmd:
                ref = offset + amp * math.sin(TWO_PI * freq * el)
                link.set_rpm(ref)
                nxt_cmd += cmd_dt

            link.pump()

            if now >= nxt_log:
                L = link.last
                if L:
                    r = {
                        "t": round(el, 2),
                        "ref": round(offset + amp * math.sin(TWO_PI * freq * el), 1),
                        "ref_fw": round(L["ref_rpm"], 1),
                        "act": round(L["w_elec"] / POLE / RAD_PER_RPM, 1),
                        "iq": round(L["iq_tar"], 3),
                        "vbus": round(L["vbus"], 2),
                        "tg": round(L["temp_gan"], 1),
                        "tm": round(L["temp_motor"], 1),
                    }
                    rows.append(r)
                    if r["vbus"] > vmax:
                        vmax = r["vbus"]
                    if verbose and len(rows) % 20 == 1:
                        print("  t=%7.1f  ref=%7.1f  act=%7.1f  iq=%6.2f  V=%5.1f  tg=%5.1f"
                              % (r["t"], r["ref_fw"], r["act"], r["iq"], r["vbus"], r["tg"]))
                nxt_log += log_dt

            if watch_fault and now >= nxt_chk:
                nxt_chk += 2.0
                st = link.status()
                fv = st.get("f")
                if fv is not None and fv != "0x00":
                    fault_hit = st.get("line", "")
                    print("\n  [FAULT] " + fault_hit)
                    aborted = True
                    break
                if st.get("moe") == "0":
                    print("\n  [FAULT] MOE 已掉 (无 fault 回显被顶): " + st.get("line", ""))
                    aborted = True
                    break

            time.sleep(0.002)
    except KeyboardInterrupt:
        print("\n  [INT] 用户中断")

    link.write(b"D")
    time.sleep(0.3)
    st = link.status()
    print("[4] 收尾: " + st.get("line", "(no echo)"))

    dV = vmax - v0
    print("\n" + "-" * 78)
    print("  样本 %d  遥测帧 %d  CRC错 %d" % (len(rows), link.frames, link.bad))
    print("  母线 %.2f -> 峰值 %.2f  (dV = %+.2f V)" % (v0, vmax, dV))
    if aborted:
        print("  ⚠ 中途跳闸，数据不完整")
    print("-" * 78)

    if rows and not dry:
        path = os.path.join(OUTDIR, "spd_sine_%s.csv" % (tag or time.strftime("%H%M%S")))
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print("  -> %s" % path)

    return {"v0": v0, "vmax": vmax, "dV": dV, "rows": rows,
            "aborted": aborted, "fault": fault_hit}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--amp", type=float, default=2000.0)
    ap.add_argument("--freq", type=float, default=0.1)
    ap.add_argument("--secs", type=float, default=600.0)
    ap.add_argument("--offset", type=float, default=0.0)
    ap.add_argument("--cmd-ms", type=float, default=50.0)
    ap.add_argument("--log-ms", type=float, default=100.0)
    ap.add_argument("--tag", default="")
    ap.add_argument("--probe", action="store_true", help="±300 rpm × 3 周期量泵升")
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    if a.probe:
        a.amp, a.freq, a.secs = 300.0, 0.333, 9.0
        a.tag = a.tag or "probe"

    link = Link(a.port)
    try:
        res = run(link, a.amp, a.freq, a.secs, a.offset,
                  a.cmd_ms / 1000.0, a.log_ms / 1000.0,
                  True, a.tag, a.dry, a.verbose)
    finally:
        link.close()

    if res and a.probe and not res["aborted"]:
        dV = res["dV"]
        k = (2000.0 / a.amp) ** 2
        print("\n外推（能量 ∝ ω²，dV ∝ ΔE/C）：")
        print("  ±%.0f rpm -> dV %+.2f V" % (a.amp, dV))
        print("  ±2000 rpm -> dV %+.1f V  =>  母线峰值 ≈ %.0f V" % (dV * k, res["v0"] + dV * k))
        print("  OV 门限 58 V，安全余量需 >= 6 V（即峰值 <= 52 V）")
        print("  判定：%s" % ("可行" if res["v0"] + dV * k <= 52.0 else "不可行 —— 会顶 OV"))


if __name__ == "__main__":
    sys.exit(main())
