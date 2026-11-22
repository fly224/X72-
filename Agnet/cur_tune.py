# -*- coding: utf-8 -*-
"""
cur_tune.py —— 用 iq/id 电流阶跃自动扫增益，整定电流环 PI

依赖固件：T=iq 阶跃 / U=id 阶跃 / V=双轴 / G=Kp 缩放循环 / H=Ki 缩放循环 / J=重发结果。
阶跃在 20 kHz 里采样、片内算指标，串口只传结果，所以不受 100 Hz 遥测带宽限制。

流程：
  1) D -> X（封锁 + 清故障）
  2) S 读状态：cal=0 发 C；z=0 发 Z（约 9 s，电角度对齐）
  3) 扫 Kp 缩放（Ki 固定 1.00x），每点跑一次阶跃
  4) 用第 3 步最好的一档 Kp，再扫 Ki
  5) 打印汇总表与建议

用法：
  python cur_tune.py                 # COM27, id 轴
  python cur_tune.py COM5 --axis iq  # 换口 / 换轴（iq 会产生转矩，转子会转）
  python cur_tune.py --trace         # 额外打印 32 点波形
"""
import re
import sys
import time

import serial

BAUD = 921600
SCALES = ["0.50", "0.75", "1.00", "1.25", "1.50", "2.00"]


class Rig:
    """直接对原始字节流做匹配：实测行切分器会在二进制帧边界上丢行，
    而"在流里找 [CMD] 再截到下一个 CRLF"这条路径是稳定的。"""

    def __init__(self, ser):
        self.ser = ser
        self.raw = bytearray()

    def pump(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            c = self.ser.read(8192)
            if c:
                self.raw.extend(c)

    def send(self, s):
        self.ser.write(s.encode())
        time.sleep(0.02)

    def _take_line(self, tag):
        i = self.raw.find(tag)
        if i < 0:
            return None
        j = self.raw.find(b"\r\n", i)
        if j < 0:
            return None
        line = self.raw[i:j].decode("ascii", "replace")
        del self.raw[:j + 2]
        return line

    def wait_for(self, pattern, timeout=2.0, cmd=None):
        rx = re.compile(pattern)
        if cmd:
            self.raw = bytearray()
            self.send(cmd)
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump(0.03)
            while True:
                ln = self._take_line(b"[CMD]")
                if ln is None:
                    break
                if rx.search(ln):
                    return ln
        return ""

    def wait_lines(self, tags, pattern, timeout=3.0, cmd=None):
        """收集所有匹配行；tags 是起始标记列表（如 [b'[STEP]', b'[TRC]']）"""
        rx = re.compile(pattern)
        if cmd:
            self.raw = bytearray()
            self.send(cmd)
        got = []
        t0 = time.time()
        last = time.time()
        while time.time() - t0 < timeout:
            self.pump(0.03)
            while True:
                idx = [(self.raw.find(t), t) for t in tags]
                idx = [(i, t) for (i, t) in idx if i >= 0]
                if not idx:
                    break
                i, t = min(idx)
                j = self.raw.find(b"\r\n", i)
                if j < 0:
                    break
                ln = self.raw[i:j].decode("ascii", "replace")
                del self.raw[:j + 2]
                if rx.search(ln):
                    got.append(ln)
                    last = time.time()
            if len(got) >= 2 and (time.time() - last) > 0.25:
                break
        return got


def parse_status(line):
    d = {}
    for k in ("en", "moe", "cal", "z", "cur", "f", "dm", "clamp"):
        m = re.search(r"\b%s=([0-9a-fA-Fx]+)" % k, line)
        if m:
            d[k] = m.group(1)
    return d


def parse_step(line):
    m = re.search(r"\[STEP\] (\w+) (.*)$", line)
    if not m:
        return None, None
    ax, rest = m.group(1), m.group(2)
    if "invalid" in rest:
        return ax, None
    d = {}
    for key, pat in (("amp", r"amp=([-\d.]+)"), ("tr", r"tr=(\d+)"),
                     ("os", r"os=([-\d.]+)"), ("ts", r"ts=(\d+)"),
                     ("ss", r"ss=([-\d.]+)"), ("pk", r"pk=([-\d.]+)"),
                     ("vp", r"vp=([-\d.]+)")):
        mm = re.search(pat, rest)
        if mm:
            d[key] = float(mm.group(1))
    return ax, d


def ensure_ready(rig, log):
    log.append("D/X: " + rig.wait_for(r"\[CMD\] OFF", 1.5, "D"))
    log.append("X  : " + rig.wait_for(r"\[CMD\] fault cleared", 1.5, "X"))

    st_line = rig.wait_for(r"\[CMD\] st ", 2.5, "S")
    st = parse_status(st_line)
    log.append("st : " + st_line)

    if st.get("cal") != "1":
        log.append("未标定 -> C")
        log.append("C  : " + rig.wait_for(r"\[CMD\] [Cc][Aa][Ll]", 3.0, "C"))
        # 标定 200 ms 完成后 cal 才置 1，用状态轮询确认（不依赖回显文案）
        for _ in range(12):
            st = parse_status(rig.wait_for(r"\[CMD\] st ", 1.2, "S"))
            if st.get("cal") == "1":
                break
        log.append("标定完成: cal=%s" % st.get("cal"))

    if st.get("z") != "1":
        log.append("未对齐 -> Z（约 9 s）")
        rig.send("Z")
        log.append("Z  : " + rig.wait_for(r"IDENT ok|IDENT FAIL", 14.0))
        for _ in range(8):
            st = parse_status(rig.wait_for(r"\[CMD\] st ", 1.2, "S"))
            if st.get("z") == "1":
                break
        log.append("对齐完成: z=%s" % st.get("z"))

    log.append("就绪: cal=%s z=%s" % (st.get("cal"), st.get("z")))
    return st


def set_scale(rig, which, target):
    """which='g'(Kp) / 'h'(Ki)：循环发送直到回显缩放等于 target。"""
    pat = r"\[CMD\] kp x([0-9.]+)" if which == "g" else r"\[CMD\] ki x([0-9.]+)"
    for _ in range(8):
        ln = rig.wait_for(pat, 1.0, which.upper())
        if not ln:
            continue
        m = re.search(pat, ln)
        if m and abs(float(m.group(1)) - float(target)) < 1e-3:
            return True
    return False


def run_step(rig, axis):
    lines = rig.wait_lines([b"[STEP]", b"[TRC]"], r"\[STEP\]|\[TRC\]", 4.0,
                            "U" if axis == "id" else "T")
    res, trc = {}, []
    for ln in lines:
        if ln.startswith("[TRC]"):
            trc.append(ln)
            continue
        ax, d = parse_step(ln)
        if ax:
            res[ax] = d
    err = ""
    if not res:
        err = rig.wait_for(r"\[CMD\] STEP", 0.4)   # 没指标时捞一条拒绝原因
    return res, trc, err


def fmt_row(tag, d, err=""):
    if not d:
        return "  %-10s  (无指标) %s" % (tag, err)
    return ("  %-10s  tr=%4.0f us  os=%+6.2f %%  ts=%5.0f us  ss=%+6.2f %%  pk=%.3f  vp=%.2f V"
            % (tag, d["tr"], d["os"], d["ts"], d["ss"], d["pk"], d["vp"]))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    port = args[0] if args else "COM27"
    axis = "id"
    if "--axis" in sys.argv:
        axis = sys.argv[sys.argv.index("--axis") + 1]
    want_trace = "--trace" in sys.argv

    ser = serial.Serial(port, BAUD, timeout=0.005)
    print("OPEN_OK %s @ %d" % (ser.name, BAUD))
    time.sleep(0.2)
    ser.reset_input_buffer()

    rig = Rig(ser)
    log = []
    rows = []
    trc_dump = []
    try:
        ensure_ready(rig, log)

        print("\n[1] 扫 Kp 缩放（Ki 固定 1.00x，阶跃轴=%s）" % axis)
        set_scale(rig, "h", "1.00")
        best = None
        for s in SCALES:
            if not set_scale(rig, "g", s):
                print("  !! 设置 Kp=%sx 失败" % s)
                continue
            res, trc, err = run_step(rig, axis)
            d = res.get(axis)
            print(fmt_row("kp x" + s, d, err))
            if d:
                rows.append((float(s), 1.0, d))
                score = abs(d["os"] - 8.0) * 2.0 + d["tr"] / 100.0
                if best is None or score < best[0]:
                    best = (score, s)
            if want_trace and trc:
                trc_dump.append(("kp x" + s, trc))

        if best:
            kp_best = best[1]
            set_scale(rig, "g", kp_best)
            print("\n[2] 扫 Ki 缩放（Kp 固定 %sx）" % kp_best)
            for s in SCALES:
                if not set_scale(rig, "h", s):
                    print("  !! 设置 Ki=%sx 失败" % s)
                    continue
                res, trc, err = run_step(rig, axis)
                d = res.get(axis)
                print(fmt_row("ki x" + s, d, err))
                if d:
                    rows.append((float(kp_best), float(s), d))
                if want_trace and trc:
                    trc_dump.append(("ki x" + s, trc))
    finally:
        try:
            rig.wait_for(r"\[CMD\] OFF", 1.0, "D")
        except Exception:
            pass
        ser.close()

    print("\n--- 过程 ---")
    for ln in log:
        print("  |", ln)

    if rows:
        print("\n汇总（%s 轴，1 A 阶跃）" % axis)
        print("  kp_x   ki_x    tr(us)   os(%)   ts(us)   ss(%)   pk(A)   vp(V)")
        print("  " + "-" * 66)
        for kp, ki, d in rows:
            print("  %4.2f   %4.2f  %6.0f  %+6.2f  %6.0f  %+6.2f  %6.3f  %5.2f"
                  % (kp, ki, d["tr"], d["os"], d["ts"], d["ss"], d["pk"], d["vp"]))
        print("\n判据：tr 与 250us 同量级；os 5~15% 较理想；ss 应接近 0；vp 不应接近限幅(约 16.5V)。")
        print("      os 随 kp 增大而增大；tr 不再下降说明已到采样/滤波极限。")

    if want_trace:
        for tag, lines in trc_dump[-2:]:
            print("\n波形 %s（t_ms id_ref id_now iq_ref iq_now）" % tag)
            for ln in lines:
                print("  " + ln)
    return 0


if __name__ == "__main__":
    sys.exit(main())
