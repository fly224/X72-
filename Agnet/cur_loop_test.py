# -*- coding: utf-8 -*-
"""
cur_loop_test.py —— 电角度对齐 + 电流环闭环实测（实机脚本，需独占 COM27）

流程：C(零漂标定) -> X(清故障) -> Z(电角度辨识) -> I(结果) -> M(+1A) -> N(-1A) -> D

判据（电流环带宽 ~1 kHz，20 kHz 采样，理论上 1~2 ms 就到给定）：
  * iq 应在 5 ms 内跟上 iq_ref，稳态误差 < 5%；
  * id 应保持在 0 附近（说明电角度对齐正确，d/q 解耦成立）；
  * 故障位必须为 0，MOE 全程为 1；
  * 正负给定下 iq 对称（符号跟随），说明没有把 θ 算反。
"""
import struct
import sys
import time

import serial

PORT = "COM27"
BAUD = 921600
POLE_PAIRS = 14.0
EX = {30025: 191, 30027: 58, 30040: 160}
TWO_PI = 6.283185307179586


def crc(b):
    c = 0xFFFF
    for x in b:
        t = (x ^ c) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        c = ((c >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return c


class Link:
    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()
        self.ascii = []
        self.last = {}
        self.series = []          # [(t, iq_ref, id, iq, rpm, enc)]

    def pump(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            chunk = self.ser.read(4096)
            if chunk:
                self.buf.extend(chunk)
                self._parse()
        return self.snapshot()

    def _parse(self):
        i = 0
        b = self.buf
        while i < len(b):
            if b[i] == 0x5B:
                j = b.find(b"\r\n", i, min(i + 240, len(b)))
                if j > 0:
                    seg = bytes(b[i:j])
                    if all(32 <= c <= 126 for c in seg):
                        self.ascii.append(seg.decode("ascii", "replace"))
                        i = j + 2
                        continue
            if b[i] != 0xFD:
                i += 1
                continue
            if i + 10 > len(b):
                break
            ln = b[i + 1]
            mid = b[i + 7] | (b[i + 8] << 8) | (b[i + 9] << 16)
            fl = 12 + ln
            if mid not in EX or i + fl > len(b):
                i += 1
                continue
            got = b[i + 10 + ln] | (b[i + 11 + ln] << 8)
            if crc(b[i + 1:i + 10 + ln] + bytes([EX[mid]])) == got:
                p = bytes(b[i + 10:i + 10 + ln])
                self.last[mid] = p
                if mid == 30025 and len(p) >= 57:
                    v = struct.unpack_from("<13fB", p, 0)
                    q = self.last.get(30027)
                    r = self.last.get(30040)
                    iq_cmd = v[1]
                    if q and len(q) >= 32:
                        idn, iqn = struct.unpack_from("<ff", q, 12)
                    else:
                        idn = iqn = float("nan")
                    enc = struct.unpack_from("<q", r, 0)[0] if (r and len(r) >= 8) else 0
                    self.series.append((time.time(), iq_cmd, idn, iqn, v[11], enc))
                i += fl
            else:
                i += 1
        self.buf = b[i:]

    def snapshot(self):
        s = {"iu": None, "iw": None, "vbus": None, "enc": None, "iq_ref": None, "id": None, "iq": None}
        p = self.last.get(30025)
        if p and len(p) >= 57:
            v = struct.unpack_from("<13fB", p, 0)
            s["iu"], s["iw"], s["vbus"] = v[2], v[4], v[6]
            s["iq_ref"] = v[1]
        q = self.last.get(30027)
        if q and len(q) >= 32:
            s["id"], s["iq"] = struct.unpack_from("<ff", q, 12)
        r = self.last.get(30040)
        if r and len(r) >= 8:
            s["enc"] = struct.unpack_from("<q", r, 0)[0]
        return s

    def take(self):
        out, self.ascii = self.ascii, []
        return out


def show(rows, tag):
    print()
    print("%s  (t=0 为发命令时刻，每行 1 ms)" % tag)
    print("=" * 70)
    print("   t(ms)   iq_ref[A]    id[A]     iq[A]     rpm    dEnc")
    print("-" * 70)
    if not rows:
        print("   (无帧)")
        return
    t0 = rows[0][0]
    enc0 = rows[0][5]
    printed = 0
    for (t, iqr, idn, iqn, om, enc) in rows:
        ms = (t - t0) * 1000.0
        if ms < -1:
            continue
        rpm = om / POLE_PAIRS * 60.0 / TWO_PI
        print("  %6.1f   %+8.3f  %+8.3f  %+8.3f  %+7.2f  %+7d"
              % (ms, iqr, idn, iqn, rpm, enc - enc0))
        printed += 1
        if printed >= 40:
            print("  ... (只显示前 40 帧)")
            break
    print("=" * 70)
    # 统计：最后 40% 样本视为稳态
    n = len(rows)
    tail = rows[int(n * 0.6):] if n >= 5 else rows
    if tail:
        iqr_m = sum(r[1] for r in tail) / len(tail)
        id_m = sum(r[2] for r in tail) / len(tail)
        iq_m = sum(r[3] for r in tail) / len(tail)
        err = (iq_m - iqr_m)
        rel = (abs(err) / abs(iqr_m) * 100.0) if abs(iqr_m) > 1e-6 else 0.0
        print("稳态(后40%%): iq_ref=%+.3f  id=%+.3f  iq=%+.3f  err=%+.3f A (%.1f%%)"
              % (iqr_m, id_m, iq_m, err, rel))


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.005)
    print("OPEN_OK %s @ %d" % (ser.name, BAUD))
    time.sleep(0.2)
    ser.reset_input_buffer()

    lk = Link(ser)
    head = []

    setup = [("C", 0.70, "cal"), ("X", 0.30, "clr"), ("Z", 12.5, "ident"), ("I", 0.40, "id_info")]
    try:
        for ch, wait, desc in setup:
            ser.write(ch.encode())
            ser.write(b"S")
            lk.pump(wait)
            lines = lk.take()
            pick = ""
            for ln in lines:
                if any(k in ln for k in ("[CMD] IDENT", "[CMD] id st", "[CMD] st en=")):
                    pick = ln
            if not pick and lines:
                pick = lines[-1]
            head.append((ch, desc, pick))

        # ---- +1 A ----
        lk.series = []
        ser.write(b"M")
        for _ in range(3):
            ser.write(b"S")
            lk.pump(0.50)
        rows_p = lk.series
        for ln in lk.take():
            if "[CMD] CUR" in ln or "[CMD] st en=" in ln:
                head.append(("M", "+1A", ln))

        # ---- -1 A ----
        lk.series = []
        ser.write(b"N")
        for _ in range(3):
            ser.write(b"S")
            lk.pump(0.50)
        rows_n = lk.series
        for ln in lk.take():
            if "[CMD] CUR" in ln or "[CMD] st en=" in ln:
                head.append(("N", "-1A", ln))
    finally:
        try:
            ser.write(b"D")
            lk.pump(0.40)
            for ln in lk.take():
                if "[CMD] OFF" in ln:
                    head.append(("D", "off", ln))
        except Exception:
            pass
        ser.close()

    print()
    for ch, desc, line in head:
        print("  %-3s %-9s %s" % (ch, desc, line))

    show(rows_p, "电流阶跃 +1.00 A（M）")
    show(rows_n, "电流阶跃 -1.00 A（N）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
