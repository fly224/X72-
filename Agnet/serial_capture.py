#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_capture.py —— 后台抓包工具（只读，不下发任何字节）

用途：
    横幅只在复位后发一次，普通"连上去读 3 秒"只能看到稳态的二进制流。
    这个脚本先占住串口持续抓 N 秒，把原始字节落盘，事后可反复分析：
    到底收到了 ASCII 横幅、二进制帧，还是什么都没收到。

用法：
    python serial_capture.py -p COM27 -t 40 -o capture.bin
"""
import argparse
import os
import sys
import time

try:
    import serial
except ImportError:
    print("ERR 缺少 pyserial：python -m pip install pyserial")
    sys.exit(2)


def summarize(raw, baud):
    out = []
    out.append("BYTES %d in %.1f s (%.1f B/s)" % (len(raw), raw and 1.0 or 0.0, 0.0))
    if not raw:
        out.append("EMPTY 一个字节都没收到")
        return "\n".join(out)

    # ASCII 可读部分
    txt = raw.decode("ascii", "replace")
    lines = [ln.strip() for ln in txt.replace("\r", "\n").split("\n")]
    lines = [ln for ln in lines if ln and sum(ch.isprintable() for ch in ln) >= len(ln) * 0.8]
    if lines:
        out.append("ASCII_LINES %d" % len(lines))
        for ln in lines[:20]:
            out.append("  | " + ln)

    # MAVLink v2 帧统计
    n, i, bad, ids = 0, 0, 0, {}
    while i < len(raw):
        if raw[i] != 0xFD:
            i += 1
            continue
        if i + 10 > len(raw):
            break
        ln = raw[i + 1]
        end = i + 10 + ln + 2
        if end > len(raw):
            break
        mid = raw[i + 7] | (raw[i + 8] << 8) | (raw[i + 9] << 16)
        ids[mid] = ids.get(mid, 0) + 1
        n += 1
        i = end
    out.append("MAVLINK_FRAMES %d  bad_gap %d  ids %s" % (n, bad, ids))
    out.append("HEAD_HEX %s" % raw[:24].hex(" "))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default="COM27")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("-t", "--time", type=float, default=40.0)
    ap.add_argument("-o", "--out", default="capture.bin")
    a = ap.parse_args()

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.1)
    except Exception as e:
        print("OPEN_FAIL %s: %s" % (type(e).__name__, e))
        return 1
    print("OPEN_OK %s @ %d  抓 %.0f s -> %s" % (ser.name, a.baud, a.time, a.out))
    sys.stdout.flush()

    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < a.time:
        chunk = ser.read(8192)
        if chunk:
            buf += chunk
    ser.close()

    with open(a.out, "wb") as f:
        f.write(buf)

    print("---- SUMMARY ----")
    print(summarize(bytes(buf), a.baud))
    return 0


if __name__ == "__main__":
    sys.exit(main())
