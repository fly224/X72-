# -*- coding: utf-8 -*-
"""param_probe.py —— 单次参数辨识快照（@ 在线 R/L + + 联合 ψ/R/L）

为什么单独一个脚本：10 分钟正弦漂移测试要在"冷态 / 热态"各取一次快照，
必须能一次调用拿到全部量（R/L/ψ/Kt/r2/dev + 温度 + 母线），且不依赖交互。

用法：
    python param_probe.py                 # COM27，做 @ 与 +
    python param_probe.py --fast          # 只做 @（0.4 s，用于中途插测）
    python param_probe.py --tag cold      # 结果写进 param_snapshots.csv 并加标签
    python param_probe.py COM5 --tag hot

注意（踩过的坑）：
  · @ 与 + 都是单缓冲回显，辨识期间**不能发任何其它命令**（连 S 都会把结果行顶掉）；
  · + 要转到 1000 rpm 且约 9 s，必须等够 14 s 再收尾；
  · 两个命令都要求 s_armed=0，所以必须先 D。
"""
import argparse
import csv
import os
import re
import sys
import time

import serial

BAUD = 921600
CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), "param_snapshots.csv")


def cmd(ser, c, wait=0.35, quiet=False):
    ser.reset_input_buffer()
    ser.write(c.encode())
    ser.flush()
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < wait:
        d = ser.read(256)
        if d:
            buf += d
            i = buf.rfind(b"[CMD]")
            if i >= 0 and b"\r\n" in buf[i:]:
                break
    i = buf.rfind(b"[CMD]")
    line = bytes(buf[i:]).split(b"\r\n")[0].decode("ascii", "replace") if i >= 0 else "(no echo)"
    if not quiet:
        print("    " + line)
    return line


def collect(ser, secs, stop_words):
    """收全部回显直到超时或命中停止词。辨识期间绝不插入其它命令。"""
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < secs:
        d = ser.read(512)
        if d:
            buf += d
            txt = bytes(buf)
            if any(w.encode() in txt for w in stop_words):
                break
    return bytes(buf).decode("ascii", "replace")


def num(pat, txt, cast=float):
    m = re.search(pat, txt)
    return cast(m.group(1)) if m else None


def field(line, key):
    m = re.search(r"\b" + key + r"=(-?[\w.]+)", line)
    return m.group(1) if m else None


def read_status(ser):
    line = cmd(ser, "S", 0.4, quiet=True)
    d = {
        "en": field(line, "en"),
        "moe": field(line, "moe"),
        "cal": field(line, "cal"),
        "z": field(line, "z"),
        "fault": field(line, "f"),
        "vbus": field(line, "v"),
    }
    return line, d


def probe(ser, do_flux=True, tag=""):
    out = {"tag": tag, "t": time.strftime("%H:%M:%S")}

    print("\n[1] 停机并清故障")
    cmd(ser, "D", 0.3)
    st_line, st = read_status(ser)
    print("    " + st_line)
    if st["fault"] not in (None, "0x00"):
        print("    [WARN] 有故障位，先发 X")
        cmd(ser, "X", 0.4)
        st_line, st = read_status(ser)
        print("    " + st_line)
    out.update({"vbus": st["vbus"], "fault_before": st["fault"]})

    if st["cal"] != "1":
        print("\n[2] 电流零点标定 C")
        cmd(ser, "C", 0.5)
        time.sleep(0.6)
    if st["z"] != "1":
        print("\n[3] 电角度对齐 Z（约 9 s，转子会转）")
        cmd(ser, "Z", 0.3)
        for _ in range(16):
            time.sleep(1.0)
            ln, s2 = read_status(ser)
            if s2["z"] == "1":
                print("    " + ln)
                break
        else:
            print("    [FAIL] Z 未完成")
            return out

    print("\n[4] @ 在线辨识 R/L（约 0.4 s）")
    ser.reset_input_buffer()
    ser.write(b"@")
    ser.flush()
    t0 = time.time()
    txt = collect(ser, 3.0, ["RL ok", "RL FAIL"])
    print("    " + " / ".join([l.strip() for l in txt.splitlines() if "RL " in l][-1:]) or "    (无结果)")
    out["R_mohm"] = num(r"\bR=([\d.]+)mohm", txt)
    out["L_uH_at"] = num(r"\bL=([\d.]+)uH", txt)
    out["rl_pts"] = num(r"\bpts=(\d+)", txt, int)
    out["rl_spread"] = num(r"\bspread=([\d.]+)", txt)
    out["rl_ok"] = 1 if "RL ok" in txt else 0

    if do_flux:
        print("\n[5] + 联合辨识 ψ/R/L（1000 rpm，约 9 s）")
        ser.reset_input_buffer()
        ser.write(b"+")
        ser.flush()
        t0 = time.time()
        txt = collect(ser, 16.0, ["FI Kt="])
        ok = "FI R=" in txt and "FI Kt=" in txt
        out["fl_ok"] = 1 if ok else 0
        out["R_mohm_fl"] = num(r"FI R=([\d.]+)mohm", txt)
        out["L_uH"] = num(r"mohm L=([\d.]+)uH", txt)
        out["psi_mWb"] = num(r"uH psi=([\d.]+)mWb", txt)
        out["Kt"] = num(r"FI Kt=([\d.]+)", txt)
        out["Ke"] = num(r"Ke=([\d.]+)", txt)
        out["r2"] = num(r"r2=([\d.]+)/", txt)
        out["dev_pct"] = num(r"dev=([\d.]+)%", txt)
        for ln in txt.splitlines():
            if "FI R=" in ln or "FI Kt=" in ln or "FI FAIL" in ln:
                print("    " + ln.strip())
        if not ok:
            print("    [WARN] ψ 辨识未给完整结果（约 %.0f s）" % (time.time() - t0))

    st_line, st = read_status(ser)
    out["temp_gan"] = num(r"t=([-\d.na]+)/", "t=" + (field(st_line, "t") or ""))
    # S 行不含温度：温度走 30025 的 driver_temp 字段，测试脚本另取
    out["vbus_after"] = st["vbus"]
    out["fault_after"] = st["fault"]
    cmd(ser, "D", 0.25, quiet=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--fast", action="store_true", help="只做 @，不做 +")
    ap.add_argument("--tag", default="")
    a = ap.parse_args()

    ser = serial.Serial(a.port, BAUD, timeout=0.1)
    print("=" * 70)
    print("参数辨识快照   port=%s  tag=%s  flux=%s" % (a.port, a.tag or "-", not a.fast))
    print("=" * 70)

    out = probe(ser, do_flux=not a.fast, tag=a.tag)
    ser.close()

    print("\n" + "-" * 70)
    for k in ("tag", "t", "vbus", "R_mohm", "L_uH_at", "R_mohm_fl", "L_uH", "psi_mWb",
              "Kt", "Ke", "r2", "dev_pct", "rl_spread", "fault_after"):
        if out.get(k) is not None:
            print("  %-12s %s" % (k, out[k]))

    if a.tag:
        new = not os.path.exists(CSV)
        cols = ["tag", "t", "vbus", "R_mohm", "L_uH_at", "R_mohm_fl", "L_uH", "psi_mWb",
                "Kt", "Ke", "r2", "dev_pct", "rl_spread", "rl_ok", "fl_ok", "fault_after"]
        with open(CSV, "a", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            if new:
                w.writeheader()
            w.writerow(out)
        print("\n  -> 追加到 %s" % CSV)


if __name__ == "__main__":
    sys.exit(main())
