#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""位置环测试：保持稳定性 + 点对点定位（手写链路，2026-11-17）。

用法：
  python pos_test.py COM27 --kind hold --secs 3
  python pos_test.py COM27 --kind move --delta 0.5 --secs 4
  python pos_test.py COM27 --kind move --delta -3.14 --kp 40

位置来源是 30040 帧（enc_sum int64，2^21/圈），100 Hz，不受 ASCII 单槽影响。
"""
import argparse
import re
import math
import struct
import time

from cog_ff_test import Rig, x25

MSG_POS, LEN_POS, CRC_POS = 30040, 12, 160
ENC_REV = 2097152.0
TWO_PI = 2.0 * math.pi


def crc_x25(data, crc=0xFFFF):
    """保留占位：实际用 cog_ff_test.x25（它会把 crc_extra 并入）。
    MAVLink 的校验必须累加 crc_extra，漏了会一帧都匹配不上 —— 这里踩过一次。"""
    return x25(data, CRC_POS)


def ask(rig, txt, wait=0.7):
    """发命令取最后一条 [CMD] 行。
    不能用 Rig.cmd：它在二进制遥测流里 rfind(b"[CMD]") 会切到半行。"""
    rig.s.reset_input_buffer()
    rig.s.write(txt.encode() if isinstance(txt, str) else txt)
    rig.s.flush()
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < wait:
        d = rig.s.read(1024)
        if d:
            buf += d
    s = "".join(c if 32 <= ord(c) < 127 else "\n" for c in bytes(buf).decode("ascii", "replace"))
    lines = [ln.strip() for ln in s.split("\n") if ln.strip().startswith("[CMD]")]
    return lines[-1] if lines else "(no echo)"


def scan_pos(buf, out):
    """从字节流里抠 30040 帧，返回已消费到的位置"""
    i = 0
    n = len(buf)
    while i + 12 <= n:
        if buf[i] != 0xFD:
            i += 1
            continue
        ln = buf[i + 1]
        mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        b = i + 10
        if b + ln + 2 > n:
            break
        if mid != MSG_POS or ln != LEN_POS:
            i += 1
            continue
        crc = buf[b + ln] | (buf[b + ln + 1] << 8)
        if crc_x25(buf[i + 1:b + ln], 0xFFFF) != crc:
            i += 1
            continue
        enc = struct.unpack_from("<q", buf, b)[0]
        out.append(enc / ENC_REV * TWO_PI)
        i = b + ln + 2
    return i


def collect(rig, secs):
    """采位置序列 (t, rad)，期间保活"""
    pts = []
    t0 = time.time()
    nxt = t0 + 0.8
    rig.s.reset_input_buffer()
    del rig.raw[:]
    while time.time() - t0 < secs:
        d = rig.s.read(4096)
        now = time.time() - t0
        if d:
            rig.raw += d
        i = scan_pos(rig.raw, pts)
        del rig.raw[:i]
        if now > 0.8 and time.time() > nxt:
            rig.s.write(b"S")
            rig.s.flush()
            nxt += 0.8
    # 时间戳用均匀分配（帧率 100 Hz，实测 115 Hz 帧率有抖动）
    n = len(pts)
    return pts, n


def stamp(pts, fs=100.0):
    return [(k / fs, v) for k, v in enumerate(pts)]


def stats(v):
    m = sum(v) / len(v)
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
    return m, sd, min(v), max(v)


def metrics(ts, p0, target):
    """上升时间 / 超调 / 到位时间 / 稳态误差"""
    if not ts:
        return {}
    v = [p[1] for p in ts]
    d = target - p0
    if abs(d) < 1e-6:
        return {}
    sgn = 1.0 if d > 0 else -1.0
    # 归一化进度
    prog = [(x - p0) / d for x in v]
    t10 = t90 = None
    for (t, _), g in zip(ts, prog):
        if t10 is None and g >= 0.1: t10 = t
        if t90 is None and g >= 0.9: t90 = t
    # prog = (x − p0)/d 恒为 0→1 的正向进度（d 带符号，负行程也一样），
    # 所以超调一律取 max(prog)−1，不要按 sgn 分支（那样反向会把起点当超调）
    os_pct = (max(prog) - 1.0) * 100.0
    # 到位：进入 ±2% 且不再离开
    band = 0.02
    t_set = None
    for k in range(len(prog)):
        if all(abs(g - 1.0) < band for g in prog[k:]):
            t_set = ts[k][0]
            break
    # 速度（差分）
    vmax = 0.0
    for k in range(1, len(v)):
        dv = (v[k] - v[k - 1]) / max(1e-6, ts[k][0] - ts[k - 1][0])
        vmax = max(vmax, abs(dv))
    return dict(tr=(t90 - t10) * 1000.0 if (t10 is not None and t90 is not None) else float("nan"),
                os=os_pct, ts_ms=t_set * 1000.0 if t_set is not None else float("nan"),
                ss=v[-1], err=(v[-1] - target), vmax=vmax, n=len(v))


def do_hold(rig, secs):
    print("[1] P + CR -> 保持当前位置")
    print("   ", ask(rig, b"P\r", 0.7))

    pts, n = collect(rig, secs)
    if len(pts) < 10:
        print("   采样不足 n=%d" % len(pts))
        return
    m, sd, lo, hi = stats(pts)
    print("   位置 mean=%.6f rad  sd=%.6f  ppk=%.6f  (n=%d)" % (m, sd, hi - lo, len(pts)))
    print("   sd 折合 %.4f deg，ppk 折合 %.4f deg" % (sd * 57.2958, (hi - lo) * 57.2958))


def do_move(rig, delta, secs, hold_first=True):
    # 先保持，从回显里拿**控制器坐标系**的起点。
    # 不能拿 30040 的 enc_sum 当基准：它是"相对上电点"的累积量，
    # 而控制器用 BspServo_GetThetaMulti()（带绝对零点），两者差一个常量。
    # 混用会让目标算到 4.5 rad 之外 —— 实测直接满速反向跑掉 4 rad。
    resp = ask(rig, b"P\r", 0.8)
    print("   hold:", resp)
    m = re.search(r"tgt=(-?\d+\.\d+)", resp)
    if not m:
        print("   拿不到控制器起点，退出")
        return {}
    p0_ctl = float(m.group(1))

    pts, _ = collect(rig, 0.5)
    if not pts:
        print("   拿不到初始位置")
        return {}
    p0_enc = pts[-1]              # enc_sum 坐标，仅用于轨迹分析
    target_ctl = p0_ctl + delta   # 发给控制器的目标（控制器坐标）
    print("[2] 定位：ctl %.4f -> %.4f rad (delta=%+.4f = %+.2f deg)"
          % (p0_ctl, target_ctl, delta, delta * 57.2958))

    rig.s.reset_input_buffer()
    rig.s.write(b"P")
    rig.s.flush()
    time.sleep(0.05)
    for ch in ("%.4f" % target_ctl):
        rig.s.write(ch.encode())
        rig.s.flush()
        time.sleep(0.010)
    rig.s.write(b"\r")
    rig.s.flush()

    pts, n = collect(rig, secs)
    ts = stamp(pts)
    mt = metrics(ts, p0_enc, p0_enc + delta)
    if not mt:
        print("   采样不足 n=%d" % len(pts))
        return {}
    print("   tr(10-90%%) = %6.1f ms   os = %+6.2f %%   到位(±2%%) = %6.1f ms"
          % (mt["tr"], mt["os"], mt["ts_ms"]))
    print("   终点 %.6f rad  误差 %+.6f rad (%+.4f deg)  峰值速度 %.2f rad/s"
          % (mt["ss"], mt["err"], mt["err"] * 57.2958, mt["vmax"]))
    tail = pts[-20:] if len(pts) >= 20 else pts
    mm, sdd, _, _ = stats(tail)
    print("   末段 %d 帧: mean=%.4f sd=%.6f rad (%.4f deg)" % (len(tail), mm, sdd, sdd * 57.2958))
    k = max(1, len(ts) // 24)
    print("   轨迹:", " ".join("%.3f" % p[1] for p in ts[::k][:24]))
    return mt


def read_cmd(rig, wait=0.6):
    """只读回显，**不清输入缓冲** —— 用于命令刚发完、回显已经在路上的场景。
    ask() 会先 reset_input_buffer，那种场景下会把回显直接冲掉。"""
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < wait:
        d = rig.s.read(1024)
        if d:
            buf += d
    s = "".join(c if 32 <= ord(c) < 127 else "\n" for c in bytes(buf).decode("ascii", "replace"))
    lines = [ln.strip() for ln in s.split("\n") if ln.strip().startswith("[CMD]")]
    return lines[-1] if lines else "(no echo)"


def set_kp(rig, kp):
    rig.s.reset_input_buffer()
    rig.s.write(b"`")
    rig.s.flush()
    time.sleep(0.06)
    for ch in ("%.2f" % kp):
        rig.s.write(ch.encode())
        rig.s.flush()
        time.sleep(0.010)
    rig.s.write(b"\r")
    rig.s.flush()
    return read_cmd(rig, 0.8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--kind", choices=["hold", "move"], default="move")
    ap.add_argument("--delta", type=float, default=0.5, help="相对当前位置的目标（rad）")
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--kp", type=float, default=None)
    a = ap.parse_args()

    rig = Rig(a.port)
    try:
        time.sleep(2.0)
        st = "(no echo)"
        for _ in range(5):
            st = ask(rig, "S", 0.8)
            if st != "(no echo)":
                break
            time.sleep(0.3)
        print("S:", st[:170])
        if "cal=1" not in st:
            print("!! 电流未标定，先发 C")
            return
        if "z=1" not in st:
            print("!! 电角度未对齐，先发 Z")
            return
        if a.kp is not None:
            print("   Kp ->", set_kp(rig, a.kp))
        if a.kind == "hold":
            do_hold(rig, a.secs)
        else:
            do_move(rig, a.delta, a.secs)
        print("--- 退出 ---")
        ask(rig, "D", 0.6)
    finally:
        try:
            ask(rig, "D", 0.6)
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    main()
