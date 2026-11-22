# -*- coding: utf-8 -*-
"""enc_rpm.py —— 用多圈计数（30040.enc_sum）独立核算转速，交叉验证 30025 的 omega 字段

为什么需要这一步：
  speed_test.py 看到 30 rpm 时转速读数在 -10..94 之间跳，但 iq 只有 0.8 A。
  0.8 A 对应 0.13 N·m，即使 J=1e-4 也只能让 30 rpm 的转子在 50 us 内改变 0.03 rad/s，
  绝无可能产生 ±100 rpm 的抖动。所以"跳"要么是测速链路的读数问题，要么是
  omega 字段本身有 bug —— 必须用一条独立的测量路径（位置差分）来判断。

两条路径：
  A) 30040[0..7] 多圈计数差分 / 2^21 / dt      -> 机械 rev/s（原始编码器，不经低通）
  B) 30025[44..47] / 14 -> 电角速度 / 极对数  -> 机械 rad/s（控制律实际用的量）
  C) 30025[20..23] 电角度单圈值的展开差分     -> 再算一遍转速（不依赖 30040 与低通）

时间基准：同一批 read 里的多帧共享一个 time.time()，直接用会把 dt 算成 0。
所以按"时间戳间隔 > 3 ms 才算新的一拍"去重，再差分。
"""
import math
import struct
import sys
import time

import serial

MSG_IN, LEN_IN, EXTRA_IN = 30025, 57, 191
MSG_POS, LEN_POS, EXTRA_POS = 30040, 12, 160
POLE_PAIRS = 14.0
COUNTS_PER_REV = 2097152.0
TWO_PI = 2.0 * math.pi


def x25(data, extra):
    crc = 0xFFFF

    def acc(c, b):
        t = (b ^ (c & 0xFF)) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        return ((c >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF

    for b in data:
        crc = acc(crc, b)
    return acc(crc, extra)


def dedup(seq, gap=0.003):
    """去掉同一批 read 产生的重复时间戳：只有时间前进超过 gap 才认为是新的一拍"""
    out = []
    for t, v in seq:
        if not out or (t - out[-1][0]) > gap:
            out.append((t, v))
    return out


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 1.2

    s = serial.Serial(port, 921600, timeout=0.1)
    buf = bytearray()
    pos, om, ang = [], [], []

    mode = sys.argv[3] if len(sys.argv) > 3 else "spd"
    s.reset_input_buffer()
    if mode == "vel":
        # 对照组：E 使能（位置模式）+ F 速度轨迹 3 rpm。
        # 这条路径此前实测过 2.86~3.25 rpm 且被认为"平滑"，
        # 用同一套 omega 字段采一遍就能判断测速链路本身是否可信。
        s.write(b"E"); s.flush(); time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b"F"); s.flush()
    else:
        s.write(b"W"); s.flush()
    t0 = time.time()
    while time.time() - t0 < secs:
        c = s.read(1024)
        if c:
            buf += c
        i = 0
        while i + 24 <= len(buf):
            if buf[i] != 0xFD:
                i += 1
                continue
            ln = buf[i + 1]
            mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            if (ln, mid) not in ((LEN_POS, MSG_POS), (LEN_IN, MSG_IN)):
                i += 1
                continue
            body = i + 10
            if body + ln + 2 > len(buf):
                break
            crc = buf[body + ln] | (buf[body + ln + 1] << 8)
            extra = EXTRA_POS if mid == MSG_POS else EXTRA_IN
            if x25(buf[i + 1:body + ln], extra) != crc:
                i += 1
                continue
            now = time.time()
            if mid == MSG_POS:
                pos.append((now, struct.unpack_from("<q", buf, body)[0]))
            else:
                om.append((now, struct.unpack_from("<f", buf, body + 44)[0]))
                ang.append((now, struct.unpack_from("<f", buf, body + 20)[0]))
            i = body + ln + 2
        del buf[:i]
    s.write(b"D")
    s.flush()
    s.close()

    pos = dedup(pos)
    om = dedup(om)
    ang = dedup(ang)
    print("去重后帧数：pos=%d omega=%d ang=%d" % (len(pos), len(om), len(ang)))

    def stats(name, vals, unit="rpm"):
        if not vals:
            print("  %s: 无数据" % name)
            return
        m = sum(vals) / len(vals)
        sd = math.sqrt(sum((x - m) ** 2 for x in vals) / len(vals))
        print("  %-22s n=%3d  min=%8.2f  max=%8.2f  mean=%8.2f  std=%7.2f %s"
              % (name, len(vals), min(vals), max(vals), m, sd, unit))
        return sd

    print("\n--- A) 多圈计数差分（原始编码器，无低通）---")
    pv, pd = [], []
    for k in range(1, len(pos) - 1):
        dt = pos[k + 1][0] - pos[k][0]
        if dt <= 0.002:
            continue
        d = pos[k + 1][1] - pos[k][1]
        pv.append(d / COUNTS_PER_REV / dt * 60.0)
        pd.append(d)
    for k in range(min(8, len(pd))):
        print("  d=%7d counts -> %.2f rpm" % (pd[k], pv[k]))
    sda = stats("位置差分", pv)

    print("\n--- B) 30025 omega 字段（控制律实际用的量，含 95 Hz 低通）---")
    ov = [x[1] / POLE_PAIRS * 60.0 / TWO_PI for x in om]
    sdb = stats("omega 字段", ov)
    if ov:
        print("  前 12 个：" + " ".join("%.1f" % x for x in ov[:12]))

    print("\n--- C) 电角度单圈展开后差分（不经 30040 也不经低通）---")
    av = []
    unwrapped = []
    for k in range(len(ang)):
        a = ang[k][1]
        if k == 0:
            unwrapped.append(a)
            continue
        d = a - ang[k - 1][1]
        while d > math.pi:
            d -= TWO_PI
        while d < -math.pi:
            d += TWO_PI
        unwrapped.append(unwrapped[-1] + d)
    for k in range(1, len(ang) - 1):
        dt = ang[k + 1][0] - ang[k][0]
        if dt <= 0.002:
            continue
        de = unwrapped[k + 1] - unwrapped[k]          # 电角 rad
        av.append(de / POLE_PAIRS / dt / TWO_PI * 60.0)
    stats("电角度差分", av)

    print("\n--- 判读 ---")
    if sda is not None and sdb is not None:
        print("  A 位置差分 std=%.2f rpm ，B omega 字段 std=%.2f rpm" % (sda, sdb))
        if sda < 3.0 and sdb > 10.0:
            print("  -> 真实转速平稳，是 omega 字段（测速链路）有问题")
        elif sda > 10.0:
            print("  -> 真实转速也在跳：控制/机械侧问题（不是测速）")
        else:
            print("  -> 两条路径一致，转速读数可信")


if __name__ == "__main__":
    main()
