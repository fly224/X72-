# -*- coding: utf-8 -*-
"""MIT 模式下真正"跑电机"：正弦位置轨迹跟踪。

两个测量上的讲究：
  1. **误差必须按位置帧自己的时间戳算**。遥测 100 Hz，而轨迹 0.5 Hz、
     峰值速度 3.14 rad/s —— 拿"当前时刻的 p_des"去比"5 ms 前采到的位置"，
     会凭空多出 16 mrad 的假误差，和真实误差同量级。
  2. 收尾要**连续发帧**。只发一帧的话，万一那帧丢了，最后一帧的 v_des
     会留在 kd 项里继续推电机（这也是固件里加 20 ms 命令超时的原因）。

用法:
    python mit_run.py COM27 --amp 1.0 --freq 0.5 --secs 8 --kp 10 --kd 0.5
"""
import argparse
import math
import struct
import time

from cog_ff_test import Rig, x25
from mit_test import pack, cmd_line

MSG_POS, LEN_POS, CRC_POS = 30040, 12, 160
MSG_OUT, LEN_OUT, CRC_OUT = 30027, 32, 58
ENC_REV = 2097152.0            # 2^21 counts/rev


def scan(buf, out):
    """从字节流里抠出 30040 / 30027 帧，返回已消费到的位置"""
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
        if not ((mid == MSG_POS and ln == LEN_POS) or (mid == MSG_OUT and ln == LEN_OUT)):
            i += 1
            continue
        crc = buf[b + ln] | (buf[b + ln + 1] << 8)
        ex = CRC_POS if mid == MSG_POS else CRC_OUT
        if x25(buf[i + 1:b + ln], ex) != crc:
            i += 1
            continue
        if mid == MSG_POS:
            out.append(("pos", struct.unpack_from("<q", buf, b)[0]))
        else:
            out.append(("iq", struct.unpack_from("<f", buf, b + 16)[0]))
        i = b + ln + 2
    return i


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--amp", type=float, default=1.0, help="正弦幅值 rad")
    ap.add_argument("--freq", type=float, default=0.5, help="Hz")
    ap.add_argument("--secs", type=float, default=8.0)
    ap.add_argument("--kp", type=float, default=30.0,
                    help="位置刚度 N·m/rad。默认 30：精度已见底（kp>20 被摩擦/测量噪声占住），"
                         "再大只增出力不增精度")
    ap.add_argument("--kd", type=float, default=0.05,
                    help="阻尼 N·m·s/rad。上限 0.2；0.5 会在 244 Hz 起极限环、iq 顶 5 A 限幅。"
                         "判据 kd/J ≪ 电流环带宽（J≈5.5e-5，1 kHz）")
    ap.add_argument("--rate", type=float, default=500.0)
    a = ap.parse_args()

    rig = Rig(a.port)
    dt = 1.0 / a.rate
    try:
        time.sleep(2.0)
        rig.cmd("D", 0.5)
        rig.cmd("X", 0.5)

        # --- 使能：kp=0 一帧，读出 p0 与 enc0 ---
        rig.s.write(pack(0.0, 0.0, 0.0, 0.08, 0.0))
        rig.s.flush()
        time.sleep(0.4)
        cl = cmd_line(rig.cmd("S", 0.8))
        if " p=" not in cl:
            print("!! 没拿到 MIT 位置，S =", cl[-60:])
            return
        p0 = float(cl.split(" p=")[1].split()[0])

        rig.s.reset_input_buffer()
        enc0 = None
        buf = bytearray()
        got = []
        t = time.time()
        while time.time() - t < 0.5 and enc0 is None:
            buf += rig.s.read(4096)
            got.clear()
            k = scan(buf, got)
            del buf[:k]
            for kind, v in got:
                if kind == "pos":
                    enc0 = v
        if enc0 is None:
            print("!! 没拿到编码器多圈计数")
            return
        print("p0 = %.4f rad  enc0 = %d  (kp=%.0f kd=%.2f amp=%.2f f=%.2f)"
              % (p0, enc0, a.kp, a.kd, a.amp, a.freq))

        # --- 正弦轨迹 ---
        w = 2.0 * math.pi * a.freq
        t0 = time.time()
        nxt = t0
        pos_log = []          # (t_recv, p_meas)
        iq_log = []           # (t_recv, iq)
        buf = bytearray()
        frames = 0
        while True:
            now = time.time() - t0
            if now > a.secs:
                break
            pd = p0 + a.amp * math.sin(w * now)
            vd = a.amp * w * math.cos(w * now)
            rig.s.write(pack(pd, vd, a.kp, a.kd, 0.0))
            frames += 1

            d = rig.s.read(max(1, rig.s.in_waiting))
            if d:
                buf += d
                got.clear()
                k = scan(buf, got)
                del buf[:k]
                for kind, v in got:
                    if kind == "pos":
                        pos_log.append((now, p0 + (v - enc0) * 2.0 * math.pi / ENC_REV))
                    else:
                        iq_log.append((now, v))

            nxt += dt
            slp = nxt - time.time()
            if slp > 0:
                time.sleep(slp)

        # --- 收尾：连续发 0.3 s 保持帧 ---
        for _ in range(int(0.3 * a.rate)):
            rig.s.write(pack(p0 + a.amp * math.sin(w * a.secs), 0.0, a.kp, a.kd, 0.0))
            rig.s.flush()
            time.sleep(dt)
        time.sleep(0.2)

        # --- 误差按各帧自己的时间戳算 ---
        errs = []
        for t_recv, p in pos_log:
            if t_recv < 0.3:
                continue
            pd = p0 + a.amp * math.sin(w * t_recv)
            errs.append(pd - p)
        if not errs:
            print("!! 采样不足（pos 帧 %d 个）" % len(pos_log))
            return
        ae = sorted(abs(x) for x in errs)
        rms = math.sqrt(sum(x * x for x in errs) / len(errs))
        ps = [p for _, p in pos_log]
        print("位置帧 %d 个，有效误差样本 %d" % (len(pos_log), len(errs)))
        print("跟踪误差  RMS %.4f rad  |e|max %.4f  |e|中位 %.4f"
              % (rms, ae[-1], ae[len(ae) // 2]))
        print("行程 %.3f rad（目标峰峰 %.3f）  |iq|峰值 %.3f A"
              % (max(ps) - min(ps), 2 * a.amp,
                 max(abs(v) for _, v in iq_log) if iq_log else 0.0))

        cl = cmd_line(rig.cmd("S", 0.8))
        print("S:", cl[-52:])
    finally:
        rig.cmd("D", 0.4)
        rig.close()


if __name__ == "__main__":
    main()
