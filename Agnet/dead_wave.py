# -*- coding: utf-8 -*-
"""dead_wave.py —— 相电流按电角度周期同步平均，直接看过零段有没有死区平台

为什么这么做：
  死区效应在稳态 iq 频谱上看不出来（见 死区补偿_标定与启用.md 第 2 节的纠正），
  它伤害的是"相电流过零那几十微秒"。要看到它必须：
    1) 采 abc 原始相电流（不是 dq）
    2) 同拍采电角度，按角度分箱平均 —— 靠时间反推相位不行，
       100 rpm 的转速有 9% 波动，几个周期就糊了
    3) 多周期叠加降噪：1024 点 / 12 周期 ≈ 85 点每周期，每 5° 箱约 1.2 点，
       叠 3 次才勉强可用

判据：
  正常正弦：过零处斜率最大、连续穿越
  死区平台：过零附近连续几个角度箱电流都贴近 0（电流"钉住"），
            或者正负半周不对称
  定量指标：|iu| < 2% 峰值的角度箱数（正弦理论值 ≈ 1~2 个 5° 箱）

用法:
    python dead_wave.py COM27 --rpm 100 --div 10
    python dead_wave.py COM27 --rpm 100 --div 10 --dus 0,59 --rep 3
"""
import argparse
import math
import re
import time

from cog_ff_test import Rig
from sweep_1_1000 import set_rpm
from torque_test import send_num

TWO_PI = 2.0 * math.pi
SCALE_I = 131.0


def grab_rec2(ser, timeout=8.0):
    """发 `&`，解析 [REC2]（每点 iu 4hex + theta 4hex）"""
    ser.reset_input_buffer()
    ser.write(b"&")
    ser.flush()
    raw = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = ser.read(4096)
        if d:
            raw += d
        if b"REC end" in raw:
            break

    pts = []
    for m in re.finditer(rb"\[REC2\]((?:[0-9a-f]{4})+)", bytes(raw)):
        body = m.group(1)
        for k in range(0, len(body), 8):
            if k + 8 > len(body):
                break
            u = int(body[k:k + 4], 16)
            iu = (u - 0x10000 if u >= 0x8000 else u) / SCALE_I
            th = int(body[k + 4:k + 8], 16) / 65536.0 * TWO_PI
            pts.append((iu, th))
    return pts


def bin_wave(pts, nbins):
    """按电角度分箱取**中位数**（不是均值）。

    均值会被单个尖峰抬起来：实测出现过某 bin 均值 0.48 A，而稳态相电流
    峰值只有 0.15 A —— 一个坏采样就能把整箱拉偏。中位数对这种离群值免疫。
    """
    buckets = [[] for _ in range(nbins)]
    for iu, th in pts:
        b = int(th / TWO_PI * nbins) % nbins
        buckets[b].append(iu)
    wave = []
    cnt = []
    for bk in buckets:
        if bk:
            bk.sort()
            wave.append(bk[len(bk) // 2])
            cnt.append(len(bk))
        else:
            wave.append(0.0)
            cnt.append(0)
    return wave, cnt


def zero_metrics(wave, nbins):
    """过零段指标。

    flat     ：|i| 小于 2% 峰值的角度箱数（正弦理论 ≈1，死区平台会变大）
    cross    ：过零段电流符号翻转次数（正弦 =1 次干净穿越；
               死区平台会在 0 附近来回，翻转多次）
    slope    ：过零段相邻箱的平均跳变（越大越连续）
    """
    amp = max(abs(v) for v in wave) or 1.0
    thr = 0.02 * amp
    flat = sum(1 for v in wave if abs(v) < thr)

    half = max(4, nbins // 8)
    seg = [wave[(i) % nbins] for i in range(-half, half + 1)]

    band = 0.01 * amp
    cross = 0
    prev = 0
    for v in seg:
        s = 1 if v > band else (-1 if v < -band else 0)
        if s != 0:
            if (prev != 0) and (s != prev):
                cross += 1
            prev = s

    sl = []
    for i in range(nbins):
        if abs(wave[i]) < 0.25 * amp:
            sl.append(abs(wave[(i + 1) % nbins] - wave[i]))
    return dict(amp=amp, flat=flat, thr=thr, cross=cross,
                slope=(sum(sl) / len(sl) if sl else 0.0),
                seg=seg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--rpm", type=float, default=100.0)
    ap.add_argument("--div", type=int, default=10, help="录制分频（10=2kHz/512ms/12周期）")
    ap.add_argument("--dus", default="0,59")
    ap.add_argument("--rep", type=int, default=3)
    ap.add_argument("--bins", type=int, default=72, help="每电周期分箱数（72=5°）")
    a = ap.parse_args()

    dus = [float(x) for x in a.dus.split(",")]
    fs = 20000.0 / a.div
    f_elec = a.rpm * 14.0 / 60.0
    win = 51.2 * a.div
    print("转速 %.0f rpm  电频率 %.2f Hz  录制 fs=%.0f Hz 窗口 %.0f ms（%.1f 周期）"
          % (a.rpm, f_elec, fs, win, win / 1000.0 * f_elec))

    rig = Rig(a.port)
    res = {}
    try:
        print("S:", rig.cmd("S", 0.6))
        # div + 通道编码：100+div = 相电流+角度
        print("  ", send_num(rig, "'", 100 + a.div, dec=0))
        print(set_rpm(rig, a.rpm))
        rig.keep(2.0)

        for du in dus:
            print("\n  ", send_num(rig, "\\", du, dec=0))
            time.sleep(0.8)
            allp = []
            for _ in range(a.rep):
                p = grab_rec2(rig.s)
                allp.extend(p)
                time.sleep(0.2)
            if len(allp) < 256:
                print("   %6.0f 采集不足 %d 点" % (du, len(allp)))
                continue
            wave, cnt = bin_wave(allp, a.bins)
            m = zero_metrics(wave, a.bins)
            res[du] = (wave, m, len(allp))
            print("   du=%3.0f  n=%5d  峰值 %.4f A  平台箱 %d  过零翻转 %d 次  "
                  "斜率 %.4f"
                  % (du, len(allp), m["amp"], m["flat"], m["cross"], m["slope"]))
            # 角度分布诊断：各箱样本数应该均匀。若极不均匀，
            # 说明角度编码/映射有问题，后面所有幅值都不可信。
            nz = [c for c in cnt if c > 0]
            print("        箱样本数 min=%d max=%d 均值=%.1f 空箱=%d/%d"
                  % (min(nz) if nz else 0, max(nz) if nz else 0,
                     len(allp) / a.bins, a.bins - len(nz), a.bins))

        rig.cmd("D", 0.3)

        # 打印过零段 + 完整波形
        for du in res:
            wave, m = res[du][0], res[du][1]
            n = a.bins
            print("\n  du=%.0f 过零段(θ≈0, 每箱 %.1f°)：平台 %d 箱  翻转 %d 次"
                  % (du, 360.0 / n, m["flat"], m["cross"]))
            print("   ", " ".join("%+.3f" % v for v in m["seg"]))
            print("   完整波形（%d 箱，每箱 %.1f°）：" % (n, 360.0 / n))
            for k in range(0, n, 12):
                print("   ", " ".join("%+.3f" % v for v in wave[k:k + 12]))

        if len(res) == 2:
            d0, d1 = dus[0], dus[1]
            print("\n=== 对比 ===")
            print("  过零平台箱数:  du=%.0f → %d   du=%.0f → %d"
                  % (d0, res[d0][1]["flat"], d1, res[d1][1]["flat"]))
            print("  过零斜率:      du=%.0f → %.4f  du=%.0f → %.4f"
                  % (d0, res[d0][1]["slope"], d1, res[d1][1]["slope"]))
            print("  （正弦理论：|i|<2%%峰值的箱数 ≈ %d；斜率越大说明过零越连续）"
                  % max(1, int(round(2 * a.bins * math.asin(0.02) / math.pi))))
        return 0
    finally:
        rig.cmd("D", 0.3)
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
