# -*- coding: utf-8 -*-
"""转速纹波-转速扫描：判断波动是"环自激"还是"转速同步的机械/编码器扰动"

判据：
  · 波动频率与转速无关（固定 ~4 Hz）  → 环路极限环 / 外部固定扰动
  · 波动频率 ∝ 转速（每转整数次）      → 齿槽、偏心、编码器、联轴器
  · 波动幅值随 kp 变化                → 速度环增益相关

用法: python spd_sweep.py 600 1000 1400 1800 2000
"""
import sys, time, math, struct, csv
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, 'G:/FOC/X72/Agnet')
from spd_ripple import (Link, MSG_ST, MSG_LP, MSG_PS, MSG_MODE, X_PS, X_MODE,
                        f32, pack_speed)

PORT = 'COM27'


def stats(v):
    m = sum(v) / len(v)
    return m, max(v) - min(v), math.sqrt(sum((x - m) ** 2 for x in v) / len(v))


def main_freq(ts, vs):
    """自相关估主周期（0.05~1.0 s），比 DFT 对非正弦稳"""
    fs = 200.0
    dt = 1.0 / fs
    t0, t1 = ts[0], ts[-1]
    n = int((t1 - t0) / dt)
    if n < 64:
        return 0.0, 0.0
    y = []
    j = 0
    for k in range(n):
        t = t0 + k * dt
        while j < len(ts) - 2 and ts[j + 1] < t:
            j += 1
        y.append(vs[j])
    m = sum(y) / n
    y = [x - m for x in y]
    var = sum(x * x for x in y)

    def acf(lag):
        a = y[:n - lag]
        b = y[lag:]
        return sum(p * q for p, q in zip(a, b)) / var if var > 0 else 0.0

    best = (0, 0.0)
    for lag in range(int(0.05 * fs), min(int(0.80 * fs), n // 2)):
        c = acf(lag)
        if c > best[1]:
            best = (lag, c)
    if best[0] == 0:
        return 0.0, 0.0
    return fs / best[0], best[1]


lk = Link(PORT)
lk.poll(0.3)
targets = [float(x) for x in sys.argv[1:]] or [600, 1000, 1400, 1800, 2000]

# 先确认/建立速度模式
lk.send(MSG_MODE, bytes([3]), X_MODE)
time.sleep(0.05)

print('%8s | %8s %7s %6s | %7s %6s | %7s %6s | %7s' %
      ('目标rpm', '实测', 'pp', 'rms', 'iq均值', 'iq_pp', '周期s', '频率Hz', '自相关'))
print('-' * 108)

rows = []
for target in targets:
    pl = pack_speed(target)

    def ka():
        lk.send(MSG_PS, pl, X_PS)

    lk.send(MSG_PS, pl, X_PS)
    lk.poll(3.5, ka=ka, ka_period=0.4)         # 爬坡 + 稳定

    raw = []
    lk.poll(6.0, sink=raw, ka=ka, ka_period=0.4)
    tr = [(t, f32(p, 12)) for t, mid, p in raw if mid == MSG_ST and len(p) >= 38]
    ti = [(t, f32(p, 12)) for t, mid, p in raw if mid == MSG_LP and len(p) >= 32]

    if len(tr) < 100:
        print('%8.0f | 样本不足' % target)
        continue
    rm, rpp, rrms = stats([v for _, v in tr])
    im, ipp, irms = stats([v for _, v in ti]) if ti else (0, 0, 0)
    f, c = main_freq([t for t, _ in tr], [v for _, v in tr])
    print('%8.0f | %8.1f %7.2f %6.2f | %7.3f %6.3f | %7.3f %7.2f | %7.2f'
          % (target, rm, rpp, rrms, im, ipp,
             1.0 / f if f else 0, f, c))
    rows.append((target, rm, rpp, rrms, im, ipp, f, c))

# 收尾：给 0 再失能
lk.send(MSG_PS, pack_speed(0), X_PS)
time.sleep(0.3)
for _ in range(3):
    lk.send(MSG_PS, pack_speed(0), X_PS)
    time.sleep(0.1)
print('\n已给 0 rpm；2 s 后 deadman 自动失能')
