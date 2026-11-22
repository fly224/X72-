# -*- coding: utf-8 -*-
"""sine_analyze.py —— 速度正弦测试数据分析

输入 spd_sine.py 产出的 CSV，输出：
  1) 跟踪质量：误差 RMS / 峰值 / 相位延迟（用互相关估计）
  2) 电流：iq 均值、峰值、正负半周对称性（四象限是否一致）
  3) 热与母线：温度斜率、母线 min/max
  4) 分段趋势：每 60 s 一段的 rpm σ、iq 均值，用来看长时间漂移

用法：
    python sine_analyze.py spd_sine_run2000.csv
    python sine_analyze.py spd_sine_run2000.csv --seg 60 --freq 0.1
"""
import argparse
import csv
import math
import os
import sys


def load(path):
    with open(path, encoding="utf-8-sig") as f:
        return [dict((k, float(v)) for k, v in r.items()) for r in csv.DictReader(f)]


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def std(xs):
    if len(xs) < 2:
        return float("nan")
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def interp(ts, vs, t):
    import bisect
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return vs[0]
    if i >= len(ts):
        return vs[-1]
    t0, t1 = ts[i - 1], ts[i]
    if t1 <= t0:
        return vs[i]
    k = (t - t0) / (t1 - t0)
    return vs[i - 1] + (vs[i] - vs[i - 1]) * k


def best_lag(rows, max_lag=1.2, step=0.005):
    """以 ref 列为参考，搜使 RMS(act - ref(t-lag)) 最小的延迟。

    直接对 ref 做平移而不是重建 sin：CSV 的 t 是"记录时刻"，与命令发出的
    时刻差一个有界量，重建 sin 会把这个偏置算成相位误差（上一版就栽在这）。
    """
    if len(rows) < 20:
        return 0.0, float("nan"), float("nan")
    ts = [r["t"] for r in rows]
    ref = [r["ref"] for r in rows]
    act = [r["act"] for r in rows]

    e_raw = rms([act[i] - ref[i] for i in range(len(rows))])
    best_e, best_l = e_raw, 0.0
    lag = step
    while lag <= max_lag:
        errs = [act[i] - interp(ts, ref, ts[i] - lag) for i in range(len(rows))]
        e = rms(errs)
        if e < best_e:
            best_e, best_l = e, lag
        lag += step
    return best_l, best_e, e_raw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--freq", type=float, default=0.1)
    ap.add_argument("--seg", type=float, default=60.0)
    ap.add_argument("--amp", type=float, default=None)
    a = ap.parse_args()

    rows = load(a.csv)
    if not rows:
        print("空文件")
        return 1

    dur = rows[-1]["t"] - rows[0]["t"]
    amp = a.amp or max(abs(r["ref"]) for r in rows)
    print("=" * 74)
    print("正弦跟踪分析  %s" % os.path.basename(a.csv))
    print("  时长 %.1f s  样本 %d  频率 %.3f Hz  幅值 %.0f rpm"
          % (dur, len(rows), a.freq, amp))
    print("=" * 74)

    ref = [r["ref"] for r in rows]
    act = [r["act"] for r in rows]
    err = [act[i] - ref[i] for i in range(len(rows))]
    iq = [r["iq"] for r in rows]

    # 跳过起步 2 s（使能瞬态）
    k0 = next((i for i, r in enumerate(rows) if r["t"] > 2.0), 0)
    e2 = err[k0:]
    a2 = act[k0:]

    print("\n[1] 跟踪质量（跳过前 2 s）")
    print("  误差 RMS      %8.1f rpm  (%.1f%% FS)" % (rms(e2), 100 * rms(e2) / amp))
    print("  误差 峰值     %8.1f rpm  (%.1f%% FS)"
          % (max(abs(x) for x in e2), 100 * max(abs(x) for x in e2) / amp))
    print("  实测幅值      %8.1f rpm  (给定 %.0f, 幅值比 %.3f)"
          % ((max(a2) - min(a2)) / 2, amp, (max(a2) - min(a2)) / 2 / amp))
    print("  实测均值      %8.1f rpm  (直流偏置)" % mean(a2))

    lag, e_al, e_raw = best_lag(rows[k0:])
    print("  原始误差 RMS  %8.1f rpm  (%.1f%% FS)" % (e_raw, 100 * e_raw / amp))
    if lag == lag:
        print("  最佳对齐延迟  %8.1f ms  (%.1f ° @ %.2f Hz)"
              % (lag * 1000, lag * a.freq * 360, a.freq))
        print("  对齐后误差    %8.1f rpm  (%.1f%% FS)  <- 速度环真实跟踪能力"
              % (e_al, 100 * e_al / amp))
        print("  延迟来源：50 ms 命令量化 + ASCII 往返 + 遥测 10 ms + 测速 2 ms")

    print("\n[2] 电流（iq 给定）")
    pos = [x for x in iq[k0:] if x > 0]
    neg = [x for x in iq[k0:] if x < 0]
    print("  均值 %7.3f A   峰值 %+.3f / %+.3f A   RMS %.3f A"
          % (mean(iq[k0:]), max(iq[k0:]), min(iq[k0:]), rms(iq[k0:])))
    print("  正半周均值 %7.3f A (n=%d)   负半周均值 %7.3f A (n=%d)"
          % (mean(pos) if pos else float("nan"), len(pos),
             mean(neg) if neg else float("nan"), len(neg)))
    print("  四象限对称性（|负/正| 均值比）: %.3f"
          % ((abs(mean(neg)) / mean(pos)) if pos and neg and mean(pos) else float("nan")))

    print("\n[3] 热与母线")
    for k, name in (("tg", "GaN"), ("tm", "电机")):
        v = [r[k] for r in rows]
        print("  %-5s %6.1f -> %6.1f °C   斜率 %+.2f °C/min"
              % (name, v[0], v[-1], (v[-1] - v[0]) / (dur / 60)))
    vb = [r["vbus"] for r in rows]
    print("  母线  %6.2f ~ %6.2f V   均值 %.2f" % (min(vb), max(vb), mean(vb)))

    nseg = max(1, int(dur / a.seg))
    print("\n[4] 分段趋势（每 %.0f s）" % a.seg)
    print("  段    t范围        act_sigma   iq_mean   iq_peak  err_rms   tg")
    for s in range(nseg):
        lo, hi = s * a.seg, (s + 1) * a.seg
        ss = [r for r in rows if lo <= r["t"] < hi]
        if len(ss) < 5:
            continue
        print("  %2d  %5.0f-%-5.0f  %9.1f  %8.3f  %7.3f  %7.1f  %5.1f"
              % (s + 1, lo, hi,
                 std([r["act"] for r in ss]),
                 mean([r["iq"] for r in ss]),
                 max(abs(r["iq"]) for r in ss),
                 rms([r["act"] - r["ref"] for r in ss]),
                 ss[-1]["tg"]))

    print("\n[5] 判读")
    print("  · 幅值比 <0.95 且 iq 触到 ±3 A → 电压/电流限幅，给定到不了")
    print("  · 四象限比偏离 1 超过 20% → 正反向摩擦/齿槽不对称")
    print("  · 误差 RMS 里约 50 ms 是命令量化，剩余才是速度环带宽")
    return 0


if __name__ == "__main__":
    sys.exit(main())
