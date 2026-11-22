# -*- coding: utf-8 -*-
"""cog_spec.py —— 齿槽阶次谱分析：前馈到底压掉了哪根线

为什么要谱分析：总 σ 里混着测速噪声、摩擦黏滑、母线波动，
齿槽前馈只针对「24 阶机械」这一根。σ 没变不代表补偿无效，
也可能有效但被别的成分掩盖；反过来 σ 偶然变小也不代表有效。

判据：在 24×rpm/60 Hz 处，OFF 的峰值应明显高于 ON。

实现：不依赖 numpy（这台机器没装），用 Hann 窗 + 单点 DFT 在目标频率
±1.5 Hz 内扫峰。点数只有 600，几十个频点，纯 Python 足够快。

用法: python cog_spec.py COM27 --rpm 30 --secs 6 --repeat 2
"""
import argparse
import math

from cog_ff_test import Rig

COG_ORDER = 24.0        # 齿槽阶次（每转 24 次）


def hann(n):
    if n < 2:
        return [1.0]
    return [0.5 - 0.5 * math.cos(2.0 * math.pi * i / (n - 1)) for i in range(n)]


def amp_at(xm, w, fs, f, wsum):
    """去均值后的序列 xm，Hann 窗 w，单频幅值（正弦峰值）"""
    c = 2.0 * math.pi * f / fs
    re = 0.0
    im = 0.0
    for i, x in enumerate(xm):
        a = c * i
        re += x * w[i] * math.cos(a)
        im += x * w[i] * math.sin(a)
    return 2.0 * math.hypot(re, im) / wsum


def band_peak(x, fs, f0, tol=1.5, step=0.05):
    xm = [v - sum(x) / len(x) for v in x]
    w = hann(len(x))
    wsum = sum(w)
    best, bf = 0.0, f0
    k = int(round((2 * tol) / step))
    for j in range(k + 1):
        f = f0 - tol + j * step
        if f <= 0.2:
            continue
        a = amp_at(xm, w, fs, f, wsum)
        if a > best:
            best, bf = a, f
    return best, bf


def capture(rig, rpm, secs):
    rig.cmd("D", 0.3)
    rig.set_rpm(rpm)
    echo = rig.cmd("W", 0.45)
    if "refused" in echo or "failed" in echo:
        return None, echo
    rig.keep(1.2)
    fr = rig.collect(secs)
    rig.cmd("D", 0.3)
    if len(fr) < 64:
        return None, "帧太少 %d" % len(fr)
    return fr, echo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--rpm", type=float, default=30.0)
    ap.add_argument("--secs", type=float, default=6.0)
    ap.add_argument("--repeat", type=int, default=2)
    a = ap.parse_args()

    f_cog = COG_ORDER * a.rpm / 60.0
    print("转速 %.0f rpm -> 齿槽基频 %.1f Hz（%.0f 阶）" % (a.rpm, f_cog, COG_ORDER))

    rig = Rig(a.port)
    orig_on = False
    rows = []
    try:
        st = rig.cmd(":", 0.5)
        print("[状态] %s" % st)
        if " v=1" not in st:
            return 2
        orig_on = " en=1" in st

        for rep in range(a.repeat):
            got = {}
            for want in (False, True):
                st = rig.cmd(":", 0.3)
                if (" en=1" in st) != want:
                    rig.cmd(";", 0.3)
                fr, echo = capture(rig, a.rpm, a.secs)
                if fr is None:
                    print("  rep%d %s 失败: %s" % (rep + 1, want, echo))
                    continue
                t = [x[0] for x in fr]
                span = t[-1] - t[0]
                fs = (len(t) - 1) / span if span > 0 else 0.0
                if fs <= 2.0 * f_cog:
                    print("  rep%d 帧率 %.1f Hz 不足以看 %.1f Hz" % (rep + 1, fs, f_cog))
                    continue
                v = [x[1] for x in fr]
                iq = [x[2] for x in fr]
                mv = sum(v) / len(v)
                sd = math.sqrt(sum((x - mv) ** 2 for x in v) / len(v))
                p, fp = band_peak(v, fs, f_cog)
                q, fq = band_peak(iq, fs, f_cog)
                got[want] = (p, fp, q, fq, sd, mv, fs, len(t))
            if len(got) == 2:
                o, n = got[False], got[True]
                rows.append((o[0], n[0], o[2], n[2], o[4], n[4]))
                print("\n  rep%d  fs=%.1f Hz  n=%d  spd=%.2f rpm" % (rep + 1, o[6], o[7], o[5]))
                print("    速度谱 @%.1fHz: OFF %.4f  ON %.4f  rpm   (%+.1f%%)"
                      % (f_cog, o[0], n[0], (o[0] - n[0]) / o[0] * 100 if o[0] else 0.0))
                print("    电流谱 @%.1fHz: OFF %.4f  ON %.4f  A     (%+.1f%%)"
                      % (f_cog, o[2], n[2], (o[2] - n[2]) / o[2] * 100 if o[2] else 0.0))
                print("    总 σ: OFF %.3f  ON %.3f rpm" % (o[4], n[4]))
                print("    实测峰频: OFF %.2f Hz  ON %.2f Hz" % (o[1], n[1]))

        if rows:
            def cm(i):
                return sum(r[i] for r in rows) / len(rows)
            print("\n=== 汇总（%d 轮）===" % len(rows))
            print("  速度谱@%.1fHz  OFF %.4f   ON %.4f   %+.1f%%"
                  % (f_cog, cm(0), cm(1), (cm(0) - cm(1)) / cm(0) * 100 if cm(0) else 0))
            print("  电流谱@%.1fHz  OFF %.4f   ON %.4f   %+.1f%%"
                  % (f_cog, cm(2), cm(3), (cm(2) - cm(3)) / cm(2) * 100 if cm(2) else 0))
            print("  总 σ           OFF %.3f   ON %.3f   %+.1f%%"
                  % (cm(4), cm(5), (cm(4) - cm(5)) / cm(4) * 100 if cm(4) else 0))
        return 0
    finally:
        rig.cmd("D", 0.3)
        st = rig.cmd(":", 0.3)
        if (" en=1" in st) != orig_on:
            rig.cmd(";", 0.3)
        print("\n[收尾] %s" % rig.cmd(":", 0.4))
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
