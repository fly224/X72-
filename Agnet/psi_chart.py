# -*- coding: utf-8 -*-
"""
psi_chart.py —— ψ / R 温度漂移曲线（双轴散点 + 线性拟合），输出纯 SVG

左轴 ψ（mWb，随温度下降），右轴 R（mΩ，随温度上升），横轴电机 NTC 温度。
两个参数放在一张图上，是因为它们对温度的反应方向相反、量级可比，
一眼能看出"磁钢退磁"与"铜阻上升"是两件独立的事。

用法：
    python psi_chart.py psi_1000rpm_20261119_125847.csv
    python psi_chart.py a.csv b.csv --out merged.svg     # 多文件合并
"""
import argparse
import csv
import os
import sys


def load(paths):
    pts = []
    for p in paths:
        with open(p, "r", encoding="utf-8-sig") as f:
            for r in csv.DictReader(f):
                try:
                    t = float(r["temp_mot"])
                    psi = float(r["psi_mwb"])
                    rr = float(r["r_mohm"])
                except (KeyError, TypeError, ValueError):
                    continue
                pts.append((t, psi, rr))
    return pts


def linfit(xs, ys):
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx <= 1e-12:
        return my, 0.0, 0.0
    b = sxy / sxx
    a = my - b * mx
    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (a + b * x)) ** 2 for x, y in zip(xs, ys))
    r2 = 0.0 if ss_tot < 1e-12 else 1 - ss_res / ss_tot
    return a, b, r2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    pts = load(args.csv)
    if len(pts) < 3:
        print("[ERR] 有效点不足（%d）" % len(pts))
        return 1

    ts = [p[0] for p in pts]
    psis = [p[1] for p in pts]
    rs = [p[2] for p in pts]
    a_p, b_p, r2_p = linfit(ts, psis)
    a_r, b_r, r2_r = linfit(ts, rs)
    psi20 = a_p + b_p * 20.0
    alpha_p = b_p / psi20 * 100.0
    r20 = a_r + b_r * 20.0
    alpha_r = b_r / r20 * 100.0

    W, H = 900, 560
    ML, MR, MT, MB = 96, 104, 66, 66
    pw, ph = W - ML - MR, H - MT - MB

    tmin, tmax = min(ts), max(ts)
    tpad = max(1.0, (tmax - tmin) * 0.12)
    tlo, thi = tmin - tpad, tmax + tpad

    pmin, pmax = min(psis), max(psis)
    ppad = max(0.004, (pmax - pmin) * 0.25)
    plo, phi = pmin - ppad, pmax + ppad

    rmin, rmax = min(rs), max(rs)
    rpad = max(0.4, (rmax - rmin) * 0.25)
    rlo, rhi = rmin - rpad, rmax + rpad

    def sx(t):
        return ML + (t - tlo) / (thi - tlo) * pw

    def sy_p(v):
        return MT + ph - (v - plo) / (phi - plo) * ph

    def sy_r(v):
        return MT + ph - (v - rlo) / (rhi - rlo) * ph

    o = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
         'font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">' % (W, H, W, H),
         '<rect width="%d" height="%d" fill="#ffffff"/>' % (W, H),
         '<text x="%d" y="32" font-size="17" font-weight="600" fill="#1a1a1a">'
         'ψ / R 温度漂移（%d 点，%.1f→%.1f °C）</text>' % (ML, len(pts), tmin, tmax),
         '<rect x="%d" y="%d" width="%d" height="%d" fill="#fafbfc" stroke="#e3e6ea"/>'
         % (ML, MT, pw, ph)]

    # 横向网格 + 左轴 ψ 刻度
    for i in range(6):
        y = MT + i * ph / 5.0
        v = phi - i * (phi - plo) / 5.0
        o.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#eceff3"/>'
                 % (ML, y, ML + pw, y))
        o.append('<text x="%d" y="%.1f" font-size="10.5" fill="#2f7fd1" text-anchor="end">%.3f</text>'
                 % (ML - 9, y + 4, v))
        rv = rhi - i * (rhi - rlo) / 5.0
        o.append('<text x="%d" y="%.1f" font-size="10.5" fill="#e05545" text-anchor="start">%.1f</text>'
                 % (ML + pw + 9, y + 4, rv))

    # 纵向网格 + 温度刻度
    for i in range(5):
        x = ML + i * pw / 4.0
        t = thi - i * (thi - tlo) / 4.0
        o.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#f2f4f7"/>'
                 % (x, MT, x, MT + ph))
        o.append('<text x="%.1f" y="%d" font-size="10.5" fill="#6b7280" text-anchor="middle">%.1f</text>'
                 % (x, MT + ph + 18, t))

    # 轴标题
    o.append('<text x="%d" y="%d" font-size="12" fill="#2f7fd1" transform="rotate(-90 %d %d)" '
             'text-anchor="middle">ψ (mWb)</text>' % (20, MT + ph // 2, 20, MT + ph // 2))
    o.append('<text x="%d" y="%d" font-size="12" fill="#e05545" transform="rotate(90 %d %d)" '
             'text-anchor="middle">R (mΩ)</text>' % (W - 18, MT + ph // 2, W - 18, MT + ph // 2))
    o.append('<text x="%d" y="%d" font-size="12" fill="#374151" text-anchor="middle">'
             '电机 NTC 温度 (°C)</text>' % (ML + pw // 2, H - 18))

    # 拟合线
    for (a, b, col, yy) in ((a_p, b_p, "#2f7fd1", sy_p), (a_r, b_r, "#e05545", sy_r)):
        x1, y1 = sx(tlo), yy(a + b * tlo)
        x2, y2 = sx(thi), yy(a + b * thi)
        o.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" '
                 'stroke-width="2.2" stroke-dasharray="7 4" opacity="0.85"/>'
                 % (x1, y1, x2, y2, col))

    # 散点
    for (t, psi, rr) in pts:
        o.append('<circle cx="%.1f" cy="%.1f" r="3.6" fill="#2f7fd1" opacity="0.85"/>'
                 % (sx(t), sy_p(psi)))
    for (t, psi, rr) in pts:
        o.append('<circle cx="%.1f" cy="%.1f" r="3.6" fill="#e05545" opacity="0.7"/>'
                 % (sx(t), sy_r(rr)))

    # 结论框
    bx, by, bw, bh = ML + 16, MT + 14, 336, 96
    o.append('<rect x="%d" y="%d" width="%d" height="%d" rx="6" fill="#ffffff" '
             'fill-opacity="0.94" stroke="#d7dce3"/>' % (bx, by, bw, bh))
    o.append('<text x="%d" y="%d" font-size="12" fill="#1a1a1a">'
             'ψ(T) = %.4f %+.6f·T  (R²=%.3f)</text>' % (bx + 12, by + 22, a_p, b_p, r2_p))
    o.append('<text x="%d" y="%d" font-size="12" fill="#2f7fd1">'
             'α_ψ = %+.3f %%/K　→ 10 K 变化 %+.2f%%</text>'
             % (bx + 12, by + 44, alpha_p, alpha_p * 10))
    o.append('<text x="%d" y="%d" font-size="12" fill="#e05545">'
             'α_R = %+.3f %%/K　→ 10 K 变化 %+.2f%%</text>'
             % (bx + 12, by + 66, alpha_r, alpha_r * 10))
    o.append('<text x="%d" y="%d" font-size="11" fill="#6b7280">'
             'ψ20=%.4f mWb  R20=%.2f mΩ  Kt20=%.4f N·m/A</text>'
             % (bx + 12, by + 86, psi20, r20, 1.5 * 14.0 * psi20 * 1e-3))

    o.append("</svg>")

    out = args.out or (os.path.splitext(args.csv[0])[0] + "_chart.svg")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(o))

    print("拟合 ψ(T) = %.5f %+.7f·T mWb, R²=%.4f, ψ20=%.4f, α_ψ=%+.4f %%/K"
          % (a_p, b_p, r2_p, psi20, alpha_p))
    print("拟合 R(T) = %.4f %+.6f·T mΩ, R²=%.4f, R20=%.2f, α_R=%+.4f %%/K"
          % (a_r, b_r, r2_r, r20, alpha_r))
    print("Kt20 = %.5f N·m/A；温度跨度 %.2f→%.2f °C（%.2f K）"
          % (1.5 * 14.0 * psi20 * 1e-3, tmin, tmax, tmax - tmin))
    print("[out] %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
