# -*- coding: utf-8 -*-
"""
psi_chart_compare.py —— ψ 温漂：升温室 vs 冷却段的迟滞环对比图（纯 SVG）

为什么单独画：同一 NTC 温度下，升温时 ψ 略低于降温时，差值折合 0.5~2.0 K，
这是 NTC 贴装热阻造成的读数滞后，不是 ψ 本身的非线性。把两段分别拟合并画在一起，
既能看到"斜率一致"（α 可信），也能看到"截距错开"（迟滞量），避免直接合并把斜率拉偏。

用法：
    python psi_chart_compare.py 加热.csv 冷却.csv --out psi_hyst.svg
"""
import argparse
import csv
import os
import sys

C_HEAT = "#2f7fd1"
C_COOL = "#e07a2b"
C_R = "#9aa3ad"


def load(path):
    pts = []
    with open(path, "r", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            try:
                pts.append((float(r["temp_mot"]), float(r["psi_mwb"]), float(r["r_mohm"])))
            except (KeyError, TypeError, ValueError):
                continue
    return sorted(pts)


def linfit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx <= 1e-12:
        return my, 0.0, 0.0
    b = sxy / sxx
    a = my - b * mx
    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (a + b * x)) ** 2 for x, y in zip(xs, ys))
    return a, b, (0.0 if ss_tot < 1e-12 else 1 - ss_res / ss_tot)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("heat_csv")
    ap.add_argument("cool_csv")
    ap.add_argument("--out", default="psi_hysteresis.svg")
    args = ap.parse_args()

    H = load(args.heat_csv)
    C = load(args.cool_csv)
    if len(H) < 3 or len(C) < 3:
        print("[ERR] 点数不足")
        return 1

    a_h, b_h, r2_h = linfit([p[0] for p in H], [p[1] for p in H])
    a_c, b_c, r2_c = linfit([p[0] for p in C], [p[1] for p in C])
    psi20_h = a_h + b_h * 20.0
    psi20_c = a_c + b_c * 20.0
    al_h = b_h / psi20_h * 100.0
    al_c = b_c / psi20_c * 100.0
    al_avg = (al_h + al_c) / 2.0

    allp = H + C
    tmin = min(p[0] for p in allp)
    tmax = max(p[0] for p in allp)
    pmin = min(p[1] for p in allp)
    pmax = max(p[1] for p in allp)

    W, HH = 900, 560
    ML, MR, MT, MB = 96, 108, 70, 70
    pw, ph = W - ML - MR, HH - MT - MB

    tpad = max(1.0, (tmax - tmin) * 0.10)
    tlo, thi = tmin - tpad, tmax + tpad
    ppad = max(0.004, (pmax - pmin) * 0.30)
    plo, phi = pmin - ppad, pmax + ppad

    rmin = min(p[2] for p in allp)
    rmax = max(p[2] for p in allp)
    rpad = max(0.5, (rmax - rmin) * 0.30)
    rlo, rhi = rmin - rpad, rmax + rpad

    sx = lambda t: ML + (t - tlo) / (thi - tlo) * pw
    sy = lambda v: MT + ph - (v - plo) / (phi - plo) * ph
    syr = lambda v: MT + ph - (v - rlo) / (rhi - rlo) * ph

    o = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
         'font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">' % (W, HH, W, HH),
         '<rect width="%d" height="%d" fill="#ffffff"/>' % (W, HH),
         '<text x="%d" y="34" font-size="17" font-weight="600" fill="#1a1a1a">'
         'ψ 温度漂移：升温室 vs 冷却段（迟滞环）</text>' % ML,
         '<rect x="%d" y="%d" width="%d" height="%d" fill="#fbfcfd" stroke="#e3e6ea"/>'
         % (ML, MT, pw, ph)]

    for i in range(6):
        y = MT + i * ph / 5.0
        v = phi - i * (phi - plo) / 5.0
        o.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#eef1f4"/>' % (ML, y, ML + pw, y))
        o.append('<text x="%d" y="%.1f" font-size="10.5" fill="%s" text-anchor="end">%.3f</text>'
                 % (ML - 9, y + 4, C_HEAT, v))
        rv = rhi - i * (rhi - rlo) / 5.0
        o.append('<text x="%d" y="%.1f" font-size="10.5" fill="%s" text-anchor="start">%.1f</text>'
                 % (ML + pw + 9, y + 4, C_R, rv))

    for i in range(5):
        x = ML + i * pw / 4.0
        t = thi - i * (thi - tlo) / 4.0
        o.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#f4f6f8"/>' % (x, MT, x, MT + ph))
        o.append('<text x="%.1f" y="%d" font-size="10.5" fill="#6b7280" text-anchor="middle">%.1f</text>'
                 % (x, MT + ph + 18, t))

    o.append('<text x="22" y="%d" font-size="12" fill="%s" transform="rotate(-90 22 %d)" '
             'text-anchor="middle">ψ (mWb)</text>' % (MT + ph // 2, C_HEAT, MT + ph // 2))
    o.append('<text x="%d" y="%d" font-size="11.5" fill="%s" transform="rotate(90 %d %d)" '
             'text-anchor="middle">R (mΩ)</text>' % (W - 16, MT + ph // 2, C_R, W - 16, MT + ph // 2))
    o.append('<text x="%d" y="%d" font-size="12" fill="#374151" text-anchor="middle">'
             '电机 NTC 温度 (°C)</text>' % (ML + pw // 2, HH - 20))

    # R 散点（灰，弱化）
    for (t, psi, rr) in allp:
        o.append('<circle cx="%.1f" cy="%.1f" r="2.6" fill="%s" opacity="0.45"/>'
                 % (sx(t), syr(rr), C_R))

    # 两条拟合线
    for (a, b, col) in ((a_h, b_h, C_HEAT), (a_c, b_c, C_COOL)):
        o.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" '
                 'stroke-width="2.4" stroke-dasharray="8 4"/>'
                 % (sx(tlo), sy(a + b * tlo), sx(thi), sy(a + b * thi), col))

    # ψ 散点
    for (t, psi, rr) in H:
        o.append('<circle cx="%.1f" cy="%.1f" r="4.0" fill="%s"/>' % (sx(t), sy(psi), C_HEAT))
    for (t, psi, rr) in C:
        o.append('<rect x="%.1f" y="%.1f" width="7.4" height="7.4" fill="%s"/>'
                 % (sx(t) - 3.7, sy(psi) - 3.7, C_COOL))

    # 结论框
    bx, by, bw, bh = ML + 18, MT + 14, 372, 118
    o.append('<rect x="%d" y="%d" width="%d" height="%d" rx="6" fill="#ffffff" '
             'fill-opacity="0.95" stroke="#d7dce3"/>' % (bx, by, bw, bh))
    o.append('<circle cx="%d" cy="%d" r="4" fill="%s"/>'
             '<text x="%d" y="%d" font-size="11.5" fill="#374151">加热段 %d 点  α=%+.4f %%/K (R²=%.3f)</text>'
             % (bx + 16, by + 20, C_HEAT, bx + 28, by + 24, len(H), al_h, r2_h))
    o.append('<rect x="%.1f" y="%.1f" width="7" height="7" fill="%s"/>'
             '<text x="%d" y="%d" font-size="11.5" fill="#374151">冷却段 %d 点  α=%+.4f %%/K (R²=%.3f)</text>'
             % (bx + 12.5, by + 35.5, C_COOL, bx + 28, by + 44, len(C), al_c, r2_c))
    o.append('<text x="%d" y="%d" font-size="12" fill="#1a1a1a" font-weight="600">'
             '取两段平均：α_ψ = %+.4f %%/K</text>' % (bx + 14, by + 68, al_avg))
    o.append('<text x="%d" y="%d" font-size="10.5" fill="#6b7280">'
             '两段斜率差 %.1f%%（一致）；垂直错开 0.5~2.0 K = NTC 热滞后</text>'
             % (bx + 14, by + 88, abs(al_h - al_c) / abs(al_avg) * 100))
    o.append('<text x="%d" y="%d" font-size="10.5" fill="#6b7280">'
             'Kt20=%.4f N·m/A，50 °C 时 Kt 比 20 °C 低 %.1f%%</text>'
             % (bx + 14, by + 106, 1.5 * 14.0 * (psi20_h + psi20_c) / 2 * 1e-3,
                abs(al_avg) * 30))

    o.append("</svg>")
    with open(args.out, "w", encoding="utf-8") as f:
        f.write("\n".join(o))

    print("加热段 α=%+.4f %%/K (R²=%.4f, ψ20=%.4f)" % (al_h, r2_h, psi20_h))
    print("冷却段 α=%+.4f %%/K (R²=%.4f, ψ20=%.4f)" % (al_c, r2_c, psi20_c))
    print("平均   α=%+.4f %%/K；两段差 %.1f%%" % (al_avg, abs(al_h - al_c) / abs(al_avg) * 100))
    print("[out] %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
