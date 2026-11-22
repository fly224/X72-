# -*- coding: utf-8 -*-
"""合并两段温升步进数据并拟合 ψ/R/L 的温度系数。"""
import csv
import glob
import sys

FILES = [
    "thermal_step_2000rpm_20261120_110950.csv",   # 30 点，29.7~60.2 °C（步长退化为 1 K）
    "thermal_step_2000rpm_20261120_113821.csv",   # 15 点，59.1~84.3 °C（2 K）
]


def load():
    rows = []
    for p in FILES:
        try:
            for r in csv.DictReader(open(p, encoding="utf-8-sig")):
                rows.append(dict(
                    temp=float(r["temp_post"]), temp_pre=float(r["temp_pre"]),
                    tdrv=float(r["temp_drv"]), psi=float(r["psi_mwb"]),
                    r_ohm=float(r["r_mohm"]), l=float(r["l_uh"]),
                    rpm=float(r["rpm_meas"]), iq=float(r["iq_ref"]),
                    src=p))
        except FileNotFoundError:
            print("missing: %s" % p)
    rows.sort(key=lambda x: x["temp"])
    return rows


def fit(x, y):
    n = len(x)
    mx = sum(x) / n
    my = sum(y) / n
    sxx = sum((a - mx) ** 2 for a in x)
    sxy = sum((a - mx) * (b - my) for a, b in zip(x, y))
    k = sxy / sxx
    b = my - k * mx
    syy = sum((bb - my) ** 2 for bb in y)
    ssr = sum((yy - (k * xx + b)) ** 2 for xx, yy in zip(x, y))
    r2 = 1 - ssr / syy if syy > 0 else 0.0
    resid = [yy - (k * xx + b) for xx, yy in zip(x, y)]
    sd = (sum(v * v for v in resid) / n) ** 0.5
    return k, b, r2, sd


def main():
    rows = load()
    if len(rows) < 5:
        print("数据不足: %d" % len(rows))
        return 1
    T = [r["temp"] for r in rows]
    P = [r["psi"] for r in rows]
    R = [r["r_ohm"] for r in rows]
    L = [r["l"] for r in rows]
    I = [r["iq"] for r in rows]
    D = [r["tdrv"] for r in rows]

    print("=== 合并数据：%d 点，T = %.2f → %.2f °C（%+.2f K）==="
          % (len(rows), T[0], T[-1], T[-1] - T[0]))
    print("驱动温度 %.2f → %.2f °C\n" % (D[0], D[-1]))

    for name, arr, unit in (("psi", P, "mWb"), ("R", R, "mΩ"), ("L", L, "µH")):
        k, b, r2, sd = fit(T, arr)
        alpha = k / (sum(arr) / len(arr)) * 100.0
        print("%-4s = %9.4f %+.6f·T   α = %+7.4f %%/K   R² = %.4f   σ_res = %.4f %s"
              % (name, b, k, alpha, r2, sd, unit))
        print("     在 %.1f °C: %.4f   →   %.1f °C: %.4f  （变化 %+.2f%%）"
              % (T[0], arr[0], T[-1], arr[-1], (arr[-1] / arr[0] - 1) * 100))

    # 分段斜率（检查非线性）
    print("\n=== 分段 α（检查线性度）===")
    for lo, hi in ((30, 50), (50, 70), (70, 86)):
        xs = [t for t in T if lo <= t < hi]
        if len(xs) < 3:
            continue
        idx = [i for i, t in enumerate(T) if lo <= t < hi]
        for name, arr in (("psi", P), ("R", R), ("L", L)):
            k, b, r2, sd = fit([T[i] for i in idx], [arr[i] for i in idx])
            mu = sum(arr[i] for i in idx) / len(idx)
            print("  %2d~%2d °C  %-3s  α=%+7.4f %%/K  (n=%d, R²=%.3f)"
                  % (lo, hi, name, k / mu * 100, len(idx), r2))
        print()

    # iq 随温度
    k, b, r2, sd = fit(T, I)
    print("iq_ref: 均值 %.3f A，斜率 %+.5f A/K（R²=%.3f）" %
          (sum(I) / len(I), k, r2))

    # 明细表
    print("\n=== 明细（每 3 点抽样）===")
    print("%8s %8s %9s %9s %9s %7s" % ("T_mot", "T_drv", "psi", "R", "L", "iq"))
    for i in range(0, len(rows), 3):
        r = rows[i]
        print("%8.2f %8.2f %9.4f %9.2f %9.2f %7.3f"
              % (r["temp"], r["tdrv"], r["psi"], r["r_ohm"], r["l"], r["iq"]))
    r = rows[-1]
    print("%8.2f %8.2f %9.4f %9.2f %9.2f %7.3f  (末点)"
          % (r["temp"], r["tdrv"], r["psi"], r["r_ohm"], r["l"], r["iq"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
