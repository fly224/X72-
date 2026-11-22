# -*- coding: utf-8 -*-
"""
analyze_thermal.py —— 温漂长测数据分析（2026-11-19）

读 thermal_soak.py 产出的 CSV，量化"温度 → 参数"的漂移：

  1) 温度：起止值、温升速率（前/后 10 min）、是否进入稳态
  2) 转速精度：err_rpm = 实测 − 给定，均值/σ/max，对温度的线性回归斜率
  3) 电流需求：iq_ref / iq_now 均值漂移，对温度的斜率
  4) 相电阻：R 冷热两点 → ΔR/R、实测等效温度系数 α_eff，与铜 α=0.00393/K 对比，
     并反推绕组热点温度（NTC 读数通常显著低于热点）

输出：
  * 终端表格
  * <csv>.report.md（UTF-8 BOM，便于飞书导入）
  * <csv>.svg（三面板曲线：温度 / 转速误差 / iq）

用法：
    python analyze_thermal.py thermal_1000rpm_20261119_111800.csv
"""
import csv
import math
import os
import statistics
import struct
import sys

ALPHA_CU = 0.00393          # 铜电阻温度系数 1/K（20 °C 附近）


def load(path):
    rows = []
    with open(path, "r", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            def g(k, d=None):
                v = r.get(k, "")
                if v is None or v == "":
                    return d
                try:
                    return float(v)
                except ValueError:
                    return d
            rows.append({
                "t": g("t_s"), "iso": r.get("iso", ""),
                "tdrv": g("temp_drv"), "tdrv_max": g("temp_drv_max"),
                "tmot": g("temp_mot"), "tmot_max": g("temp_mot_max"),
                "ref": g("spd_ref_rpm"), "rpm": g("rpm_meas"), "rpm_enc": g("rpm_enc"),
                "err": g("err_rpm"), "iq": g("iq_ref"), "iq_now": g("iq_now"),
                "vbus": g("vbus"), "active": g("active"), "en": g("en"),
                "fault": g("fault"), "frames": g("frames"), "crc": g("crc_err"),
            })
    return [r for r in rows if r["t"] is not None and r["rpm"] is not None]


def linfit(xs, ys):
    """最小二乘 y = a + b·x，返回 (a, b, r2)。"""
    n = len(xs)
    if n < 3:
        return None, None, None
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
    r2 = 0.0 if ss_tot <= 1e-12 else 1.0 - ss_res / ss_tot
    return a, b, r2


def seg(rows, t0, t1):
    return [r for r in rows if t0 <= (r["t"] or 0) <= t1]


def stat(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None, None, None, None
    return (sum(vals) / len(vals), statistics.pstdev(vals) if len(vals) > 1 else 0.0,
            min(vals), max(vals))


def svg_chart(rows, path, title):
    """三面板折线图：温度 / 转速误差 / iq。纯 SVG，不依赖绘图库。"""
    W, H = 960, 720
    ML, MR, MT, MB = 78, 26, 52, 44
    pw = W - ML - MR
    panels = [
        ("温度 (°C)", ["tdrv", "tmot"], ["#e05545", "#2f7fd1"], ["驱动 NTC", "电机 NTC"]),
        ("转速误差 实测−给定 (rpm)", ["err"], ["#7b4fd1"], ["err"]),
        ("iq 给定 (A)", ["iq"], ["#1f9d70"], ["iq_ref"]),
    ]
    ph = (H - MT - MB - 2 * 46) / 3.0

    ts = [r["t"] for r in rows]
    tmax = max(ts) if ts else 1.0
    tmax = max(tmax, 1.0)

    def sx(t):
        return ML + (t / tmax) * pw

    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
           'font-family="-apple-system,Segoe UI,Helvetica,Arial,sans-serif">' % (W, H, W, H),
           '<rect width="%d" height="%d" fill="#ffffff"/>' % (W, H),
           '<text x="%d" y="30" font-size="17" font-weight="600" fill="#1a1a1a">%s</text>' % (ML, title)]

    for pi, (ylab, keys, colors, names) in enumerate(panels):
        top = MT + pi * (ph + 46)
        bot = top + ph
        vals = []
        for k in keys:
            vals += [r[k] for r in rows if r.get(k) is not None]
        if not vals:
            continue
        vmin, vmax = min(vals), max(vals)
        if vmax - vmin < 1e-9:
            vmin -= 0.5
            vmax += 0.5
        pad = (vmax - vmin) * 0.12
        vmin -= pad
        vmax += pad

        def sy(v):
            return bot - (v - vmin) / (vmax - vmin) * ph

        out.append('<rect x="%d" y="%.1f" width="%d" height="%.1f" fill="#fafbfc" stroke="#e3e6ea"/>'
                   % (ML, top, pw, ph))
        # y 网格 4 条
        for i in range(5):
            y = bot - i * ph / 4.0
            v = vmin + i * (vmax - vmin) / 4.0
            out.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#eceff3"/>'
                       % (ML, y, ML + pw, y))
            out.append('<text x="%d" y="%.1f" font-size="11" fill="#6b7280" text-anchor="end">%.1f</text>'
                       % (ML - 8, y + 4, v))
        out.append('<text x="%d" y="%.1f" font-size="12" fill="#374151">%s</text>'
                   % (ML, top - 8, ylab))

        for k, c, nm in zip(keys, colors, names):
            pts = [(sx(r["t"]), sy(r[k])) for r in rows if r.get(k) is not None]
            if len(pts) < 2:
                continue
            d = "M" + " L".join("%.2f,%.2f" % p for p in pts)
            out.append('<path d="%s" fill="none" stroke="%s" stroke-width="1.9" stroke-linejoin="round"/>'
                       % (d, c))
            lx = ML + pw - 8 - (len(keys) - 1 - keys.index(k)) * 108
            out.append('<text x="%d" y="%.1f" font-size="11" fill="%s" text-anchor="end">%s</text>'
                       % (lx, top + 14, c, nm))

    # x 轴
    ax_y = H - MB + 14
    out.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#9ca3af"/>'
               % (ML, ax_y, ML + pw, ax_y))
    for i in range(7):
        t = tmax * i / 6.0
        x = sx(t)
        out.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#9ca3af"/>'
                   % (x, ax_y, x, ax_y + 5))
        out.append('<text x="%.1f" y="%d" font-size="11" fill="#6b7280" text-anchor="middle">%.0f</text>'
                   % (x, ax_y + 19, t / 60.0))
    out.append('<text x="%d" y="%d" font-size="12" fill="#374151" text-anchor="middle">时间 (min)</text>'
               % (ML + pw // 2, H - 8))
    out.append("</svg>")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(out))


def get_arg_str(name):
    for i, a in enumerate(sys.argv):
        if a == name and i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return None


def parse_r_points(s):
    """解析 "T:R,T:R,..." → [(T, R), ...]"""
    pts = []
    if not s:
        return pts
    for seg in s.split(","):
        seg = seg.strip()
        if not seg or ":" not in seg:
            continue
        t, r = seg.split(":", 1)
        try:
            pts.append((float(t), float(r)))
        except ValueError:
            continue
    return pts


def main():
    if len(sys.argv) < 2:
        print("用法: python analyze_thermal.py <csv> [--r-cold 278.6 --t-cold 20.5 --r-hot 285.4 --t-hot 22.4]")
        return 1
    path = sys.argv[1]
    argd = {}
    for i, a in enumerate(sys.argv):
        if a.startswith("--"):
            key = a[2:].replace("-", "_")
            if i + 1 < len(sys.argv):
                try:
                    argd[key] = float(sys.argv[i + 1])
                except ValueError:
                    pass

    rows = load(path)
    if not rows:
        print("[ERR] CSV 无有效数据")
        return 1

    t_end = rows[-1]["t"]
    print("=" * 76)
    print("温漂分析  %s" % os.path.basename(path))
    print("样本 %d 点，时长 %.1f min" % (len(rows), t_end / 60.0))
    print("=" * 76)

    # ---- 1. 温度 ----
    tdrv_mean, _, tdrv_min, tdrv_max = stat([r["tdrv"] for r in rows])
    tmot_mean, _, tmot_min, tmot_max = stat([r["tmot"] for r in rows])
    # 温度分段：前段/末段各取 "10 min 或总时长 1/3 中较小者"，保证短数据也有意义
    win = min(600.0, t_end / 3.0) if t_end > 0 else 1.0
    head = seg(rows, 0, win)
    tail = seg(rows, max(0.0, t_end - win), t_end)
    d_head = (head[-1]["tdrv"] - head[0]["tdrv"]) if len(head) > 1 else 0.0
    d_tail = (tail[-1]["tdrv"] - tail[0]["tdrv"]) if len(tail) > 1 else 0.0
    m_head = (head[-1]["tmot"] - head[0]["tmot"]) if len(head) > 1 else 0.0
    m_tail = (tail[-1]["tmot"] - tail[0]["tmot"]) if len(tail) > 1 else 0.0

    print("\n[1] 温度")
    print("    驱动 NTC : %.2f → %.2f °C（起→末），峰值 %.2f，全程均值 %.2f"
          % (rows[0]["tdrv"], rows[-1]["tdrv"], tdrv_max, tdrv_mean))
    print("    电机 NTC : %.2f → %.2f °C（起→末），峰值 %.2f，全程均值 %.2f"
          % (rows[0]["tmot"], rows[-1]["tmot"], tmot_max, tmot_mean))
    print("    温升速率 : 驱动 前段 %.2f °C / 末段 %.2f °C；电机 前段 %.2f °C / 末段 %.2f °C"
          % (d_head, d_tail, m_head, m_tail))
    settled = (abs(d_tail) < 0.35 and abs(m_tail) < 0.35)
    print("    稳态判定 : %s（末段 10 min 温升 < 0.35 °C 视为进入稳态）"
          % ("已进入" if settled else "未完全进入"))

    # ---- 2. 转速精度 ----
    errs = [r["err"] for r in rows if r["err"] is not None]
    e_mean, e_sd, e_min, e_max = stat(errs)
    # 去掉起步段（前 30 s）再统计稳态
    steady = [r for r in rows if (r["t"] or 0) > 30]
    errs_s = [r["err"] for r in steady if r["err"] is not None]
    es_mean, es_sd, es_min, es_max = stat(errs_s)
    refs = [r["ref"] for r in steady if r["ref"]]
    rpms = [r["rpm"] for r in steady if r["rpm"] is not None]
    encs = [r["rpm_enc"] for r in steady if r["rpm_enc"] is not None]
    rpm_mean, rpm_sd, rpm_min, rpm_max = stat(rpms)
    ref_mean = stat(refs)[0] or 0.0
    print("\n[2] 转速精度（剔除前 30 s 起步段）")
    print("    给定 %.1f rpm；实测均值 %.2f rpm（σ %.2f，min %.1f，max %.1f）"
          % (ref_mean, rpm_mean or 0, rpm_sd or 0, rpm_min or 0, rpm_max or 0))
    if encs:
        print("    编码器差分均值 %.2f rpm（独立于电角速度通道，用于交叉验证）"
              % (stat(encs)[0] or 0))
    print("    跟随误差 : 均值 %+.3f rpm（%+.4f%%），σ %.3f，极值 %+.2f / %+.2f"
          % (es_mean or 0, (es_mean or 0) / (ref_mean or 1) * 100, es_sd or 0,
             es_min or 0, es_max or 0))

    # ---- 3. iq / 母线 ----
    iqs = [r["iq"] for r in steady if r["iq"] is not None]
    iqn = [r["iq_now"] for r in steady if r["iq_now"] is not None]
    iq_mean, iq_sd, iq_min, iq_max = stat(iqs)
    vb = [r["vbus"] for r in steady if r["vbus"] is not None]
    print("\n[3] 电流与母线")
    print("    iq 给定 : 均值 %.4f A（σ %.4f，min %.4f，max %.4f）" % (iq_mean, iq_sd, iq_min, iq_max))
    if iqn:
        print("    iq 实测 : 均值 %.4f A" % (stat(iqn)[0]))
    print("    母线     : 均值 %.2f V（σ %.3f）" % (stat(vb)[0], stat(vb)[1] or 0))

    # ---- 4. 温度相关性 ----
    print("\n[4] 温度相关性（对电机 NTC，最小二乘）")
    xs = [r["tmot"] for r in steady if r["tmot"] is not None and r["err"] is not None]
    ys = [r["err"] for r in steady if r["tmot"] is not None and r["err"] is not None]
    a_e, b_e, r2_e = linfit(xs, ys)
    xs_i = [r["tmot"] for r in steady if r["tmot"] is not None and r["iq"] is not None]
    ys_i = [r["iq"] for r in steady if r["tmot"] is not None and r["iq"] is not None]
    a_i, b_i, r2_i = linfit(xs_i, ys_i)
    if b_e is not None:
        print("    转速误差 vs Tmot : %.4f rpm/°C  (R²=%.3f)  → 10 °C 温升约 %.2f rpm"
              % (b_e, r2_e, b_e * 10))
    if b_i is not None:
        print("    iq       vs Tmot : %.5f A/°C    (R²=%.3f)  → 10 °C 温升约 %.4f A"
              % (b_i, r2_i, b_i * 10))

    # ---- 5. 相电阻 ----
    r_cold = argd.get("r_cold")
    t_cold = argd.get("t_cold")
    r_hot = argd.get("r_hot")
    t_hot = argd.get("t_hot")
    print("\n[5] 相电阻 R（'@' 2 A 注入辨识）")
    if r_cold and r_hot and t_cold is not None and t_hot is not None:
        dr = r_hot - r_cold
        rel = dr / r_cold
        dt = t_hot - t_cold
        alpha_eff = rel / dt if abs(dt) > 1e-6 else float("nan")
        print("    冷态 R = %.2f mΩ @ Tmot %.2f °C" % (r_cold, t_cold))
        print("    热态 R = %.2f mΩ @ Tmot %.2f °C" % (r_hot, t_hot))
        print("    ΔR = %+.2f mΩ（%+.2f%%），NTC 温差 %.2f K" % (dr, rel * 100, dt))
        print("    实测等效温度系数 α_eff = %.4f /K，铜理论 0.00393 /K，比值 %.2f×"
              % (alpha_eff, alpha_eff / ALPHA_CU))
        hot_est = t_cold + rel / ALPHA_CU
        print("    按铜 α 反推绕组热点温度 ≈ %.1f °C（NTC 读数 %.1f °C，低 %.1f K）"
              % (hot_est, t_hot, hot_est - t_hot))
    else:
        print("    未提供 R 冷热两点（用 --r-cold/--t-cold/--r-hot/--t-hot 传入）")

    # 多点回归：单次 R 辨识重复性 CV≈2%，两点法噪声大，多点最小二乘更稳
    r_pts = parse_r_points(get_arg_str("--r-points"))
    reg = None
    if len(r_pts) >= 3:
        xs_r = [p[0] for p in r_pts]
        ys_r = [p[1] for p in r_pts]
        a_r, b_r, r2_r = linfit(xs_r, ys_r)
        r20 = a_r + b_r * 20.0
        alpha_reg = b_r / r20
        reg = {"a": a_r, "b": b_r, "r2": r2_r, "r20": r20, "alpha": alpha_reg,
               "n": len(r_pts)}
        print("\n[5b] 相电阻多点回归（%d 个温度点）" % len(r_pts))
        for t, r in r_pts:
            fit = a_r + b_r * t
            print("    T=%6.2f °C  R=%7.2f mΩ  (拟合 %7.2f，残差 %+6.2f)"
                  % (t, r, fit, r - fit))
        print("    拟合 R(T) = %.2f + %.4f·T mΩ，R² = %.3f" % (a_r, b_r, r2_r))
        print("    R20 = %.2f mΩ，α = %.5f /K（铜 0.00393 /K，比值 %.2f×）"
              % (r20, alpha_reg, alpha_reg / ALPHA_CU))

    # ---- 6. 健康度 ----
    faults = [r["fault"] for r in rows if r["fault"]]
    crcs = [r["crc"] for r in rows if r["crc"] is not None]
    print("\n[6] 链路健康")
    print("    故障采样点 %d 个；末帧计数 %d，CRC 累计错 %d"
          % (sum(1 for f in faults if f and f > 0), rows[-1]["frames"] or 0, rows[-1]["crc"] or 0))

    # ---- 输出 ----
    svg_path = os.path.splitext(path)[0] + ".svg"
    svg_chart(rows, svg_path, "X72 速度模式 %.0f rpm 温漂长测" % ref_mean)
    print("\n[out] 曲线: %s" % svg_path)

    md_path = os.path.splitext(path)[0] + ".report.md"
    L = []
    L.append("# X72 速度模式 %.0f rpm 温漂长测报告\n" % (stat(refs)[0] or 0))
    L.append("- 数据：`%s`（%d 点 / %.1f min）" % (os.path.basename(path), len(rows), t_end / 60.0))
    L.append("- 曲线：`%s`" % os.path.basename(svg_path))
    L.append("- 判定：%s" % ("全程无故障，参数漂移见下表" if not faults else "出现故障采样点，见健康度"))
    L.append("\n## 一、温度\n")
    L.append("| 通道 | 起始 | 结束 | 峰值 | 全程均值 |")
    L.append("|---|---|---|---|---|")
    L.append("| 驱动 NTC_GAN (°C) | %.2f | %.2f | %.2f | %.2f |"
             % (rows[0]["tdrv"], rows[-1]["tdrv"], tdrv_max, tdrv_mean))
    L.append("| 电机 NTC_MOTOR (°C) | %.2f | %.2f | %.2f | %.2f |"
             % (rows[0]["tmot"], rows[-1]["tmot"], tmot_max, tmot_mean))
    L.append("\n温升速率：驱动前段 %.2f °C / 末段 %.2f °C；电机前段 %.2f °C / 末段 %.2f °C（末段 10 min）。"
             % (d_head, d_tail, m_head, m_tail))
    L.append("稳态：%s。\n" % ("已进入" if settled else "未完全进入（末段仍有可见温升）"))
    L.append("## 二、稳态性能\n")
    L.append("| 指标 | 数值 |")
    L.append("|---|---|")
    L.append("| 给定转速 | %.1f rpm |" % ref_mean)
    L.append("| 实测转速（电角速度通道） | %.2f ± %.2f rpm |" % (rpm_mean or 0, rpm_sd or 0))
    if encs:
        L.append("| 实测转速（编码器差分交叉验证） | %.2f rpm |" % (stat(encs)[0] or 0))
    L.append("| 跟随误差 | %+.3f rpm（%+.4f%%） |" % (es_mean or 0, (es_mean or 0) / (ref_mean or 1) * 100))
    L.append("| iq 给定均值 | %.4f A |" % iq_mean)
    L.append("| 母线电压 | %.2f V |" % (stat(vb)[0]))
    L.append("\n## 三、温度相关漂移\n")
    L.append("| 参数 | 对电机 NTC 的斜率 | R² | 折算 10 °C |")
    L.append("|---|---|---|---|")
    if b_e is not None:
        L.append("| 转速跟随误差 | %+.4f rpm/°C | %.3f | %+.2f rpm |" % (b_e, r2_e, b_e * 10))
    if b_i is not None:
        L.append("| iq 给定 | %+.5f A/°C | %.3f | %+.4f A |" % (b_i, r2_i, b_i * 10))
    L.append("\n## 四、相电阻温漂\n")
    if r_cold and r_hot:
        L.append("| 项 | 值 |")
        L.append("|---|---|")
        L.append("| 冷态 R | %.2f mΩ @ %.2f °C |" % (r_cold, t_cold))
        L.append("| 热态 R | %.2f mΩ @ %.2f °C |" % (r_hot, t_hot))
        L.append("| ΔR | %+.2f mΩ（%+.2f%%） |" % (dr, rel * 100))
        L.append("| 实测 α_eff | %.4f /K（铜 0.00393，比值 %.2f×） |" % (alpha_eff, alpha_eff / ALPHA_CU))
        L.append("| 反推绕组热点 | %.1f °C（NTC 低 %.1f K） |" % (hot_est, hot_est - t_hot))
    else:
        L.append("未提供 R 冷热两点数据。")
    if reg:
        L.append("\n多点回归（%d 个温度点，最小二乘）：\n" % reg["n"])
        L.append("| T (°C) | R 实测 (mΩ) | 拟合 (mΩ) | 残差 |")
        L.append("|---|---|---|---|")
        for t, r in r_pts:
            fit = reg["a"] + reg["b"] * t
            L.append("| %.2f | %.2f | %.2f | %+.2f |" % (t, r, fit, r - fit))
        L.append("\n拟合式 `R(T) = %.2f + %.4f·T mΩ`，R² = %.3f；R20 = %.2f mΩ，"
                 "**α = %.5f /K（铜 0.00393 /K，比值 %.2f×）**。"
                 % (reg["a"], reg["b"], reg["r2"], reg["r20"], reg["alpha"],
                    reg["alpha"] / ALPHA_CU))
    L.append("\n## 五、链路健康\n")
    L.append("- 故障采样点：%d" % sum(1 for f in faults if f and f > 0))
    L.append("- 接收帧数：%d，CRC 累计错：%d" % (rows[-1]["frames"] or 0, rows[-1]["crc"] or 0))
    with open(md_path, "w", encoding="utf-8-sig") as f:
        f.write("\n".join(L) + "\n")
    print("[out] 报告: %s" % md_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
