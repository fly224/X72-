# -*- coding: utf-8 -*-
"""ident_mech.py —— 机械参数辨识（一键）

一条命令跑完机械侧全部辨识，按"快 -> 慢"排序，失败即停：

    $  摩擦两点前馈标定   3 -> 100 rpm，定 Tc 与 visc       约 6  s
    (  谐振扫频 Bode      20..1000 Hz 对数 11 点            约 5  s
    )  摩擦曲线 8 点      1..200 rpm 稳态 iq                约 30 s
    <  折算惯量 J         iq 1 A / 2 A 加速度差分           约 8  s
    }  齿槽转矩 128 箱    10 rpm 正反各 1 圈                约 15 s

为什么这个顺序：
    $ 最快且能立刻暴露"速度环转不起来"（后面四项全都靠它转），放在最前
    当探针。$ 失败就没必要再等 30 s 的摩擦曲线。
    } 放最后：它会把表装进 CogFf 缓存（`->cog ok`），一旦装错，后面的
    测试全被污染，所以必须最后一个做，且做完立刻 A 落盘。

为什么不用 `_<n>` 数值命令启动：
    实机出现过引导字符与数字在同一批 Poll 里时数字丢失，表现为"发了 _1
    却没启动"。四个单字符不可能丢（一个字节就是一条命令），所以用 ( ) < }。

用法：
    python ident_mech.py                     # COM27，五项全跑（约 65 s）
    python ident_mech.py --skip fric2,cog    # 跳过指定项
    python ident_mech.py --no-save
    python ident_mech.py --dry-run

产物：
    控制台表 + 机械参数辨识_报告.md + ident_mech_history.csv
    + cog_table_<时间>.csv（128 点齿槽表，可与 CogFf 里的表直接对照）

安全：全程转子会动，最高 200 rpm（摩擦曲线）、最低 1 rpm（可能长时间堵转
      在 1 rpm 附近，电流约 0.5 A）。运行前确认负载侧无人无限位。
"""
import argparse
import math
import os
import re
import sys
import time

from ident_lib import (Link, banner, step, precheck, save_flash, num, lines_with,
                       clean, rel_pct, verdict, MARK, md_table, save_text,
                       append_csv, HERE)

# 各项超时（s）。给的是"实测耗时 × 2~3"，不是估计值：
# 卡住时必须能自己退出，否则整个一键流程会挂死。
TMO = {"fc": 25.0, "sweep": 25.0, "fric2": 70.0, "inertia": 30.0, "cog": 45.0}
NAME = {"fc": "摩擦两点前馈", "sweep": "谐振扫频", "fric2": "摩擦曲线",
        "inertia": "折算惯量", "cog": "齿槽转矩"}
CMD = {"fc": "$", "sweep": "(", "fric2": ")", "inertia": "<", "cog": "}"}
ORDER = ["fc", "sweep", "fric2", "inertia", "cog"]


def fmt(v, nd=3, unit=""):
    if v is None:
        return "—"
    return ("%." + str(nd) + "f%s") % (v, unit)


def parse_sweep(txt):
    """全局搜索，不依赖行首。

    为什么不能用逐行 match：100 Hz 遥测帧（二进制，占串口流量 60%+）会插在
    ASCII 行之间，甚至直接贴在行首 —— 实测拿到的是
        b'...\\xef\\xbf\\xbd\\x1di[MI] SW 95.64 1.7931 -89.50\\r\\n'
    行首是遥测尾巴，splitlines 后整行以二进制开头，match 匹配不上就被当成
    "少了一个点"。实际固件发全了 11 行，是解析器自己丢的。
    """
    pts = []
    for m in re.finditer(r"\[MI\] SW\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)", txt):
        pts.append({"f": float(m.group(1)), "g": float(m.group(2)),
                    "ph": float(m.group(3))})
    return pts


def parse_fric(txt):
    pts = []
    for m in re.finditer(r"\[MI\] FR\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)", txt):
        pts.append({"rpm": float(m.group(1)), "rpm_act": float(m.group(2)),
                    "iq": float(m.group(3))})
    return pts


def parse_cog(txt):
    """返回 (表, 实收行数)。

    每行 16 个值，行数由 [MI] CG 的出现次数决定；不足 128 时按索引线性插值
    补齐，并在报告里标注 —— 少一行是 11.25° 的缺口，直接前馈会在缺口处产生
    一个台阶，插值比留 0 安全得多。
    """
    vals = []
    nrows = 0
    for m in re.finditer(r"\[MI\] CG ([^\r\n]*)", txt):
        nrows += 1
        for tok in m.group(1).split():
            try:
                vals.append(float(tok))
            except ValueError:
                pass
    return vals, nrows


def interp_to(vals, n):
    """把 vals 线性插值/截断到 n 点。"""
    if not vals or n <= 0:
        return []
    if len(vals) == n:
        return list(vals)
    out = []
    m = len(vals)
    for i in range(n):
        x = i * (m - 1) / (n - 1) if n > 1 else 0.0
        i0 = int(math.floor(x))
        i1 = min(i0 + 1, m - 1)
        t = x - i0
        out.append(vals[i0] * (1 - t) + vals[i1] * t)
    return out


def linfit(xs, ys):
    """最小二乘 y = a + b·x，返回 (a, b, R²)。点数不足返回 None。"""
    n = len(xs)
    if n < 3:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    a = my - b * mx
    ss_res = sum((y - (a + b * x)) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    return a, b, r2


def main():
    ap = argparse.ArgumentParser(description="机械参数辨识（一键）")
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--skip", default="", help="跳过项，逗号分隔：fc,sweep,fric2,inertia,cog")
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--tag", default="")
    a = ap.parse_args()

    skip = set(s.strip() for s in a.skip.split(",") if s.strip())
    bad = skip - set(ORDER)
    if bad:
        print("[FAIL] --skip 里有未知项：%s（可选 %s）" % (bad, ",".join(ORDER)))
        return 2
    todo = [k for k in ORDER if k not in skip]
    if not todo:
        print("[FAIL] 全部被跳过，无事可做")
        return 2

    banner("机械参数辨识（一键）",
           "port=%s   执行 %s   合计预计 %d s"
           % (a.port, " ".join(NAME[k] for k in todo),
              sum(int(TMO[k] / 2.5) for k in todo)))

    try:
        lk = Link(a.port)
    except Exception as e:
        print("[FAIL] 打不开串口 %s：%s" % (a.port, e))
        return 2

    try:
        step(1, len(todo) + 2, "前置联锁（停机 / 清故障 / C / Z）")
        ok, st, log, zres = precheck(lk, allow_align=True, force_align=False)
        if not ok:
            return 2
        if zres.get("dir") is not None and zres["dir"] < 0:
            print("    注意：dir=-1，扫频增益已由固件乘 dir 归一化")

        if a.dry_run:
            print("\n[dry-run] 前置已通过，计划的激励如下（未执行）：")
            for k in todo:
                print("    %s %-8s %-14s 超时 %.0f s" % (CMD[k], k, NAME[k], TMO[k]))
            return 0

        res = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "tag": a.tag, "port": a.port}
        steps = list(log)
        raw = []
        sweep, fric, cog = [], [], []

        # ---------------- 逐项执行 ----------------
        for i, k in enumerate(todo):
            step(i + 2, len(todo) + 2, "%s %s" % (CMD[k], NAME[k]))
            txt, took, refused = lk.run(CMD[k], ["MI end", "MI FAIL", "FC ok", "FC FAIL"],
                                        TMO[k], k)
            raw.append((k, txt))
            if refused:
                print("    [FAIL] 被拒收：" + refused)
                steps.append((NAME[k], CMD[k], took, refused))
                return 3

            if k == "fc":
                line = clean((lines_with(txt, "FC ") or ["(no echo)"])[-1])
                print("    " + line)
                steps.append((NAME[k], CMD[k], took, line))
                okk = "FC ok" in txt
                res.update({
                    "fc_iq1": num(r"iq1=([\d.]+)A", txt),
                    "fc_iq2": num(r"iq2=([\d.]+)A", txt),
                    "fc_visc": num(r"visc=(-?[\d.]+)e-3", txt),
                    "fc_tc": num(r"Tc=([\d.]+)Nm", txt),
                    "fc_rpm1": num(r"rpm=([\d.]+)/", txt),
                    "fc_rpm2": num(r"rpm=[\d.]+/([\d.]+)", txt),
                    "fc_ok": 1 if okk else 0,
                })
                if res.get("fc_visc") is not None:
                    res["fc_visc"] *= 1e-3
                if not okk:
                    # 不终止：$ 只产出摩擦前馈的两个系数，后面四项（扫频/摩擦曲线/
                    # 惯量/齿槽）只要求"速度环能转"，与 $ 的可信度无关。
                    # 实测冷态 3 rpm 的 iq 离散度大，$ 会报 unstable samples，
                    # 但紧接着的 ( ) < } 全部正常 —— 因一个可选快速标定丢掉
                    # 后面 60 s 的有效数据是错的。
                    print("    [WARN] $ 未通过（多为冷态低速 iq 离散大），继续后面四项")
                    print("           冷态下 1–3 rpm 的 iq 要爬 1–2 s 才稳，FricCal 的")
                    print("           500 ms 平均窗口截到的是爬升段。预热 5–10 min 后单跑：")
                    print("           python ident_mech.py %s --skip sweep,fric2,inertia,cog" % a.port)

            elif k == "sweep":
                sweep = parse_sweep(txt)
                line = clean((lines_with(txt, "MI end") or ["(no echo)"])[-1])
                print("    %d 点   %s" % (len(sweep), line))
                for p in sweep:
                    print("      %7.1f Hz   gain=%8.4f   phase=%7.1f°"
                          % (p["f"], p["g"], p["ph"]))
                steps.append((NAME[k], CMD[k], took, "%d 点" % len(sweep)))
                res["sw_n"] = len(sweep)
                res["sw_wmean"] = num(r"wmean=([\d.]+)", txt)
                res["sw_wamp"] = num(r"wamp=([\d.]+)", txt)
                res["sw_iamp"] = num(r"iamp=([\d.]+)", txt)

            elif k == "fric2":
                fric = parse_fric(txt)
                line = clean((lines_with(txt, "MI end") or ["(no echo)"])[-1])
                print("    %d 点   %s" % (len(fric), line))
                for p in fric:
                    print("      %6.1f rpm  (实测 %6.1f)  iq=%7.4f A"
                          % (p["rpm"], p["rpm_act"], p["iq"]))
                steps.append((NAME[k], CMD[k], took, "%d 点" % len(fric)))
                res["fr_n"] = len(fric)
                res["fr_a"] = num(r"fric_a=([\d.]+)A", txt)
                res["fr_visc"] = num(r"visc=(-?[\d.]+)e-3", txt)
                if res.get("fr_visc") is not None:
                    res["fr_visc"] *= 1e-3

            elif k == "inertia":
                # 固件 servo_cmd.c 的 case 3U 用了 break 而不是 return，导致同一拍
                # 又 emit 了一次 "MI end"，把 IG 行末尾的 \r\n 覆盖掉 —— 原始串口里
                # 看到的是 "tq=0.0910/0.1820[CMD] MI end"。数据本身没丢，
                # 这里按字段重排，避免把 "[CMD] MI end" 打进行尾。
                jv = num(r"J=([\d.]+)e-4", txt)
                a1 = num(r"acc=([\d.]+)/", txt)
                a2 = num(r"acc=[\d.]+/([\d.]+)", txt)
                t1 = num(r"tq=([\d.]+)/", txt)
                t2 = num(r"tq=[\d.]+/([\d.]+)", txt)
                if jv is not None:
                    ig = ("[MI] IG J=%.4fe-4 acc=%.2f/%.2f tq=%.4f/%.4f"
                          % (jv, a1 or 0, a2 or 0, t1 or 0, t2 or 0))
                else:
                    ig = clean((lines_with(txt, "IG ") or ["(no echo)"])[-1])
                print("    " + ig)
                steps.append((NAME[k], CMD[k], took, ig))
                res.update({
                    "J": num(r"J=([\d.]+)e-4", txt),
                    "acc1": num(r"acc=([\d.]+)/", txt),
                    "acc2": num(r"acc=[\d.]+/([\d.]+)", txt),
                    "tq1": num(r"tq=([\d.]+)/", txt),
                    "tq2": num(r"tq=[\d.]+/([\d.]+)", txt),
                })
                if res.get("J") is not None:
                    res["J"] *= 1e-4

            elif k == "cog":
                raw_cog, nrows = parse_cog(txt)
                cog = interp_to(raw_cog, 128) if raw_cog else []
                line = clean((lines_with(txt, "MI end") or ["(no echo)"])[-1])
                pp_txt = num(r"pp=([\d.]+)A", txt)
                print("    %d/%d 点（%d 行）   pp=%s A"
                      % (len(raw_cog), 128, nrows, fmt(pp_txt)))
                if raw_cog and len(raw_cog) != 128:
                    print("    [WARN] 少 %d 点，已线性插值补齐到 128"
                          "（缺口处不会留台阶）" % (128 - len(raw_cog)))

                # 装表结果用回读验证，不看回显里的 "->cog ok"：
                # 那一行末尾会被遥测帧挤掉（实测收到 "rev=1.20" 就断了），
                # 用字符串匹配会把"装成功"误判成"被拒绝"。
                cq = lk.query(":")
                n_now = num(r"\bn=(\d+)", cq, int)
                v_now = num(r"\bv=(\d)", cq, int)
                pp_now = num(r"pp=([\d.]+)A", cq)
                zd_now = num(r"zd=(-?[\d.]+)deg", cq)
                tol = max(0.05, 0.1 * (pp_txt or 0.0))
                loaded = (v_now == 1 and n_now == len(raw_cog)
                          and pp_txt is not None and pp_now is not None
                          and abs(pp_now - pp_txt) <= tol)
                print("    回读 %s" % cq)
                print("    -> %s（判据：v=1 且 n=%d 且 pp≈%.3f）"
                      % ("装载成功" if loaded else "未装载", len(raw_cog), pp_txt or 0))

                steps.append((NAME[k], CMD[k], took, "%d/%d 点" % (len(raw_cog), 128)))
                res.update({
                    "cog_n": len(raw_cog),
                    "cog_pp": pp_txt,
                    "cog_pp_fwd": num(r"fwd=([\d.]+)", txt),
                    "cog_pp_rev": num(r"rev=([\d.]+)", txt),
                    "cog_dc": num(r"dc=(-?[\d.]+)", txt),
                    "cog_load": 1 if loaded else 0,
                    "cog_n_flash": n_now,
                    "cog_v": v_now,
                    "cog_zd_deg": zd_now,
                })

        # ---------------- 判定 ----------------
        step(len(todo) + 2, len(todo) + 2, "判定")
        checks = []

        if "fc" in todo:
            if res.get("fc_ok"):
                i1, i2 = res.get("fc_iq1"), res.get("fc_iq2")
                if i1 and i2 and i2 > i1 * 1.02:
                    checks.append(("$ 电流随转速上升", "PASS",
                                   "3 rpm %.3f A -> 100 rpm %.3f A" % (i1, i2)))
                else:
                    checks.append(("$ 电流随转速上升", "WARN",
                                   "%.3f -> %s，黏滞项不显著（可能是低黏润滑脂）"
                                   % (i1, fmt(i2))))
                tc = res.get("fc_tc")
                if tc is not None and 0.002 <= tc <= 0.5:
                    checks.append(("$ 库仑转矩 Tc", "PASS", "%.4f N·m" % tc))
                else:
                    checks.append(("$ 库仑转矩 Tc", "WARN", "%s 超出 2–500 mN·m" % fmt(tc, 4)))
            else:
                checks.append(("$ 完成", "WARN",
                               "未通过，摩擦前馈未更新（其余四项不受影响）"))

        if sweep:
            if res.get("sw_wmean") is not None and res["sw_wmean"] > 0.01:
                checks.append(("( 测速通道", "PASS",
                               "wmean=%.4f rad/s（差分通道有响应）" % res["sw_wmean"]))
            else:
                checks.append(("( 测速通道", "FAIL",
                               "wmean=%s，差分通道没数据（theta 未推进？）"
                               % fmt(res.get("sw_wmean"), 4)))
            if len(sweep) >= 11:
                checks.append(("( 点数", "PASS", "11/11"))
            else:
                checks.append(("( 点数", "FAIL", "%d/11，结果行被顶掉或中途失败" % len(sweep)))
            if len(sweep) >= 2:
                if sweep[0]["g"] > sweep[-1]["g"]:
                    checks.append(("( 增益单调下降", "PASS",
                                   "%.4f @%.0fHz -> %.4f @%.0fHz（惯性环节应有 -20 dB/dec）"
                                   % (sweep[0]["g"], sweep[0]["f"],
                                      sweep[-1]["g"], sweep[-1]["f"])))
                else:
                    checks.append(("( 增益单调下降", "WARN",
                                   "低频 %.4f 未高于高频 %.4f，检查 dir 与解调"
                                   % (sweep[0]["g"], sweep[-1]["g"])))
                p0 = sweep[0]["ph"]
                if -120.0 <= p0 <= -60.0:
                    checks.append(("( 低频相位", "PASS", "%.1f°（惯性环节期望 -90°）" % p0))
                elif -150.0 <= p0 <= -30.0:
                    checks.append(("( 低频相位", "WARN", "%.1f°，偏离 -90° 较大" % p0))
                else:
                    checks.append(("( 低频相位", "WARN",
                                   "%.1f°，相位定义或延迟补偿可能有问题" % p0))

        if fric:
            if len(fric) >= 8:
                checks.append((") 点数", "PASS", "8/8"))
            else:
                checks.append((") 点数", "FAIL", "%d/8" % len(fric)))
            # 只判 ≥12 rpm：低速段是 Stribeck 区（静摩擦 > 动摩擦），iq 本来
            # 就随转速先降后升，拿"单调不减"去要求 1–6 rpm 会必然误报。
            hi = [p for p in fric if p["rpm"] >= 12.0]
            if len(hi) >= 2:
                inc = sum(1 for i in range(1, len(hi)) if hi[i]["iq"] >= hi[i - 1]["iq"] * 0.9)
                checks.append((") 黏滞段 iq 递增", "PASS" if inc >= len(hi) - 1 else "WARN",
                               "%d/%d 段满足（只看 ≥12 rpm）" % (inc, len(hi) - 1)))
                fit = linfit([p["rpm"] * math.pi / 30.0 for p in hi], [p["iq"] for p in hi])
                if fit:
                    a0, b0, r2 = fit
                    checks.append((") 黏滞段回归", "PASS" if r2 >= 0.80 else "WARN",
                                   "fric=%.4f A  visc=%.5f A/(rad/s)  R²=%.3f（4 点）"
                                   % (a0, b0, r2)))
                    res["fr_fit_a"] = a0
                    res["fr_fit_visc"] = b0
                    res["fr_fit_r2"] = r2
            lo = [p for p in fric if p["rpm"] < 12.0]
            if lo:
                checks.append((") 低速段", "PASS",
                               "1–6 rpm 电流 %.3f–%.3f A，Stribeck 区不参与线性拟合"
                               % (min(p["iq"] for p in lo), max(p["iq"] for p in lo))))

        if res.get("J") is not None:
            j = res["J"]
            if 1e-6 <= j <= 0.1:
                checks.append(("< 折算惯量 J", "PASS", "%.3e kg·m²" % j))
            else:
                checks.append(("< 折算惯量 J", "FAIL", "%.3e 超出 1e-6..0.1" % j))
            if res.get("acc2") and res.get("acc1") and res["acc2"] > res["acc1"]:
                checks.append(("< 两档加速度", "PASS",
                               "%.2f < %.2f rad/s²（2 A 应更快）"
                               % (res["acc1"], res["acc2"])))
            else:
                checks.append(("< 两档加速度", "WARN",
                               "acc1=%s acc2=%s，差分法要求 acc2>acc1"
                               % (fmt(res.get("acc1")), fmt(res.get("acc2")))))

        if cog:
            n_raw = res.get("cog_n", 0)
            if n_raw >= 128:
                checks.append(("} 箱数", "PASS", "128/128"))
            else:
                checks.append(("} 箱数", "WARN",
                               "%d/128，已线性插值补齐（缺口处精度略降）" % n_raw))
            pp = res.get("cog_pp")
            if pp is not None and pp > 0.02:
                checks.append(("} 峰峰值", "PASS", "%.3f A" % pp))
            else:
                checks.append(("} 峰峰值", "WARN",
                               "%s A，接近噪声（齿槽本来就小或分箱错位）" % fmt(pp)))
            if res.get("cog_pp_fwd") and res.get("cog_pp_rev") and pp:
                d = abs(res["cog_pp_fwd"] - res["cog_pp_rev"]) / pp * 100.0
                checks.append(("} 正反一致性", "PASS" if d <= 30.0 else "WARN",
                               "fwd %.3f / rev %.3f，差 %.0f%%（判据 ≤30%%）"
                               % (res["cog_pp_fwd"], res["cog_pp_rev"], d)))
                res["cog_fr_dev"] = d
            if res.get("cog_dc") is not None and pp:
                d = abs(res["cog_dc"]) / pp * 100.0
                checks.append(("} 直流残留", "PASS" if d <= 30.0 else "WARN",
                               "dc=%.4f A，占 pp 的 %.0f%%（两向平均应抵消摩擦常值）"
                               % (res["cog_dc"], d)))
            checks.append(("} 装表", "PASS" if res.get("cog_load") else "FAIL",
                           ("回读 n=%s v=1 pp=%.2fA，零点偏差 %s°"
                            % (res.get("cog_n_flash"), res.get("cog_pp") or 0,
                               fmt(res.get("cog_zd_deg"), 2)))
                           if res.get("cog_load") else
                           "回读 v=%s，零点偏差 %s° —— 零点不一致（重做过 Z？）或表未装入"
                           % (res.get("cog_v"), fmt(res.get("cog_zd_deg"), 2))))

        overall = verdict(checks)
        print()
        for name, tag, note in checks:
            print("  %s %-18s %s" % (MARK[tag], name, note))
        print("\n  综合判定：%s" % overall)

        # ---------------- 落盘 ----------------
        if not a.no_save:
            save_flash(lk)
        else:
            print("\n[保存] --no-save，跳过 A")

        st_end = lk.status()
        res["verdict"] = overall
        res["vbus"] = st_end.get("v")
        res["fault_after"] = st_end.get("f")

        # ---------------- 报告 ----------------
        rep = build_report(a, res, checks, overall, steps, sweep, fric, cog, todo)
        p_md = os.path.join(HERE, "机械参数辨识_报告.md")
        save_text(p_md, rep)
        p_raw = os.path.join(HERE, "ident_mech_raw.log")
        save_text(p_raw, "\n\n".join("=== %s ===\n%s" % (k, t) for k, t in raw))

        if cog:
            p_cog = os.path.join(HERE, "cog_table_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
            rows = ["idx,deg_from_start,iq_A"]
            for i, v in enumerate(cog):
                rows.append("%d,%.2f,%.4f" % (i, i * 360.0 / len(cog), v))
            save_text(p_cog, "\n".join(rows))
            print("\n齿槽表：%s" % p_cog)

        cols = ["t", "tag", "port", "fc_iq1", "fc_iq2", "fc_visc", "fc_tc",
                "sw_n", "fr_n", "fr_a", "fr_visc", "fr_fit_r2",
                "J", "acc1", "acc2", "cog_n", "cog_pp", "cog_pp_fwd", "cog_pp_rev",
                "cog_dc", "verdict", "vbus", "fault_after"]
        append_csv(os.path.join(HERE, "ident_mech_history.csv"), cols, res)

        print("\n报告：%s" % p_md)
        print("原始：%s" % p_raw)
        print("历史：%s" % os.path.join(HERE, "ident_mech_history.csv"))
        return 0 if overall != "FAIL" else 4

    finally:
        lk.close()


def build_report(a, res, checks, overall, steps, sweep, fric, cog, todo):
    L = []
    L.append("# 机械参数辨识报告")
    L.append("")
    L.append("**时间** %s　**端口** %s　**标签** %s　**综合判定** `%s`"
             % (res["t"], res["port"], res.get("tag") or "—", overall))
    L.append("")
    L.append("## 一、结论")
    L.append("")
    if overall == "PASS":
        L.append("全部判据通过。摩擦前馈可直接用于速度环，齿槽表已装入 CogFf 并随 `A` 落盘。")
    elif overall == "WARN":
        L.append("主体可用，有警告项（见判据明细）。警告项多为\"这台机器的固有特性\""
                 "而不是测错，例如低黏润滑脂下 `$` 的 iq2 上升不显著、"
                 "或 20 Hz 的相位落在 -60° 而不是理想的 -90°。")
        L.append("")
        L.append("**齿槽表即使有 WARN 也已装入缓存**；若警告涉及 `正反一致性` 或 "
                 "`直流残留`，建议先复核再用于前馈 —— 相位错的表比不补更糟。")
    else:
        L.append("有判据失败，**不建议**把齿槽表用于前馈，也不要据此改摩擦前馈。"
                 "先排查：点数缺失通常是解析问题（遥测二进制插行），"
                 "`->cog REJECT` 则是重做过 Z 导致零点变了。")

    L.append("")
    L.append("## 二、结果汇总")
    L.append("")
    rows = []
    if "fc" in todo:
        if res.get("fc_ok"):
            rows.append(["摩擦两点", "$",
                         "iq1=%.3f A @%.1f rpm → iq2=%.3f A @%.1f rpm" % (
                             res.get("fc_iq1") or 0, res.get("fc_rpm1") or 0,
                             res.get("fc_iq2") or 0, res.get("fc_rpm2") or 0),
                         "Tc=%.4f N·m  visc=%.5f" % (res.get("fc_tc") or 0, res.get("fc_visc") or 0)])
        else:
            rows.append(["摩擦两点", "$", "未通过（见判据明细）",
                         "摩擦前馈未更新，其余四项不受影响"])
    if "sweep" in todo:
        rows.append(["谐振扫频", "(",
                     "%s 点，%.0f–%.0f Hz" % (res.get("sw_n", 0),
                                              sweep[0]["f"] if sweep else 0,
                                              sweep[-1]["f"] if sweep else 0),
                     "wmean=%s" % fmt(res.get("sw_wmean"), 4)])
    if "fric2" in todo:
        rows.append(["摩擦曲线", ")",
                     "%s 点，1–200 rpm" % res.get("fr_n", 0),
                     "fric_a=%s A  visc=%s" % (fmt(res.get("fr_a"), 4), fmt(res.get("fr_visc"), 5))])
    if "inertia" in todo:
        rows.append(["折算惯量", "<",
                     "J=%.3e kg·m²" % (res.get("J") or 0),
                     "acc %.2f → %.2f rad/s²" % (res.get("acc1") or 0, res.get("acc2") or 0)])
    if "cog" in todo:
        rows.append(["齿槽转矩", "}",
                     "%s/128 点，pp=%.3f A" % (res.get("cog_n", 0), res.get("cog_pp") or 0),
                     "fwd %.3f / rev %.3f  dc=%.4f" % (res.get("cog_pp_fwd") or 0,
                                                       res.get("cog_pp_rev") or 0,
                                                       res.get("cog_dc") or 0)])
    L.append(md_table(["环节", "命令", "主结果", "副结果"], rows))

    L.append("")
    L.append("## 三、判据明细")
    L.append("")
    L.append(md_table(["判据", "档位", "说明"], [[n, t, d] for n, t, d in checks]))

    if sweep:
        L.append("")
        L.append("## 谐振扫频（Bode）")
        L.append("")
        L.append("传递函数是 **ω/i**（电流到速度，单位 rad/s per A）。"
                 "纯惯性段应为 -20 dB/dec、相位 -90°；偏离处即机械谐振或电流环衰减。")
        L.append("")
        L.append(md_table(["#", "频率 Hz", "增益 rad/s per A", "相位 °"],
                          [[i + 1, "%.1f" % p["f"], "%.4f" % p["g"], "%.1f" % p["ph"]]
                           for i, p in enumerate(sweep)]))
        L.append("")
        L.append("> 增益分母用的是**实测 iq 幅值**（不是给定幅值），"
                 "所以高频段已经扣掉了电流环自身的衰减与滞后 —— 这张表反映的是机械环节。")
        L.append("> 相位已补偿 8 拍差分的群延迟（`+360·f·N·Ts/2`）与 sinc 低通（`/sinc`）。")

    if fric:
        L.append("")
        L.append("## 摩擦曲线")
        L.append("")
        rows3 = []
        for p in fric:
            w = p["rpm"] * math.pi / 30.0
            rows3.append(["%.1f" % p["rpm"], "%.1f" % p["rpm_act"],
                          "%.4f" % p["iq"], "%.4f" % (p["iq"] / w if w > 0.01 else 0)])
        L.append(md_table(["给定 rpm", "实测 rpm", "稳态 iq A", "iq/ω A/(rad/s)"], rows3))
        if res.get("fr_fit_r2") is not None:
            L.append("")
            L.append("**黏滞段（≥12 rpm 的 4 点）线性回归** `iq = fric + visc·ω`："
                     "fric=%.4f A，visc=%.5f A/(rad/s)，R²=%.3f"
                     % (res["fr_fit_a"], res["fr_fit_visc"], res["fr_fit_r2"]))
            L.append("")
            L.append("R² 不高是正常的：真实摩擦是 Stribeck 曲线（低速段有负斜率），"
                     "用直线拟合必然有残差。要更准就把 8 点表直接做查表前馈，"
                     "而不是只用回归的两个系数。")
            if res.get("fr_a") is not None:
                fv = res.get("fr_visc")
                L.append("")
                L.append("**固件自己的回归（全 8 点）**：fric_a=%.4f A、visc=%.6f A/(rad/s)。"
                         % (res["fr_a"], fv or 0))
                L.append("")
                if fv is not None and fv < 0:
                    L.append("注意 visc **为负** —— 这不是噪声，而是把 1–6 rpm 的 Stribeck 区"
                             "也算进线性回归的必然结果：静摩擦大于动摩擦，低速段 iq 随转速"
                             "下降，负斜率盖过了高速段的正常黏滞上升。")
                    L.append("")
                    L.append("**不要把这个 visc 直接当速度环的黏滞前馈系数**（会让低速段前馈反向）。"
                             "上表的「黏滞段回归」只用 ≥12 rpm 的 4 个点，那才是真正的黏滞斜率。")
                else:
                    L.append("与上表「黏滞段回归」应当接近；差得多说明低速 Stribeck 区"
                             "把整体斜率拉偏了（8 点拟合 vs 4 点拟合）。")

    if cog:
        L.append("")
        L.append("## 齿槽转矩")
        L.append("")
        pp = res.get("cog_pp") or 0
        L.append("| 指标 | 值 | 说明 |")
        L.append("|---|---|---|")
        L.append("| 分箱数 | %d/%d | 每 %.2f° 一箱%s |"
                 % (len(cog), 128, 360.0 / len(cog),
                    "" if res.get("cog_n", 0) >= 128 else "（实收 %d 点，已插值）" % res.get("cog_n", 0)))
        L.append("| 峰峰值 pp | %.3f A | %.4f N·m（Kt=0.0910） |" % (pp, pp * 0.0910))
        L.append("| 正向 pp | %.3f A | 单圈，含摩擦常值偏置 |" % (res.get("cog_pp_fwd") or 0))
        L.append("| 反向 pp | %.3f A | 单圈，摩擦偏置反号 |" % (res.get("cog_pp_rev") or 0))
        L.append("| 直流残留 dc | %.4f A | 两向平均后应≈0，偏大说明摩擦没抵消干净 |"
                 % (res.get("cog_dc") or 0))
        L.append("| 极值位置 | 最大 %d 箱 / 最小 %d 箱 | 约 %.1f° / %.1f° |"
                 % (cog.index(max(cog)), cog.index(min(cog)),
                    cog.index(max(cog)) * 360.0 / len(cog),
                    cog.index(min(cog)) * 360.0 / len(cog)))
        L.append("")
        L.append("128 点表已单独存为 `cog_table_*.csv`，可直接与 CogFf 里的表对照。")
        L.append("")
        L.append("**装表结果是用 `:` 回读验证的**，不是看回显里的 `->cog ok` —— "
                 "那一行末尾会被 100 Hz 遥测帧挤掉（实测只收到 `rev=1.20` 就断了），"
                 "用字符串匹配会把\"装成功\"误判成\"被拒绝\"。")
        L.append("")
        L.append("回读 `v=1` 且 `n`/`pp` 与本次一致，才说明表真的进了缓存。"
                 "若 `v=0`，看 `zd`（零点偏差）：超过 1° 说明重做过 Z，表的相位基准已失效，"
                 "必须重测齿槽 —— 相位错的表会把扰动放大一倍，比不补更糟。")

    L.append("")
    L.append("## 步骤与耗时")
    L.append("")
    L.append(md_table(["步骤", "命令", "耗时 s", "结果"],
                      [[n, c, "%.1f" % t, ln] for n, c, t, ln in steps]))

    L.append("")
    L.append("## 复现")
    L.append("")
    L.append("```")
    L.append("python ident_mech.py %s%s" % (
        a.port, (" --skip " + a.skip) if a.skip else ""))
    L.append("```")
    L.append("")
    L.append("原始回显见 `ident_mech_raw.log`，历史追加到 `ident_mech_history.csv`。")
    L.append("")
    L.append("---")
    L.append("")
    L.append("### 为什么惯量 J 只能当量级看")
    L.append("")
    L.append("`mech_ident.c` 里换转矩用的是硬编码 `MI2_KT = 0.0910`，"
             "而 ψ 实测在 4.295–4.366 mWb 之间浮动，对应 Kt 0.0902–0.0917，"
             "**±1.6% 直接进 J**。所以：")
    L.append("")
    L.append("1. 先跑 `ident_elec.py` 拿到 ψ，再跑本脚本，J 才有意义；")
    L.append("2. J 的重复性受 1 rpm 段的速度环积分建立时间影响，"
             "两次测量差 5% 属正常；")
    L.append("3. J 只用于加速度前馈与增益调度，不参与稳定性判据 —— "
             "差 10% 不会让环路失稳。")
    return "\n".join(L)


if __name__ == "__main__":
    sys.exit(main())
