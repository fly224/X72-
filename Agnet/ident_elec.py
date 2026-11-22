# -*- coding: utf-8 -*-
"""ident_elec.py —— 电气参数辨识（一键）

一条命令跑完电气侧全部辨识，自动交叉验证并给判定：

    C  电流零点（按需）      -> 偏置，所有电流量的前提
    Z  极对数 / 方向 / 零点  -> 编码器与电角度的对齐关系
    @  相电阻 R / dq 电感 L  -> 虚拟角锁死，静态法，R 准
    +  磁链 ψ / R / L 联合   -> 实际转速下测，ψ 准

为什么把 @ 和 + 都跑一遍而不是二选一：
    两者口径不同，能互相验证。
      · @ 在 ωe=0 下测 R（不含 ωe·ψ 与 ωe·L·i 项），R 准到 ±1.2%；
        L 只能用 1 kHz 注入，测的是小信号 Ld。
      · + 在实际工作点测，R/L 里混入了死区电压与饱和，但 ψ 只有它测得出。
    判据就建立在"两个独立口径应当接近"上：R 差 ≤10%、L 差 ≤25%。
    只跑一个的话，得到的数字没有任何参照，错了也看不出来。

用法：
    python ident_elec.py                  # COM27，全套
    python ident_elec.py COM5
    python ident_elec.py --force-align    # 即使 z=1 也重做 Z（换编码器/换电机后）
    python ident_elec.py --no-save        # 只测不写 Flash
    python ident_elec.py --dry-run        # 只读状态、打印计划，不动电机

产物：
    控制台表 + 电气参数辨识_报告.md + ident_elec_history.csv

安全：Z 与 + 都会让转子转动（Z 正反各一圈；+ 转到 1000 rpm 约 9 s）。
      运行前确认联轴器/负载侧无人、无限位。
"""
import argparse
import os
import re
import sys
import time

from ident_lib import (Link, banner, step, precheck, save_flash, num, lines_with,
                       clean, rel_pct, verdict, MARK, md_table, save_text,
                       append_csv, HERE)

FW_DIR = os.path.join(HERE, "..", "Firmware", "FM_Servo_V3",
                      "embedded_project", "STM32G474_App", "Board")

# 固件常量抓不到时的兜底（2026-11-18 实测值）
FW_DEFAULT = {
    "SERVO_CUR_PSI": 4.333e-3,      # Wb
    "SERVO_CUR_LD": 85e-6,          # H
    "SERVO_CUR_KP": 0.5341,
    "SERVO_CUR_KI_TS": 0.12566,
    "SERVO_KT_NM_PER_A": 0.0910,
}


def read_fw_consts():
    """从 servo_isr.c 直接抓常量，避免脚本与固件脱节。

    为什么值得做：SERVO_CUR_PSI 从 7.593 mWb 改成 4.333 mWb 那次，
    模型路径上的副本没跟着改，两边差了 75% 却一直没人发现。
    脚本硬编码常量会重演同一个问题。
    """
    consts = dict(FW_DEFAULT)
    src = {}
    for fn in ("servo_isr.c", "servo_isr.h"):
        p = os.path.join(FW_DIR, fn)
        if os.path.exists(p):
            try:
                src[fn] = open(p, encoding="utf-8", errors="replace").read()
            except Exception:
                pass
    blob = "\n".join(src.values())
    for k in list(consts):
        m = re.search(r"#define\s+" + k + r"\s+\(?\s*([0-9.eE+-]+)f?\s*\)?", blob)
        if m:
            try:
                consts[k] = float(m.group(1))
            except ValueError:
                pass
    return consts


def fmt(v, nd=3, unit=""):
    if v is None:
        return "—"
    return ("%." + str(nd) + "f%s") % (v, unit)


def main():
    ap = argparse.ArgumentParser(description="电气参数辨识（一键）")
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--force-align", action="store_true", help="z=1 也重做 Z")
    ap.add_argument("--no-save", action="store_true", help="不写 Flash")
    ap.add_argument("--dry-run", action="store_true", help="只读状态，不启动激励")
    ap.add_argument("--tag", default="", help="写入 CSV 的标签，如 cold/hot")
    a = ap.parse_args()

    fw = read_fw_consts()

    banner("电气参数辨识（一键）",
           "port=%s   固件常量 ψ=%.3f mWb  L=%.1f µH  Kp=%.4f  KiTs=%.4f  Kt=%.4f"
           % (a.port, fw["SERVO_CUR_PSI"] * 1e3, fw["SERVO_CUR_LD"] * 1e6,
              fw["SERVO_CUR_KP"], fw["SERVO_CUR_KI_TS"], fw["SERVO_KT_NM_PER_A"]))

    try:
        lk = Link(a.port)
    except Exception as e:
        print("[FAIL] 打不开串口 %s：%s" % (a.port, e))
        return 2

    try:
        # ---------------- 前置 ----------------
        step(1, 4, "前置联锁（停机 / 清故障 / C / Z）")
        ok, st, log, zres = precheck(lk, allow_align=True, force_align=a.force_align)
        if not ok:
            return 2

        if a.dry_run:
            print("\n[dry-run] 前置已通过，计划的激励如下（未执行）：")
            print("    @  R/L 在线辨识        约 0.5 s   虚拟角锁死 + 1 kHz 注入")
            print("    +  ψ/R/L 联合辨识      约 9   s   1000 rpm，id 扫 0/-1/-2/-3 A")
            print("    合计约 10 s，其中 + 转子转到 1000 rpm")
            return 0

        res = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "tag": a.tag, "port": a.port}
        res.update(zres)
        steps = list(log)
        raw = []

        # ---------------- @ ----------------
        step(2, 4, "@ 相电阻 / dq 电感（虚拟角锁死，约 0.5 s）")
        txt, took, refused = lk.run("@", ["RL ok", "RL FAIL"], 10.0, "R/L")
        raw.append(("@", txt))
        line = clean((lines_with(txt, "RL ") or ["(no echo)"])[-1])
        print("    " + line)
        steps.append(("@ R/L", "@", took, line))
        if refused:
            print("    [FAIL] 被拒收：" + refused)
            return 3
        rl_ok = "RL ok" in txt
        res.update({
            "R_at": num(r"R=([\d.]+)mohm", txt),
            "L_at": num(r"L=([\d.]+)uH", txt),
            "Z_ohm": num(r"Z=([\d.]+)ohm", txt),
            "iamp": num(r"iamp=([\d.]+)A", txt),
            "rl_n": num(r"\bn=(\d+)", txt, int),
            "v_avg": num(r"\|v\|=([\d.]+)V", txt),
            "i_avg": num(r"\|i\|=([\d.]+)A", txt),
            "rl_ok": 1 if rl_ok else 0,
        })
        if not rl_ok:
            print("    [FAIL] @ 失败，后续 + 仍可单独给出 ψ，但无法交叉验证")

        # ---------------- + ----------------
        step(3, 4, "+ ψ/R/L 联合辨识（1000 rpm，约 9 s，转子会转）")
        txt2, took2, refused2 = lk.run("+", ["FI Kt=", "FI FAIL"], 30.0, "ψ 辨识")
        raw.append(("+", txt2))
        for ln in lines_with(txt2, "FI R=") + lines_with(txt2, "FI Kt=") + lines_with(txt2, "FI FAIL"):
            print("    " + ln)
        steps.append(("+ ψ/R/L", "+", took2, (lines_with(txt2, "FI Kt=") or ["(no echo)"])[-1]))
        if refused2:
            print("    [FAIL] 被拒收：" + refused2)
        fl_ok = "FI Kt=" in txt2
        res.update({
            "R_fl": num(r"FI R=([\d.]+)mohm", txt2),
            "L_fl": num(r"mohm L=([\d.]+)uH", txt2),
            "psi": num(r"uH psi=([\d.]+)mWb", txt2),
            "Kt": num(r"FI Kt=([\d.]+)", txt2),
            "Ke": num(r"Ke=([\d.]+)", txt2),
            "r2d": num(r"r2=([\d.]+)/", txt2),
            "r2q": num(r"r2=[\d.]+/([\d.]+)", txt2),
            "dev": num(r"dev=([\d.]+)%", txt2),
            "fl_ok": 1 if fl_ok else 0,
            "pts": num(r"FI ok pts=(\d+)", txt2, int),
        })
        if not fl_ok:
            print("    [FAIL] + 未给出完整结果")
            m = re.search(r"FI FAIL ([^\r\n]*)", txt2)
            if m:
                print("    原因：" + m.group(0).strip())
                print("    诊断：f=0x.. 是故障位，om/wref 是失败瞬间的实测/给定转速；")
                print("          om≈0 而 wref=104 说明速度环没转起来（方向/极对数/联锁）。")

        # ---------------- 判定 ----------------
        step(4, 4, "交叉验证与判定")
        checks = []

        if rl_ok:
            r = res["R_at"]
            if r is not None and 50.0 <= r <= 2000.0:
                checks.append(("@ R 量程", "PASS", "%.1f mΩ 在 50–2000 mΩ" % r))
            else:
                checks.append(("@ R 量程", "FAIL", "R=%s 超出 50–2000 mΩ" % fmt(r, 1)))
            l = res["L_at"]
            if l is not None and 5.0 <= l <= 3000.0:
                checks.append(("@ L 量程", "PASS", "%.1f µH 在 5–3000 µH" % l))
            else:
                checks.append(("@ L 量程", "FAIL", "L=%s 超出 5–3000 µH" % fmt(l, 1)))
        else:
            checks.append(("@ 完成", "FAIL", "未给出 RL ok"))

        if fl_ok:
            if res["r2d"] is not None and res["r2d"] >= 0.99:
                checks.append(("+ r²(vd)", "PASS", "%.3f ≥ 0.99" % res["r2d"]))
            else:
                checks.append(("+ r²(vd)", "WARN", "%s < 0.99，R 的线性度不足" % fmt(res["r2d"])))
            if res["r2q"] is not None and res["r2q"] >= 0.99:
                checks.append(("+ r²(vq)", "PASS", "%.3f ≥ 0.99" % res["r2q"]))
            else:
                checks.append(("+ r²(vq)", "WARN", "%s < 0.99，L 的线性度不足" % fmt(res["r2q"])))
            if res["dev"] is not None and res["dev"] <= 3.0:
                checks.append(("ψ 双速一致", "PASS", "dev=%.2f%% ≤ 3%%" % res["dev"]))
            elif res["dev"] is not None and res["dev"] <= 8.0:
                checks.append(("ψ 双速一致", "WARN", "dev=%.2f%%（1000 vs 300 rpm）" % res["dev"]))
            else:
                checks.append(("ψ 双速一致", "FAIL", "dev=%s" % fmt(res["dev"], 2, "%")))
        else:
            checks.append(("+ 完成", "FAIL", "未给出 FI Kt="))

        dR = rel_pct(res.get("R_at"), res.get("R_fl"))
        if dR is not None:
            tag = "PASS" if dR <= 10.0 else ("WARN" if dR <= 20.0 else "FAIL")
            checks.append(("R 双口径一致", tag,
                           "@ %.1f vs + %.1f mΩ，差 %.1f%%（判据 ≤10%%）"
                           % (res["R_at"], res["R_fl"], dR)))
        dL = rel_pct(res.get("L_at"), res.get("L_fl"))
        if dL is not None:
            tag = "PASS" if dL <= 25.0 else ("WARN" if dL <= 40.0 else "FAIL")
            checks.append(("L 双口径一致", tag,
                           "@ %.1f vs + %.1f µH，差 %.1f%%（判据 ≤25%%，注入法 vs 饱和）"
                           % (res["L_at"], res["L_fl"], dL)))
        if res.get("psi") is not None:
            dp = rel_pct(res["psi"] / 1e3, fw["SERVO_CUR_PSI"])
            if dp is not None:
                tag = "PASS" if dp <= 5.0 else "WARN"
                checks.append(("ψ vs 固件常量", tag,
                               "%.3f vs %.3f mWb，差 %.1f%%"
                               % (res["psi"], fw["SERVO_CUR_PSI"] * 1e3, dp)))

        overall = verdict(checks)
        print()
        for name, tag, note in checks:
            print("  %s %-14s %s" % (MARK[tag], name, note))
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
        rep = build_report(a, fw, res, checks, overall, steps)
        p_md = os.path.join(HERE, "电气参数辨识_报告.md")
        save_text(p_md, rep)
        p_raw = os.path.join(HERE, "ident_elec_raw.log")
        save_text(p_raw, "\n\n".join("=== %s ===\n%s" % (c, t) for c, t in raw))

        cols = ["t", "tag", "port", "ratio", "pn", "dir", "off_deg", "R_at", "L_at",
                "R_fl", "L_fl", "psi", "Kt", "Ke", "r2d", "r2q", "dev",
                "verdict", "vbus", "fault_after"]
        append_csv(os.path.join(HERE, "ident_elec_history.csv"), cols, res)

        print("\n报告：%s" % p_md)
        print("原始：%s" % p_raw)
        print("历史：%s" % os.path.join(HERE, "ident_elec_history.csv"))
        return 0 if overall != "FAIL" else 4

    finally:
        lk.close()


def build_report(a, fw, res, checks, overall, steps):
    L = []
    L.append("# 电气参数辨识报告")
    L.append("")
    L.append("**时间** %s　**端口** %s　**标签** %s　**综合判定** `%s`"
             % (res["t"], res["port"], res.get("tag") or "—", overall))
    L.append("")
    L.append("## 一、结论")
    L.append("")
    if overall == "PASS":
        L.append("全部判据通过。R、L、ψ 三个量在两种独立口径下一致，可用于更新固件常量或作为热态基线。")
    elif overall == "WARN":
        L.append("主体可用，但有一项以上落在警告区（见第三节）。警告项通常是"
                 "\"口径本身有差异\"而不是\"测错了\"，例如 @ 的 L 是小信号值、"
                 "+ 的 L 含饱和。改常量前建议先复核警告项。")
    else:
        L.append("存在判据失败项，**不建议**把本次结果写进固件常量或用于热态对比。"
                 "先按第三节的失败项排查（多为前置条件或接线问题）。")

    L.append("")
    L.append("## 二、辨识结果")
    L.append("")
    if not res.get("ratio"):
        L.append("> 极对数/方向/零点显示为占位符：本次 `z=1` 沿用了上次结果，没有重做 Z。")
        L.append("> 要刷新这组值用 `--force-align`（会让转子正反各转一圈，约 9 s）。")
        L.append("")
    rows = [
        ["极对数", ("ratio=%.3f → pn=%s" % (res["ratio"], res.get("pn")))
         if res.get("ratio") else "—（沿用上次）",
         "方向 dir", res.get("dir") if res.get("dir") is not None else "—"],
        ["电角度零点", ("%.2f°" % res["off_deg"]) if res.get("off_deg") is not None else "—（沿用上次）",
         "编码器零点", ("%.2f°" % res["zero_deg"]) if res.get("zero_deg") is not None else "—"],
        ["相电阻 R（@ 静态）", fmt(res.get("R_at"), 1, " mΩ"), "相电阻 R（+ 工作点）", fmt(res.get("R_fl"), 1, " mΩ")],
        ["dq 电感 L（@ 注入）", fmt(res.get("L_at"), 1, " µH"), "dq 电感 L（+ 回归）", fmt(res.get("L_fl"), 1, " µH")],
        ["永磁磁链 ψ", fmt(res.get("psi"), 3, " mWb"), "Kt=1.5·p·ψ", fmt(res.get("Kt"), 4, " N·m/A")],
        ["反电势 Ke", fmt(res.get("Ke"), 4, " V/(rad/s)"), "拟合 r²(vd/vq)",
         "%s / %s" % (fmt(res.get("r2d"), 3), fmt(res.get("r2q"), 3))],
    ]
    L.append(md_table(["量", "本口径", "量", "另一口径"], rows))

    L.append("")
    L.append("## 三、判据明细")
    L.append("")
    L.append(md_table(["判据", "档位", "说明"], [[n, t, d] for n, t, d in checks]))

    L.append("")
    L.append("## 四、与固件常量的偏差")
    L.append("")
    rows2 = []
    if res.get("psi") is not None:
        d = rel_pct(res["psi"] / 1e3, fw["SERVO_CUR_PSI"])
        rows2.append(["`SERVO_CUR_PSI`", "%.3f mWb" % (fw["SERVO_CUR_PSI"] * 1e3),
                      "%.3f mWb" % res["psi"], "%.1f%%" % d if d is not None else "—",
                      "不用改" if d is not None and d <= 5 else "建议复核"])
    if res.get("L_fl") is not None:
        d = rel_pct(res["L_fl"] * 1e-6, fw["SERVO_CUR_LD"])
        rows2.append(["`SERVO_CUR_LD/LQ`", "%.1f µH" % (fw["SERVO_CUR_LD"] * 1e6),
                      "%.1f µH" % res["L_fl"], "%.1f%%" % d if d is not None else "—",
                      "不用改" if d is not None and d <= 10 else "要改就连 Kp 一起"])
    if res.get("R_fl") is not None:
        rows2.append(["隐含 R（由 KiTs 反推）", "—", "%.1f mΩ" % res["R_fl"], "—",
                      "见下方建议"])
    if res.get("Kt") is not None:
        d = rel_pct(res["Kt"], fw["SERVO_KT_NM_PER_A"])
        rows2.append(["`SERVO_KT_NM_PER_A`", "%.4f" % fw["SERVO_KT_NM_PER_A"],
                      "%.4f" % res["Kt"], "%.1f%%" % d if d is not None else "—",
                      "不用改" if d is not None and d <= 5 else "建议改"])
    L.append(md_table(["常量", "当前值", "实测", "偏差", "结论"], rows2))

    L.append("")
    L.append("### 若要改，必须成对改")
    L.append("")
    L.append("电流环的 Kp 与 Ki 是按 L、R 推出来的，单改一个会破坏零极点对消：")
    L.append("")
    if res.get("L_fl"):
        kp_new = fw["SERVO_CUR_KP"] * (res["L_fl"] * 1e-6) / fw["SERVO_CUR_LD"]
        L.append("- `SERVO_CUR_KP` = %.4f × %.1f/%.1f = **%.4f**（现 %.4f）"
                 % (fw["SERVO_CUR_KP"], res["L_fl"], fw["SERVO_CUR_LD"] * 1e6, kp_new, fw["SERVO_CUR_KP"]))
    if res.get("R_fl"):
        r_const = 0.40
        kit_new = fw["SERVO_CUR_KI_TS"] * (res["R_fl"] / 1e3) / r_const
        L.append("- `SERVO_CUR_KI_TS` 若按 R=%.0f mΩ 反推（现常量隐含 0.40 Ω）：**%.5f**（现 %.5f）"
                 % (res["R_fl"], kit_new, fw["SERVO_CUR_KI_TS"]))
    L.append("")
    L.append("改完必须重跑电流阶跃（`U` 2 A）复测 tr/os/ts，不能只看静态量。")

    L.append("")
    L.append("## 五、步骤与耗时")
    L.append("")
    L.append(md_table(["步骤", "命令", "耗时 s", "结果行"],
                      [[n, c, "%.1f" % t, ln] for n, c, t, ln in steps]))

    L.append("")
    L.append("## 六、复现")
    L.append("")
    L.append("```")
    L.append("python ident_elec.py %s%s" % (a.port, " --force-align" if a.force_align else ""))
    L.append("```")
    L.append("")
    L.append("原始回显见 `ident_elec_raw.log`，历史追加到 `ident_elec_history.csv`。")
    L.append("")
    L.append("---")
    L.append("")
    L.append("### 口径说明（为什么 @ 与 + 会不一样）")
    L.append("")
    L.append("- **@ 的 R 偏小**：它累加的是电流环 PI 的限幅输出 vd/vq，而实际加在电机上的"
             "电压还要经 `DeadbandComp_Apply` 在占空比域补一个 du，所以 @ 系统性偏低。")
    L.append("- **@ 的 L 偏小**：1 kHz 注入时铁芯已开始饱和，且死区电压（~0.18 V）混在 vd 里；"
             "它更接近\"额定工况等效 L\"而不是小信号 Ld。")
    L.append("- **+ 的 ψ 才是真值**：ψ 只出现在 ωe·ψ 项里，300 rpm 时该项占 vq 的 97%，"
             "扣掉 R·iq 后精度很高。")
    return "\n".join(L)


if __name__ == "__main__":
    sys.exit(main())
