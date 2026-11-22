# -*- coding: utf-8 -*-
"""
psi_repeat.py —— ψ（磁链）/R/L 联合辨识重复性 + 温度点采集（2026-11-19）

背景：`+` 命令（FluxIdent）一次辨识给出 R/L/ψ，但它必须在 D 状态启动，
内部自己拉起速度环跑 1000→300 rpm、注入 id 0/-1/-2/-3，约 9 s，结束封 MOE。
绕过 do_speed 因此不设 s_armed，deadman 不会递减（servo_cmd.c:2775 在 s_armed=0 直接 return），
所以辨识期间无需保活。

为什么先做重复性：ψ 的温度系数只有约 -0.11 %/K，52→30 °C 的 22 K 跨度也只有 2.4%。
若单次辨识的重复性 CV ≥ 1%，曲线就是噪声。必须先量化 σ 再决定温度跨度与平均次数。

用法：
    python psi_repeat.py COM27 --n 5                    # 同温重复性
    python psi_repeat.py COM27 --n 3 --out psi_50c.json
"""
import argparse
import json
import re
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import (Rig, strip_frames, MSG_CTRL_WORD, MSG_PERMISSION,
                          RAD_S_TO_RPM, POLE_PAIRS)

FI_OK_RE = re.compile(r"FI R=([\d.]+)mohm L=([\d.]+)uH psi=([\d.]+)mWb")
FI_ANY_RE = re.compile(r"FI (?:R=|FAIL|refused)")


def fi_ident(rig, timeout=24.0):
    """发 ASCII '+' 做 ψ/R/L 联合辨识。

    必须独占串口：辨识会连续吐 n_pts 行 [FI] 明细 + 结果行，
    读线程并行 read 会把这些 ASCII 撕碎。
    """
    rig.rx.pause_evt.set()
    time.sleep(0.08)
    try:
        try:
            rig.s.reset_input_buffer()
        except Exception:
            pass
        with rig.wlock:
            rig.s.write(b"+")
            rig.s.flush()
        t0 = time.time()
        raw = bytearray()
        txt = ""
        while time.time() - t0 < timeout:
            d = rig.s.read(8192)
            if d:
                raw += d
            txt = strip_frames(raw).decode("ascii", "replace")
            if FI_ANY_RE.search(txt):
                # 结果行可能在明细之后，再多收 0.3 s 确保完整
                time.sleep(0.3)
                d = rig.s.read(8192)
                if d:
                    raw += d
                txt = strip_frames(raw).decode("ascii", "replace")
                break
            time.sleep(0.02)
        m = FI_OK_RE.search(txt)
        if m:
            return {"r_mohm": float(m.group(1)), "l_uh": float(m.group(2)),
                    "psi_mwb": float(m.group(3)), "ok": True, "raw": txt.strip()}
        return {"r_mohm": None, "l_uh": None, "psi_mwb": None, "ok": False,
                "raw": txt.strip()[-400:]}
    finally:
        try:
            rig.s.reset_input_buffer()
        except Exception:
            pass
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def wait_still(rig, max_s=10.0, thr_rpm=3.0):
    t0 = time.time()
    while time.time() - t0 < max_s:
        snap, st, _, _, _ = rig.rx.get_snap()
        w = snap.get("w_elec") if snap else None
        rpm = 0.0 if w is None else abs(w) / POLE_PAIRS * RAD_S_TO_RPM
        if rpm < thr_rpm:
            return True, rpm
        time.sleep(0.2)
    return False, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--n", type=int, default=5)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    rig = Rig(args.port, args.baud)
    try:
        time.sleep(0.8)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.2)
        rig.send(MSG_CTRL_WORD, bytes([0, 0, 1]))     # halt，确保 D 状态
        time.sleep(0.8)

        ok, rpm = wait_still(rig, 10.0)
        snap, _, _, _, _ = rig.rx.get_snap()
        t_mot = snap.get("temp_mot") if snap else None
        t_drv = snap.get("temp_drv") if snap else None
        print("起始状态：静止=%s  Tdrv=%.2f °C  Tmot=%.2f °C"
              % (ok, t_drv or 0, t_mot or 0))

        recs = []
        for i in range(args.n):
            ok, rpm = wait_still(rig, 12.0)
            if not ok:
                print("  #%d 未停稳（rpm=%s），跳过" % (i + 1, rpm))
                continue
            snap, _, _, _, _ = rig.rx.get_snap()
            tm = snap.get("temp_mot") if snap else None
            td = snap.get("temp_drv") if snap else None
            r = fi_ident(rig)
            if r["ok"]:
                recs.append({"psi_mwb": r["psi_mwb"], "r_mohm": r["r_mohm"],
                             "l_uh": r["l_uh"], "t_mot": tm, "t_drv": td})
                print("  #%d  psi=%.3f mWb  R=%.2f mΩ  L=%.2f uH  Tmot=%.2f °C"
                      % (i + 1, r["psi_mwb"], r["r_mohm"], r["l_uh"], tm or 0))
            else:
                print("  #%d  失败: %s" % (i + 1, r["raw"][:160].replace("\n", " | ")))
            time.sleep(0.6)

        out = {"t_mot": t_mot, "t_drv": t_drv, "recs": recs}
        if len(recs) >= 2:
            for key, unit in (("psi_mwb", "mWb"), ("r_mohm", "mΩ"), ("l_uh", "uH")):
                vals = [r[key] for r in recs if r.get(key)]
                if len(vals) < 2:
                    continue
                m = statistics.mean(vals)
                sd = statistics.pstdev(vals)
                out[key] = {"mean": m, "sd": sd, "cv_pct": sd / m * 100,
                            "range_pct": (max(vals) - min(vals)) / m * 100,
                            "sem": sd / (len(vals) ** 0.5)}
                print("%-8s 均值 %.4f %s  σ %.4f  CV %.3f%%  极差 %.3f%%  SEM %.4f"
                      % (key, m, unit, sd, out[key]["cv_pct"], out[key]["range_pct"],
                         out[key]["sem"]))

            # 可分辨的最小温差：ψ 每 K 变 -0.11%，要 3σ 才可信
            if "psi_mwb" in out:
                cv = out["psi_mwb"]["cv_pct"]
                alpha_psi = 0.11          # %/K
                print("\n判读：单次 CV=%.3f%%，ψ 温漂 %.2f %%/K"
                      % (cv, alpha_psi))
                print("      单点 3σ 噪声 = %.3f%%，等效温差 %.1f K"
                      % (3 * cv, 3 * cv / alpha_psi))
                print("      若每点平均 %d 次，噪声降到 %.3f%%，等效温差 %.1f K"
                      % (len(recs), cv / (len(recs) ** 0.5),
                         3 * cv / (len(recs) ** 0.5) / alpha_psi))
        else:
            print("\n[WARN] 有效点不足，无法统计")

        if args.out:
            with open(args.out, "w", encoding="utf-8") as f:
                json.dump(out, f, ensure_ascii=False, indent=2)
            print("已写 %s" % args.out)
        return 0
    finally:
        try:
            rig.send(MSG_CTRL_WORD, bytes([0, 0, 1]))
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    sys.exit(main())
