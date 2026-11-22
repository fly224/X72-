# -*- coding: utf-8 -*-
"""
rl_repeat.py —— 相电阻 R 重复性测试（2026-11-19）

为什么需要：温漂长测里同一温度附近的两次 '@' 辨识给出 304.26 / 294.29 mΩ
（差 3.4%），而整个 22→35 °C 的温度效应只有 4.9%。单次 R 的重复性噪声
与待测信号同量级，必须多次重复取均值才能给出可信的 α_eff。

本脚本在电机静止（disarm）状态下连续做 N 次 R/L 辨识，输出：
  * 每次的 R / L
  * 均值、标准差、极差、变异系数
  * 均值标准误（SEM = σ/√N），用来判断均值能分辨多小的差异

用法：
    python rl_repeat.py COM27 --n 5
    python rl_repeat.py COM27 --n 5 --out rl_repeat_hot.json
"""
import argparse
import json
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, MSG_CTRL_WORD, MSG_PERMISSION, RAD_S_TO_RPM, POLE_PAIRS


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
        # 权限 + 确保封锁（R 辨识要求 D 状态）
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.2)
        rig.send(MSG_CTRL_WORD, bytes([0, 0, 1]))
        time.sleep(0.8)

        # 等转子停稳
        t0 = time.time()
        while time.time() - t0 < 8.0:
            snap, st, _, _, _ = rig.rx.get_snap()
            w = snap.get("w_elec") if snap else None
            rpm = 0.0 if w is None else abs(w) / POLE_PAIRS * RAD_S_TO_RPM
            if rpm < 3.0:
                break
            time.sleep(0.2)

        snap, _, _, _, _ = rig.rx.get_snap()
        t_mot = snap.get("temp_mot")
        t_drv = snap.get("temp_drv")
        print("起始温度：Tmot=%.2f °C  Tdrv=%.2f °C" % (t_mot or 0, t_drv or 0))

        rs, ls = [], []
        for i in range(args.n):
            r = rig.rl_ident()
            if r["r_mohm"]:
                rs.append(r["r_mohm"])
                if r["l_uh"]:
                    ls.append(r["l_uh"])
                print("  #%d  R=%.2f mΩ  L=%s uH" % (i + 1, r["r_mohm"], r["l_uh"]))
            else:
                print("  #%d  失败: %s" % (i + 1, r["raw"][:80]))
            time.sleep(1.2)

        out = {"t_mot": t_mot, "t_drv": t_drv, "n": len(rs), "r": rs, "l": ls}
        if len(rs) >= 2:
            m = statistics.mean(rs)
            sd = statistics.pstdev(rs)
            out.update({"mean": m, "sd": sd, "min": min(rs), "max": max(rs),
                        "cv_pct": sd / m * 100, "sem": sd / (len(rs) ** 0.5),
                        "range_pct": (max(rs) - min(rs)) / m * 100})
            print("\nR 统计：均值 %.2f mΩ  σ %.2f  极差 %.2f%%  变异系数 %.2f%%  SEM %.2f"
                  % (m, sd, out["range_pct"], out["cv_pct"], out["sem"]))
            if ls:
                print("L 统计：均值 %.2f uH  σ %.2f" % (statistics.mean(ls), statistics.pstdev(ls)))
        print(json.dumps(out, ensure_ascii=False))

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
