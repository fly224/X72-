# -*- coding: utf-8 -*-
"""温升步进测试：2000 rpm 恒速，每升温 2 °C 做一次电气辨识，共 30 组。

触发逻辑：
  1) 2000 rpm 闭环运行，0.5 s 读一次电机温度
  2) T >= 上一组基准 + 2.0 °C 时触发一组：
       降速 -> D 封锁（+ 要求 MOE=0）-> '+' 辨识 R/L/psi -> 重新升速 -> 稳定 2 s
       以"升速后温度"作为下一组基准（补偿停机期间的温降）
  3) 30 组 / 电机 100 °C / 驱动 90 °C / 单组等 10 min 未升温 2 K（热稳态）→ 停止

用法：python thermal_step.py COM27 [--rpm 2000] [--dt 2] [--groups 30]
"""
import argparse
import csv
import re
import statistics
import sys
import time
from datetime import datetime

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402
from combo_sweep import cmd, fault, toggle_expect  # noqa: E402

RE_FI = re.compile(r"FI R=([\d.]+)mohm L=([\d.]+)uH psi=([\d.]+)mWb")
RE_MEAS = re.compile(r"MI ok mode=(\d)")

ENC = 2097152.0


def cmd_slow(rig, c, wait):
    """带 pause 的命令（会暂停接收线程），只用于切换/辨识。"""
    return cmd(rig, c, wait)


def read_env(rig):
    s = rig.rx.get_snap()[0]
    if not s:
        return None, None, None, None
    return (s.get("temp_mot"), s.get("temp_drv"),
            s.get("iq_ref"), s.get("w_elec"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--rpm", type=float, default=2000.0)
    ap.add_argument("--dt", type=float, default=2.0)
    ap.add_argument("--groups", type=int, default=30)
    ap.add_argument("--t-stop", type=float, default=100.0)
    ap.add_argument("--t-drv-stop", type=float, default=90.0)
    args = ap.parse_args()

    out = "thermal_step_%drpm_%s.csv" % (int(args.rpm), datetime.now().strftime("%Y%m%d_%H%M%S"))
    rig = Rig(args.port)
    rows = []
    stop_reason = None
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.25)
        cmd(rig, "D", 0.6)
        cmd(rig, "X", 0.7)
        cmd(rig, "Y", 0.4)
        toggle_expect(rig, "%", False)          # 谐波抑制关（组合配置低速会失稳/高速吃电压）
        toggle_expect(rig, "B", True)           # 死区补偿保持默认 ON
        cmd(rig, "\\59\r", 0.4)

        tm, td, _, _ = read_env(rig)
        t0_c = tm if tm else 0.0
        print("起点: 电机 %.2f °C / 驱动 %.2f °C，目标 %.0f rpm，每 %.1f °C 一组，共 %d 组"
              % (tm or 0, td or 0, args.rpm, args.dt, args.groups))
        print("安全上限: 电机 %.0f °C / 驱动 %.0f °C\n" % (args.t_stop, args.t_drv_stop))

        # 建环
        for _ in range(3):
            rig.s.write(("=%g\r" % args.rpm).encode())
            rig.s.flush()
            time.sleep(0.4)
        time.sleep(1.0)

        next_trig = (tm if tm else 25.0) + args.dt
        t_start = time.time()
        last_trig_t = t_start
        group = 0
        print("%5s | %7s | %7s | %7s | %s" % ("组", "t(min)", "T_pre", "T_post", "psi / R / L"))
        print("-" * 78)

        while group < args.groups:
            # 保活 + 读温度
            rig.s.write(("=%g\r" % args.rpm).encode())
            rig.s.flush()
            time.sleep(0.5)
            tm, td, iq, _ = read_env(rig)

            if tm is not None and td is not None:
                if tm >= args.t_stop:
                    stop_reason = "电机温度 %.1f °C 达上限" % tm
                    break
                if td >= args.t_drv_stop:
                    stop_reason = "驱动温度 %.1f °C 达上限" % td
                    break

            if tm is None or tm < next_trig:
                if time.time() - t_start > 3 * 3600:
                    stop_reason = "总时长超 3 h"
                    break
                # 热稳态判据：距上一组超过 10 min 仍未升温 dt，再等也无意义
                if (time.time() - last_trig_t) > 600.0:
                    stop_reason = ("10 min 内未再升温 %.1f K（T=%.1f °C），判热稳态"
                                   % (args.dt, tm if tm else 0))
                    break
                continue

            # ---- 触发一组 ----
            t_pre = tm
            cmd_slow(rig, "=0\r", 0.35)
            cmd_slow(rig, "D", 0.6)
            txt = cmd_slow(rig, "+", 20.0)
            m = RE_FI.search(txt)
            psi = float(m.group(3)) if m else float("nan")
            r_fi = float(m.group(1)) if m else float("nan")
            l_fi = float(m.group(2)) if m else float("nan")

            # 重新升速
            for _ in range(2):
                rig.s.write(("=%g\r" % args.rpm).encode())
                rig.s.flush()
                time.sleep(0.4)
            time.sleep(1.0)
            tm2, td2, iq2, we = read_env(rig)
            group += 1
            elapsed = (time.time() - t_start) / 60.0
            print("%5d | %7.2f | %6.2f | %6.2f | %.4f mWb / %.2f mΩ / %.2f µH"
                  % (group, elapsed, t_pre, tm2 or 0, psi, r_fi, l_fi))

            rows.append(dict(group=group, t_min=round(elapsed, 3),
                             temp_pre=round(t_pre, 2),
                             temp_post=round(tm2 if tm2 else 0, 2),
                             temp_drv=round((td2 if td2 else 0), 2),
                             psi_mwb=round(psi, 4), r_mohm=round(r_fi, 2),
                             l_uh=round(l_fi, 2),
                             rpm_meas=round((we / 14.0 * 60 / (2 * 3.141592653589793))
                                            if we else 0, 2),
                             iq_ref=round(iq2 if iq2 else 0, 4),
                             fault=fault(rig)))
            with open(out, "w", newline="", encoding="utf-8-sig") as fp:
                w = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)

            # 基准必须用"触发时的温度"：辨识停机期间会降温，
            # 若用 T_post 作基准，下一点会一升温就立刻再触发，步长退化成 ~1 K
            next_trig = t_pre + args.dt
            last_trig_t = time.time()

        if stop_reason is None:
            stop_reason = "已完成 %d 组" % group
    finally:
        try:
            rig.s.write(b"=0\r")
            time.sleep(0.3)
            rig.s.write(b"D")
        except Exception:
            pass
        try:
            cmd(rig, "X", 0.5)
        except Exception:
            pass
        rig.close()

    print("\n" + "=" * 78)
    print("结束原因: %s" % (stop_reason or "未知"))
    if rows:
        T = [r["temp_post"] for r in rows]
        P = [r["psi_mwb"] for r in rows]
        R = [r["r_mohm"] for r in rows]
        L = [r["l_uh"] for r in rows]
        print("温度范围 %.2f → %.2f °C（%+.2f K），共 %d 组"
              % (T[0], T[-1], T[-1] - T[0], len(rows)))

        def fit(x, y, name, unit):
            n = len(x)
            if n < 2:
                return
            mx = sum(x) / n
            my = sum(y) / n
            sxx = sum((a - mx) ** 2 for a in x)
            sxy = sum((a - mx) * (b - my) for a, b in zip(x, y))
            if sxx <= 0:
                return
            k = sxy / sxx
            b = my - k * mx
            syy = sum((bb - my) ** 2 for bb in y)
            ssr = sum((yy - (k * xx + b)) ** 2 for xx, yy in zip(x, y))
            r2 = 1 - ssr / syy if syy > 0 else 0.0
            print("  %-4s = %8.4f %+.6f·T   α=%+.4f %%/K  R²=%.4f"
                  % (name, b, k, k / my * 100 if my else 0, r2))
        fit(T, P, "psi", "mWb")
        fit(T, R, "R", "mΩ")
        fit(T, L, "L", "µH")
        print("[out] %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
