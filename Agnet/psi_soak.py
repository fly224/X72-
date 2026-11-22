# -*- coding: utf-8 -*-
"""
psi_soak.py —— ψ（磁链）温度漂移曲线采集（2026-11-19）

为什么这么做：ψ 直接决定 Kt = 1.5·p·ψ = 0.0910 N·m/A，进而决定所有力矩换算与
加速度前馈 J/Kt。ψ 的温度系数是钕铁硼的 -0.11 %/K 量级，比 R 的 +0.4 %/K 温和，
但对力矩精度是**直接**误差（R 只是间接影响电流环动态）。

可行性已由 psi_repeat.py 验证：ψ 单次辨识 CV=0.048%，3σ 等效温差 1.3 K，
远优于 0.11 %/K 的信号，因此每个温度点单次辨识即可。

流程（每 interval 分钟一轮）：
  1. 保活给定降到 0，等转子停稳（期间保活继续发 0，避免 2 s deadman 触发）
  2. 停保活 → halt（必须 D 状态才能启动 FluxIdent）
  3. 发 ASCII '+'，FluxIdent 自己拉速度环跑 1000→300 rpm、注入 id 0/-1/-2/-3，
     约 9~12 s，结束封 MOE
  4. 记录 ψ/R/L + 双路温度
  5. 重新使能回到目标转速继续加热

关键：FluxIdent 直接调 Servo_SetSpeedMode + PwmEnable，不设 s_armed，
所以辨识期间 deadman 不递减（servo_cmd.c:2775 在 s_armed=0 直接 return），无需保活。

用法：
    python psi_soak.py COM27 --rpm 1000 --minutes 36 --interval 3
    python psi_soak.py COM27 --minutes 20 --interval 2      # 短跑预演
"""
import argparse
import csv
import json
import os
import struct
import sys
import threading
import time
from datetime import datetime

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import (Rig, MSG_CTRL_WORD, MSG_PERMISSION, MSG_OP_MODE,
                          MSG_READ_PARAM, MSG_STATUS, MSG_SYS_CMD,
                          RAD_S_TO_RPM, POLE_PAIRS, ST_NAME, sw_parse,
                          WARN_DRV, WARN_MOT, STOP_DRV, STOP_MOT)
from psi_repeat import fi_ident, wait_still


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--rpm", type=float, default=1000.0, help="加热转速")
    ap.add_argument("--minutes", type=float, default=36.0, help="总时长")
    ap.add_argument("--interval", type=float, default=3.0, help="辨识间隔（分钟）")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--cool", action="store_true",
                    help="冷却模式：辨识后保持封锁不重启速度环（测降温段，用于判定 NTC 迟滞）")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_csv = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "psi_%drpm_%s.csv" % (int(args.rpm), stamp))
    rows = []
    aborted = {"why": None}

    print("=" * 74)
    print("ψ 温漂曲线采集  转速 %.0f rpm  时长 %.0f min  间隔 %.0f min"
          % (args.rpm, args.minutes, args.interval))
    print("CSV: %s" % out_csv)
    print("=" * 74)

    rig = Rig(args.port, args.baud)
    stop_evt = threading.Event()
    ka = {"en": False, "rpm": args.rpm}

    def keepalive():
        while not stop_evt.is_set():
            if ka["en"]:
                try:
                    rig.send_ps(ka["rpm"])
                except Exception:
                    pass
            time.sleep(0.4)

    threading.Thread(target=keepalive, daemon=True).start()

    def temps():
        snap, _, _, _, _ = rig.rx.get_snap()
        if not snap:
            return None, None
        return snap.get("temp_drv"), snap.get("temp_mot")

    def rpm_now():
        snap, _, _, _, _ = rig.rx.get_snap()
        w = snap.get("w_elec") if snap else None
        return 0.0 if w is None else w / POLE_PAIRS * RAD_S_TO_RPM

    def check_fault():
        rig.send(MSG_READ_PARAM, struct.pack("<I", MSG_STATUS))
        time.sleep(0.12)
        sw, ew, _ = rig.rx.get_status()
        en, flt, act = sw_parse(sw)
        return en, flt, act, ew

    def speed_stop_and_ident(tag):
        """停稳 → halt → '+' 辨识 → 返回记录。失败返回 None。"""
        # 给定降到 0，保活继续发 0（关键：不能停保活，否则 deadman 2 s 触发）
        ka["rpm"] = 0.0
        t0 = time.time()
        while time.time() - t0 < 8.0:
            if abs(rpm_now()) < 8.0:
                break
            time.sleep(0.15)
        # 停稳后停保活、封锁（s_armed=0 后 deadman 不再递减）
        ka["en"] = False
        time.sleep(0.5)
        rig.send(MSG_CTRL_WORD, bytes([0, 0, 1]))
        time.sleep(0.8)

        td, tm = temps()
        r = fi_ident(rig)
        rec = None
        if r["ok"]:
            rec = {"tag": tag, "t_s": round(time.time() - t_start, 1),
                   "iso": datetime.now().strftime("%H:%M:%S"),
                   "temp_drv": round(td, 2) if td is not None else "",
                   "temp_mot": round(tm, 2) if tm is not None else "",
                   "psi_mwb": round(r["psi_mwb"], 4),
                   "r_mohm": round(r["r_mohm"], 2),
                   "l_uh": round(r["l_uh"], 2)}
            print("  #%-2d %s  psi=%.4f mWb  R=%.2f mΩ  L=%.2f uH  Tdrv=%.2f Tmot=%.2f °C"
                  % (len(rows) + 1, rec["iso"], r["psi_mwb"], r["r_mohm"],
                     r["l_uh"], td or 0, tm or 0))
        else:
            print("  [ERR] 辨识失败: %s" % r["raw"][:180].replace("\n", " | "))
        return rec

    def speed_restart():
        """重新回到目标转速。冷却模式下不重启，保持封锁让转子自然冷却。"""
        if args.cool:
            return
        ka["rpm"] = args.rpm
        ka["en"] = True
        rig.send_ps(args.rpm)
        time.sleep(0.4)
        rig.send(MSG_CTRL_WORD, bytes([1, 0, 0]))
        time.sleep(0.3)

    t_start = time.time()
    try:
        time.sleep(0.8)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.25)
        rig.send(MSG_CTRL_WORD, bytes([0, 0, 1]))
        time.sleep(0.8)

        en, flt, act, ew = check_fault()
        print("[ok] 初始：使能=%s 故障=%s 在环=%s err=0x%04X" % (en, flt, act, ew or 0))
        if flt:
            rig.send(MSG_SYS_CMD, bytes([3]))
            time.sleep(0.3)
            print("[ok] 已清故障")

        # 选速度模式 + 起点辨识（当前温度）
        rig.send(MSG_OP_MODE, bytes([3]))
        time.sleep(0.25)
        rec = speed_stop_and_ident("start")
        if rec:
            rows.append(rec)

        speed_restart()
        time.sleep(0.5)
        if not args.cool:
            en, flt, act, ew = check_fault()
            print("[ok] 升速后：使能=%s 故障=%s 在环=%s" % (en, flt, act))
            if act != 3:
                aborted["why"] = "active_mode=%s" % act

        # 循环
        t_end = t_start + args.minutes * 60.0
        next_at = time.time() + args.interval * 60.0
        while aborted["why"] is None and time.time() < t_end:
            time.sleep(1.0)
            # 安全
            td, tm = temps()
            if tm is not None and tm >= STOP_MOT:
                aborted["why"] = "motor_temp_%.0fC" % tm
                break
            if td is not None and td >= STOP_DRV:
                aborted["why"] = "driver_temp_%.0fC" % td
                break
            if tm is not None and tm >= WARN_MOT:
                print("  [WARN] 电机温度 %.1f °C 已达降额起点" % tm)
            if rig.rx.last_rx and (time.time() - rig.rx.last_rx) > 3.0:
                aborted["why"] = "link_silent"
                break

            if time.time() >= next_at:
                next_at += args.interval * 60.0
                rec = speed_stop_and_ident("run")
                if rec:
                    rows.append(rec)
                    # 落盘
                    with open(out_csv, "w", newline="", encoding="utf-8-sig") as f:
                        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                        wr.writeheader()
                        wr.writerows(rows)
                en, flt, act, ew = check_fault()
                if flt:
                    aborted["why"] = "fault_0x%04X" % (ew or 0)
                    break
                speed_restart()

    except KeyboardInterrupt:
        aborted["why"] = "ctrl_c"
    except Exception as e:
        aborted["why"] = "exception_%r" % e
    finally:
        # 收尾：停保活 → 减速 → 封锁
        ka["rpm"] = 0.0
        try:
            t0 = time.time()
            while time.time() - t0 < 6.0 and abs(rpm_now()) > 5.0:
                time.sleep(0.15)
        except Exception:
            pass
        ka["en"] = False
        time.sleep(0.4)
        try:
            rig.send(MSG_CTRL_WORD, bytes([0, 0, 1]))
            time.sleep(0.3)
        except Exception:
            pass
        rig.close()

    # 拟合
    print("\n" + "=" * 74)
    print("拟合")
    print("=" * 74)
    fit = None
    pts = [(r["temp_mot"], r["psi_mwb"]) for r in rows
           if isinstance(r["temp_mot"], float) and r["psi_mwb"]]
    if len(pts) >= 3:
        n = len(pts)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        mx = sum(xs) / n
        my = sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        if sxx > 1e-9:
            b = sxy / sxx
            a = my - b * mx
            ss_tot = sum((y - my) ** 2 for y in ys)
            ss_res = sum((y - (a + b * x)) ** 2 for x, y in zip(xs, ys))
            r2 = 0.0 if ss_tot < 1e-12 else 1 - ss_res / ss_tot
            psi20 = a + b * 20.0
            alpha = b / psi20 * 100.0          # %/K
            fit = {"a": a, "b": b, "r2": r2, "psi20": psi20, "alpha_pct_per_K": alpha,
                   "n": n, "t_min": min(xs), "t_max": max(xs)}
            print("ψ(T) = %.5f %+.6f·T  mWb    R² = %.4f" % (a, b, r2))
            print("ψ20  = %.4f mWb" % psi20)
            print("α_ψ  = %+.4f %%/K   （钕铁硼典型 -0.11 %%/K）" % alpha)
            print("Kt20 = %.5f N·m/A  （1.5·14·ψ20）" % (1.5 * 14.0 * psi20 * 1e-3))
            print("温度跨度 %.1f→%.1f °C（%.1f K），ψ 变化 %.2f%%"
                  % (min(xs), max(xs), max(xs) - min(xs),
                     (max(ys) - min(ys)) / my * 100))
        else:
            print("[WARN] 温度跨度太小，无法拟合")
    else:
        print("[WARN] 有效点 %d 个，不足 3 个" % len(pts))

    out = {"csv": out_csv, "rows": rows, "fit": fit, "aborted": aborted["why"],
           "rpm": args.rpm, "minutes": args.minutes}
    jpath = os.path.splitext(out_csv)[0] + ".json"
    with open(jpath, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("\n[out] %s\n[out] %s" % (out_csv, jpath))
    if aborted["why"]:
        print("[ABORT] %s" % aborted["why"])
    return 0 if aborted["why"] is None else 2


if __name__ == "__main__":
    sys.exit(main())
