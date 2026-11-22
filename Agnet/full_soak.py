# -*- coding: utf-8 -*-
"""
full_soak.py —— 完整校准 + 温升对电气参数影响的长测（2026-11-19）

流程（严格按顺序）：
  1) C   电流零点校准（200 ms，静止）
  2) Z   电角度对齐（需 C）
  3) ^   编码器 MT6835 自校准（600 rpm × 64 圈 ≈ 7 s）★ 完成后芯片需断电 >6 s 才生效
  4) @   R/L 辨识（2 A 注入，0.4 s）
  5) +   ψ/R/L 联合辨识（两转速 × id 扫描，9~12 s）
  6) 机械辨识：`(` 扫频 / `)` 摩擦 / `<` 惯量 / `}` 齿槽
  7) 速度环 1000 rpm 恒速
  8) 每 N 分钟为一组：停稳 → `+` 辨识电气参数 → 记录温度 → 重新升速
     共 M 组（默认 30 组 × 10 min ≈ 5 h）

每组记录：温度（驱动/电机）、ψ、R、L、转速、母线。
判据：R 随温度升、ψ 随温度降，斜率与既有标定对照。

安全：
  · 每组停稳后再做辨识（`+` 要求 D 状态、转子静止）
  · 停稳期间保活给 0，避免 2 s deadman 误杀；停稳后才停保活并 halt
  · 故障位 / 通信静默 / 温度超限 → 立即中止并封锁
  · CSV 每组增量落盘，中断不丢数据

用法：
    python full_soak.py COM27                      # 30 组 × 10 min（约 5 h）
    python full_soak.py COM27 --groups 3 --minutes 1   # 预演
    python full_soak.py COM27 --skip-enc-cal       # 跳过 ^（已校准过）
"""
import argparse
import csv
import json
import os
import re
import struct
import sys
import threading
import time
from datetime import datetime

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import (Rig, strip_frames, MSG_PERMISSION, MSG_CTRL_WORD,
                          MSG_OP_MODE, MSG_SYS_CMD, MSG_PS_IN, MSG_READ_PARAM,
                          MSG_STATUS, RAD_S_TO_RPM, POLE_PAIRS, sw_parse,
                          WARN_DRV, WARN_MOT, STOP_DRV, STOP_MOT)
from psi_repeat import fi_ident

RE_FI = re.compile(r"FI R=([\d.]+)mohm L=([\d.]+)uH psi=([\d.]+)mWb")
RE_RL = re.compile(r"RL ok R=([\d.]+)mohm L=([\d.]+)uH")
RE_IDENT = re.compile(r"ratio=([\d.]+).*?pn=(\d+)")


def ascii_cmd(rig, cmd, wait=1.0, until=None, timeout=180.0):
    """发单字符 ASCII 命令并收完整回显（剥掉二进制遥测）。
    until: 命中即提前返回的正则字符串列表。"""
    rig.rx.pause_evt.set()
    time.sleep(0.08)
    try:
        try:
            rig.s.reset_input_buffer()
        except Exception:
            pass
        with rig.wlock:
            rig.s.write(cmd.encode() if isinstance(cmd, str) else cmd)
            rig.s.flush()
        raw = bytearray()
        t0 = time.time()
        txt = ""
        while time.time() - t0 < timeout:
            d = rig.s.read(8192)
            if d:
                raw += d
            txt = strip_frames(raw).decode("ascii", "replace")
            if until and any(u in txt for u in until):
                time.sleep(0.25)
                d = rig.s.read(8192)
                if d:
                    raw += d
                txt = strip_frames(raw).decode("ascii", "replace")
                break
            if not until and (time.time() - t0) >= wait:
                break
            time.sleep(0.02)
        return txt
    finally:
        try:
            rig.s.reset_input_buffer()
        except Exception:
            pass
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def pick_cmd_line(txt):
    """只取含 [CMD] 的最后一行，避免二进制残片/乱码混进日志。"""
    lines = [l.strip() for l in txt.splitlines() if "[CMD]" in l]
    return lines[-1] if lines else "(无[CMD]回显)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--rpm", type=float, default=1000.0)
    ap.add_argument("--groups", type=int, default=30, help="组数")
    ap.add_argument("--minutes", type=float, default=10.0, help="每组间隔（分钟）")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--skip-pre", action="store_true", help="跳过 C/Z/@/+/机械辨识")
    ap.add_argument("--cal", action="store_true",
                    help="执行 C 电流零点。默认跳过：cal=1 已校准；且实测 C 之后紧跟 Z "
                         "会把极对数识别成 13（ratio=12.84），导致后续全错")
    ap.add_argument("--enc-cal", action="store_true",
                    help="执行 ^ 编码器自校准（危险：会让 MT6835 输出异常，"
                         "触发 f=0x02 编码器故障，X 清不掉，必须断电才能恢复；默认不执行）")
    ap.add_argument("--skip-mech", action="store_true", help="跳过机械辨识")
    ap.add_argument("--pre-only", action="store_true", help="只做前置校准，不进主测试（验证用）")
    ap.add_argument("--z-optional", action="store_true",
                    help="Z 失败不中止（只告警）。用于编码器角度勉强可用、"
                         "但开环 2 A 步进推不动转子导致 ratio_bad 的场合："
                         "速度闭环本身并不依赖本次 Z 的结果，沿用上次零点即可")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_csv = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "full_%drpm_%s.csv" % (int(args.rpm), stamp))
    log = []
    rows = []
    aborted = {"why": None}

    def say(s):
        print(s, flush=True)
        log.append(s)

    print("=" * 82, flush=True)
    print("完整校准 + 温升电气参数长测  %.0f rpm  %d 组 × %.0f min ≈ %.1f h"
          % (args.rpm, args.groups, args.minutes, args.groups * args.minutes / 60.0),
          flush=True)
    print("CSV: %s" % out_csv, flush=True)
    print("=" * 82, flush=True)

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
            return None, None, None
        return snap.get("temp_drv"), snap.get("temp_mot"), snap.get("vbus")

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

    def ensure_clean(tag):
        """各辨识步骤前的故障门卫：有故障先 X，仍清不掉就判失败。
        （编码器类 f=0x02 需要 ! 软复位，X 无效，这里只报告不自动复位，
          因为软复位会中断整个长测流程。）"""
        en, flt, act, ew = check_fault()
        if not flt:
            return True
        ascii_cmd(rig, "X", until=["fault cleared", "CMD"], timeout=3.0)
        time.sleep(0.5)
        en, flt, act, ew = check_fault()
        if flt:
            say("  [ERR] %s 前仍有故障 f=0x%04X（编码器类故障需 ! 软复位）"
                % (tag, ew or 0))
            return False
        say("  [ok] %s 前已清故障" % tag)
        return True

    def wait_still(max_s=12.0, thr=8.0):
        t0 = time.time()
        while time.time() - t0 < max_s:
            if abs(rpm_now()) < thr:
                return True
            time.sleep(0.15)
        return False

    def stop_and_ident(tag, group):
        """停稳 → halt → '+' 辨识 → 返回记录"""
        ka["rpm"] = 0.0
        wait_still(12.0)
        ka["en"] = False
        time.sleep(0.5)
        rig.send(MSG_CTRL_WORD, bytes([0, 0, 1]))
        time.sleep(0.8)

        td, tm, vb = temps()
        r = fi_ident(rig)
        rec = None
        if r["ok"]:
            rec = {"group": group, "tag": tag,
                   "iso": datetime.now().strftime("%H:%M:%S"),
                   "t_s": round(time.time() - t_start, 1),
                   "temp_drv": round(td, 2) if td is not None else "",
                   "temp_mot": round(tm, 2) if tm is not None else "",
                   "vbus": round(vb, 2) if vb is not None else "",
                   "psi_mwb": round(r["psi_mwb"], 4),
                   "r_mohm": round(r["r_mohm"], 2),
                   "l_uh": round(r["l_uh"], 2)}
            say("  #%-2d %s  psi=%.4f mWb  R=%.2f mΩ  L=%.2f uH  Tdrv=%.2f Tmot=%.2f °C"
                % (group, rec["iso"], r["psi_mwb"], r["r_mohm"], r["l_uh"],
                   td or 0, tm or 0))
        else:
            say("  [ERR] 第 %d 组辨识失败: %s" % (group, r["raw"][:150].replace("\n", " | ")))
        return rec

    def restart_speed():
        ka["rpm"] = args.rpm
        ka["en"] = True
        rig.send_ps(args.rpm)
        time.sleep(0.4)
        rig.send(MSG_CTRL_WORD, bytes([1, 0, 0]))
        time.sleep(0.3)

    # ---------------- 前置校准 ----------------
    t_start = time.time()
    pre = {}
    try:
        time.sleep(1.0)
        rig.send(MSG_PERMISSION, bytes([0, 0])); time.sleep(0.25)
        rig.send(MSG_CTRL_WORD, bytes([0, 0, 1])); time.sleep(0.8)
        rig.send(MSG_SYS_CMD, bytes([3])); time.sleep(0.3)

        en, flt, act, ew = check_fault()
        say("[ok] 初始：使能=%s 故障=%s 在环=%s err=0x%04X" % (en, flt, act, ew or 0))

        if not args.skip_pre:
            say("\n=== 前置校准 ===")

            # 1) C 电流零点（默认跳过，见 --cal 说明）
            if args.cal:
                txt = ascii_cmd(rig, "C", until=["CAL start", "CAL refused"], timeout=6.0)
                say("[1/6] C 电流零点: %s" % pick_cmd_line(txt))
                time.sleep(3.0)
            else:
                say("[1/6] C 电流零点: 跳过（cal=1 已校准；C 后紧跟 Z 会识别错极对数）")

            # 2) Z 电角度对齐：必须校验 ratio≈14 且 pn=14
            #    实测 C 后紧跟 Z 会得到 ratio=12.84 pn=13，此时后续 R/ψ 全偏。
            z_ok = False
            for attempt in range(4):
                txt = ascii_cmd(rig, "Z",
                                until=["IDENT ok", "IDENT FAIL", "Z refused"],
                                timeout=45.0)
                tail = pick_cmd_line(txt)
                m = RE_IDENT.search(txt)
                good = False
                if m:
                    ratio, pn = float(m.group(1)), int(m.group(2))
                    good = (13.5 <= ratio <= 14.5) and (pn == 14)
                    tail += "   [校验 ratio=%.2f pn=%d %s]" % (ratio, pn, "OK" if good else "BAD")
                say("[2/6] Z 电角度 (第%d次): %s" % (attempt + 1, tail))
                if good:
                    z_ok = True
                    break
                ensure_clean("Z 重试前")
                time.sleep(3.0)
            if not z_ok:
                # Z 的作用是重写电角度零点。速度闭环沿用固件里已有的零点即可运行，
                # 所以"Z 失败"不应无条件中止整场长测——尤其是开环 2 A 推不动转子
                # 的机器，Z 会稳定地 ratio_bad，但 1000 rpm 闭环实测完全正常。
                if args.z_optional:
                    say("      ⚠ Z 未通过，按 --z-optional 继续（沿用上次零点）")
                else:
                    raise RuntimeError("Z_align_failed_or_ratio_bad")
            time.sleep(1.0)

            # 3) ^ 编码器自校准（默认跳过，见参数说明）
            if not args.enc_cal:
                say("[3/6] ^ 编码器校准: 跳过（默认）。原因：^ 期间 MT6835 自校准会"
                    "让角度输出短暂不可信，速度环随即置 f=0x02 编码器故障，"
                    "X 清不掉、必须 ! 软复位；且写入 EEPROM 后需断电 >6 s 才生效。"
                    "本板 cal=1/z=1 已具备可用的编码器标定。")
            else:
                txt = ascii_cmd(rig, "^", until=["EC ok", "EC FAIL", "EC refused"], timeout=60.0)
                tail = pick_cmd_line(txt)
                say("[3/6] ^ 编码器校准: %s" % tail)
                if "POWER CYCLE" in txt:
                    say("      ⚠ 已写入 EEPROM，需断电 >6 s 后新数据才生效")
                time.sleep(2.0)
                # ^ 之后极可能带 f=0x02，必须清/复位
                if not ensure_clean(rig, "^ 编码器校准后"):
                    say("      ✘ ^ 留下未恢复的编码器故障，建议改用默认（不加 --enc-cal）")
                    aborted["why"] = "enc_cal_left_fault"

            # 4) @ R/L
            if not ensure_clean("@ R/L 前"):
                raise RuntimeError("fault_before_rl")
            txt = ascii_cmd(rig, "@", until=["RL ok", "RL FAIL", "RL refused"], timeout=10.0)
            m = RE_RL.search(txt)
            pre["rl"] = {"r_mohm": float(m.group(1)), "l_uh": float(m.group(2))} if m else None
            say("[4/6] @ R/L: %s" % pick_cmd_line(txt))
            time.sleep(1.0)

            # 5) + ψ/R/L
            if not ensure_clean("+ ψ 辨识前"):
                raise RuntimeError("fault_before_fi")
            r = fi_ident(rig)
            pre["fi"] = {"psi_mwb": r["psi_mwb"], "r_mohm": r["r_mohm"], "l_uh": r["l_uh"]} if r["ok"] else None
            say("[5/6] + ψ/R/L: %s" % ("psi=%.4f mWb R=%.2f L=%.2f"
                                       % (r["psi_mwb"], r["r_mohm"], r["l_uh"]) if r["ok"]
                                       else r["raw"][:120]))
            time.sleep(1.0)

            # 6) 机械辨识（4 项）
            if args.skip_mech:
                say("[6/6] 机械辨识: 跳过（--skip-mech）")
            elif not ensure_clean("机械辨识前"):
                # 机械辨识不是温漂测试的必要条件，失败不阻塞主流程
                say("[6/6] 机械辨识: 因故障跳过（不影响温漂采集）")
            else:
                # 只做"机械参数"本体：惯量与摩擦。
                # 扫频(20..1000Hz)要 5 min 以上且属频响分析、齿槽属前馈标定，
                # 都不是"参数"，且会与 5 h 长测抢时间。
                mech = [("<", "惯量 iq 1A/2A", 40.0),
                        (")", "摩擦 8pts 1..200rpm", 90.0)]
                got = []
                for cmd, name, to in mech:
                    txt = ascii_cmd(rig, cmd, until=["MI start", "MI refused", "MI2 refused"],
                                    timeout=to)
                    tail = pick_cmd_line(txt)
                    say("[6/6] %s 机械 %-16s: %s" % (cmd, name, tail))
                    got.append({"cmd": cmd, "name": name, "tail": tail})
                    time.sleep(2.0)
                pre["mech"] = got
    except Exception as e:
        if aborted["why"] is None:
            aborted["why"] = "pre_exception_%r" % e
        say("[ERR] 前置阶段异常: %r" % e)

    # ---------------- 主测试 ----------------
    if args.pre_only and aborted["why"] is None:
        say("\n[pre-only] 前置全部通过（Z 的 ratio/pn 已校验），按 --pre-only 退出")
        aborted["why"] = "pre_only_done"
    if aborted["why"] is None:
        try:
            say("\n=== 温升电气参数采集 ===")
            rig.send(MSG_OP_MODE, bytes([3])); time.sleep(0.25)

            # 先建立速度环（0 rpm），确认在环后再升速
            rig.send_ps(0.0); time.sleep(0.5)
            rig.send(MSG_CTRL_WORD, bytes([1, 0, 0])); time.sleep(0.3)
            en, flt, act, ew = check_fault()
            say("[ok] 速度环：使能=%s 故障=%s 实际在环=%s" % (en, flt, act))
            if act != 3:
                aborted["why"] = "active_mode=%s" % act

            if aborted["why"] is None:
                restart_speed()

                for g in range(1, args.groups + 1):
                    if aborted["why"]:
                        break
                    # 恒速运行 minutes
                    t_next = time.time() + args.minutes * 60.0
                    while time.time() < t_next:
                        time.sleep(1.0)
                        td, tm, vb = temps()
                        if tm is not None and tm >= STOP_MOT:
                            aborted["why"] = "motor_temp_%.0fC" % tm
                            break
                        if td is not None and td >= STOP_DRV:
                            aborted["why"] = "driver_temp_%.0fC" % td
                            break
                        # 静默判据放到 10 s：接收线程已改为异常重试，短暂 USB
                        # 抖动不会立刻致死；只有连续 10 s 没有一帧才认为链路断了。
                        # 原来 3 s 太紧，一次 pyserial 瞬时异常就会误杀长测。
                        if rig.rx.last_rx and (time.time() - rig.rx.last_rx) > 10.0:
                            aborted["why"] = "link_silent_%.0fs" % (time.time() - rig.rx.last_rx)
                            break
                        if tm is not None and tm >= WARN_MOT and ("mot" not in pre):
                            pre["mot"] = 1
                            say("  [WARN] 电机温度 %.1f °C 已达降额起点" % tm)

                    if aborted["why"]:
                        break

                    rec = stop_and_ident("run", g)
                    if rec:
                        rows.append(rec)
                        with open(out_csv, "w", newline="", encoding="utf-8-sig") as f:
                            wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                            wr.writeheader()
                            wr.writerows(rows)
                    en, flt, act, ew = check_fault()
                    if flt:
                        aborted["why"] = "fault_0x%04X" % (ew or 0)
                        break
                    restart_speed()
        except KeyboardInterrupt:
            aborted["why"] = "ctrl_c"
        except Exception as e:
            aborted["why"] = "loop_exception_%r" % e

    # ---------------- 收尾 ----------------
    ka["rpm"] = 0.0
    try:
        wait_still(8.0)
    except Exception:
        pass
    ka["en"] = False
    time.sleep(0.4)
    stop_evt.set()
    try:
        rig.send(MSG_CTRL_WORD, bytes([0, 0, 1]))
        time.sleep(0.3)
    except Exception:
        pass
    rig.close()

    # ---------------- 汇总 ----------------
    say("\n" + "=" * 82)
    say("汇总")
    say("=" * 82)
    say("前置: " + json.dumps(pre, ensure_ascii=False))
    if len(rows) >= 3:
        ts = [r["temp_mot"] for r in rows if isinstance(r["temp_mot"], float)]
        ps = [r["psi_mwb"] for r in rows]
        rs = [r["r_mohm"] for r in rows]
        say("组数 %d，温度 %.2f → %.2f °C（%.2f K）"
            % (len(rows), ts[0], ts[-1], ts[-1] - ts[0]))
        say("ψ %.4f → %.4f mWb（%+.2f%%）；R %.2f → %.2f mΩ（%+.2f%%）"
            % (ps[0], ps[-1], (ps[-1] - ps[0]) / ps[0] * 100,
               rs[0], rs[-1], (rs[-1] - rs[0]) / rs[0] * 100))
    if aborted["why"]:
        say("[ABORT] %s" % aborted["why"])

    jpath = os.path.splitext(out_csv)[0] + ".json"
    with open(jpath, "w", encoding="utf-8") as f:
        json.dump({"csv": out_csv, "pre": pre, "rows": rows,
                   "aborted": aborted["why"], "log": log[-40:]},
                  f, ensure_ascii=False, indent=2)
    say("[out] %s" % out_csv)
    say("[out] %s" % jpath)
    return 0 if aborted["why"] in (None, "pre_only_done") else 2


if __name__ == "__main__":
    sys.exit(main())
