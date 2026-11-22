# -*- coding: utf-8 -*-
"""pos_rep_multi.py —— 三项重复定位精度

  1) fwd ：从 A 正转 20 圈到 B，记录 B；再反转回 A。重复 n 次 → B 的单向重复性
  2) rev ：从 A 反转 20 圈到 B，记录 B；再正转回 A。重复 n 次 → 反向单向重复性
  3) sway：A ↔ A+90° 往返 n 次，两端各记 n 点 → 双向差 + 两端各自重复性

为什么每次都要"回 A"而不是一路正转累加：
  · 控制器位置是 float。累计到 463 圈（2910 rad）时 ulp≈50 角秒，
    且实测该状态下 `P` 命令会得到 tgt=nan（2026-11-18 现场）。
    往返方案把工作区间锁在 [A, A+20圈]，ulp≈1.5 角秒，量化地板可见且可控。
  · 单向重复性本来就要求"每次从同一方向、同一段距离接近目标"。

测量基准用 30040 的 enc_sum（int64，2^21/圈），换算在 Python double 里做，
不经过固件 float，因此测量侧没有精度损失。

用法：
    python pos_rep_multi.py COM27 --test sway --n 100
    python pos_rep_multi.py COM27 --test fwd  --n 100 --revol 20
    python pos_rep_multi.py COM27 --test all  --n 100
"""
import argparse
import csv
import math
import os
import re
import statistics
import sys
import time

from cog_ff_test import Rig
from pos_test import ask, collect

TWO_PI = 2.0 * math.pi
ARCSEC = 180.0 * 3600.0 / math.pi
OUTDIR = os.path.dirname(os.path.abspath(__file__))


def send_p(rig, target):
    return b"P" + ("%.4f" % target).encode() + b"\r"


def p_bytes(rig, target):
    return send_p(rig, target)


def keepalive(rig, secs):
    """注意：`S` **不刷新** deadman（只有动作命令才续期）。
    这里发 S 仅用于在短等待里维持链路活跃，长行程必须用 move_wait 重发目标。"""
    t0 = time.time()
    nxt = t0
    while time.time() - t0 < secs:
        if time.time() >= nxt:
            rig.s.write(b"S")
            rig.s.flush()
            nxt += 0.4
        time.sleep(0.02)


def move_wait(rig, target, settle, tail=0.35):
    """发目标并等到位：期间每 1.35 s **重发同一目标**保活（deadman 2 s）。

    为什么重发目标而不是发 S：
      · deadman 只被动作命令续期，`S` 是只读查询、不续期；
      · 20 圈单程要 5.5 s，远超 2 s，不发动作命令会被 do_disable 中途封锁；
      · 重发同一目标只置 s_restart，规划器用**实测位置与速度**重新起步
        （profile_start(p_meas, v_meas)），目标不变、速度连续，不打断运动。
    实测 8.5 s 轨迹里速度稳定 24.5 rad/s，无停顿。
    """
    cmd = p_bytes(rig, target)
    rig.s.write(cmd)
    rig.s.flush()
    t0 = time.time()
    nxt = t0 + 1.35
    t_end = t0 + settle
    while time.time() < t_end - tail:
        if time.time() >= nxt:
            rig.s.write(cmd)
            rig.s.flush()
            nxt += 1.35
        time.sleep(0.02)
    if t_end - time.time() > 0:
        time.sleep(max(0.0, t_end - tail - time.time()))
    pts, _ = collect(rig, tail)
    if not pts:
        return None
    n = min(8, len(pts))
    return sum(pts[-n:]) / n


def read_A(rig):
    """读当前位置：返回 (A_ctrl, A_enc)。A_ctrl 用来算目标，A_enc 作测量基准。

    注意 P 空回车 = 保持当前位置（会 do_pos），不是查询。所以这里顺便
    完成"使能位置环 + 目标贴到当前"，随后再采样一次作为基准。
    """
    line = ask(rig, b"P\r", 0.9)
    m = re.search(r"tgt=(-?\d+\.\d+)", line)
    a_ctrl = float(m.group(1)) if m else None
    if a_ctrl is None:
        return None, None, line
    a_enc = move_wait(rig, a_ctrl, 0.6, 0.35)
    return a_ctrl, a_enc, line


def run_linear(rig, name, a_ctrl, a_enc, delta, n, t_move, t_back):
    """每次 A→B 单向到达并记录，再回 A。"""
    b_ctrl = a_ctrl + delta
    print("\n  %s: A=%.4f  B=%.4f  单程 %.1f s  预计 %.0f s"
          % (name, a_ctrl, b_ctrl, t_move, n * (t_move + t_back) / 60))
    got = []
    t_start = time.time()
    for i in range(n):
        p = move_wait(rig, b_ctrl, t_move, 0.35)
        if p is None:
            print("    [%3d] 采样失败，跳过" % (i + 1))
            continue
        got.append(p)
        if (i + 1) % 10 == 0:
            el = time.time() - t_start
            print("    [%3d/%d] dev=%+7.1f\"  (%.0f s, 余 %.0f s)"
                  % (i + 1, n, (p - a_enc - delta) * ARCSEC,
                     el, el / (i + 1) * (n - i - 1)))
        move_wait(rig, a_ctrl, t_back, 0.08)
    return got


def run_sway(rig, a_ctrl, a_enc, deg, n, t_move):
    """A ↔ B 往返，两端都记。"""
    b_ctrl = a_ctrl + math.radians(deg)
    print("\n  sway: A=%.4f  B=%.4f  ±%.0f°  单程 %.1f s  预计 %.0f s"
          % (a_ctrl, b_ctrl, deg, t_move, n * 2 * t_move / 60))
    pa, pb = [], []
    t_start = time.time()
    for i in range(n):
        p = move_wait(rig, b_ctrl, t_move, 0.30)
        if p is not None:
            pb.append(p)
        p = move_wait(rig, a_ctrl, t_move, 0.30)
        if p is not None:
            pa.append(p)
        if (i + 1) % 10 == 0:
            el = time.time() - t_start
            print("    [%3d/%d] B dev=%+6.1f\"  A dev=%+6.1f\"  (%.0f s)"
                  % (i + 1, n,
                     (pb[-1] - a_enc - math.radians(deg)) * ARCSEC if pb else float("nan"),
                     (pa[-1] - a_enc) * ARCSEC if pa else float("nan"), el))
    return pa, pb


def stat_block(name, vals, nominal_enc):
    if len(vals) < 2:
        print("  %-10s 无数据" % name)
        return None
    dev = [(v - nominal_enc) * ARCSEC for v in vals]
    m = statistics.mean(dev)
    sd = statistics.pstdev(dev)
    rng = max(dev) - min(dev)
    # 不再统计"模 2π"的 σ：enc_sum 是连续累积量，取模会在 2π 边界处把同一个
    # 位置拆成 0 与 360°，算出上百万角秒的假散布（实测出现过 625698"）。
    print("  %-12s n=%3d  mean=%+8.1f\"  σ=%6.1f\"  range=%7.1f\"  ±3σ=%7.1f\""
          % (name, len(dev), m, sd, rng, 3 * sd))
    return dict(n=len(dev), mean=m, sd=sd, rng=rng, dev=dev)


def save(rows, tag):
    if not rows:
        return
    path = os.path.join(OUTDIR, "pos_rep_%s.csv" % tag)
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["scheme", "i", "enc_rad", "dev_arcsec", "mod2pi_arcsec"])
        w.writerows(rows)
    print("\n  -> %s" % path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--test", default="all", choices=["fwd", "rev", "sway", "all"])
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--revol", type=float, default=20.0)
    ap.add_argument("--sway-deg", type=float, default=90.0)
    ap.add_argument("--move-20r", type=float, default=6.05, help="20 圈单程等待 s（运动 5.5 + 采样）")
    ap.add_argument("--move-sway", type=float, default=0.62, help="90° 单程等待 s")
    a = ap.parse_args()

    rig = Rig(a.port)
    print("=" * 78)
    print("重复定位精度  port=%s  test=%s  n=%d" % (a.port, a.test, a.n))
    print("=" * 78)

    st = ""
    for _ in range(4):
        st = ask(rig, b"S", 0.7)
        if "f=0x" in st:
            break
        time.sleep(0.2)
    print("[1] " + st)
    if "f=0x" not in st:
        print("    [ABORT] 状态行读不到（回显被遥测顶掉）")
        rig.close()
        return 2
    if "f=0x00" not in st:
        print("    [ABORT] 有故障位")
        rig.close()
        return 2

    if "moe=0" in st:
        print("[2] 使能位置环（P 保持当前位置）")
        rig.s.write(b"P\r")
        rig.s.flush()
        time.sleep(0.6)
    else:
        print("[2] 位置环已使能")

    a_ctrl, a_enc, line = read_A(rig)
    print("[3] 基准: " + line)
    if a_ctrl is None or a_enc is None:
        print("    [ABORT] 读不到基准（tgt 是否 nan？）")
        rig.close()
        return 3

    # 归零：控制器位置是 float，|theta| 越大 ulp 越大。把工作区间挪到 0 附近，
    # 20 圈处 ulp 从 3.1" 降到 1.6"，避免量化地板掩盖真实重复性。
    if abs(a_ctrl) > 5.0:
        print("[3b] 先归零（当前 %.2f rad，行程 %.1f rad）" % (a_ctrl, a_ctrl))
        move_wait(rig, 0.0, 6.5, 0.35)
        a_ctrl, a_enc, line = read_A(rig)
        print("     归零后: " + line)
        if a_ctrl is None or a_enc is None:
            print("    [ABORT] 归零后读不到基准")
            rig.close()
            return 3

    print("    A_ctrl=%.4f rad   A_enc=%.4f rad" % (a_ctrl, a_enc))

    delta = a.revol * TWO_PI
    enc_lo = min(a_enc, a_enc + delta, a_enc - delta)
    enc_hi = max(a_enc, a_enc + delta, a_enc - delta)
    abs_max = max(abs(enc_lo), abs(enc_hi))
    ulp = 2.0 ** (math.floor(math.log2(abs_max)) - 23) if abs_max > 0 else 1e-7
    print("    区间 [%.2f, %.2f] rad  -> 控制器 float ulp ≈ %.2f\" (量化地板)"
          % (enc_lo, enc_hi, ulp * ARCSEC))

    res = {}
    rows = []
    try:
        if a.test in ("sway", "all"):
            pa, pb = run_sway(rig, a_ctrl, a_enc, a.sway_deg, a.n, a.move_sway)
            res["swayA"] = stat_block("sway A(0°)", pa, a_enc)
            res["swayB"] = stat_block("sway B(90°)", pb, a_enc + math.radians(a.sway_deg))
            if res["swayA"] and res["swayB"]:
                print("  双向差 (B均值 - A均值 - 90°) = %+.1f\""
                      % (res["swayB"]["mean"] - res["swayA"]["mean"]))
            for i, v in enumerate(pb):
                rows.append(["swayB", i, "%.6f" % v,
                             "%.2f" % ((v - a_enc - math.radians(a.sway_deg)) * ARCSEC),
                             "%.2f" % ((v % TWO_PI) * ARCSEC)])
            for i, v in enumerate(pa):
                rows.append(["swayA", i, "%.6f" % v,
                             "%.2f" % ((v - a_enc) * ARCSEC),
                             "%.2f" % ((v % TWO_PI) * ARCSEC)])

        if a.test in ("fwd", "all"):
            g = run_linear(rig, "fwd(+20 rev)", a_ctrl, a_enc, delta, a.n,
                           a.move_20r, a.move_20r * 0.96)
            res["fwd"] = stat_block("fwd +20rev", g, a_enc + delta)
            for i, v in enumerate(g):
                rows.append(["fwd", i, "%.6f" % v,
                             "%.2f" % ((v - a_enc - delta) * ARCSEC),
                             "%.2f" % ((v % TWO_PI) * ARCSEC)])

        if a.test in ("rev", "all"):
            g = run_linear(rig, "rev(-20 rev)", a_ctrl, a_enc, -delta, a.n,
                           a.move_20r, a.move_20r * 0.96)
            res["rev"] = stat_block("rev -20rev", g, a_enc - delta)
            for i, v in enumerate(g):
                rows.append(["rev", i, "%.6f" % v,
                             "%.2f" % ((v - a_enc + delta) * ARCSEC),
                             "%.2f" % ((v % TWO_PI) * ARCSEC)])
    except KeyboardInterrupt:
        print("\n  [INT] 中断，已采数据仍会落盘")
    finally:
        rig.s.write(b"D")
        rig.s.flush()
        time.sleep(0.3)
        print("\n[4] " + ask(rig, b"S", 0.7))
        rig.close()

    save(rows, a.test)

    print("\n" + "=" * 78)
    print("汇总（全部相对各自的理想到达点，单位角秒；1\" = 4.85 µrad）")
    print("  量化地板 %.1f\"（控制器 float 在目标位置处的 ulp）" % (ulp * ARCSEC))
    for k in ("fwd", "rev", "swayA", "swayB"):
        if k in res and res[k]:
            r = res[k]
            print("  %-8s σ=%6.1f\"  range=%7.1f\"  mean=%+7.1f\"  n=%d"
                  % (k, r["sd"], r["rng"], r["mean"], r["n"]))
    if res.get("fwd") and res.get("rev"):
        print("  正反均值差 = %+.1f\"" % (res["fwd"]["mean"] - res["rev"]["mean"]))
    print("=" * 78)
    return 0


if __name__ == "__main__":
    sys.exit(main())
