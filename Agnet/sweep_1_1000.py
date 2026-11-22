# -*- coding: utf-8 -*-
"""sweep_1_1000.py —— 速度闭环 1~1000 rpm 全程扫描

记录每点的：给定 / 稳态均值 / σ / 峰峰值 / 误差 / iq 均值 / 母线电压，
落 CSV（UTF-8 BOM，便于直接拖进飞书），并做分段分析。

调速方式：`=` 数值命令（不是 Y/O 档位）——档位只有 3/30/100/500/1500/3000
六个固定值，做不出 1~1000 的连续曲线。

**引导符与数字必须分开发**：一次性发 "=100\\r" 时，ASCII 解析器在同一次
Poll 里既看到引导符又看到数字，偶发丢字节（历史上 `_1` 就是这么坏的）。
这里 `=` 单独发、停 40 ms，再逐字符发数字，每字符停 6 ms。

用法:
    python sweep_1_1000.py COM27
    python sweep_1_1000.py COM27 --secs 2.5 --out sweep.csv
"""
import argparse
import math
import struct
import time

from cog_ff_test import LEN_IN, MSG_IN, Rig, x25

POLE = 14.0
TWO_PI = 2.0 * math.pi
RAD_PER_RPM = TWO_PI / 60.0


def collect_rich(rig, secs):
    """比 Rig.collect 多取两个字段：母线电压(24) 与斜坡后给定(57)

    30025 帧布局：4=iq_ref 24=vbus 44=ωe 53=temp_motor 57=spd_ref_rpm（LEN=61）
    """
    frames = []
    t0 = time.time()
    nxt = t0 + 0.8
    while time.time() - t0 < secs:
        d = rig.s.read(4096)
        if d:
            rig.raw += d
        if time.time() > nxt:
            rig.s.write(b"S")
            rig.s.flush()
            nxt += 0.8
        i = 0
        n = len(rig.raw)
        while i + 12 <= n:
            if rig.raw[i] != 0xFD:
                i += 1
                continue
            ln = rig.raw[i + 1]
            mid = rig.raw[i + 7] | (rig.raw[i + 8] << 8) | (rig.raw[i + 9] << 16)
            if ln != LEN_IN or mid != MSG_IN:
                i += 1
                continue
            b = i + 10
            if b + ln + 2 > n:
                break
            crc = rig.raw[b + ln] | (rig.raw[b + ln + 1] << 8)
            if x25(rig.raw[i + 1:b + ln], 191) != crc:
                i += 1
                continue
            iq = struct.unpack_from("<f", rig.raw, b + 4)[0]
            vbus = struct.unpack_from("<f", rig.raw, b + 24)[0]
            we = struct.unpack_from("<f", rig.raw, b + 44)[0]
            ref = struct.unpack_from("<f", rig.raw, b + 57)[0]
            frames.append((time.time(), we / POLE / RAD_PER_RPM, iq, vbus, ref))
            i = b + ln + 2
        del rig.raw[:i]
    return frames

# 对数分布，低速密、高速疏：低速段是误差和黏滑最容易暴露的地方
RPM_LIST = [1, 2, 3, 5, 7, 10, 15, 20, 30, 50, 70,
            100, 150, 200, 300, 400, 500, 700, 850, 1000]


def set_rpm(rig, rpm):
    """`=` 数值调速：引导符单独发，数字逐字符发

    发送后必须校验回显。历史上踩过的坑：ASCII 回显单槽（s_out_pending），
    碰上遥测 DMA 在飞会整行丢掉，看起来像"命令没生效"。所以这里若拿不到
    回显就整条重发一次，绝不把"没回显"当成"已执行"。
    """
    for attempt in (1, 2):
        rig.s.reset_input_buffer()
        rig.s.write(b"\r")          # 先把可能残留的数值解析状态清掉
        rig.s.flush()
        time.sleep(0.05)
        rig.s.write(b"=")
        rig.s.flush()
        time.sleep(0.06)
        for ch in ("%d" % rpm):
            rig.s.write(ch.encode())
            rig.s.flush()
            time.sleep(0.012)
        rig.s.write(b"\r")
        rig.s.flush()

        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < 0.6:
            d = rig.s.read(512)
            if d:
                buf += d
        txt = "".join(ch if 32 <= ord(ch) < 127 else "\n"
                      for ch in buf.decode("ascii", "replace"))
        lines = [ln.strip() for ln in txt.split("\n") if ln.strip().startswith("[CMD]")]
        if lines:
            return lines[-1]
        if attempt == 1:
            print("      (无回显，重发一次)")
    return "(no echo)"


def stats(fr, given):
    n = max(6, int(len(fr) * 0.5))
    tail = fr[-n:]
    v = [f[1] for f in tail]
    iq = [f[2] for f in tail]
    vb = [f[3] for f in tail]
    ref = [f[4] for f in tail]
    m = sum(v) / len(v)
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
    return dict(
        n=len(fr),
        spd_ref=sum(ref) / len(ref),
        spd_mean=m,
        spd_sd=sd,
        spd_min=min(v),
        spd_max=max(v),
        err=m - given,
        err_pct=(m - given) / given * 100.0 if given else 0.0,
        rip_pct=sd / given * 100.0 if given else 0.0,
        iq_mean=sum(iq) / len(iq),
        iq_sd=math.sqrt(sum((x - sum(iq) / len(iq)) ** 2 for x in iq) / len(iq)),
        vbus=sum(vb) / len(vb),
        vbus_min=min(vb),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--secs", type=float, default=3.0, help="每点采集时长")
    ap.add_argument("--settle", type=float, default=3.0,
                    help="到速后稳定时间。1.5 s 不够：从上一档切过来时，"
                         "低速黏滑要几秒才稳，早期 σ 会系统性偏高"
                         "（同一 260 rpm 点 1.5 s 稳定测得 27，3 s 测得 17）")
    ap.add_argument("--out", default="sweep_1_1000.csv")
    ap.add_argument("--rpms", default=",".join(str(x) for x in RPM_LIST))
    a = ap.parse_args()

    rpms = [float(x) for x in a.rpms.split(",")]
    rig = Rig(a.port)
    rows = []
    try:
        st = rig.cmd("S", 0.5)
        print("[状态] %s" % st)
        cg = rig.cmd(":", 0.4)
        print("[齿槽] %s" % cg)

        print("\n%6s %9s %9s %8s %8s %8s %8s %7s"
              % ("rpm", "给定", "实测", "误差%", "σ", "σ%", "iq(A)", "vbus"))
        print("-" * 74)

        for want in rpms:
            echo = set_rpm(rig, want)
            if "refused" in echo or "failed" in echo:
                print("%6.0f  设定被拒: %s" % (want, echo))
                continue
            rig.keep(a.settle)
            fr = collect_rich(rig, a.secs)
            if len(fr) < 32:
                print("%6.0f  帧太少 %d" % (want, len(fr)))
                continue
            s = stats(fr, want)
            s["rpm_cmd"] = want
            rows.append(s)
            print("%6.0f %9.2f %9.2f %+7.2f%% %8.3f %7.2f%% %8.3f %7.2f"
                  % (want, s["spd_ref"], s["spd_mean"], s["err_pct"],
                     s["spd_sd"], s["rip_pct"], s["iq_mean"], s["vbus"]))

        rig.cmd("D", 0.4)      # 收尾只下使能；不要发 `=0`，0 给定下 MOE 仍开、无意义

        if not rows:
            print("没有有效数据")
            return 1

        with open(a.out, "w", encoding="utf-8-sig", newline="") as f:
            f.write("rpm_cmd,spd_ref,spd_mean,spd_sd,spd_min,spd_max,"
                    "err,err_pct,rip_pct,iq_mean,iq_sd,vbus,vbus_min,n\n")
            for s in rows:
                f.write("%.0f,%.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.3f,%.3f,%d\n"
                        % (s["rpm_cmd"], s["spd_ref"], s["spd_mean"], s["spd_sd"],
                           s["spd_min"], s["spd_max"], s["err"], s["err_pct"],
                           s["rip_pct"], s["iq_mean"], s["iq_sd"], s["vbus"],
                           s["vbus_min"], s["n"]))
        print("\nCSV -> %s" % a.out)

        # ---- 分析 ----
        print("\n=== 分段统计 ===")
        segs = [(0, 5), (5, 20), (20, 100), (100, 500), (500, 1001)]
        print("  区间(rpm)     点数   平均|误差%|   平均σ%   平均iq(A)")
        for lo, hi in segs:
            g = [s for s in rows if lo <= s["rpm_cmd"] < hi]
            if not g:
                continue
            print("  %4d-%-4d     %3d      %6.2f      %6.2f     %6.3f"
                  % (lo, hi - 1, len(g),
                     sum(abs(s["err_pct"]) for s in g) / len(g),
                     sum(s["rip_pct"] for s in g) / len(g),
                     sum(s["iq_mean"] for s in g) / len(g)))

        worst = max(rows, key=lambda s: abs(s["err_pct"]))
        print("\n  最大稳态误差: %.0f rpm  %+.2f%%  (%.2f rpm)"
              % (worst["rpm_cmd"], worst["err_pct"], worst["spd_mean"]))
        worstr = max(rows, key=lambda s: s["rip_pct"])
        print("  最大纹波:     %.0f rpm  σ=%.3f (%.1f%%)"
              % (worstr["rpm_cmd"], worstr["spd_sd"], worstr["rip_pct"]))

        # iq 对转速做线性拟合：iq = a + b*rpm，b 反映黏滞
        n = len(rows)
        sx = sum(s["rpm_cmd"] for s in rows)
        sy = sum(s["iq_mean"] for s in rows)
        sxx = sum(s["rpm_cmd"] ** 2 for s in rows)
        sxy = sum(s["rpm_cmd"] * s["iq_mean"] for s in rows)
        den = n * sxx - sx * sx
        if den:
            b = (n * sxy - sx * sy) / den
            aa = (sy - b * sx) / n
            print("  iq 拟合: iq = %.4f + %.3e·rpm  A  (黏滞等效)"
                  % (aa, b))
        print("  母线: min %.2f V @ %.0f rpm"
              % (min(s["vbus_min"] for s in rows),
                 min(rows, key=lambda s: s["vbus_min"])["rpm_cmd"]))
        return 0
    finally:
        try:
            rig.cmd("D", 0.3)
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
