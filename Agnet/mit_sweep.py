# -*- coding: utf-8 -*-
"""MIT 参数扫描：kp/kd 网格，量化跟踪精度 vs iq 峰值/限幅次数。

每组独立 D→X→kp=0 使能读基准→正弦轨迹→收尾保持帧→D，避免组间状态串扰。
误差按位置帧自身时间戳解析算 p_des（见 MIT模式_实现与实测.md 8.3），
不用"当前时刻 p_des"减"5 ms 前位置"那种会引入假误差的口径。

指标：
  RMS/中位/|e|max  跟踪误差
  travel          实际行程
  iq_peak         |iq| 峰值（限幅 5 A）
  iq_p95          |iq| 95 分位（反映持续出力，避开单点尖峰）
  iq_sd           末段 iq 标准差（极限环指标：kd 过大时 244 Hz 振荡会暴涨）
  n_limit         |iq|≥4.9 A 的帧数（顶限幅计数）

用法:
  python mit_sweep.py COM27                                # 默认网格
  python mit_sweep.py COM27 --grid 10:0.5,10:0.1,30:0.05   # 自定义网格
  python mit_sweep.py COM27 --amp 0.5 --freq 0.5 --secs 4  # 轨迹参数
  python mit_sweep.py COM27 --smooth                       # 余弦起步 vd(0)=0（对比用）
"""
import argparse
import csv
import math
import struct
import time

from cog_ff_test import Rig, x25
from mit_test import pack, cmd_line

MSG_POS, LEN_POS, CRC_POS = 30040, 12, 160
MSG_OUT, LEN_OUT, CRC_OUT = 30027, 32, 58
ENC_REV = 2097152.0
TWO_PI = 2.0 * math.pi
IQ_LIMIT = 5.0


def scan(buf, out):
    """从字节流抠 30040(pos) / 30027(iq)，返回消费到的位置"""
    i = 0
    n = len(buf)
    while i + 12 <= n:
        if buf[i] != 0xFD:
            i += 1
            continue
        ln = buf[i + 1]
        mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        b = i + 10
        if b + ln + 2 > n:
            break
        if not ((mid == MSG_POS and ln == LEN_POS) or (mid == MSG_OUT and ln == LEN_OUT)):
            i += 1
            continue
        crc = buf[b + ln] | (buf[b + ln + 1] << 8)
        ex = CRC_POS if mid == MSG_POS else CRC_OUT
        if x25(buf[i + 1:b + ln], ex) != crc:
            i += 1
            continue
        if mid == MSG_POS:
            out.append(("pos", struct.unpack_from("<q", buf, b)[0]))
        else:
            out.append(("iq", struct.unpack_from("<f", buf, b + 16)[0]))
        i = b + ln + 2
    return i


def enable_and_baseline(rig):
    """kp=0 使能，读 p0（控制器坐标）与 enc0（编码器多圈）。失败返回 (None,None,cl)"""
    rig.cmd("D", 0.3)
    rig.cmd("X", 0.3)
    rig.s.write(pack(0.0, 0.0, 0.0, 0.08, 0.0))
    rig.s.flush()
    time.sleep(0.4)
    cl = cmd_line(rig.cmd("S", 0.6))
    if " p=" not in cl:
        return None, None, cl
    p0 = float(cl.split(" p=")[1].split()[0])
    # enc0 必须与 p0 取自**同一静止时刻**。上一组 D 后电机可能仍在惯性滑行，
    # 若读到的是"p0 之后又转了一段"的 enc，整段误差会带一个固定偏置
    # （实测随机出现 RMS 0.048 rad、|e|中位 0.047，与 kd 无关）。
    # 故要求连续 3 帧 enc 变化 < 8 counts（≈0.0024 rad）才认基准。
    rig.s.reset_input_buffer()
    enc0 = None
    buf = bytearray()
    recent = []
    t = time.time()
    while time.time() - t < 1.5:
        d = rig.s.read(4096)
        if d:
            buf += d
        got = []
        k = scan(buf, got)
        del buf[:k]
        for kind, v in got:
            if kind == "pos":
                recent.append(v)
                if len(recent) > 4:
                    recent.pop(0)
                if len(recent) >= 3 and max(recent) - min(recent) <= 8:
                    enc0 = recent[-1]
                    break
        if enc0 is not None:
            break
        time.sleep(0.005)
    return p0, enc0, cl


def traj(smooth, p0, amp, w, t):
    """返回 (p_des, v_des)"""
    if smooth:
        return p0 + amp * (1.0 - math.cos(w * t)), amp * w * math.sin(w * t)
    return p0 + amp * math.sin(w * t), amp * w * math.cos(w * t)


def run_one(rig, kp, kd, amp, freq, secs, rate, smooth):
    p0, enc0, cl = enable_and_baseline(rig)
    if p0 is None or enc0 is None:
        return None
    w = TWO_PI * freq
    t0 = time.time()
    nxt = t0
    frames = 0
    pos_log = []
    iq_log = []
    buf = bytearray()
    while True:
        now = time.time() - t0
        if now > secs:
            break
        pd, vd = traj(smooth, p0, amp, w, now)
        rig.s.write(pack(pd, vd, kp, kd, 0.0))
        frames += 1
        d = rig.s.read(4096)          # timeout=0，非阻塞
        if d:
            buf += d
            got = []
            k = scan(buf, got)
            del buf[:k]
            for kind, v in got:
                if kind == "pos":
                    pos_log.append((now, p0 + (v - enc0) * TWO_PI / ENC_REV))
                else:
                    iq_log.append((now, v))
        nxt += 1.0 / rate
        slp = nxt - time.time()
        if slp > 0:
            time.sleep(slp)

    # 收尾：连续发保持帧 0.2 s（mit_run.py 的教训：只发一帧丢了会顶着限幅）
    pend, _ = traj(smooth, p0, amp, w, secs)
    for _ in range(int(0.2 * rate)):
        rig.s.write(pack(pend, 0.0, kp, kd, 0.0))
        rig.s.flush()
        time.sleep(1.0 / rate)
    time.sleep(0.1)

    # 误差用**相对轨迹**：p_meas − p_meas[0] 对 amp·sin(w·t)。
    # 不用 p0/enc0 绝对基准 —— 两者采样时刻并不同步（上一组 D 后可能还在
    # 惯性滑行），实测会随机注入 ~0.0475 rad 的固定偏置，与 kd 无关，
    # 足以淹没 0.005 rad 的真实跟踪误差。相对基准把这个偏置完全消掉。
    t_first, p_first = pos_log[0]
    errs = []
    for tr, p in pos_log:
        if tr < 0.3:
            continue
        # 相位零点用首帧的实际时刻，不用名义 t=0（首帧到达时 tr 已 >0，
        # 直接 sin(w·tr) 会引入 amp·sin(w·t0)≈0.03 rad 的假误差）
        pd, _ = traj(smooth, 0.0, amp, w, tr - t_first)
        errs.append(pd - (p - p_first))
    if not errs or not iq_log:
        return None

    ae = sorted(abs(x) for x in errs)
    rms = math.sqrt(sum(x * x for x in errs) / len(errs))
    ps = [x for _, x in pos_log]
    aqs = sorted(abs(x) for _, x in iq_log)
    tail = [x for _, x in iq_log if _ > 0.3]
    m_tail = sum(tail) / len(tail) if tail else 0.0
    iq_sd = math.sqrt(sum((x - m_tail) ** 2 for x in tail) / len(tail)) if tail else 0.0
    return dict(
        kp=kp, kd=kd,
        rms=rms, emax=ae[-1], emed=ae[len(ae) // 2],
        travel=max(ps) - min(ps),
        iq_peak=aqs[-1], iq_p95=aqs[int(len(aqs) * 0.95)],
        iq_sd=iq_sd,
        n_limit=sum(1 for x in aqs if x >= IQ_LIMIT - 0.1),
        n_pos=len(pos_log), n_iq=len(iq_log), frames=frames,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--grid", default="10:0.5,10:0.2,10:0.1,10:0.05,10:0.02,30:0.05,60:0.05,100:0.05")
    ap.add_argument("--amp", type=float, default=0.5)
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--secs", type=float, default=4.0)
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--smooth", action="store_true")
    ap.add_argument("--csv", default="")
    ap.add_argument("--md", default="")
    a = ap.parse_args()

    grid = []
    for item in a.grid.split(","):
        kps, kds = item.strip().split(":")
        grid.append((float(kps), float(kds)))

    rig = Rig(a.port)
    rig.s.timeout = 0                 # 非阻塞读，保住 500 Hz 发帧节奏
    rows = []
    try:
        time.sleep(2.0)
        print("MIT 参数扫描  amp=%.2f rad  f=%.2f Hz  secs=%.1f  rate=%.0f Hz  %s"
              % (a.amp, a.freq, a.secs, a.rate, "余弦起步" if a.smooth else "正弦起步"))
        print("网格: " + "  ".join("%g/%g" % g for g in grid))
        print("  kp    kd  |  RMS(rad)  |e|max  |e|中位  行程   iq峰  iq_p95  iq_sd  顶限幅")
        print("  " + "-" * 78)
        for i, (kp, kd) in enumerate(grid):
            r = run_one(rig, kp, kd, a.amp, a.freq, a.secs, a.rate, a.smooth)
            if r is None:
                print("  %5g %6g  |  采集失败（基准读取或帧不足）" % (kp, kd))
                continue
            rows.append(r)
            print("  %5g %6g  |  %8.4f  %7.4f  %7.4f  %5.3f  %5.2f  %6.2f  %5.3f  %4d"
                  % (r["kp"], r["kd"], r["rms"], r["emax"], r["emed"], r["travel"],
                     r["iq_peak"], r["iq_p95"], r["iq_sd"], r["n_limit"]))
            rig.cmd("D", 0.2)
            print("      [%d/%d]  pos %d / iq %d / 发帧 %d" % (i + 1, len(grid),
                  r["n_pos"], r["n_iq"], r["frames"]))
        rig.cmd("D", 0.4)

        if rows:
            print("\n判读：")
            best = min(rows, key=lambda x: x["rms"])
            quiet = min(rows, key=lambda x: x["iq_peak"])
            print("  精度最优: kp=%g kd=%g  RMS %.4f rad" % (best["kp"], best["kd"], best["rms"]))
            print("  最省电流: kp=%g kd=%g  iq峰 %.2f A  RMS %.4f rad"
                  % (quiet["kp"], quiet["kd"], quiet["iq_peak"], quiet["rms"]))
            bad = [x for x in rows if x["n_limit"] > 0 or x["iq_sd"] > 0.5]
            if bad:
                print("  顶限幅/振荡: " + "  ".join("kp%g/kd%g(iq峰%.1f,sd%.2f)" %
                      (x["kp"], x["kd"], x["iq_peak"], x["iq_sd"]) for x in bad))
            else:
                print("  全部组合无顶限幅、无极限环")

        if a.csv and rows:
            with open(a.csv, "w", newline="", encoding="utf-8-sig") as f:
                wcsv = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                wcsv.writeheader()
                wcsv.writerows(rows)
            print("CSV ->", a.csv)
        if a.md and rows:
            with open(a.md, "w", encoding="utf-8-sig") as f:
                f.write("# MIT 参数扫描结果\n\n")
                f.write("amp=%.2f rad, f=%.2f Hz, secs=%.1f, rate=%.0f Hz, %s\n\n"
                        % (a.amp, a.freq, a.secs, a.rate, "余弦起步" if a.smooth else "正弦起步"))
                f.write("| kp | kd | RMS(rad) | |e|max | |e|中位 | 行程 | iq峰 | iq_p95 | iq_sd | 顶限幅 |\n")
                f.write("|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")
                for r in rows:
                    f.write("| %g | %g | %.4f | %.4f | %.4f | %.3f | %.2f | %.2f | %.3f | %d |\n"
                            % (r["kp"], r["kd"], r["rms"], r["emax"], r["emed"], r["travel"],
                               r["iq_peak"], r["iq_p95"], r["iq_sd"], r["n_limit"]))
            print("MD  ->", a.md)
    finally:
        rig.cmd("D", 0.3)
        rig.close()


if __name__ == "__main__":
    main()
