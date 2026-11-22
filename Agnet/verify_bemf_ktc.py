# -*- coding: utf-8 -*-
"""验证 BEMF 前馈的温度补偿（dec_vq 的 ψ 项乘 s_kt_comp）。

判据只有一个，但它是决定性的：

    vq 总和 = PI 输出 + 前馈。稳态下 PI→0，所以**总和只由真实 BEMF 决定**，
    看不出前馈准不准。真正镜像"前馈估计误差"的是**积分项**：
        改前（前馈用固定 ψ）：升温后前馈高估，int 被压成负值，
            2000 rpm / 84 °C 理论偏置 = -ωe·ψ·(1-ktc) ≈ -0.98 V
        改后（前馈乘 ktc）：int 回到 0 附近

    所以本脚本测两件事：
      A) 冷态 int 基线（≈0，两种实现都该是 0）
      B) 升温后 int 相对冷态的偏移 Δ
         Δ ≈ 0            → 补偿生效
         Δ ≈ -0.98 V      → 补偿没接上（或 ktc 被关）
         中间值            → 部分生效（检查是否只有 Kt 补了）

测量手段：`'205` 把录波切到 ch=2（vq 积分，mV），`&` 触发 1024 点，
          div=5 → 4 kHz / 256 ms。取均值即稳态偏置。

用法：python verify_bemf_ktc.py COM27 [--rpm 2000] [--t-target 70] [--t-max 40]
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

# 与固件一致：ψ 温漂系数、极对数
KT_ALPHA = -0.001197
POLE = 14.0
RAD_PER_RPM = 0.10471975512


def cmd(rig, c, wait=0.5):
    """发命令并收回显，期间暂停接收线程（避免遥测帧插入）。"""
    rig.rx.pause_evt.set()
    time.sleep(0.03)
    try:
        rig.s.reset_input_buffer()
        rig.s.write(c.encode() if isinstance(c, str) else c)
        rig.s.flush()
        raw = b""
        t0 = time.time()
        while time.time() - t0 < wait:
            d = rig.s.read(8192)
            if d:
                raw += d
        return strip_frames(raw).decode("ascii", "replace")
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def status(rig):
    """S 行（拿 ktc / 故障字）+ 遥测快照（拿温度 / iq / 电角速度）合并成 dict。

    温度只在 30025 遥测里，S 行没有；ktc 只在 ASCII 行里。两边都得取。
    """
    txt = cmd(rig, "S", 0.55)
    d = {}
    for line in txt.splitlines():
        if "[CMD] st" in line:
            for m in re.finditer(r"([A-Za-z_]\w*)=(-?[0-9a-fA-Fx.]+)", line):
                d[m.group(1)] = m.group(2)
            d["line"] = line
            break
    snap = rig.rx.get_snap()[0] or {}
    for k in ("temp_mot", "temp_drv", "iq_ref", "w_elec", "vbus", "spd_ref_rpm"):
        if k in snap:
            d[k] = snap[k]
    return d


def read_psi(rig):
    """从 K 行取控制律正在用的 ψ（mWb）。预测改前偏置必须用实测值，
    用编译期常量 4.333 会引入 3% 的模型误差。"""
    txt = cmd(rig, "K", 0.55)
    m = re.search(r"psi=([0-9.]+)m", txt)
    return float(m.group(1)) / 1000.0 if m else 4.333e-3


def record(rig, timeout=1.2):
    """`&` 触发并读回 [REC] 行，返回原始 int16 列表。"""
    rig.rx.pause_evt.set()
    time.sleep(0.03)
    try:
        rig.s.reset_input_buffer()
        rig.s.write(b"&")
        rig.s.flush()
        raw = bytearray()
        t0 = time.time()
        while time.time() - t0 < timeout:
            d = rig.s.read(4096)
            if d:
                raw += d
            if b"REC end" in raw:
                break
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()

    vals = []
    for m in re.finditer(rb"\[REC\]((?:[0-9a-f]{4})+)", bytes(raw)):
        body = m.group(1)
        for k in range(0, len(body), 4):
            u = int(body[k:k + 4], 16)
            vals.append(u - 0x10000 if u >= 0x8000 else u)
    return vals


def keepalive(rig, rpm, secs):
    """速度模式 deadman 2 s：边跑边重发给定。"""
    t0 = time.time()
    while time.time() - t0 < secs:
        with rig.wlock:
            rig.s.write(("=%g\r" % rpm).encode())
            rig.s.flush()
        time.sleep(0.35)


def one_point(rig, rpm, tag):
    """采一个工况：受控给定 + 稳定 + 录 vq 积分 + 读状态。"""
    keepalive(rig, rpm, 2.0)                      # 建立/维持闭环
    # 切到 ch=2（vq 积分），div=5 → 4 kHz / 256 ms
    cmd(rig, "'205\r", 0.45)
    keepalive(rig, rpm, 0.8)                      # 录波前再保活一次
    vals = record(rig)
    keepalive(rig, rpm, 0.5)
    st = status(rig)

    if not vals:
        return None
    mean_mv = statistics.mean(vals)
    sd_mv = statistics.pstdev(vals) if len(vals) > 1 else 0.0

    t_mot = float(st.get("temp_mot", "nan"))
    w_e = float(st.get("w_elec", "nan"))
    rpm_meas = w_e / POLE / RAD_PER_RPM if w_e == w_e else None

    ktc = 1.0
    if st.get("ktc") not in (None, "off"):
        try:
            ktc = float(st["ktc"])
        except ValueError:
            ktc = 1.0
    return dict(tag=tag, t_mot=round(t_mot, 2), n=len(vals),
                vqint_V=round(mean_mv / 1000.0, 4),
                vqint_sd_V=round(sd_mv / 1000.0, 4),
                ktc=round(ktc, 4),
                rpm=round(rpm_meas, 1) if rpm_meas else None,
                iq=round(float(st.get("iq_ref", 0.0)), 3),
                fault=st.get("f"), line=st.get("line"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--rpm", type=float, default=2000.0)
    ap.add_argument("--t-target", type=float, default=70.0)
    ap.add_argument("--t-max", type=float, default=40.0, help="总时长上限 min")
    ap.add_argument("--t-stop", type=float, default=100.0)
    ap.add_argument("--rep", type=int, default=1,
                    help="到温后重复测几次（vq 积分 sd 有 1 V 级波动，多点取均值）")
    ap.add_argument("--no-cold", action="store_true",
                    help="跳过冷态基线（已在别的温度点采过基线时用）")
    ap.add_argument("--base-t", type=float, default=0.0, help="外部基线温度 °C")
    ap.add_argument("--base-vqint", type=float, default=0.0, help="外部基线 vq 积分 V")
    ap.add_argument("--base-ktc", type=float, default=1.0, help="外部基线 ktc")
    args = ap.parse_args()

    out = "bemf_ktc_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S")
    rig = Rig(args.port)
    rows = []
    psi_used = [4.333e-3]
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.3)
        cmd(rig, "D", 0.5)                        # 先确保停
        cmd(rig, "E", 0.6)
        keepalive(rig, args.rpm, 3.0)
        psi_used[0] = read_psi(rig)

        t_start = time.time()
        if args.no_cold:
            print("[1] 跳过基线，用外部基线 T=%.2f C vqint=%+.4f V ktc=%.3f"
                  % (args.base_t, args.base_vqint, args.base_ktc), flush=True)
            rows.append(dict(tag="cold", t_mot=args.base_t, n=0,
                             vqint_V=args.base_vqint, vqint_sd_V=0.0,
                             ktc=args.base_ktc, rpm=None, iq=None,
                             fault="0x00", line="external baseline"))
        else:
            print("[1] 基线点（当前温度）", flush=True)
            r = one_point(rig, args.rpm, "cold")
            if r:
                rows.append(r)
                print("    T=%.2f C  rpm=%s  iq=%s  ktc=%.4f  vqint=%.4f V (sd %.4f, n=%d)"
                      % (r["t_mot"], r["rpm"], r["iq"], r["ktc"], r["vqint_V"],
                         r["vqint_sd_V"], r["n"]), flush=True)

        t0 = time.time()
        while True:
            el = (time.time() - t0) / 60.0
            if el >= args.t_max:
                print("[!] 达到时长上限 %.0f min" % args.t_max, flush=True)
                break
            st = status(rig)
            t_now = float(st.get("temp_mot", "nan"))
            fault = st.get("f", "0x00")
            if fault not in ("0x00", None):
                print("[!] 故障 f=%s，停止" % fault, flush=True)
                break
            if t_now >= args.t_target:
                print("[2] 到温 %.1f C，测 %d 次" % (t_now, args.rep), flush=True)
                for k in range(args.rep):
                    r = one_point(rig, args.rpm, "hot%d" % (k + 1))
                    if r:
                        rows.append(r)
                        print("    [%d] T=%.2f C  rpm=%s  iq=%s  ktc=%.4f  vqint=%.4f V (sd %.4f, n=%d)"
                              % (k + 1, r["t_mot"], r["rpm"], r["iq"], r["ktc"],
                                 r["vqint_V"], r["vqint_sd_V"], r["n"]), flush=True)
                    keepalive(rig, args.rpm, 0.6)
                break
            if t_now >= args.t_stop:
                print("[!] 电机 %.1f C 超限，停止" % t_now, flush=True)
                break
            print("    ... T=%.1f C  (%.0f/%.0f min)" % (t_now, el, args.t_max), flush=True)
            keepalive(rig, args.rpm, 60.0)

        # 收尾：多做一次中途点（若还没到目标温度）
        if len(rows) == 1 and time.time() - t0 > 300:
            r = one_point(rig, args.rpm, "mid")
            if r:
                rows.append(r)
                print("    mid: T=%.2f C  vqint=%.4f V  ktc=%.4f"
                      % (r["t_mot"], r["vqint_V"], r["ktc"]), flush=True)
    finally:
        try:
            cmd(rig, "D", 0.6)
        except Exception:
            pass
        rig.close()

    if len(rows) < 2:
        print("[!] 有效点不足 2 个，无法给出 Δ")
        return 1

    # ---- 判定 ----
    cold = rows[0]
    hot_rows = [x for x in rows if x["tag"].startswith("hot")] or [rows[-1]]
    hv = statistics.mean(x["vqint_V"] for x in hot_rows)
    hk = statistics.mean(x["ktc"] for x in hot_rows)
    ht = statistics.mean(x["t_mot"] for x in hot_rows)
    hsd = statistics.pstdev([x["vqint_V"] for x in hot_rows]) if len(hot_rows) > 1 else 0.0

    d_meas = hv - cold["vqint_V"]
    we = POLE * args.rpm * RAD_PER_RPM
    psi = psi_used[0]
    d_pred_old = -we * psi * (1.0 - hk)   # 改前理论偏置

    print("\n=== 结果 ===")
    print("基线: T=%.2f C  ktc=%.4f  vqint=%+.4f V" % (cold["t_mot"], cold["ktc"], cold["vqint_V"]))
    print("高温: T=%.2f C  ktc=%.4f  vqint=%+.4f V  (n=%d 点, 组内 sd=%.4f)"
          % (ht, hk, hv, len(hot_rows), hsd))
    print("实测 Δint = %+.4f V" % d_meas)
    print("改前理论 Δ = %+.4f V  (前馈不补，ωe·ψ·(1−ktc))" % d_pred_old)
    if abs(d_pred_old) > 0.2:
        frac = d_meas / d_pred_old
        print("残余比例 = %.1f%% （0%%=完全补偿，100%%=完全没补）" % (frac * 100.0))

    with open(out, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print("CSV: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
