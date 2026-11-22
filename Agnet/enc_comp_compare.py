"""
enc_comp_compare.py —— 编码器角度误差补偿：三种软件方案对比

方案
----
A  常数偏置       ：把误差曲线的均值补到 eleoff_deg（已有旋钮，不新增代码）
B  低阶谐波（8 阶）：只补 1~8 次机械谐波
C  高阶谐波（16 阶）：补到 14 阶（=每电周期一次的误差）以上，等效 64 点 LUT

指标
----
1) 角度误差残差：δ(θm) 去均值后的 RMS（越小越好），由 vd 积分反推，见 enc_err_measure.py
2) iq 纹波：1/2/4/14 阶幅值 + 去直流后的 AC RMS

用法
----
    python enc_comp_compare.py COM27 500
"""
import sys
import time
import math
import csv

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink  # noqa: E402

POLE = 14.0
NBIN = 256
TWO_PI = 2.0 * math.pi


def connect(port):
    lk = SerialLink()
    r = lk.connect(port, 921600)
    if not r.get("ok"):
        raise SystemExit("连接失败：%s" % r.get("error"))
    time.sleep(1.4)
    lk.set_permission(0, 0)
    return lk


def amp_at(x, fs, f):
    """Hann 窗单频幅值。"""
    n = len(x)
    if n < 8:
        return 0.0
    sc = 0.0
    ss = 0.0
    sw = 0.0
    for i, v in enumerate(x):
        w = 0.5 - 0.5 * math.cos(TWO_PI * i / (n - 1))
        ph = TWO_PI * f * i / fs
        sc += w * v * math.cos(ph)
        ss += w * v * math.sin(ph)
        sw += w
    return 2.0 * math.hypot(sc, ss) / sw


def iq_metric(lk, rpm):
    """连采 3 段取平均：单段 512 ms 的谱幅值抖动可达 ±15%。"""
    acc = {"dc": 0.0, "ac_rms": 0.0, "h1": 0.0, "h2": 0.0, "h4": 0.0, "h14": 0.0}
    n = 0
    for _ in range(3):
        r = lk.rec_start(ch=0, div=10)
        if not r.get("ok"):
            return None
        time.sleep(0.75)
        d = lk.rec_read()
        if not d.get("ok"):
            return None
        x = d["val"]
        fs = 20000.0 / 10.0
        dc = sum(x) / len(x)
        ac = [v - dc for v in x]
        fm = rpm / 60.0
        acc["dc"] += dc
        acc["ac_rms"] += math.sqrt(sum(v * v for v in ac) / len(ac))
        acc["h1"] += amp_at(ac, fs, fm)
        acc["h2"] += amp_at(ac, fs, 2 * fm)
        acc["h4"] += amp_at(ac, fs, 4 * fm)
        acc["h14"] += amp_at(ac, fs, 14 * fm)
        n += 1
    return {k: v / n for k, v in acc.items()}


def measure(lk, segs=5):
    """返回 (curve[256] 去均值, mean_rad, rms_rad, psi)。"""
    lp = lk.loop_status()
    psi = lk.dbg_read()["floats"]["psi"]
    rpm = lp["wmeas"]
    we = POLE * rpm * TWO_PI / 60.0
    if abs(we) < 10.0 or psi <= 0:
        raise RuntimeError("转速/psi 异常：rpm=%.1f psi=%.5f" % (rpm, psi))

    sums = [0.0] * NBIN
    cnts = [0] * NBIN
    for _ in range(segs):
        r = lk.rec_start(ch=3, div=10)
        if not r.get("ok"):
            raise RuntimeError("rec_start(ch=3) 失败：%s" % r)
        time.sleep(0.85)
        d = lk.rec_read()
        if not d.get("ok"):
            raise RuntimeError("rec_read 失败：%s" % d)
        for v, t in zip(d["val"], d["th"]):
            dd = v / (we * psi)
            if abs(dd) > 0.5:
                continue
            b = int(t / TWO_PI * NBIN) % NBIN
            sums[b] += dd
            cnts[b] += 1

    raw = [sums[i] / cnts[i] if cnts[i] else float("nan") for i in range(NBIN)]
    good = [c for c in raw if c == c]
    mean = sum(good) / len(good)
    for i in range(NBIN):
        if raw[i] != raw[i]:
            raw[i] = mean
    curve = [c - mean for c in raw]
    rms = math.sqrt(sum(c * c for c in curve) / NBIN)
    return curve, mean, rms, psi


def fit(curve, n):
    a, b = [], []
    for k in range(1, n + 1):
        sa = sb = 0.0
        for i in range(NBIN):
            x = TWO_PI * i / NBIN
            sa += curve[i] * math.sin(k * x)
            sb += curve[i] * math.cos(k * x)
        a.append(2.0 * sa / NBIN)
        b.append(2.0 * sb / NBIN)
    return a, b


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 500.0

    lk = connect(port)
    st = lk.ctrl_status()
    if st["fault"]:
        lk.action(3)
        time.sleep(0.6)
    if not st["cal"] or not st["z"]:
        raise SystemExit("cal=%d z=%d：先做 C/Z" % (st["cal"], st["z"]))
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.4)

    # 起点强制归零：上一轮若中途异常退出，可能留下非零 eleoff / 系数
    eleoff0 = 0.0
    lk.enc_comp_off()
    lk.dbg_write(floats={"eleoff_deg": eleoff0})
    lk.set_mode(3)
    lk.set_speed_accel(400.0)
    lk.set_speed_traj(0.0)
    lk.set_speed_rpm(rpm)
    lk.set_keepalive(True, 3, 5.0)
    time.sleep(0.3)
    lk.set_controlword(0x0F, 0)
    time.sleep(2.6)

    rows = []

    def snap(name):
        c, mean, rms, psi = measure(lk)
        iqm = iq_metric(lk, rpm)
        rows.append({"name": name, "mean_deg": mean * 180 / math.pi,
                     "rms_deg": rms * 180 / math.pi,
                     "pp_deg": (max(c) - min(c)) * 180 / math.pi,
                     "iq_dc": iqm["dc"] if iqm else float("nan"),
                     "iq_ac": iqm["ac_rms"] if iqm else float("nan"),
                     "h14": iqm["h14"] if iqm else float("nan"),
                     "h1": iqm["h1"] if iqm else float("nan"),
                     "h4": iqm["h4"] if iqm else float("nan")})
        print("  %-14s δRMS %.4f°  峰峰 %.4f°  iq: DC %.4fA AC %.4fA h14 %.5fA"
              % (name, rms * 180 / math.pi, (max(c) - min(c)) * 180 / math.pi,
                 iqm["dc"] if iqm else 0, iqm["ac_rms"] if iqm else 0,
                 iqm["h14"] if iqm else 0))
        with open("enc_curve_%s.csv" % name, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["theta_deg", "delta_deg"])
            for i in range(NBIN):
                w.writerow(["%.3f" % (360.0 * i / NBIN), "%.6f" % (c[i] * 180 / math.pi)])
        return c

    print("\n[0] 基线（补偿关闭）")
    c0 = snap("baseline")
    a1, b1 = fit(c0, 1)
    a8, b8 = fit(c0, 8)
    a16, b16 = fit(c0, 16)
    print("     14 阶幅值 %.4f°" % (math.hypot(a16[13], b16[13]) * 180 / math.pi))

    print("\n[A] 常数偏置：eleoff += %.4f°" % (rows[0]["mean_deg"] * -1))
    lk.dbg_write(floats={"eleoff_deg": eleoff0 - rows[0]["mean_deg"]})
    time.sleep(0.6)
    snap("A_const")

    print("\n[B] 8 阶谐波补偿")
    lk.enc_comp(a=[-v for v in a8], b=[-v for v in b8], en=True)
    time.sleep(0.8)
    rb = lk.enc_comp()
    print("     回读 n_harm=%s en=%s ok=%s" % (rb.get("n_harm"), rb.get("en"), rb.get("ok")))
    snap("B_h8")

    print("\n[C] 16 阶谐波补偿")
    lk.enc_comp(a=[-v for v in a16], b=[-v for v in b16], en=True)
    time.sleep(0.8)
    rb = lk.enc_comp()
    print("     回读 n_harm=%s en=%s ok=%s" % (rb.get("n_harm"), rb.get("en"), rb.get("ok")))
    snap("C_h16")

    print("\n[收尾] 关闭补偿、恢复 eleoff")
    lk.enc_comp_off()
    lk.dbg_write(floats={"eleoff_deg": eleoff0})
    lk.set_keepalive(False)
    lk.set_speed_rpm(0)
    time.sleep(0.8)
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.3)

    print("\n%-12s %9s %9s %9s %9s %10s %10s"
          % ("方案", "δ均值°", "δRMS°", "iq_DC", "iq_AC", "h1(A)", "h14(A)"))
    for r in rows:
        print("%-12s %9.3f %9.4f %9.4f %9.4f %10.5f %10.5f"
              % (r["name"], r["mean_deg"], r["rms_deg"], r["iq_dc"], r["iq_ac"],
                 r["h1"], r["h14"]))
    lk.disconnect()


if __name__ == "__main__":
    main()
