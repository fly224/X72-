"""
enc_err_measure.py —— 测量编码器电角度误差曲线（不依赖外部仪器）

原理
----
速度环稳态、id=0 时，电流环 d 轴 PI 的输出等于"角度误差在 d 轴上的投影"：

    PI_d = ωe · ψ · sin(δ) ≈ ωe · ψ · δ        (δ = 估计角 − 真实角)

所以  δ(θm) = vd_int / (ωe · ψ)，按机械角 θm 分箱平均就得到误差曲线。
vd_int 由 30094 的通道 3（vd 积分 mV + 单圈机械角）采回。

用法
----
    python enc_err_measure.py COM27 500          # 500 rpm，采 5 段
    python enc_err_measure.py COM27 500 --fit    # 顺便打印 1~8 阶拟合系数

输出：终端统计 + enc_err_<rpm>.csv（θm, delta_deg, 拟合残差）
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


def ensure_ready(lk):
    st = lk.ctrl_status()
    if st is None:
        raise SystemExit("30090 无应答")
    if st["fault"]:
        lk.action(3)
        time.sleep(0.6)
    if not st["cal"]:
        raise SystemExit("cal=0：先做 C（30069 action=4）")
    if not st["z"]:
        raise SystemExit("z=0：先做 Z（30069 action=5）")
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.4)


def spin(lk, rpm):
    lk.set_mode(3)
    lk.set_speed_accel(400.0)
    lk.set_speed_traj(0.0)
    lk.set_speed_rpm(rpm)
    lk.set_keepalive(True, 3, 5.0)
    time.sleep(0.3)
    lk.set_controlword(0x0F, 0)
    time.sleep(2.6)


def capture(lk, seg=0.70):
    r = lk.rec_start(ch=3, div=10)          # 2 kHz，1024 点 = 512 ms
    if not r.get("ok"):
        raise RuntimeError("rec_start 失败：%s" % r)
    time.sleep(seg + 0.15)
    d = lk.rec_read()
    if not d.get("ok"):
        raise RuntimeError("rec_read 失败：%s" % d)
    return d["val"], d["th"]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 500.0
    do_fit = "--fit" in sys.argv

    lk = connect(port)
    ensure_ready(lk)
    spin(lk, rpm)

    lp = lk.loop_status()
    psi = lk.dbg_read()["floats"]["psi"]
    w_mech = lp["wmeas"] * TWO_PI / 60.0
    we = POLE * w_mech
    print("稳态：%.1f rpm  ψ=%.4f mWb  ωe=%.1f rad/s  ωeψ=%.3f V"
          % (lp["wmeas"], psi * 1000, we, we * psi))

    sums = [0.0] * NBIN
    cnts = [0] * NBIN
    n_total = 0

    for seg in range(5):
        val, th = capture(lk)
        for v, t in zip(val, th):
            d = v / (we * psi)              # δ，rad（电角度）
            if abs(d) > 0.5:                # 明显异常点（换向/瞬态）丢弃
                continue
            b = int(t / TWO_PI * NBIN) % NBIN
            sums[b] += d
            cnts[b] += 1
            n_total += 1
        print("  段 %d：%d 点" % (seg + 1, len(val)))

    curve = []
    for i in range(NBIN):
        curve.append(sums[i] / cnts[i] if cnts[i] else float("nan"))
    filled = [c for c in curve if c == c]
    empty = NBIN - len(filled)
    mean = sum(filled) / len(filled)
    curve = [c - mean for c in curve]       # 去均值：常数项交给 eleoff_deg

    # 空箱用相邻插值补
    for i in range(NBIN):
        if curve[i] != curve[i]:
            j = (i + 1) % NBIN
            while curve[j] != curve[j]:
                j = (j + 1) % NBIN
            k = (i - 1) % NBIN
            while curve[k] != curve[k]:
                k = (k - 1) % NBIN
            curve[i] = 0.5 * (curve[j] + curve[k])

    rms = math.sqrt(sum(c * c for c in curve) / NBIN)
    print("\n采样 %d 点，空箱 %d；δ 均值 %.3f°，去均值后 RMS %.4f°（峰峰 %.4f°）"
          % (n_total, empty, mean * 180 / math.pi,
             rms * 180 / math.pi,
             (max(curve) - min(curve)) * 180 / math.pi))

    # 谐波拟合：d(θ)=Σ a_k sin(kθ)+b_k cos(kθ)
    coef = []
    for k in range(1, 9):
        sa = sb = 0.0
        for i in range(NBIN):
            x = TWO_PI * i / NBIN
            sa += curve[i] * math.sin(k * x)
            sb += curve[i] * math.cos(k * x)
        coef.append((2.0 * sa / NBIN, 2.0 * sb / NBIN))

    print("\n阶次   幅值(°)   相位基准 a_k(°)   b_k(°)")
    for k, (a, b) in enumerate(coef, 1):
        amp = math.hypot(a, b) * 180 / math.pi
        print("  %d    %8.4f   %10.4f  %10.4f" % (k, amp, a * 180 / math.pi, b * 180 / math.pi))

    for n in (1, 2, 4, 8):
        res = []
        for i in range(NBIN):
            x = TWO_PI * i / NBIN
            s = 0.0
            for k in range(n):
                s += coef[k][0] * math.sin((k + 1) * x) + coef[k][1] * math.cos((k + 1) * x)
            res.append(curve[i] - s)
        rr = math.sqrt(sum(r * r for r in res) / NBIN) * 180 / math.pi
        print("  去掉前 %d 阶后残差 RMS = %.4f°" % (n, rr))

    with open("enc_err_%d.csv" % int(rpm), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["theta_deg", "delta_deg"])
        for i in range(NBIN):
            w.writerow(["%.3f" % (360.0 * i / NBIN), "%.6f" % (curve[i] * 180 / math.pi)])

    if do_fit:
        print("\n可直接下发的补偿系数（= −δ 的拟合值，rad）：")
        print("a = [%s]" % ", ".join("%.6f" % (-a) for a, _ in coef))
        print("b = [%s]" % ", ".join("%.6f" % (-b) for _, b in coef))
        print("常量项：eleoff_deg += %.4f" % (-mean * 180 / math.pi))

    lk.set_keepalive(False)
    lk.set_speed_rpm(0)
    time.sleep(0.8)
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.4)
    print("\n收尾：en=%d fault=0x%04X" % (lk.ctrl_status()["en"], lk.ctrl_status()["fault"]))
    lk.disconnect()


if __name__ == "__main__":
    main()
