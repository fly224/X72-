# -*- coding: utf-8 -*-
"""dead_cal.py —— 死区电压标定（d 轴扫描法）

原理：
  死区让逆变器实际输出电压比指令少 `Vdead·sign(i)`，Vdead = (Tdead/Ts)·Vbus。
  电流环是闭环，稳态时会用积分把这部分补回来 —— 于是**指令电压**里就
  多出了一份固定偏置：
      vd_cmd = R·id + Vdead·sign(id)
  对 id 做正负对称扫描，拟合上面这个式子，斜率就是 R、截距就是 Vdead。
  正负两侧截距反号，所以不会和"电流零点偏置"混在一起。

为什么用 id 不用 iq：
  id 沿转子磁链方向，表贴式（Ld=Lq）下不产生力矩，转子不动。
  用 iq 的话 0.3 A 就能让空载电机加速到 300 rpm 撞超速保护。

取电压的方式：
  30027 的 +0/+4/+8 是三相桥臂电压指令（(duty-0.5)·Vbus），
  用与电流环同一套 Park 变换折到 dq：
      valpha = ua,  vbeta = (ua + 2·ub)/√3
      vd =  cos(th)·valpha + sin(th)·vbeta
      vq = -sin(th)·valpha + cos(th)·vbeta
  th 用遥测的 elec_theta（14·θm）+ π/2，与 servo_isr.c 的 th_e 同源。

判据：拟合出的 R 应接近 0.40 Ω（`@` 在线辨识给 395–405 mΩ）。
      若 R 明显不对，先怀疑 dir 符号或 Park 相位，不要急着信 Vdead。

用法:
    python dead_cal.py COM27
    python dead_cal.py COM27 --ids -0.5,-0.3,-0.15,-0.05,0.05,0.15,0.3,0.5
"""
import argparse
import math
import struct
import time

from cog_ff_test import LEN_IN, MSG_IN, Rig, x25
from torque_test import send_num

LEN_OUT = 32
MSG_OUT = 30027
CRC_OUT = 58

TDEAD_NS = 294.0       # DTG=50 × tDTS(5.88 ns)
TS_US = 50.0           # 20 kHz
DIR_SIGN = 1.0         # 本项目 dm=0，dir=+1；若 R 拟合为负，先改这里


def collect_v(rig, secs):
    """采 30025（theta/vbus）与 30027（ua/ub/uc + id/iq 实测）"""
    vin, vout = [], []
    t0 = time.time()
    while time.time() - t0 < secs:
        d = rig.s.read(4096)
        if d:
            rig.raw += d
        i = 0
        n = len(rig.raw)
        while i + 12 <= n:
            if rig.raw[i] != 0xFD:
                i += 1
                continue
            ln = rig.raw[i + 1]
            mid = rig.raw[i + 7] | (rig.raw[i + 8] << 8) | (rig.raw[i + 9] << 16)
            if mid not in (MSG_IN, MSG_OUT):
                i += 1
                continue
            if (mid == MSG_IN and ln != LEN_IN) or (mid == MSG_OUT and ln != LEN_OUT):
                i += 1
                continue
            b = i + 10
            if b + ln + 2 > n:
                break
            crc = rig.raw[b + ln] | (rig.raw[b + ln + 1] << 8)
            extra = 191 if mid == MSG_IN else CRC_OUT
            if x25(rig.raw[i + 1:b + ln], extra) != crc:
                i += 1
                continue
            f = lambda o: struct.unpack_from("<f", rig.raw, b + o)[0]
            if mid == MSG_IN:
                vin.append((f(20), f(24)))
            else:
                vout.append((f(0), f(4), f(8), f(12), f(16)))
            i = b + ln + 2
        del rig.raw[:i]
    return vin, vout


def dq_of(ua, ub, uc, the):
    va = ua
    vb = (ua + 2.0 * ub) / math.sqrt(3.0)
    th = DIR_SIGN * the + math.pi / 2.0
    c, s = math.cos(th), math.sin(th)
    return c * va + s * vb, -s * va + c * vb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--ids", default="-0.5,-0.4,-0.3,-0.2,-0.1,0.1,0.2,0.3,0.4,0.5")
    ap.add_argument("--settle", type=float, default=0.6)
    ap.add_argument("--secs", type=float, default=0.5)
    a = ap.parse_args()

    ids = [float(x) for x in a.ids.split(",")]
    rig = Rig(a.port)
    pts = []
    try:
        print("S:", rig.cmd("S", 0.6))
        print(rig.cmd("X", 0.4))
        print(send_num(rig, ">", 0.0))          # 进力矩模式，iq=0
        print(send_num(rig, "/", 0.0))

        print("\n%8s %9s %9s %9s %9s %9s" %
              ("id_ref", "id_meas", "vd(V)", "vq(V)", "iq_meas", "vbus"))
        print("-" * 60)

        for want in ids:
            send_num(rig, "/", want)
            time.sleep(a.settle)
            vin, vout = collect_v(rig, a.secs)
            if not vout or not vin:
                print("%8.2f  帧不足" % want)
                continue
            # 各取均值（theta 变化缓慢，逐帧配对取均值即可）
            nv = min(len(vin), len(vout))
            the = vin[-1][0]
            vb = vin[-1][1]
            ua = sum(x[0] for x in vout) / len(vout)
            ub = sum(x[1] for x in vout) / len(vout)
            uc = sum(x[2] for x in vout) / len(vout)
            idm = sum(x[3] for x in vout) / len(vout)
            iqm = sum(x[4] for x in vout) / len(vout)
            vd, vq = dq_of(ua, ub, uc, the)
            pts.append((want, idm, vd, vq, iqm, vb))
            print("%8.2f %9.4f %9.4f %9.4f %9.4f %9.2f"
                  % (want, idm, vd, vq, iqm, vb))

        send_num(rig, "/", 0.0)
        rig.cmd(">0" if False else "D", 0.3)

        # 两参数最小二乘：vd = R·id + Vdead·sgn(id)
        use = [p for p in pts if abs(p[1]) > 0.02]
        if len(use) < 4:
            print("\n有效点不足，无法拟合")
            return 1
        sx = sum(p[1] for p in use)
        sy = sum(p[2] for p in use)
        ss = sum(1.0 if p[1] > 0 else -1.0 for p in use)
        sxx = sum(p[1] * p[1] for p in use)
        sxs = sum(p[1] * (1.0 if p[1] > 0 else -1.0) for p in use)
        sxy = sum(p[1] * p[2] for p in use)
        sys = sum(p[2] * (1.0 if p[1] > 0 else -1.0) for p in use)
        n = len(use)
        # [sxx sxs; sxs n] [R; Vdead] = [sxy; sys]
        det = sxx * n - sxs * sxs
        if abs(det) < 1e-9:
            print("\n矩阵退化")
            return 1
        R = (sxy * n - sys * sxs) / det
        Vd = (sxx * sys - sxs * sxy) / det

        vb_mean = sum(p[5] for p in use) / n
        vd_theory = (TDEAD_NS * 1e-9) / (TS_US * 1e-6) * vb_mean
        print("\n=== 拟合结果（%d 点）===" % n)
        print("  R      = %.4f Ω   （@RL 在线辨识参考 0.395–0.405）" % R)
        print("  Vdead  = %.4f V   （理论 %.4f V）" % (Vd, vd_theory))
        print("  比值   = %.3f" % (Vd / vd_theory if vd_theory else 0.0))
        print("\n  残差：")
        for p in use:
            pred = R * p[1] + Vd * (1.0 if p[1] > 0 else -1.0)
            print("    id=%+.3f  vd=%+.4f  预测=%+.4f  差=%+.4f"
                  % (p[1], p[2], pred, p[2] - pred))
        print("\n死区补偿增益建议: %.2f （= Vdead / (0.0059·Vbus)，`\"` 不适用，用 Servo_SetCurDeadGain）"
              % (Vd / vd_theory if vd_theory else 0.0))
        return 0
    finally:
        try:
            rig.cmd("D", 0.3)
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
