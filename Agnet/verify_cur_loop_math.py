# -*- coding: utf-8 -*-
"""
verify_cur_loop_math.py —— 电流闭环模式的离线复算（不接硬件）

复算 Board/servo_isr.c 里 s_cur_* 分支的每一条运算：
  th_e   = 14*(dir*theta_m) + pi/2 + 1.005e-3*(dir*omega_m)
  id/iq  = Park(i_alpha, i_beta, th_e)
  vd/vq  = Kp*e + integ (- dec_vd) / (+ dec_vq + bemf)
  圆限幅 + 积分回退 0.98
  逆 Park -> 三相占空比 -> 对象 di/dt = (v - R*i - e)/L

关键约定（与辨识一致）：
  * 零点标定后的定义就是 "14*(dir*theta_m) + pi/2 == 真实电角度"，
    所以仿真里“控制器看到的角”等于“转子真实电角度 + 注入的误差”，
    dir 只改变机械转动的符号，不改变这条等式（这正是 Z 的作用）。
  * 因此 dir=-1 的验证是**代数恒等式**检查，而不是重新跑一遍动力学。

检查项：
  1. +1 A / -1 A 阶跃：上升时间、超调、稳态误差、id 泄漏
  2. dir 符号链路的恒等式（θ 折算与 dir 无关）
  3. 零点误差 0/2/5/10/20 度：id 泄漏与 iq 幅值损失（现场判据）
  4. 母线 < 5 V 的保护分支：不产生 NaN/除零，电流不增长
  5. 超速保护：空载 1 A 到 60 rpm 的时间
"""
import math

# ---------------- 与 C 代码一致的常量 ----------------
KP        = 0.71314153
KI_TS     = 0.11890653
TS        = 5e-5
VLIM_FRAC = 0.5375
POLE      = 14.0
ELEC_OFF  = 1.57079637
COMP      = 1.05e-3
LD        = 113.5e-6
LQ        = 113.5e-6
PSI       = 7.59298e-3
R         = 0.378481
VBUS      = 30.78
WMAX      = 31.415927      # 300 rpm（电机侧）

fails = []


def ck(name, cond, detail=""):
    print("  [%s] %-42s %s" % ("PASS" if cond else "FAIL", name, detail))
    if not cond:
        fails.append(name)


def run_step(iq_ref, ang_err=0.0, vbus=VBUS, t_end=0.010, J=5e-4, Tf=0.03):
    vb = vbus
    integ_d = integ_q = 0.0
    id_m = iq_m = 0.0
    i_alpha = i_beta = 0.0
    th_plant = 0.0            # 转子 d 轴真实电角度（零点标定后 θ 的定义）
    w_mech = 0.0              # 机械角速度
    hist = []
    t = 0.0
    for _ in range(int(t_end / TS)):
        th_used = th_plant + ang_err          # 控制器读到的电角度
        vd = vq = 0.0
        vmag = 0.0
        if vb < 5.0:
            da = db = dc = 0.5                # C 里的保护分支：固定中点
        else:
            cs, sn = math.cos(th_used), math.sin(th_used)
            id_now = cs * i_alpha + sn * i_beta
            iq_now = -sn * i_alpha + cs * i_beta
            id_m, iq_m = id_now, iq_now

            integ_d += (0.0 - id_now) * KI_TS
            integ_q += (iq_ref - iq_now) * KI_TS

            om_e = POLE * w_mech              # 电角速度
            dec_vd = iq_now * om_e * LQ
            dec_vq = id_now * om_e * LD + om_e * PSI

            vd = KP * (0.0 - id_now) + integ_d - dec_vd
            vq = KP * (iq_ref - iq_now) + integ_q + dec_vq

            vmax = vb * VLIM_FRAC
            vmag = math.hypot(vd, vq)
            if vmag > vmax > 0.0:
                kk = vmax / vmag
                vd *= kk
                vq *= kk
                integ_d *= 0.98
                integ_q *= 0.98

            valpha = cs * vd - sn * vq
            vbeta = sn * vd + cs * vq
            da = 0.5 + valpha / vb
            db = 0.5 + (-0.5 * valpha + 0.86602540 * vbeta) / vb
            dc = 0.5 + (-0.5 * valpha - 0.86602540 * vbeta) / vb

        # ---- 对象：占空比 -> 相电压 -> Clarke -> dq -> 电气动态 ----
        ua = (da - 0.5) * vb
        ub = (db - 0.5) * vb
        uc = (dc - 0.5) * vb
        u_alpha = ua
        u_beta = (ua + 2.0 * ub) / math.sqrt(3.0)

        cs2, sn2 = math.cos(th_plant), math.sin(th_plant)
        vd_obj = cs2 * u_alpha + sn2 * u_beta
        vq_obj = -sn2 * u_alpha + cs2 * u_beta
        e_q = POLE * w_mech * PSI

        id_m += (vd_obj - R * id_m) / LD * TS
        iq_m += (vq_obj - R * iq_m - e_q) / LQ * TS

        i_alpha = id_m * cs2 - iq_m * sn2
        i_beta = id_m * sn2 + iq_m * cs2

        # ---- 机械（简化：Kt·iq 与库仑摩擦）----
        Kt = 1.5 * POLE * PSI
        Te = Kt * iq_m
        sign = 0.0 if abs(w_mech) < 1e-9 else (1.0 if w_mech > 0 else -1.0)
        w_mech += (Te - Tf * sign) / J * TS
        th_plant = (th_plant + POLE * w_mech * TS) % (2.0 * math.pi)

        t += TS
        hist.append((t, iq_m, id_m, w_mech, vmag))
    return hist


def stats(hist, iq_ref):
    sgn = 1.0 if iq_ref >= 0 else -1.0
    t10 = t90 = None
    for (t, iq, idn, w, vm) in hist:
        if t10 is None and sgn * iq >= 0.1 * abs(iq_ref):
            t10 = t
        if t90 is None and sgn * iq >= 0.9 * abs(iq_ref):
            t90 = t
    tail = hist[int(len(hist) * 0.7):]
    iq_ss = sum(h[1] for h in tail) / len(tail)
    id_ss = sum(h[2] for h in tail) / len(tail)
    peak = max(abs(h[1]) for h in hist)
    return {
        "tr": (t90 - t10) * 1e6 if (t10 is not None and t90 is not None) else float("nan"),
        "os": (peak / abs(iq_ref) - 1) * 100 if iq_ref else 0.0,
        "iq_ss": iq_ss, "id_ss": id_ss,
        "err": (iq_ss - iq_ref) / abs(iq_ref) * 100 if iq_ref else 0.0,
        "vmax": max(h[4] for h in hist),
    }


print("=" * 78)
print("1) 阶跃响应（空载 J=5e-4, Tf=0.03, 角度误差 0）")
print("=" * 78)
for ref in (+1.0, -1.0):
    h = run_step(ref)
    s = stats(h, ref)
    print("  iq_ref=%+.1f A: tr(10-90%%)=%.0f us  超调=%.1f%%  稳态 iq=%+.4f (%+.2f%%)  id=%+.4f A  峰值电压=%.1f V"
          % (ref, s["tr"], s["os"], s["iq_ss"], s["err"], s["id_ss"], s["vmax"]))
    ck("iq_ref=%+.1f 上升时间 < 1 ms" % ref, s["tr"] < 1000, "%.0f us" % s["tr"])
    ck("iq_ref=%+.1f 超调 < 10%%" % ref, abs(s["os"]) < 10, "%.1f %%" % s["os"])
    ck("iq_ref=%+.1f 稳态误差 < 3%%" % ref, abs(s["err"]) < 3.0, "%+.2f %%" % s["err"])
    ck("iq_ref=%+.1f id 泄漏 < 0.05 A" % ref, abs(s["id_ss"]) < 0.05, "%+.4f A" % s["id_ss"])

print()
print("=" * 78)
print("2) dir 符号链路：零点标定后 '14*(dir*theta_m)+pi/2' 应与 dir 无关")
print("=" * 78)
worst = 0.0
for dir_sign in (+1, -1):
    for k in range(7):
        th_mech = k * 0.37
        th_enc = dir_sign * th_mech           # 编码器计数（dir=-1 时反向）
        th_from_chain = POLE * (dir_sign * th_enc) + ELEC_OFF
        th_ref_chain = POLE * th_mech + ELEC_OFF
        worst = max(worst, abs(th_from_chain - th_ref_chain))
ck("dir=±1 的 θ 折算恒等", worst < 1e-9, "最大偏差 %.2e rad" % worst)

print()
print("=" * 78)
print("3) 电角度零点误差的影响（现场判据：Z 之后 S 里 id 应当很小）")
print("=" * 78)
print("  角度误差   稳态 iq      id 泄漏     iq 幅值损失")
print("  " + "-" * 54)
for deg in (0.0, 2.0, 5.0, 10.0, 20.0):
    h = run_step(1.0, ang_err=math.radians(deg))
    s = stats(h, 1.0)
    loss = (1 - s["iq_ss"]) * 100
    print("  %5.1f deg  %+8.4f A  %+8.4f A  %7.2f %%" % (deg, s["iq_ss"], s["id_ss"], loss))
    if deg <= 5.0:
        ck("%.0f deg 时 id 泄漏 < 0.15 A" % deg, abs(s["id_ss"]) < 0.15, "%+.3f A" % s["id_ss"])
print("  说明：id 泄漏 ≈ iq_ref·sin(误差)，误差 5 度时约 8.7%。")
print("        所以 Z 之后先看 S 的 id：>0.2 A 就说明零点没对齐，别急着加电流。")

print()
print("=" * 78)
print("4) 母线 < 5 V 的保护分支")
print("=" * 78)
h = run_step(1.0, vbus=3.0)
s = stats(h, 1.0)
ck("vbus=3 V 时电流不增长", abs(s["iq_ss"]) < 0.01, "iq=%+.4f A" % s["iq_ss"])
print("  vbus=3 V: 占空比固定 0.5 -> iq=%+.4f A；无除零、无 NaN" % s["iq_ss"])

print()
print("=" * 78)
print("5) 超速保护：空载 1 A 到 300 rpm（电机侧）的时间")
print("=" * 78)
Kt = 1.5 * POLE * PSI
print("  Kt = 1.5*p*psi = %.4f N·m/A（1 A 约 %.2f N·m）" % (Kt, Kt))
for J in (2e-4, 5e-4, 2e-3):
    t_trip = WMAX * J / max(Kt - 0.03, 1e-9)
    print("  J=%.1e kg·m^2, Tf=0.03 N·m -> %.0f ms 到 300 rpm" % (J, t_trip * 1000))
ck("最坏情况也留出 >=40 ms（约 4 帧遥测）", WMAX * 2e-4 / (Kt - 0.03) * 1000 > 40,
   "%.0f ms" % (WMAX * 2e-4 / (Kt - 0.03) * 1000))

print()
print("=" * 78)
print("结论：%s" % ("全部 PASS" if not fails else "FAIL %d 项 -> %s" % (len(fails), ", ".join(fails))))
print("=" * 78)
