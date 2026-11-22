#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_ident_math.py —— 对 motor_ident.c 的三段式算法做纯数学复算

目的：在接硬件之前，确认「120° 步进 -> ratio/dir」「扫整圈 -> offset」
「写零点 -> 模型电角度」这一整条链路在符号与环绕上都对，特别是：

  1. dir = -1 时，零点公式里的 dir 乘子是否把 signed 坐标的增量换算回原始坐标；
  2. 模型侧固定相位 +pi/2（three_loop_deploy.c:48）是否被零点吃掉；
  3. 圆形平均 atan2(Σsin, Σcos) 在角度跨越 0/2π 时是否仍然正确。

复算对象是 C 实现的同构版本（同样的常量、同样的累积顺序），不是"重新推一遍公式"。
用法：python Agnet/verify_ident_math.py
"""
import math
import sys

TWO_PI = 6.283185307179586
PI = 3.141592653589793
ENC_COUNTS = 2097152.0
MODEL_ELEC_OFS = 1.57079632679

STEP_ELEC = 2.09439510239          # 120°
STEP_N = 30
SWEEP_PER_REV = 3
DMECH_MIN = 0.002
RATIO_TOL = 0.25

fails = []


def ck(name, cond, detail=""):
    print("  [%s] %-46s %s" % ("PASS" if cond else "FAIL", name, detail))
    if not cond:
        fails.append(name)


def wrap_pi(x):
    return x - TWO_PI * math.floor(x * (1.0 / TWO_PI) + 0.5)


class Plant:
    """被测电机 + 编码器。真实关系：elec = dir*ratio*theta_m + phi"""

    def __init__(self, ratio, d, phi, enc_counts=ENC_COUNTS, noise_rad=0.0, seed=1):
        self.ratio = ratio
        self.dir = d
        self.phi = phi
        self.n = enc_counts
        self.noise = noise_rad
        self._s = seed

    def theta_m(self, theta_v):
        """转子被磁场拖到 theta_v 时的编码器机械角（连续多圈）"""
        raw = (theta_v - self.phi) / (self.dir * self.ratio)
        if self.noise > 0.0:
            # 确定性伪随机，避免引入外部依赖
            self._s = (1103515245 * self._s + 12345) & 0x7FFFFFFF
            raw += ((self._s / 0x7FFFFFFF) - 0.5) * 2.0 * self.noise
        return round(raw * self.n / TWO_PI) * TWO_PI / self.n


def run_ident(plant, z_old=0.0):
    """完整复算一遍 C 里的状态机，返回结果 dict

    关键：虚拟角要区分两个表示
      theta_v_acc —— 连续累积值，决定转子**物理位置**（磁场方向模 2π 等价，
                     但转子是连续跟着转的，step 之间不会瞬移）
      theta_v     —— wrap 到 [-π,π] 的值，只用于 phi = wrap(θv - elec) 这类模运算
    C 侧同样只保存 wrapped 值（等价），所以这里必须按物理连续来模拟才对得上。
    """
    # ---- RAMP 结束：虚拟角 0，记录基准 ----
    theta_v_acc = 0.0
    tm_start = plant.theta_m(theta_v_acc)
    theta_start = (tm_start + z_old) % TWO_PI      # GetThetaCached()：单圈，含 z_old

    # ---- STEP：正 30 + 反 30 ----
    ssin = scos = 0.0
    dsum_fwd = 0.0
    n_step = n_fail = 0
    dmin, dmax = 1e9, -1e9
    prev = tm_start

    for p in range(STEP_N * 2):
        delta = STEP_ELEC if p < STEP_N else -STEP_ELEC
        theta_v_acc += delta
        tm_now = plant.theta_m(theta_v_acc)
        dmech = tm_now - prev
        prev = tm_now
        ad = abs(dmech)
        if ad < DMECH_MIN:
            n_fail += 1
            continue
        ssin += math.sin(ad)
        scos += math.cos(ad)
        n_step += 1
        dmin = min(dmin, ad)
        dmax = max(dmax, ad)
        if p < STEP_N:
            dsum_fwd += dmech

    if n_step < STEP_N // 2:
        return {"stuck": True, "n_step": n_step, "n_fail": n_fail}

    dmech_avg = math.atan2(ssin, scos)
    if dmech_avg < 0:
        dmech_avg = -dmech_avg
    if dmech_avg < 1.0e-6:
        return {"stuck": True, "n_step": n_step, "n_fail": n_fail}

    ratio = STEP_ELEC / dmech_avg
    ratio_int = int(ratio + 0.5)
    d = 1 if dsum_fwd >= 0.0 else -1
    resid = abs(ratio - ratio_int)

    # ---- SWEEP：正反各一遍 ----
    npoint = max(3, int(ratio_int * SWEEP_PER_REV + 0.5))
    osin = ocos = 0.0
    n_sweep = 0

    for sweep_dir in (+1, -1):
        for _ in range(npoint):
            theta_v_acc += sweep_dir * STEP_ELEC
            theta_v = wrap_pi(theta_v_acc)
            tm_now = plant.theta_m(theta_v_acc)
            th_m = theta_start + (tm_now - tm_start)
            e_elec = d * ratio_int * th_m
            phi_i = wrap_pi(theta_v - e_elec)
            osin += math.sin(phi_i)
            ocos += math.cos(phi_i)
            n_sweep += 1

    offset = math.atan2(osin, ocos)

    # ---- 写零点 ----
    # C 里是：z_old = theta_start - raw_start*(2π/N)，而 raw_start*(2π/N) 就是 tm_start 的单圈值
    raw_start_rad = tm_start % TWO_PI
    z_old_calc = theta_start - raw_start_rad
    delta = ((offset - MODEL_ELEC_OFS) / ratio_int) * d
    z_new = z_old_calc + delta

    return {
        "ratio": ratio, "ratio_int": ratio_int, "dir": d, "resid": resid,
        "offset": offset, "z_new": z_new, "n_step": n_step, "n_sweep": n_sweep,
        "n_fail": n_fail, "dmin": dmin, "dmax": dmax, "stuck": False,
    }


def check_closed_loop(plant, r, z_old=0.0, tag=""):
    """验证：写零点后，模型算出的电角度 == 真实电角度（模 2π）"""
    errs = []
    for k in range(24):
        theta_v_acc = k * 0.7 - 3.0        # 连续位置，直接作为 plant 的输入
        tm = plant.theta_m(theta_v_acc)
        # 写零点后 BSP 返回的原始坐标单圈角
        theta_orig = (tm + r["z_new"]) % TWO_PI
        # servo_isr 按 dir 符号送给模型
        model_theta = r["dir"] * theta_orig
        model_elec = r["ratio_int"] * model_theta + MODEL_ELEC_OFS
        # 真实电角度（同一位置下）
        elec_true = plant.dir * plant.ratio * tm + plant.phi
        err = wrap_pi(model_elec - elec_true)
        errs.append(abs(err))
    worst = max(errs)
    ck("闭环电角度一致（%s）" % tag, worst < 1e-3,
       "最大偏差 %.3e rad = %.5f deg" % (worst, worst * 57.2957795131))
    return worst


def main():
    print("=" * 76)
    print("motor_ident 三段式算法数学复算（C 实现的同构版本）")
    print("=" * 76)

    cases = [
        # (极对数, 方向, 真实零点 phi, 说明)
        (14, +1, 0.0, "14 对 / 正向 / phi=0"),
        (14, +1, 0.837, "14 对 / 正向 / phi=0.837"),
        (14, -1, 0.837, "14 对 / 反向 / phi=0.837"),
        (14, -1, -2.9, "14 对 / 反向 / phi=-2.9（跨 -π）"),
        (7, +1, 3.0, "7 对 / 正向 / phi=3.0（贴近 π）"),
        (21, +1, 1.2, "21 对 / 正向"),
        (4, -1, 0.1, "4 对 / 反向"),
    ]

    for ratio, d, phi, tag in cases:
        print("\n--- %s ---" % tag)
        plant = Plant(ratio, d, phi)
        r = run_ident(plant)
        ck("ratio 恢复 %.0f" % ratio, abs(r["ratio"] - ratio) < 0.02,
           "%.4f (resid %.4f)" % (r["ratio"], r["resid"]))
        ck("ratio_int == %d" % ratio, r["ratio_int"] == ratio, str(r["ratio_int"]))
        ck("dir 恢复 %+d" % d, r["dir"] == d, "%+d" % r["dir"])
        ck("样本数", r["n_step"] == 2 * STEP_N and r["n_sweep"] == 2 * r["ratio_int"] * SWEEP_PER_REV,
           "step=%d sweep=%d bad=%d" % (r["n_step"], r["n_sweep"], r["n_fail"]))
        check_closed_loop(plant, r, tag=tag)

    # 噪声鲁棒性：编码器量化 1 count + 0.3 mrad 噪声
    print("\n--- 加噪：14 对 / 正向 / ±0.3 mrad 随机噪声 ---")
    plant = Plant(14, +1, 1.1, noise_rad=0.0003)
    r = run_ident(plant)
    ck("ratio 仍然准确", abs(r["ratio"] - 14) < 0.05, "%.4f (resid %.4f)" % (r["ratio"], r["resid"]))
    ck("dir 仍然正确", r["dir"] == +1, "%+d" % r["dir"])
    check_closed_loop(plant, r, tag="含噪声")

    # 卡死检测：转子完全不动
    print("\n--- 卡死：转子不动（电流不足 / 机械卡死） ---")
    class Stuck(Plant):
        def theta_m(self, theta_v):
            return 0.0
    r = run_ident(Stuck(14, +1, 0.0))
    ck("判为卡死（不硬算 ratio）", r.get("stuck") is True and r["n_step"] == 0,
       "step=%d bad=%d" % (r["n_step"], r["n_fail"]))

    print("\n" + "=" * 76)
    if fails:
        print("结果：%d 项 FAIL -> %s" % (len(fails), ", ".join(fails)))
        return 1
    print("结果：全部 PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
