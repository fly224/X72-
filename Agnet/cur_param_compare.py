# -*- coding: utf-8 -*-
"""电流环两套参数对比：模型路径 vs 手写路径。

对象：L、R 用实测值；控制器为 PI + 采样/调制延时。
输出：增益穿越频率、相位裕度、低频环路增益、离散闭环阶跃响应。
"""
import math

TS = 50e-6              # 20 kHz
TD = 1.5 * TS           # 采样 + 调制延时（ADC 在谷值采样、本拍立即更新占空比）

# ---- 控制器参数 ----
# 模型：servo_isr.c:1883  model_params_init()
MODEL = dict(name="模型 par_c", kp=0.71314153, kits=0.11890653,
             ld=113.5e-6, lq=113.5e-6, psi=7.59298e-3)
# 手写：servo_isr.c  SERVO_CUR_KP / SERVO_CUR_KI_TS（2026-11-17 实测校正后）
HAND = dict(name="手写宏", kp=0.5341, kits=0.12566,
            ld=85.0e-6, lq=85.0e-6, psi=4.333e-3)

# ---- 实测对象 ----
PLANTS = [
    dict(name="R=0.40 L=85u（@辨识基准）", R=0.40, L=85.0e-6),
    dict(name="R=0.616 L=85u（d轴静扫）", R=0.616, L=85.0e-6),
    dict(name="R=0.378 L=113.5u（模型原始假定）", R=0.3785, L=113.5e-6),
]


def loop_response(kp, ki, R, L, f):
    """开环增益 C(jw)P(jw)e^{-jwTd}，f 单位 Hz"""
    w = 2 * math.pi * f
    # PI: kp + ki/(jw)
    cr = kp
    ci = -ki / w
    # 对象 1/(R + jwL)
    den = R * R + (w * L) ** 2
    pr = R / den
    pi_ = -w * L / den
    # 乘
    gr = cr * pr - ci * pi_
    gi = cr * pi_ + ci * pr
    # 延时
    ph = -w * TD
    gr2 = gr * math.cos(ph) - gi * math.sin(ph)
    gi2 = gr * math.sin(ph) + gi * math.cos(ph)
    return math.hypot(gr2, gi2), math.atan2(gi2, gr2)


def margins(kp, ki, R, L):
    lo, hi = 1.0, 20000.0
    # 找 |L|=1（在 10 Hz~20 kHz 内扫，取第一次穿越）
    prev = None
    fc = None
    f = 10.0
    while f < 9000.0:
        m, _ = loop_response(kp, ki, R, L, f)
        if prev is not None and prev > 1.0 >= m:
            a, b = f * 0.9, f
            for _ in range(60):
                mid = 0.5 * (a + b)
                if loop_response(kp, ki, R, L, mid)[0] > 1.0:
                    a = mid
                else:
                    b = mid
            fc = 0.5 * (a + b)
            break
        prev = m
        f *= 1.02
    if fc is None:
        return None
    pm = 180.0 + math.degrees(loop_response(kp, ki, R, L, fc)[1])
    # 低频环路增益（10 Hz）：扰动抑制能力
    g10 = loop_response(kp, ki, R, L, 10.0)[0]
    return fc, pm, g10


def sim_step(kp, kits, R, L, iref=2.0, n=2000):
    """离散闭环仿真：i[k+1]=i[k]+Ts/L*(v[k-1]-R*i[k])，PI 与固件同结构"""
    i = 0.0
    integ = 0.0
    v_prev = 0.0
    xs = []
    for k in range(n):
        e = iref - i
        integ += e * kits
        v = kp * e + integ
        # 圆限幅（这里不会碰到，保留结构）
        if v > 16.5:
            v = 16.5
        if v < -16.5:
            v = -16.5
        i = i + (TS / L) * (v_prev - R * i)
        v_prev = v
        xs.append(i)
    t = [(k + 1) * TS for k in range(n)]
    # tr: 10%→90%
    t10 = t90 = None
    for k, x in enumerate(xs):
        if t10 is None and x >= 0.1 * iref:
            t10 = t[k]
        if t90 is None and x >= 0.9 * iref:
            t90 = t[k]
            break
    tr = (t90 - t10) * 1e6 if (t10 is not None and t90 is not None) else float("nan")
    pk = max(xs)
    os = (pk - iref) / iref * 100.0
    # ts: 最后一次离开 ±10%
    ts = 0.0
    for k, x in enumerate(xs):
        if abs(x - iref) > 0.1 * iref:
            ts = t[k]
    return tr, os, ts * 1e6, pk


print("=" * 78)
print("1) 控制器自身参数（Ki = kits/Ts）")
print("-" * 78)
print("  %-12s %8s %10s %10s %10s %10s" %
      ("参数组", "Kp", "Ki", "Ki/Kp", "Kp/L(假定)", "psi"))
for p in (MODEL, HAND):
    ki = p["kits"] / TS
    print("  %-12s %8.4f %10.1f %10.0f %10.0f %10.4f"
          % (p["name"], p["kp"], ki, ki / p["kp"], p["kp"] / p["ld"], p["psi"]))
print()
print("  Ki/Kp 是 PI 零点(rad/s)，设计上应该等于对象极点 R/L：")
for pl in PLANTS:
    print("    对象 %-30s R/L = %6.0f rad/s" % (pl["name"], pl["R"] / pl["L"]))
print()

print("=" * 78)
print("2) 套到实测对象上：穿越频率与相位裕度")
print("-" * 78)
print("  %-16s %-30s %8s %8s %9s" %
      ("参数组", "对象", "fc(Hz)", "PM(°)", "10Hz环路增益"))
for p in (MODEL, HAND):
    ki = p["kits"] / TS
    for pl in PLANTS:
        r = margins(p["kp"], ki, pl["R"], pl["L"])
        if r:
            print("  %-16s %-30s %8.0f %8.1f %9.1f"
                  % (p["name"], pl["name"], r[0], r[1], r[2]))
    print()

print("=" * 78)
print("3) 离散闭环阶跃（2 A，与 CurStep 同结构）—— 实测 tr=200us os=5.3%")
print("-" * 78)
print("  %-16s %-30s %8s %8s %9s" %
      ("参数组", "对象", "tr(us)", "os(%)", "ts(us)"))
for p in (MODEL, HAND):
    for pl in PLANTS[:2]:
        tr, os, ts, pk = sim_step(p["kp"], p["kits"], pl["R"], pl["L"])
        print("  %-16s %-30s %8.0f %8.1f %9.0f"
              % (p["name"], pl["name"], tr, os, ts))
    print()
