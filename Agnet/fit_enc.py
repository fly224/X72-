# -*- coding: utf-8 -*-
"""从录波角度数据拟合 16 阶谐波误差系数（机械角域），并换算成 30098 的电角度域系数

输出：
  a_m/b_m : 机械角误差 eps(theta) = sum[a_m sin(kθ)+b_m cos(kθ)]（rad，机械角）
  a_e/b_e := -14 * a_m/b_m  -> 直接写 30098（电角度域补偿量）
  * 30098 的语义是"加到电角度上的修正量"，故取负号；/14 用于机械角速度补偿。
"""
import csv, math, glob, sys, os

TWO_PI = 6.283185307179586
NORD = 4
POLES = 14


def lstsq(rows, npar):
    """rows = [(fvec, y)]；正规方程 + 高斯消元"""
    M = [[0.0] * npar for _ in range(npar)]
    v = [0.0] * npar
    for f, y in rows:
        for i in range(npar):
            fi = f[i]
            v[i] += fi * y
            Mi = M[i]
            for j in range(i, npar):
                Mi[j] += fi * f[j]
    for i in range(npar):
        for j in range(i):
            M[i][j] = M[j][i]
    # 高斯消元（带部分主元）
    for c in range(npar):
        piv = max(range(c, npar), key=lambda r: abs(M[r][c]))
        if abs(M[piv][c]) < 1e-12:
            raise RuntimeError('奇异')
        if piv != c:
            M[c], M[piv] = M[piv], M[c]
            v[c], v[piv] = v[piv], v[c]
        pv = M[c][c]
        for j in range(c, npar):
            M[c][j] /= pv
        v[c] /= pv
        for r in range(npar):
            if r == c:
                continue
            fac = M[r][c]
            if fac == 0.0:
                continue
            for j in range(c, npar):
                M[r][j] -= fac * M[c][j]
            v[r] -= fac * v[c]
    return v


def load(f):
    th = []
    with open(f, encoding='utf-8-sig') as fh:
        for r in csv.DictReader(fh):
            th.append(float(r['theta_rad']))
    dt = 0.0005
    u = [th[0]]
    for k in range(1, len(th)):
        d = th[k] - th[k - 1]
        while d > math.pi:
            d -= TWO_PI
        while d < -math.pi:
            d += TWO_PI
        u.append(u[-1] + d)
    n = len(u)
    ts = [k * dt for k in range(n)]
    mt = sum(ts) / n
    mu = sum(u) / n
    a = sum((ts[k] - mt) * (u[k] - mu) for k in range(n)) / sum((ts[k] - mt) ** 2 for k in range(n))
    b = mu - a * mt
    res = [u[k] - (a * ts[k] + b) for k in range(n)]
    return th, res, a * 60 / TWO_PI


def fit(th, res):
    rows = []
    for k in range(len(th)):
        x = th[k]
        f = [math.sin((i + 1) * x) for i in range(NORD)] + [math.cos((i + 1) * x) for i in range(NORD)]
        rows.append((f, res[k]))
    c = lstsq(rows, 2 * NORD)
    a_m = c[:NORD]
    b_m = c[NORD:]
    # 残差
    err = []
    for k in range(len(th)):
        x = th[k]
        y = res[k]
        s = 0.0
        for i in range(NORD):
            s += a_m[i] * math.sin((i + 1) * x) + b_m[i] * math.cos((i + 1) * x)
        err.append(y - s)
    rms0 = math.sqrt(sum(x * x for x in res) / len(res))
    rms1 = math.sqrt(sum(x * x for x in err) / len(err))
    return a_m, b_m, rms0, rms1, err


def main():
    files = sys.argv[1:] or sorted(glob.glob('rec_theta_*.csv'), key=os.path.getmtime)[-2:]
    # 合并两次数据一起拟合，抑制单次噪声
    TH, RS = [], []
    for f in files:
        th, res, rpm = load(f)
        TH += th
        RS += res
        print('%s  rpm=%.1f  n=%d' % (f, rpm, len(th)))
    a_m, b_m, r0, r1, err = fit(TH, RS)
    print('\n拟合 %d 阶：残差 rms %.3f -> %.3f mrad' % (NORD, r0 * 1000, r1 * 1000))

    print('\n k    A_m(mrad)   phase(deg)   A_e(mrad)')
    for i in range(NORD):
        A = math.hypot(a_m[i], b_m[i])
        ph = math.degrees(math.atan2(b_m[i], a_m[i]))
        print('%2d  %10.3f  %10.1f  %10.3f' % (i + 1, A * 1000, ph, A * 1000 * POLES))

    a_e = [-POLES * x for x in a_m]
    b_e = [-POLES * x for x in b_m]
    print('\n--- 30098 电角度域系数（a_k, b_k，rad）---')
    print('a = [' + ', '.join('%.9ef' % x for x in a_e) + ']')
    print('b = [' + ', '.join('%.9ef' % x for x in b_e) + ']')

    # 验证：补偿后的速度脉动
    for f in files:
        th, res, rpm = load(f)
        # 用拟合的 eps 补偿，重算速度
        comp = []
        for k in range(len(th)):
            x = th[k]
            s = 0.0
            for i in range(NORD):
                s += a_m[i] * math.sin((i + 1) * x) + b_m[i] * math.cos((i + 1) * x)
            comp.append(s)
        dt = 0.0005
        def speed(seq):
            v = []
            for k in range(1, len(seq)):
                d = seq[k] - seq[k - 1]
                while d > math.pi:
                    d -= TWO_PI
                while d < -math.pi:
                    d += TWO_PI
                v.append(d / dt * 60 / TWO_PI)
            return v
        # 展开后的 theta 与补偿后的 theta
        u = [th[0]]
        for k in range(1, len(th)):
            d = th[k] - th[k - 1]
            while d > math.pi:
                d -= TWO_PI
            while d < -math.pi:
                d += TWO_PI
            u.append(u[-1] + d)
        uc = [u[k] - comp[k] for k in range(len(u))]
        v0 = speed(u)
        v1 = speed(uc)
        m0 = sum(v0) / len(v0); m1 = sum(v1) / len(v1)
        r0_ = math.sqrt(sum((x - m0) ** 2 for x in v0) / len(v0))
        r1_ = math.sqrt(sum((x - m1) ** 2 for x in v1) / len(v1))
        print('\n%s: 原始 pp=%.2f rms=%.2f rpm -> 补偿后 pp=%.2f rms=%.2f rpm'
              % (f, max(v0) - min(v0), r0_, max(v1) - min(v1), r1_))

    with open('enc_comp_coef.txt', 'w', encoding='utf-8') as fh:
        fh.write('a_e = [' + ', '.join('%.9e' % x for x in a_e) + ']\n')
        fh.write('b_e = [' + ', '.join('%.9e' % x for x in b_e) + ']\n')
    print('\n系数已存 enc_comp_coef.txt')


if __name__ == '__main__':
    main()
