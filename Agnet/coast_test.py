# -*- coding: utf-8 -*-
"""滑行测试：切断电磁力矩后看角度里是否仍有 4 次/转 波动

判据：
  · 滑行时仍有 ~1.15 mrad 的 4 次/转  → 编码器读数误差（可补偿）
  · 滑行时显著变小                    → 恒速时的波动是真实力矩脉动驱动

做法：先把 iq 限幅压到 0.05 A（30082 float[6]），再给 0 rpm → 电机基本自由滑行，
MOE 仍开（否则录波拒绝启动），512 ms 内从 1560 rpm 滑到 ~1100 rpm。
"""
import sys, time, math, struct
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import Link, MSG_PS, MSG_MODE, X_PS, X_MODE, pack_speed

MSG_REC, X_REC = 30094, 94
MSG_DBG, X_DBG = 30082, 218
spd_ripple.KNOWN[MSG_REC] = X_REC
TWO_PI = 6.283185307179586
DIV = 10
DT = DIV / 20000.0
RPM = 1560.0


def set_iqmax(lk, a):
    p = bytearray(192)
    struct.pack_into('<f', p, 6 * 4, a)
    struct.pack_into('<I', p, 64, 1 << 6)
    lk.send(MSG_DBG, bytes(p), X_DBG)
    time.sleep(0.05)


def rec(lk, op, arg=0, div=0, timeout=1.2):
    lk.send(MSG_REC, bytes([op, arg, div]) + bytes(141), X_REC)
    t = time.time()
    tn = t
    while time.time() - t < timeout:
        if time.time() - tn > 0.5:
            lk.send(MSG_PS, pack_speed(KEEP['rpm']), X_PS)
            tn = time.time()
        for mid, pl in lk.poll(0.02):
            if mid == MSG_REC and len(pl) >= 16:
                return pl
    return None


KEEP = {'rpm': RPM}
lk = Link('COM27')
lk.poll(0.3)

# 起转到 1560
lk.send(MSG_MODE, bytes([3]), X_MODE)
time.sleep(0.05)
lk.send(MSG_PS, pack_speed(RPM, 500.0), X_PS)
lk.poll(5.0, ka=lambda: lk.send(MSG_PS, pack_speed(RPM), X_PS), ka_period=0.4)

# 压 iq 限幅 → 给 0 → 立刻录波
set_iqmax(lk, 0.05)
KEEP['rpm'] = 0.0
lk.send(MSG_PS, pack_speed(0.0), X_PS)
rec(lk, 2)
rec(lk, 1, 3, DIV)
for _ in range(2):
    lk.send(MSG_PS, pack_speed(0.0), X_PS)
    lk.poll(0.4)
time.sleep(0.1)

th = []
for blk in range(64):
    r = rec(lk, 3, blk)
    if r is None:
        break
    nt = r[5]
    np_ = r[6] | (r[7] << 8)
    if nt == 0:
        break
    for k in range(nt):
        th.append(struct.unpack_from('<f', r, 80 + k * 4)[0])
    if len(th) >= np_:
        break
    time.sleep(0.003)

n = len(th)
print('滑行录波 %d 点，窗口 %.0f ms' % (n, n * DT * 1000))
u = [th[0]]
for k in range(1, n):
    d = th[k] - th[k - 1]
    while d > math.pi:
        d -= TWO_PI
    while d < -math.pi:
        d += TWO_PI
    u.append(u[-1] + d)
# 首末速度（用前/后 100 点）
def spd(a, b):
    return (u[b] - u[a]) / ((b - a) * DT) * 60 / TWO_PI
print('起始 %.0f rpm，结束 %.0f rpm' % (spd(0, 100), spd(n - 100, n - 1)))

# 二次去趋势（滑行是匀减速）
ts = [k * DT for k in range(n)]
s1 = s2 = s3 = s4 = 0.0
# 正规方程 3x3（最小二乘 a t^2 + b t + c）
import itertools
A = [[0.0] * 3 for _ in range(3)]
B = [0.0] * 3
for k in range(n):
    f = [ts[k] ** 2, ts[k], 1.0]
    for i in range(3):
        B[i] += f[i] * u[k]
        for j in range(3):
            A[i][j] += f[i] * f[j]
# 高斯消元
for c in range(3):
    pv = max(range(c, 3), key=lambda r: abs(A[r][c]))
    A[c], A[pv] = A[pv], A[c]
    B[c], B[pv] = B[pv], B[c]
    d0 = A[c][c]
    for j in range(c, 3):
        A[c][j] /= d0
    B[c] /= d0
    for r in range(3):
        if r == c:
            continue
        fac = A[r][c]
        for j in range(c, 3):
            A[r][j] -= fac * A[c][j]
        B[r] -= fac * B[c]
coef = B
res = [u[k] - (coef[0] * ts[k] ** 2 + coef[1] * ts[k] + coef[2]) for k in range(n)]
mr = sum(res) / n
res = [x - mr for x in res]

# 分箱（按 θ mod 2π）后拟合 1..6 阶
NB = 72
bins = [[0.0, 0] for _ in range(NB)]
for k in range(n):
    idx = int((th[k] % TWO_PI) / TWO_PI * NB) % NB
    bins[idx][0] += res[k]
    bins[idx][1] += 1
pts = [(i * TWO_PI / NB, bins[i][0] / bins[i][1]) for i in range(NB) if bins[i][1] > 0]
print('角度残差（二次去趋势）: pp=%.2f mrad rms=%.3f mrad，箱数 %d'
      % ((max(res) - min(res)) * 1000, math.sqrt(sum(x * x for x in res) / n) * 1000, len(pts)))
print('\n 次/转   幅值(mrad)')
for k in range(1, 7):
    re = im = 0.0
    for ang, y in pts:
        re += y * math.cos(k * ang)
        im += y * math.sin(k * ang)
    print('%6d  %10.3f' % (k, 2 * math.hypot(re, im) / len(pts) * 1000))

# 恢复限幅
set_iqmax(lk, 3.0)
lk.send(MSG_PS, pack_speed(0), X_PS)
print('\n已恢复 iqmax=3.0 A')
