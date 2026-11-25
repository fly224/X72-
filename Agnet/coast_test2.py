# -*- coding: utf-8 -*-
"""滑行标定：真正无力矩状态下分离纯编码器角度误差

关键：速度环输出限 0（iqmax=0）+ 关前馈 → 电机纯滑行，只靠摩擦减速。
短窗口 div=2（10kHz，102ms）减速仅 ~50rpm，线性去趋势足够。

判据：滑行时 4 次/转角度波动仍在 ~1 mrad → 纯编码器读数误差（可补偿）；
      消失 → 恒速时的波动是真实力矩/机械。

用法: python coast_test2.py
"""
import sys, time, math, struct
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import (Link, MSG_PS, MSG_MODE, X_PS, X_MODE, pack_speed,
                        pack_traj_t)

MSG_REC, X_REC = 30094, 94
MSG_DBG, X_DBG = 30082, 218
MSG_TRAJ, X_TRAJ = 30097, 97
MSG_SYS, X_SYS = 30069, 169
spd_ripple.KNOWN[MSG_REC] = X_REC
TWO_PI = 6.283185307179586
DIV = 2
DT = DIV / 20000.0
KEEP = {'rpm': 0.0}


def rec(lk, op, arg=0, div=0, timeout=1.5):
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


def set_dbg(lk, iqmax, ff_en):
    p = bytearray(192)
    struct.pack_into('<f', p, 6 * 4, iqmax)          # float[6]=spd_iqmax
    struct.pack_into('<I', p, 64, 1 << 6)            # float 掩码 bit6
    struct.pack_into('<I', p, 124, 1 << 2)           # int 掩码 bit2
    struct.pack_into('<I', p, 128 + 2 * 4, ff_en)    # int[2]=ff_en
    lk.send(MSG_DBG, bytes(p), X_DBG)
    time.sleep(0.05)


def main():
    lk = Link('COM27')
    lk.poll(0.3)
    lk.send(MSG_SYS, bytes([3]), X_SYS)
    time.sleep(0.05)
    lk.send(MSG_MODE, bytes([3]), X_MODE)
    time.sleep(0.05)
    lk.send(MSG_TRAJ, pack_traj_t(0.0), X_TRAJ)  # 关时间优先
    time.sleep(0.05)

    # 起转到 1560
    KEEP['rpm'] = 1560.0
    lk.send(MSG_PS, pack_speed(1560.0, 500.0), X_PS)
    lk.poll(5.0, ka=lambda: lk.send(MSG_PS, pack_speed(1560.0), X_PS), ka_period=0.4)

    # 关前馈 + iq 限 0 → 给 0 → 立即录波
    set_dbg(lk, 0.0, 0)
    KEEP['rpm'] = 0.0
    lk.send(MSG_PS, pack_speed(0.0), X_PS)
    rec(lk, 2)
    rec(lk, 1, 3, DIV)
    lk.send(MSG_PS, pack_speed(0.0), X_PS)
    time.sleep(0.105)                     # 102ms 录满 + 余量
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

    # 恢复
    set_dbg(lk, 3.0, 1)
    lk.send(MSG_PS, pack_speed(0.0), X_PS)

    n = len(th)
    if n < 100:
        print('录波失败 n=%d' % n)
        return
    # 展开
    u = [th[0]]
    for k in range(1, n):
        d = th[k] - th[k - 1]
        while d > math.pi:
            d -= TWO_PI
        while d < -math.pi:
            d += TWO_PI
        u.append(u[-1] + d)
    def spd(a, b):
        return (u[b] - u[a]) / ((b - a) * DT) * 60 / TWO_PI
    print('滑行 %.0f ms：起始 %.0f rpm → 结束 %.0f rpm'
          % (n * DT * 1000, spd(0, 50), spd(n - 50, n - 1)))

    # 二次去趋势（匀减速）
    ts = [k * DT for k in range(n)]
    A = [[0.0] * 3 for _ in range(3)]
    B = [0.0] * 3
    for k in range(n):
        f = [ts[k] ** 2, ts[k], 1.0]
        for i in range(3):
            B[i] += f[i] * u[k]
            for j in range(3):
                A[i][j] += f[i] * f[j]
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
    res = [u[k] - (B[0] * ts[k] ** 2 + B[1] * ts[k] + B[2]) for k in range(n)]
    mr = sum(res) / n
    res = [x - mr for x in res]

    # 分箱（按 θ mod 2π）拟合 1..6 阶
    NB = 72
    bins = [[0.0, 0] for _ in range(NB)]
    for k in range(n):
        idx = int((th[k] % TWO_PI) / TWO_PI * NB) % NB
        bins[idx][0] += res[k]
        bins[idx][1] += 1
    pts = [(i * TWO_PI / NB, bins[i][0] / bins[i][1]) for i in range(NB) if bins[i][1] > 0]
    print('角度残差（去趋势后）: pp=%.3f mrad rms=%.3f mrad，箱 %d'
          % ((max(res) - min(res)) * 1000, math.sqrt(sum(x * x for x in res) / n) * 1000, len(pts)))
    print(' 次/转   幅值(mrad)')
    for k in range(1, 7):
        re = im = 0.0
        for ang, y in pts:
            re += y * math.cos(k * ang)
            im += y * math.sin(k * ang)
        print('%6d  %10.3f' % (k, 2 * math.hypot(re, im) / len(pts) * 1000))


if __name__ == '__main__':
    main()
