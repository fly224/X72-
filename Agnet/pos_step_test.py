# -*- coding: utf-8 -*-
"""位置环阶跃响应：给位置阶跃，录 θ 看定位时间/超调/整定/保持精度

用法: python pos_step_test.py [阶跃幅度 rad]
"""
import sys, time, math, struct
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import Link, f32

MSG_MODE, X_MODE = 30056, 26
MSG_PS, X_PS = 30019, 8
MSG_CW, X_CW = 30054, 108
MSG_SYS, X_SYS = 30069, 169
MSG_REC, X_REC = 30094, 94
MSG_PD, X_PD = 30100, 100
spd_ripple.KNOWN[MSG_REC] = X_REC
spd_ripple.KNOWN[MSG_PD] = X_PD
TWO_PI = 6.283185307179586
DIV = 10
DT = DIV / 20000.0
ENC = 2097152.0
RAD2CNT = ENC / TWO_PI
KEEP = {'cnt': 0}


def pack_pos(rad, acc_rad_s2=0.0):
    p = bytearray(44)
    struct.pack_into('<q', p, 0, int(rad * RAD2CNT))
    if acc_rad_s2:
        struct.pack_into('<f', p, 32, acc_rad_s2 * (ENC / TWO_PI))
    return bytes(p)


def rec(lk, op, arg=0, div=0, timeout=1.5):
    lk.send(MSG_REC, bytes([op, arg, div]) + bytes(141), X_REC)
    t = time.time()
    tn = t
    while time.time() - t < timeout:
        if time.time() - tn > 0.5:
            lk.send(MSG_PS, pack_pos(KEEP['cnt'] / RAD2CNT), X_PS)
            tn = time.time()
        for mid, pl in lk.poll(0.02):
            if mid == MSG_REC and len(pl) >= 16:
                return pl
    return None


def main():
    step = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5   # rad
    acc = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0    # rad/s²，0=不改

    lk = Link('COM27')
    lk.poll(0.3)
    lk.send(MSG_SYS, bytes([3]), X_SYS)
    time.sleep(0.05)
    lk.send(MSG_MODE, bytes([1]), X_MODE)      # 位置模式
    time.sleep(0.05)
    lk.send(MSG_CW, struct.pack('<HB', 0x000F, 0), X_CW)   # 使能
    time.sleep(0.3)

    # 读当前位置（位置环多圈坐标 p_now，不是 30090 的单圈 theta）
    cur_pos = None
    for mid, pl in lk.poll(0.3)[-4:]:
        if mid == MSG_PD and len(pl) >= 20:
            cur_pos = f32(pl, 0)
    if cur_pos is None:
        print('读位置失败')
        return
    target = cur_pos + step
    KEEP['cnt'] = target * RAD2CNT

    rec(lk, 2)
    rec(lk, 1, 3, DIV)                        # 录 θ
    lk.send(MSG_PS, pack_pos(target, acc), X_PS)   # 位置阶跃
    for _ in range(int(DIV * 1024 / 20000 / 0.4) + 1):
        lk.send(MSG_PS, pack_pos(target, acc), X_PS)
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
    if n < 100:
        print('录波失败 n=%d' % n)
        return
    # 展开单圈角，转相对变化（避免单圈角与多圈坐标的基准差）
    u = [th[0]]
    for k in range(1, n):
        d = th[k] - th[k - 1]
        while d > math.pi:
            d -= TWO_PI
        while d < -math.pi:
            d += TWO_PI
        u.append(u[-1] + d)
    base0 = u[0]
    err = [(u[k] - base0) - step for k in range(n)]   # 相对阶跃误差

    # 找阶跃起点
    t0 = None
    for k in range(5, n):
        if abs(err[k]) < abs(err[0]) - 0.05:
            t0 = k
            break
    if t0 is None:
        t0 = 0
    settle = None
    for k in range(t0, n - 10):
        if all(abs(err[k + j]) < 0.010 for j in range(10)):
            settle = k
            break
    tail = err[-200:]
    hold_pp = (max(tail) - min(tail)) * 1000
    hold_rms = math.sqrt(sum(x * x for x in tail) / len(tail)) * 1000
    # 正过冲 = 位置冲过目标再回来的峰值（err 为正的最大值）
    overshoot = max(err[t0:]) * 1000 if max(err[t0:]) > 0 else 0

    print('位置阶跃 %.2f rad（%.1f°），当前 %.3f → 目标 %.3f rad'
          % (step, step * 57.3, cur_pos, target))
    print('到位时间 = %.0f ms，最大过冲 = %.2f mrad'
          % (settle * DT * 1000 if settle else -1, overshoot))
    print('保持精度：峰值 %.2f mrad，rms %.2f mrad' % (hold_pp, hold_rms))

    lk.send(MSG_CW, struct.pack('<HB', 0x0000, 1), X_CW)   # halt 失能
    time.sleep(0.2)


if __name__ == '__main__':
    main()
