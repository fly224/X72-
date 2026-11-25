# -*- coding: utf-8 -*-
"""速度环阶跃响应：给速度阶跃，录 θ 差分算转速，测上升时间/超调/稳态纹波

用法: python spd_step_test.py [目标rpm] [acc_rpm_s]
默认 0→300 rpm，acc=2000 rpm/s（接近阶跃，考验速度环动态）
"""
import sys, time, math, struct
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import (Link, MSG_PS, MSG_MODE, X_PS, X_MODE, pack_speed,
                        pack_traj_t, f32)

MSG_REC, X_REC = 30094, 94
MSG_LP, X_LP = 30091, 91
MSG_SYS, X_SYS = 30069, 169
MSG_TRAJ, X_TRAJ = 30097, 97
spd_ripple.KNOWN[MSG_REC] = X_REC
spd_ripple.KNOWN[MSG_LP] = X_LP
TWO_PI = 6.283185307179586
DIV = 20          # 1 kHz，1024 点 = 1024 ms
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


def main():
    rpm = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    acc = float(sys.argv[2]) if len(sys.argv) > 2 else 2000.0

    lk = Link('COM27')
    lk.poll(0.3)
    lk.send(MSG_SYS, bytes([3]), X_SYS)          # 清故障（力矩测试残留超速）
    time.sleep(0.05)
    lk.send(MSG_MODE, bytes([3]), X_MODE)
    time.sleep(0.05)
    lk.send(MSG_TRAJ, pack_traj_t(0.0), X_TRAJ)  # 关时间优先，用配置加速度
    time.sleep(0.05)

    KEEP['rpm'] = rpm
    lk.send(MSG_PS, pack_speed(0.0, acc), X_PS)
    lk.poll(0.3)

    rec(lk, 2)
    rec(lk, 1, 3, DIV)                    # 录 θ
    lk.send(MSG_PS, pack_speed(rpm, acc), X_PS)   # 阶跃（从 0 起步）
    for _ in range(int(DIV * 1024 / 20000 / 0.4) + 1):
        lk.send(MSG_PS, pack_speed(rpm), X_PS)
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
    w = []
    for k in range(1, n):
        d = th[k] - th[k - 1]
        while d > math.pi:
            d -= TWO_PI
        while d < -math.pi:
            d += TWO_PI
        w.append(d / DT * 60.0 / TWO_PI)

    # 找阶跃点（转速开始上升）
    t0 = None
    for k in range(5, len(w)):
        if w[k] > 30:
            t0 = k
            break
    if t0 is None:
        print('未找到阶跃')
        return
    ss = sum(w[-150:]) / 150                 # 稳态（后 150 点）
    peak = max(w[t0:])
    overshoot = (peak - ss) / ss * 100 if ss else 0
    # 上升时间 10-90%
    lo = 0.1 * ss
    hi = 0.9 * ss
    t10 = t90 = None
    for k in range(t0, len(w)):
        if t10 is None and w[k] >= lo:
            t10 = k
        if t10 is not None and w[k] >= hi:
            t90 = k
            break
    tr = (t90 - t10) * DT * 1000 if (t10 and t90) else None
    # 稳态纹波（后 300 点）
    tail = w[-300:]
    m = sum(tail) / len(tail)
    ripple_pp = max(tail) - min(tail)
    ripple_rms = math.sqrt(sum((x - m) ** 2 for x in tail) / len(tail))

    print('速度阶跃 0→%.0f rpm（acc=%.0f rpm/s）' % (rpm, acc))
    print('上升时间(10-90%%) = %.1f ms，超调 = %.1f%%，峰值 = %.1f rpm'
          % (tr, overshoot, peak))
    print('稳态均值 %.1f rpm，纹波 pp %.1f / rms %.2f rpm' % (m, ripple_pp, ripple_rms))

    lk.send(MSG_PS, pack_speed(0.0), X_PS)
    time.sleep(0.3)
    for _ in range(3):
        lk.send(MSG_PS, pack_speed(0.0), X_PS)
        time.sleep(0.1)


if __name__ == '__main__':
    main()
