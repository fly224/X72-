# -*- coding: utf-8 -*-
"""电流环阶跃响应：iq 小幅度双脉冲（0→amp→0），20kHz 录波测上升时间

力矩模式开环会让电机加速，超 95rpm 就触发超速保护 0x0020。所以用
小幅度（0.3A）+ 短保持（10ms）+ 回零，电机几乎不动；多次对齐平均消纹波。

用法: python cur_step_test.py [amp] [次数]
"""
import sys, time, math, struct
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import Link

MSG_MODE, X_MODE = 30056, 26
MSG_CUR, X_CUR = 30025, 191
MSG_TORQUE, X_TORQUE = 30095, 95
MSG_SYS, X_SYS = 30069, 169
MSG_REC, X_REC = 30094, 94
spd_ripple.KNOWN[MSG_REC] = X_REC
spd_ripple.KNOWN[MSG_TORQUE] = X_TORQUE

DIV = 1
DT = DIV / 20000.0
HOLD = 0.010          # 阶跃保持 10ms


def pack_iq(iq):
    p = bytearray(53)
    struct.pack_into('<f', p, 4, iq)
    return bytes(p)


def set_tau(lk, ms):
    p = bytearray(12)
    p[0] = 0x02
    struct.pack_into('<f', p, 8, ms)
    lk.send(MSG_TORQUE, bytes(p), X_TORQUE)


def clear_fault(lk):
    lk.send(MSG_SYS, bytes([3]), X_SYS)     # action=3 清故障
    time.sleep(0.05)


def rec(lk, op, arg=0, div=0, timeout=1.2):
    lk.send(MSG_REC, bytes([op, arg, div]) + bytes(141), X_REC)
    t = time.time()
    while time.time() - t < timeout:
        for mid, pl in lk.poll(0.02):
            if mid == MSG_REC and len(pl) >= 16:
                return pl
    return None


def capture(lk, amp):
    lk.send(MSG_CUR, pack_iq(0.0), X_CUR)
    lk.poll(0.2)
    rec(lk, 2)
    rec(lk, 1, 0, DIV)
    time.sleep(0.003)
    lk.send(MSG_CUR, pack_iq(amp), X_CUR)   # 上升沿
    time.sleep(HOLD)
    lk.send(MSG_CUR, pack_iq(0.0), X_CUR)   # 回零
    time.sleep(0.04)
    v = []
    for blk in range(64):
        r = rec(lk, 3, blk)
        if r is None:
            break
        nt = r[5]
        np_ = r[6] | (r[7] << 8)
        if nt == 0:
            break
        for k in range(nt):
            v.append(struct.unpack_from('<f', r, 16 + k * 4)[0])
        if len(v) >= np_:
            break
        time.sleep(0.003)
    return v


def analyze(v, amp):
    n = len(v)
    if n < 100:
        return None
    w = []
    for k in range(n):
        a, b = max(0, k - 2), min(n, k + 3)
        w.append(sum(v[a:b]) / (b - a))
    base = sum(w[:30]) / 30
    thr = base + 0.3 * amp
    t0 = None
    for k in range(30, n - 5):
        if all(w[k + j] > thr for j in range(5)):
            t0 = k
            break
    if t0 is None:
        return None
    peak = max(w[t0:])
    lo = base + 0.1 * (amp - base)
    hi = base + 0.9 * (amp - base)
    t10 = t90 = None
    for k in range(t0, min(t0 + int(0.02 / DT), n)):
        if t10 is None and w[k] >= lo:
            t10 = k
        if t10 is not None and w[k] >= hi:
            t90 = k
            break
    tr = (t90 - t10) * DT * 1000 if (t10 and t90) else None
    return dict(base=base, peak=peak, tr_ms=tr)


def main():
    amp = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
    times = int(sys.argv[2]) if len(sys.argv) > 2 else 20

    lk = Link('COM27')
    lk.poll(0.3)
    lk.send(MSG_MODE, bytes([4]), X_MODE)
    time.sleep(0.05)
    set_tau(lk, 0.0)

    print('电流环阶跃：0 → %.2f A（%d 次对齐平均）' % (amp, times))
    trs = []
    peaks = []
    for i in range(times):
        clear_fault(lk)
        v = capture(lk, amp)
        r = analyze(v, amp)
        if r is not None and r['tr_ms'] is not None:
            trs.append(r['tr_ms'])
            peaks.append(r['peak'])
        time.sleep(0.1)
    if trs:
        trs.sort()
        n = len(trs)
        print('有效 %d/%d 次，上升时间(10-90%%) 中位 %.3f ms，峰值中位 %.3f A'
              % (n, times, trs[n // 2], sorted(peaks)[n // 2]))
        print('上升时间范围 %.3f ~ %.3f ms' % (trs[0], trs[-1]))
    else:
        print('全部未捕获到阶跃')
    lk.send(MSG_CUR, pack_iq(0.0), X_CUR)


if __name__ == '__main__':
    main()
