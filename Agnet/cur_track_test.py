# -*- coding: utf-8 -*-
"""电流环稳态跟踪 + 过零线性度测试

给不同小 iq 给定，录 20kHz 实测 iq，算均值（跟踪误差）与 rms（噪声）。
小电流段最能暴露死区造成的过零非线性（Vdead≈0.18V / R=0.4Ω ≈ 0.45A 偏差）。

用法: python cur_track_test.py
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
    lk.send(MSG_SYS, bytes([3]), X_SYS)
    time.sleep(0.03)


def rec(lk, op, arg=0, div=0, timeout=1.2):
    lk.send(MSG_REC, bytes([op, arg, div]) + bytes(141), X_REC)
    t = time.time()
    while time.time() - t < timeout:
        for mid, pl in lk.poll(0.02):
            if mid == MSG_REC and len(pl) >= 16:
                return pl
    return None


def measure(lk, iq):
    clear_fault(lk)
    lk.send(MSG_CUR, pack_iq(iq), X_CUR)
    time.sleep(0.05)                       # 电流环 0.35ms 就位，50ms 够
    rec(lk, 2)
    rec(lk, 1, 0, 1)                       # 20kHz，51ms
    time.sleep(0.055)
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
    lk.send(MSG_CUR, pack_iq(0.0), X_CUR)
    if len(v) < 100:
        return None
    m = sum(v) / len(v)
    rms = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
    return m, rms


def main():
    lk = Link('COM27')
    lk.poll(0.3)
    lk.send(MSG_MODE, bytes([4]), X_MODE)
    time.sleep(0.05)
    set_tau(lk, 0.0)

    iqs = [0.05, 0.10, 0.15, 0.20, 0.30]
    print('%8s | %8s %8s %8s' % ('iq给定', 'iq实测', '误差', 'rms'))
    print('-' * 44)
    for iq in iqs:
        r = measure(lk, iq)
        if r is None:
            print('%8.3f | 失败' % iq)
            continue
        m, rms = r
        print('%8.3f | %8.3f %8.3f %8.3f' % (iq, m, m - iq, rms))
        time.sleep(0.1)
    lk.send(MSG_CUR, pack_iq(0.0), X_CUR)


if __name__ == '__main__':
    main()
