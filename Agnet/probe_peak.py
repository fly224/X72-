# -*- coding: utf-8 -*-
"""定位 1400/2000 rpm 尖峰：录 iq 看是否撞限幅 + 读速度环积分项

用法: python probe_peak.py <rpm> [--kp X]  （--kp 可选，降增益看频率是否移动）
"""
import sys, time, math, struct
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import Link, MSG_PS, MSG_MODE, X_PS, X_MODE, pack_speed, f32

MSG_REC, X_REC = 30094, 94
MSG_LP, X_LP = 30091, 91
MSG_DBG, X_DBG = 30082, 218
spd_ripple.KNOWN[MSG_REC] = X_REC
spd_ripple.KNOWN[MSG_LP] = X_LP
TWO_PI = 6.283185307179586
DIV = 10
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


def set_kp(lk, kp):
    p = bytearray(192)
    struct.pack_into('<f', p, 4 * 4, kp)
    struct.pack_into('<I', p, 64, 1 << 4)
    struct.pack_into('<I', p, 124, 1 << 5)
    struct.pack_into('<I', p, 128 + 5 * 4, 0)   # sched_en=0
    lk.send(MSG_DBG, bytes(p), X_DBG)
    time.sleep(0.05)


def read_lp(lk):
    for mid, pl in lk.poll(0.3)[-6:]:
        if mid == MSG_LP:
            return dict(wref=f32(pl, 4), wmeas=f32(pl, 8), iqc=f32(pl, 12),
                        iterm=f32(pl, 16), kp=f32(pl, 20), iqmax=f32(pl, 28))
    return None


def main():
    rpm = float(sys.argv[1]) if len(sys.argv) > 1 else 1400.0
    kp_arg = None
    if '--kp' in sys.argv:
        kp_arg = float(sys.argv[sys.argv.index('--kp') + 1])

    lk = Link('COM27')
    lk.poll(0.3)
    lk.send(MSG_MODE, bytes([3]), X_MODE)
    time.sleep(0.05)
    if kp_arg is not None:
        set_kp(lk, kp_arg)
        print('已设 spd_kp=%.3f（关调度）' % kp_arg)
    KEEP['rpm'] = rpm
    lk.send(MSG_PS, pack_speed(rpm, 500.0), X_PS)
    lk.poll(4.0, ka=lambda: lk.send(MSG_PS, pack_speed(rpm), X_PS), ka_period=0.4)

    lp = read_lp(lk)
    if lp:
        print('30091: wref=%.1f wmeas=%.1f iqc=%.3f iterm=%.3f kp=%.3f iqmax=%.2f'
              % (lp['wref'], lp['wmeas'], lp['iqc'], lp['iterm'], lp['kp'], lp['iqmax']))

    rec(lk, 2)
    rec(lk, 1, 0, DIV)
    for _ in range(int(DIV * 1024 / 20000 / 0.4) + 1):
        lk.send(MSG_PS, pack_speed(rpm), X_PS)
        lk.poll(0.4)
    time.sleep(0.1)
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

    m = sum(v) / len(v)
    pp = max(v) - min(v)
    rms = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
    print('iq: n=%d mean=%.3f pp=%.3f rms=%.3f min=%.3f max=%.3f'
          % (len(v), m, pp, rms, min(v), max(v)))
    fs = 1.0 / DT
    f_rev = m and None
    for k in range(1, 7):
        fq = (rpm / 60.0) * k
        if fq >= fs / 2:
            break
        wr = TWO_PI * fq / fs
        re = im = 0.0
        for j in range(len(v)):
            wnd = 0.5 - 0.5 * math.cos(TWO_PI * j / (len(v) - 1))
            re += (v[j] - m) * wnd * math.cos(wr * j)
            im -= (v[j] - m) * wnd * math.sin(wr * j)
        print('  %d次/转 %.0fHz: %.3f A' % (k, fq, 2 * math.hypot(re, im) / len(v)))

    lk.send(MSG_PS, pack_speed(0), X_PS)
    time.sleep(0.3)


if __name__ == '__main__':
    main()
