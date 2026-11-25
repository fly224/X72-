# -*- coding: utf-8 -*-
"""全转速范围转速纹波扫描（2 kHz 录波，避开 100 Hz 遥测混叠）

每档：给转速 → 稳定 → 录波 θ → 差分算速度 → 输出 mean/pp/rms + 1..6 次/转幅值。
用法: python sweep_rec.py 200 400 ... 2000
"""
import sys, time, math, struct, csv
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import Link, MSG_PS, MSG_MODE, X_PS, X_MODE, pack_speed

MSG_REC, X_REC = 30094, 94
spd_ripple.KNOWN[MSG_REC] = X_REC
TWO_PI = 6.283185307179586

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


def measure(lk, rpm, div):
    KEEP['rpm'] = rpm
    lk.send(MSG_PS, pack_speed(rpm, 500.0), X_PS)
    # 稳定：按 500 rpm/s 爬坡 + 1.2 s 裕量
    settle = abs(rpm - (M_STATE['prev'] or 0)) / 500.0 + 1.2
    M_STATE['prev'] = rpm
    lk.poll(settle, ka=lambda: lk.send(MSG_PS, pack_speed(rpm), X_PS), ka_period=0.4)
    rec(lk, 2)
    rec(lk, 1, 3, div)
    need = div * 1024 / 20000.0
    for _ in range(int(need / 0.4) + 1):
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
    return th


def analyze(th, div):
    DT = div / 20000.0
    n = len(th)
    if n < 64:
        return None
    w = []
    for k in range(1, n):
        d = th[k] - th[k - 1]
        while d > math.pi:
            d -= TWO_PI
        while d < -math.pi:
            d += TWO_PI
        w.append(d / DT * 60.0 / TWO_PI)
    m = sum(w) / len(w)
    pp = max(w) - min(w)
    rms = math.sqrt(sum((x - m) ** 2 for x in w) / len(w))
    f_rev = m / 60.0
    fs = 1.0 / DT
    amps = []
    for k in range(1, 7):
        fq = f_rev * k
        if fq >= fs / 2:
            amps.append(float('nan'))
            continue
        wr = TWO_PI * fq / fs
        re = im = 0.0
        for j in range(len(w)):
            wnd = 0.5 - 0.5 * math.cos(TWO_PI * j / (len(w) - 1))
            re += (w[j] - m) * wnd * math.cos(wr * j)
            im -= (w[j] - m) * wnd * math.sin(wr * j)
        amps.append(2.0 * math.hypot(re, im) / len(w))
    return dict(mean=m, pp=pp, rms=rms, a1=amps[0], a2=amps[1], a3=amps[2],
                a4=amps[3], a5=amps[4], a6=amps[5])


M_STATE = {'prev': None}


def main():
    targets = [float(x) for x in sys.argv[1:]] or [200, 400, 600, 800, 1000, 1200, 1400, 1560, 1800, 2000]
    lk = Link('COM27')
    lk.poll(0.3)
    lk.send(MSG_MODE, bytes([3]), X_MODE)
    time.sleep(0.05)
    lk.send(MSG_PS, pack_speed(200, 500.0), X_PS)   # 先起转，脱离死区
    lk.poll(1.0)

    rows = []
    print('%8s | %9s %7s %6s | %6s %6s %6s %6s | %6s' %
          ('目标rpm', '实测', 'pp', 'rms', '1/转', '2/转', '3/转', '4/转', '窗口ms'))
    print('-' * 86)
    for rpm in targets:
        div = 20 if rpm <= 400 else 10
        th = measure(lk, rpm, div)
        r = analyze(th, div)
        if r is None:
            print('%8.0f | 样本不足' % rpm)
            continue
        win = len(th) * div / 20.0
        print('%8.0f | %9.1f %7.2f %6.2f | %6.2f %6.2f %6.2f %6.2f | %6.0f'
              % (rpm, r['mean'], r['pp'], r['rms'], r['a1'], r['a2'], r['a3'], r['a4'], win))
        rows.append([rpm, r['mean'], r['pp'], r['rms'], r['a1'], r['a2'], r['a3'], r['a4'], r['a5'], r['a6']])

    with open('sweep_rec_%s.csv' % time.strftime('%H%M%S'), 'w', newline='', encoding='utf-8-sig') as fh:
        wr = csv.writer(fh)
        wr.writerow(['target_rpm', 'mean_rpm', 'pp', 'rms', 'a1', 'a2', 'a3', 'a4', 'a5', 'a6'])
        wr.writerows(rows)

    lk.send(MSG_PS, pack_speed(0), X_PS)
    time.sleep(0.3)
    for _ in range(3):
        lk.send(MSG_PS, pack_speed(0), X_PS)
        time.sleep(0.1)
    print('\n已给 0 rpm')


if __name__ == '__main__':
    main()
