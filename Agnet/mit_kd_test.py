# -*- coding: utf-8 -*-
"""MIT 静止振荡性质诊断：kp=30 固定，扫 kd，看 theta 振荡幅值。
欠阻尼振荡 → kd 增大显著压幅值；死区黏滑 → kd 影响小。
"""
import sys, time, struct, math
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import Link, f32

TWO_PI = 6.283185307179586
RAD2CNT = 2097152.0 / TWO_PI
PS2RADS = 2097152.0 / 60.0

MSG_MODE, X_MODE = 30056, 26
MSG_CW,   X_CW   = 30054, 108
MSG_SYS,  X_SYS  = 30069, 169
MSG_REC,  X_REC  = 30094, 94
MSG_MITC, X_MITC = 30066, 147
MSG_MIT,  X_MIT  = 30065, 17
MSG_ENC,  X_ENC  = 30040, 160
for m, x in [(MSG_REC, X_REC), (MSG_MITC, X_MITC), (MSG_MIT, X_MIT), (MSG_ENC, X_ENC)]:
    spd_ripple.KNOWN[m] = x

DIV = 4


def pack_cfg(kp, kd):
    p = bytearray(16)
    struct.pack_into('<f', p, 0, kp)
    struct.pack_into('<f', p, 4, kd)
    return bytes(p)


def pack_mit(pos_out):
    p = bytearray(32)
    struct.pack_into('<q', p, 0, int(pos_out * RAD2CNT))
    return bytes(p)


def rec(lk, op, arg=0, div=0, timeout=1.2):
    lk.send(MSG_REC, bytes([op, arg, div]) + bytes(141), X_REC)
    t = time.time()
    while time.time() - t < timeout:
        for mid, pl in lk.poll(0.02):
            if mid == MSG_REC and len(pl) >= 16:
                return pl
    return None


def read_out(lk):
    for mid, pl in lk.poll(0.3)[-4:]:
        if mid == MSG_ENC and len(pl) >= 8:
            return (struct.unpack_from('<q', pl, 0)[0] / 2097152.0) * TWO_PI / 9.0
    return None


def record(lk, kp, kd):
    cur = read_out(lk)
    if cur is None:
        return None
    lk.send(MSG_MITC, pack_cfg(0.0, 0.0), X_MITC)
    time.sleep(0.05)
    lk.send(MSG_MIT, pack_mit(cur), X_MIT)
    time.sleep(0.4)
    lk.send(MSG_MITC, pack_cfg(kp, kd), X_MITC)
    time.sleep(0.05)
    lk.send(MSG_MIT, pack_mit(cur), X_MIT)
    time.sleep(0.6)

    rec(lk, 2)
    r = rec(lk, 1, 3, DIV)
    if r is None:
        return None
    need = DIV * 1024 / 20000.0
    for _ in range(int(need / 0.4) + 1):
        lk.send(MSG_MIT, pack_mit(cur), X_MIT)
        lk.poll(0.4)
    rec(lk, 2)
    time.sleep(0.05)
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
            th.append(f32(r, 80 + k * 4))
        if len(th) >= np_:
            break
        time.sleep(0.004)
    return th


def main():
    kp = 30.0
    kds = [float(x) for x in sys.argv[1:]] or [0.02, 0.05, 0.1, 0.2]
    lk = Link('COM27')
    lk.poll(0.3)
    lk.send(MSG_SYS, bytes([3]), X_SYS)
    time.sleep(0.05)
    lk.send(MSG_MODE, bytes([12]), X_MODE)
    time.sleep(0.05)
    lk.send(MSG_CW, struct.pack('<HB', 0x000F, 0), X_CW)
    time.sleep(0.4)

    fs = 20000.0 / DIV
    for kd in kds:
        th = record(lk, kp, kd)
        if not th or len(th) < 100:
            print('kp=%.0f kd=%.2f: 录波失败' % (kp, kd))
            continue
        m = sum(th) / len(th)
        d = [x - m for x in th]
        pp = max(d) - min(d)
        rms = math.sqrt(sum(x * x for x in d) / len(d))
        # 频谱峰（20~400 Hz）
        best = (0, 0)
        f = 20.0
        while f < 400.0:
            wr = TWO_PI * f / fs
            re = im = 0.0
            n = len(d)
            for j in range(n):
                re += d[j] * math.cos(wr * j)
                im -= d[j] * math.sin(wr * j)
            amp = 2 * math.hypot(re, im) / n
            if amp > best[1]:
                best = (f, amp)
            f *= 1.01
        print('kp=%.0f kd=%.2f: theta pp=%.3f mrad rms=%.3f mrad 峰 %.0f Hz/%.3f mrad'
              % (kp, kd, pp * 1000, rms * 1000, best[0], best[1] * 1000))
        time.sleep(0.3)

    lk.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    print('done')


if __name__ == '__main__':
    main()
