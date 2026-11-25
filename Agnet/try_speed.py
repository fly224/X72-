# -*- coding: utf-8 -*-
"""最小起转试验：只用 30019（44 B）给速度，看 do_speed 完整装配是否成功"""
import sys, time, struct
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, 'G:/FOC/X72/Agnet')
from spd_ripple import (Link, MSG_ST, MSG_LP, MSG_PS, MSG_MODE, MSG_CW,
                        X_PS, X_MODE, X_CW, f32, pack_speed)

lk = Link('COM27')
lk.poll(0.3)


def show(tag):
    got = lk.poll(0.35)
    for mid, pl in got[-3:]:
        if mid == MSG_ST:
            print('%s 30090 cw=0x%04X mode=%d flags=0x%02X iqref=%.3f rpm=%.1f fault=0x%04X'
                  % (tag, struct.unpack_from('<H', pl, 0)[0], pl[2], pl[3],
                     f32(pl, 8), f32(pl, 12),
                     struct.unpack_from('<H', pl, 36)[0] if len(pl) >= 38 else 0))
        elif mid == MSG_LP:
            print('%s 30091 spdmode=%d src=%d wref=%.1f wmeas=%.1f iqc=%.3f iterm=%.3f'
                  % (tag, pl[0], pl[1], f32(pl, 4), f32(pl, 8), f32(pl, 12), f32(pl, 16)))


show('初始')
rpm = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
lk.send(MSG_MODE, bytes([3]), X_MODE)          # 30056 mode=3 速度模式
time.sleep(0.05)
pl = pack_speed(rpm)
lk.send(MSG_PS, pl, X_PS)
for k in range(5):
    lk.poll(0.4, ka=lambda: lk.send(MSG_PS, pl, X_PS), ka_period=0.4)
    show('t=%.1fs' % (0.4 * (k + 1)))
