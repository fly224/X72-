# -*- coding: utf-8 -*-
"""把拟合好的编码器谐波系数写入 30098 并回读确认

用法: python write_enc_comp.py [--off]
"""
import sys, time, struct, re
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import Link

MSG_EC, X_EC = 30098, 98
spd_ripple.KNOWN[MSG_EC] = X_EC
NORD = 16


def load_coef():
    txt = open('enc_comp_coef.txt', encoding='utf-8').read()
    a = [float(x) for x in re.search(r'a_e = \[(.*?)\]', txt, re.S).group(1).split(',')]
    b = [float(x) for x in re.search(r'b_e = \[(.*?)\]', txt, re.S).group(1).split(',')]
    return a, b


def send_pack(lk, n, en, a, b):
    p = bytearray(136)
    p[0] = 1
    p[2] = n
    p[3] = en
    for k in range(n):
        struct.pack_into('<f', p, 8 + k * 8, a[k])
        struct.pack_into('<f', p, 12 + k * 8, b[k])
    lk.send(MSG_EC, bytes(p), X_EC)
    for mid, pl in lk.poll(0.4):
        if mid == MSG_EC:
            return pl
    return None


def show(pl, tag):
    if pl is None:
        print('%s 无应答' % tag)
        return
    n, en = pl[2], pl[3]
    a = [struct.unpack_from('<f', pl, 8 + k * 8)[0] for k in range(min(n, 4))]
    print('%s status=%d n=%d en=%d a[0..3]=%s' %
          (tag, pl[1], n, en, ['%.4f' % x for x in a]))


lk = Link('COM27')
lk.poll(0.3)

lk.send(MSG_EC, bytes(136), X_EC)          # cmd=0 读回当前
cur = None
for mid, pl in lk.poll(0.3):
    if mid == MSG_EC:
        cur = pl
show(cur, '当前')

a, b = load_coef()
print('写入 %d 阶，a4=%.4f b4=%.4f (电角度 rad)' % (NORD, a[3], b[3]))
show(send_pack(lk, NORD, 1, a, b), '写入后')

if '--off' in sys.argv:
    show(send_pack(lk, NORD, 0, a, b), '已关闭')
