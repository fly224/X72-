# -*- coding: utf-8 -*-
"""陷波 A/B：验证 4 次/转（104 Hz @1560rpm）速度脉动能否被速度环陷波压掉

流程（一次使能内完成，避免重爬坡）：
  1) 1560 rpm 到速 → 录波基线（div=10，2 kHz，512 ms）
  2) 30096 设陷波 f0=104 Hz / 20 dB / Q=10 → 复测
  3) 30096 关闭 → 可选第三次

判据：pp/rms 明显下降 → 脉动确实经"速度反馈→速度环"通路放大；
      不变 → 脉动在反馈路径之外（力矩/机械），需查角度补偿或减速器
"""
import sys, time, math, struct
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import Link, MSG_PS, MSG_MODE, X_PS, X_MODE, pack_speed

MSG_REC, X_REC = 30094, 94
MSG_ADV, X_ADV = 30096, 96
spd_ripple.KNOWN[MSG_REC] = X_REC
spd_ripple.KNOWN[MSG_ADV] = X_ADV
TWO_PI = 6.283185307179586
DIV = 10
DT = DIV / 20000.0


KEEP = {'rpm': 0.0}


def rec(lk, op, arg=0, div=0, timeout=1.2):
    lk.send(MSG_REC, bytes([op, arg, div]) + bytes(141), X_REC)
    t = time.time()
    tn = t
    while time.time() - t < timeout:
        # 长超时期间也要保活 30019，否则 2 s deadman 会把电机停掉
        if time.time() - tn > 0.5:
            lk.send(MSG_PS, pack_speed(KEEP['rpm']), X_PS)
            tn = time.time()
        for mid, pl in lk.poll(0.02):
            if mid == MSG_REC and len(pl) >= 16:
                return pl
    return None


def set_notch(lk, en, f0, dep, q):
    p = bytearray(24)
    p[0] = 1 if en else 0
    struct.pack_into('<f', p, 4, f0)
    struct.pack_into('<f', p, 8, dep)
    struct.pack_into('<f', p, 12, q)
    lk.send(MSG_ADV, bytes(p), X_ADV)
    for mid, pl in lk.poll(0.15):
        if mid == MSG_ADV:
            return (pl[0], struct.unpack_from('<f', pl, 4)[0],
                    struct.unpack_from('<f', pl, 8)[0], struct.unpack_from('<f', pl, 12)[0])
    return None


def capture(lk, rpm, tag):
    KEEP['rpm'] = rpm
    lk.send(MSG_PS, pack_speed(rpm), X_PS)
    rec(lk, 2)
    r = rec(lk, 1, 3, DIV)
    need = DIV * 1024 / 20000.0
    for _ in range(int(need / 0.4) + 1):
        lk.send(MSG_PS, pack_speed(rpm), X_PS)
        lk.poll(0.4)
    time.sleep(0.1)
    th = []
    for blk in range(64):
        r = rec(lk, 3, blk)
        if r is None:
            break
        n_this = r[5]
        n_pts = r[6] | (r[7] << 8)
        if n_this == 0:
            break
        for k in range(n_this):
            th.append(struct.unpack_from('<f', r, 80 + k * 4)[0])
        if len(th) >= n_pts:
            break
        time.sleep(0.003)

    w = []
    for k in range(1, len(th)):
        d = th[k] - th[k - 1]
        while d > math.pi:
            d -= TWO_PI
        while d < -math.pi:
            d += TWO_PI
        w.append(d / DT * 60.0 / TWO_PI)
    m = sum(w) / len(w)
    pp = max(w) - min(w)
    rms = math.sqrt(sum((x - m) ** 2 for x in w) / len(w))
    print('  %-14s rpm=%7.1f  pp=%6.2f  rms=%5.2f  (n=%d)' % (tag, m, pp, rms, len(w)))
    return pp, rms


lk = Link('COM27')
lk.poll(0.3)

# 读当前陷波配置
lk.send(MSG_ADV, bytes([0xFF]) + bytes(23), X_ADV)
cur = None
for mid, pl in lk.poll(0.3):
    if mid == MSG_ADV:
        cur = (pl[0], struct.unpack_from('<f', pl, 4)[0],
               struct.unpack_from('<f', pl, 8)[0], struct.unpack_from('<f', pl, 12)[0])
print('当前陷波 en=%s f0=%.1fHz depth=%.1fdB q=%.2f' % cur if cur else '当前陷波 读回失败')

RPM = 1560.0
lk.send(MSG_MODE, bytes([3]), X_MODE)
time.sleep(0.05)
lk.send(MSG_PS, pack_speed(RPM, 500.0), X_PS)
lk.poll(5.0, ka=lambda: lk.send(MSG_PS, pack_speed(RPM), X_PS), ka_period=0.4)

print('\n--- A/B ---')
b = capture(lk, RPM, '基线(陷波关)')
print('  设陷波 → %s' % (set_notch(lk, 1, 104.0, 20.0, 10.0),))
time.sleep(0.4)
n = capture(lk, RPM, '陷波104Hz')
print('  关陷波 → %s' % (set_notch(lk, 0, 104.0, 20.0, 10.0),))
time.sleep(0.4)
c = capture(lk, RPM, '恢复')

if b[0] > 0:
    print('\npp 变化: 基线 %.2f → 陷波 %.2f (%+.0f%%) → 恢复 %.2f'
          % (b[0], n[0], (n[0] / b[0] - 1) * 100, c[0]))
    print('rms 变化: 基线 %.2f → 陷波 %.2f (%+.0f%%) → 恢复 %.2f'
          % (b[1], n[1], (n[1] / b[1] - 1) * 100, c[1]))

lk.send(MSG_PS, pack_speed(0), X_PS)
time.sleep(0.3)
for _ in range(3):
    lk.send(MSG_PS, pack_speed(0), X_PS)
    time.sleep(0.1)
print('已给 0 rpm')
