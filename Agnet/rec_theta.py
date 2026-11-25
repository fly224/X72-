# -*- coding: utf-8 -*-
"""用 30094 录波读编码器角度，反算 20 kHz 级真实转速 -> 判断"转速波动"是真实还是测速噪声

通道 3：buf = vd 积分(mV)，th = 单圈机械角(2π→65536)
分频 div：等效采样 = 20 kHz / div，1024 点
  div=1 → 51 ms；div=4 → 205 ms；div=10 → 512 ms

输出：速度序列（rpm，差分）+ 峰峰/rms + 主频，并打印前 60 点看是否阶梯/抖动
"""
import sys, time, math, struct, argparse
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import Link, MSG_PS, MSG_MODE, X_PS, X_MODE, pack_speed

MSG_REC, X_REC = 30094, 94
spd_ripple.KNOWN[MSG_REC] = X_REC      # Link.poll 只解析 KNOWN 里的 msgid
TWO_PI = 6.283185307179586


def rec(lk, op, arg=0, div=0, timeout=0.6):
    # payload 必须补到 144 B：接收端按 s_meta 的定义长度收帧，
    # 短帧会被长度门卫整帧丢弃（与 30019 短帧同样的坑）
    lk.send(MSG_REC, bytes([op, arg, div]) + bytes(141), X_REC)
    t = time.time()
    while time.time() - t < timeout:
        got = lk.poll(0.02)
        for mid, pl in got:
            if mid == MSG_REC and len(pl) >= 16:
                return pl
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--div', type=int, default=1)
    ap.add_argument('--rpm', type=float, default=1560.0)
    ap.add_argument('--run', action='store_true')
    ap.add_argument('--ch', type=int, default=3)
    a = ap.parse_args()

    lk = Link('COM27')
    lk.poll(0.3)
    if a.run:
        lk.send(MSG_MODE, bytes([3]), X_MODE)
        time.sleep(0.05)
        pl = pack_speed(a.rpm)
        lk.send(MSG_PS, pack_speed(a.rpm, 500.0), X_PS)
        lk.poll(5.0, ka=lambda: lk.send(MSG_PS, pl, X_PS), ka_period=0.4)

    # 先 STOP 清场，再启动（全程保活，避免 2 s deadman 把速度撤掉）
    rec(lk, 2)
    if a.run:
        lk.send(MSG_PS, pack_speed(a.rpm), X_PS)
    r = rec(lk, 1, a.ch, a.div)
    print('START: %s' % (r[:8].hex() if r else 'no ack'))
    need = a.div * 1024 / 20000.0
    for _ in range(int(need / 0.4) + 1):
        if a.run:
            lk.send(MSG_PS, pack_speed(a.rpm), X_PS)
        lk.poll(0.4)
    time.sleep(0.1)
    r2 = rec(lk, 2)
    print('STOP : %s' % (r2[:8].hex() if r2 else 'no ack'))
    time.sleep(0.1)

    th = []
    vd = []
    print('通道 %d（%s），div=%d' % (a.ch, {0:'iq',1:'iu+θ',2:'vq',3:'vd+θ'}.get(a.ch,'?'), a.div))
    for blk in range(64):
        r = rec(lk, 3, blk, timeout=1.2)
        if r is None:
            print('块 %d 无应答' % blk)
            break
        if blk == 0:
            print('块0 应答头: %s' % r[:16].hex())
        st_code = r[0]
        n_this = r[5]
        n_pts = r[6] | (r[7] << 8)
        if n_this == 0:
            break
        for k in range(n_this):
            v = struct.unpack_from('<f', r, 16 + k * 4)[0]
            t_ = struct.unpack_from('<f', r, 80 + k * 4)[0]
            vd.append(v)
            th.append(t_)
        if len(th) >= n_pts:
            break
        time.sleep(0.004)

    # 保持给定（否则 2 s deadman 会停机）
    pl = pack_speed(a.rpm) if a.run else None
    if pl:
        lk.send(MSG_PS, pl, X_PS)

    print('总点数 %d，div=%d，采样 %.1f kHz，窗口 %.0f ms'
          % (len(th), a.div, 20.0 / a.div, len(th) * a.div / 20.0))

    # 单圈角差分 -> 速度（处理 2π 回绕）
    dt = a.div / 20000.0
    w = []
    for k in range(1, len(th)):
        d = th[k] - th[k - 1]
        while d > math.pi:
            d -= TWO_PI
        while d < -math.pi:
            d += TWO_PI
        w.append(d / dt)
    rpm = [x * 60.0 / TWO_PI for x in w]
    m = sum(rpm) / len(rpm)
    pp = max(rpm) - min(rpm)
    rms = math.sqrt(sum((x - m) ** 2 for x in rpm) / len(rpm))
    print('速度: mean=%.2f rpm  pp=%.2f  rms=%.2f  (量化台阶 %.2f rpm)'
          % (m, pp, rms, 60.0 / TWO_PI * (TWO_PI / 65536.0) / dt))

    print('\n前 60 点（每点 %.1f ms）：' % (dt * 1000))
    for k in range(0, min(60, len(rpm))):
        print('%3d  th=%8.5f  rpm=%9.2f' % (k, th[k], rpm[k]))

    # 简单 DFT 找主频（1~2000 Hz）
    fs = 1.0 / dt
    y = [x - m for x in rpm]
    n = len(y)
    best = (0.0, 0.0)
    f = 5.0
    while f < min(2000.0, fs / 2):
        wr = TWO_PI * f / fs
        re = im = 0.0
        for k in range(n):
            wnd = 0.5 - 0.5 * math.cos(TWO_PI * k / (n - 1))
            re += y[k] * wnd * math.cos(wr * k)
            im -= y[k] * wnd * math.sin(wr * k)
        amp = 2.0 * math.hypot(re, im) / n
        if amp > best[1]:
            best = (f, amp)
        f *= 1.01
    print('\n主频 %.1f Hz 幅值 %.2f rpm  (对应每转 %.2f 次, 1560rpm=26Hz)'
          % (best[0], best[1], best[0] / 26.0))


    # ---- 角度残差谐波（机械角域）：判断 4 次/转误差是否可补偿 ----
    u = [th[0]]
    for k in range(1, len(th)):
        d = th[k] - th[k - 1]
        while d > math.pi:
            d -= TWO_PI
        while d < -math.pi:
            d += TWO_PI
        u.append(u[-1] + d)
    nn = len(u)
    ts = [k * dt for k in range(nn)]
    mt = sum(ts) / nn
    mu = sum(u) / nn
    num = sum((ts[k] - mt) * (u[k] - mu) for k in range(nn))
    den = sum((ts[k] - mt) ** 2 for k in range(nn))
    aa = num / den
    bb = mu - aa * mt
    res = [u[k] - (aa * ts[k] + bb) for k in range(nn)]
    mr = sum(res) / nn
    res = [x - mr for x in res]
    f_rev = aa / TWO_PI
    print('\n角度残差: pp=%.2f mrad rms=%.3f mrad' %
          ((max(res) - min(res)) * 1000, math.sqrt(sum(x * x for x in res) / nn) * 1000))
    print(' 次/转  freq(Hz)  amp(mrad)  phase(deg)')
    for k in range(1, 7):
        fq = f_rev * k
        if fq >= fs / 2:
            break
        wr = TWO_PI * fq / fs
        re = im = 0.0
        for j in range(nn):
            wnd = 0.5 - 0.5 * math.cos(TWO_PI * j / (nn - 1))
            re += res[j] * wnd * math.cos(wr * j)
            im -= res[j] * wnd * math.sin(wr * j)
        amp = 2.0 * math.hypot(re, im) / nn
        ph = math.degrees(math.atan2(im, re))
        print('%6d %9.1f %10.3f %10.1f' % (k, fq, amp * 1000, ph))

    with open('rec_theta_%s.csv' % time.strftime('%H%M%S'), 'w', encoding='utf-8-sig') as fh:
        fh.write('k,theta_rad,rpm,vd_mV\n')
        for k in range(len(rpm)):
            fh.write('%d,%.6f,%.3f,%.2f\n' % (k, th[k], rpm[k], vd[k] if k < len(vd) else 0))


if __name__ == '__main__':
    main()
