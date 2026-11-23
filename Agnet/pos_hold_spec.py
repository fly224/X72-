# -*- coding: utf-8 -*-
"""位置环静止保持：录 iq 与机械角，定位"嗡嗡声"的频率与幅值

为什么先测频率：不同频段指向不同根因
  < 10 Hz       位置环极限环（Kp 过高 / 摩擦死区）
  10~100 Hz     速度环振荡
  100 Hz~1 kHz  死区补偿 / 电流环
  ~1 kHz 以上   PWM 载波边带

用法: python pos_hold_spec.py COM27 [保持秒数]
"""
import sys, time, struct, math
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_CW,   X_CW   = 30054, 108
MSG_MODE, X_MODE = 30056, 26
MSG_PERM, X_PERM = 30068, 134
MSG_PS,   X_PS   = 30019, 8
MSG_ST,   X_ST   = 30090, 90
MSG_LP,   X_LP   = 30091, 91
MSG_REC,  X_REC  = 30094, 94

KNOWN = {MSG_CW: X_CW, MSG_MODE: X_MODE, MSG_PERM: X_PERM,
         MSG_PS: X_PS, MSG_ST: X_ST, MSG_LP: X_LP, MSG_REC: X_REC}
ENC_COUNTS = 2097152.0
TWO_PI = 6.283185307179586
REC_N = 1024
REC_DIV = 10          # 2 kHz，512 ms，分辨率 1.95 Hz


def x25(data, extra):
    crc = 0xFFFF
    for b in list(data) + [extra]:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class Link:
    def __init__(self, port):
        self.s = serial.Serial(port, 921600, timeout=0)
        self.seq = 0
        self.buf = bytearray()
        self.pos_cs = 0

    def send(self, msgid, payload, extra):
        hdr = bytes([len(payload) & 0xFF, 0, 0, self.seq & 0xFF, 1, 1,
                     msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
        body = hdr + payload
        c = x25(body, extra)
        self.s.write(b'\xFD' + body + bytes([c & 0xFF, c >> 8]))
        self.seq += 1

    def poll(self, dur):
        out = []
        t = time.time()
        while time.time() - t < dur:
            d = self.s.read(self.s.in_waiting or 1)
            if d:
                self.buf += d
            i, n = 0, len(self.buf)
            while i + 12 <= n:
                if self.buf[i] != 0xFD:
                    i += 1
                    continue
                ln = self.buf[i + 1]
                if i + 12 + ln > n:
                    break
                mid = self.buf[i+7] | (self.buf[i+8] << 8) | (self.buf[i+9] << 16)
                ex = KNOWN.get(mid)
                if ex is None:
                    i += 1
                    continue
                body = bytes(self.buf[i+1:i+10+ln])
                crc = self.buf[i+10+ln] | (self.buf[i+11+ln] << 8)
                if x25(body, ex) != crc:
                    i += 1
                    continue
                out.append((mid, bytes(self.buf[i+10:i+10+ln])))
                i += 12 + ln
            del self.buf[:i]
            time.sleep(0.001)
        return out

    def keepalive(self):
        """30019：位置模式给当前位置，保持不输出（on_pos）"""
        body = bytearray(44)
        struct.pack_into('<q', body, 0, self.pos_cs)
        struct.pack_into('<f', body, 28, 0.0)
        self.send(MSG_PS, bytes(body), X_PS)

    def wait(self, msgid, dur, keep=False):
        t = time.time()
        while time.time() - t < dur:
            if keep:
                self.keepalive()
            for mid, pl in self.poll(0.03):
                if mid == msgid:
                    return pl
        return None

    def rec(self, op, arg=0, div=0, retry=3, dur=0.6):
        body = bytes([op & 0xFF, arg & 0xFF, div & 0xFF]) + bytes(141)
        for _ in range(retry):
            self.send(MSG_REC, body, X_REC)
            r = self.wait(MSG_REC, dur)
            if r is not None:
                return r
        return None


def goertzel(x, fs, f):
    """单频幅值（Hanning 窗归一化），避免依赖 numpy"""
    n = len(x)
    if n < 8 or f <= 0 or f >= fs / 2:
        return 0.0
    w = 2.0 * math.pi * f / fs
    cw, sw = math.cos(w), math.sin(w)
    coeff = 2.0 * cw
    s0 = s1 = s2 = 0.0
    for i, v in enumerate(x):
        # Hanning 窗
        wv = v * 0.5 * (1.0 - math.cos(2.0 * math.pi * i / (n - 1)))
        s0 = wv + coeff * s1 - s2
        s2, s1 = s1, s0
    real = s1 - s2 * cw
    imag = s2 * sw
    mag = math.sqrt(real * real + imag * imag) * 2.0 / n
    return mag * 2.0          # Hanning 相干增益 0.5，补回


def capture(L, ch):
    r = L.rec(1, ch, REC_DIV)
    if r is None or r[12] != 0:
        print('    录波启动失败 st=%s' % ('?' if r is None else r[12]))
        return None, None
    t0 = time.time()
    while time.time() - t0 < 2.0:
        L.keepalive()
        time.sleep(0.03)
        s = L.rec(0)
        if s is None:
            continue
        n_pts = s[6] | (s[7] << 8)
        if s[4] == 2 and n_pts >= REC_N:
            break
    buf, th = [], []
    for b in range((REC_N + 15) // 16):
        r = L.rec(3, b)
        if r is None or r[12] != 0:
            print('    块 %d 读取失败' % b)
            return None, None
        n_this = r[5]
        for k in range(n_this):
            buf.append(struct.unpack_from('<f', r, 16 + k * 4)[0])
            th.append(struct.unpack_from('<f', r, 80 + k * 4)[0])
    return buf, th


def analyze(vals, dt, label, unit):
    n = len(vals)
    fs = 1.0 / dt
    mean = sum(vals) / n
    ac = [v - mean for v in vals]
    pp = max(vals) - min(vals)
    rms = math.sqrt(sum(v * v for v in ac) / n)

    best_f, best_a = 0.0, 0.0
    f = 10.0
    while f < min(900.0, fs / 2 - 50):
        a = goertzel(ac, fs, f)
        if a > best_a:
            best_a, best_f = a, f
        f += 5.0 if f < 200 else 10.0

    print('    %s: 均值 %.5f %s  峰峰 %.5f  交流RMS %.5f  %s'
          % (label, mean, unit, pp, rms, unit))
    print('      主峰 %.0f Hz (%.5f %s)' % (best_f, best_a, unit))
    return {'mean': mean, 'pp': pp, 'rms': rms, 'peak_f': best_f, 'peak_a': best_a}


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    hold = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    L = Link(port)
    time.sleep(0.3)
    L.poll(0.3)

    st = L.wait(MSG_ST, 0.6)
    if st is None or len(st) < 40:
        print('读不到 30090')
        return
    theta_m = struct.unpack_from('<f', st, 16)[0]
    print('当前单圈角 %.4f rad，位置模式目标设为同值（保持）' % theta_m)

    L.pos_cs = int(round(theta_m * ENC_COUNTS / TWO_PI))
    print('目标 counts = %d' % L.pos_cs)

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.poll(0.1)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.15)
    L.send(MSG_MODE, bytes([1]), X_MODE)      # POS
    L.poll(0.15)
    L.keepalive()
    L.poll(0.1)
    L.send(MSG_CW, struct.pack('<HB', 1, 0), X_CW)
    ack = L.wait(MSG_CW, 1.0, keep=True)
    print('使能 ack=%s' % ('无' if ack is None else ack[0]))

    print('\n保持 %.1f s，观察静止状态 ...' % hold)
    t = time.time()
    while time.time() - t < hold:
        L.keepalive()
        time.sleep(0.1)

    lp = L.wait(MSG_LP, 0.5, keep=True)
    if lp and len(lp) >= 16:
        print('  wmeas=%.2f rpm  iqc=%.4f A' % (
            struct.unpack_from('<f', lp, 8)[0],
            struct.unpack_from('<f', lp, 12)[0]))

    print('\n=== 录波 ch0 = iq（2 kHz / 512 ms）===')
    iq, _ = capture(L, 0)
    if iq:
        analyze(iq, REC_DIV / 20000.0, 'iq', 'A')

    print('\n=== 录波 ch3 = 单圈机械角（位置抖动）===')
    pos, _ = capture(L, 3)
    if pos:
        r = analyze(pos, REC_DIV / 20000.0, '机械角', 'rad')

    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.2)
    print('\n已失能')
    L.s.close()


if __name__ == '__main__':
    main()
