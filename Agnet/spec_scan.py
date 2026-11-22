# -*- coding: utf-8 -*-
"""宽频激励扫描：用录波看 iq 里到底有哪些频率成分

力矩脉动 → 结构振动 → 声音，所以 iq 谱就是"声音的激励谱"。
用两个采样率覆盖不同频段：
  div=10 → 2 kHz 采样，Nyquist 1 kHz，分辨率 1.95 Hz（看齿槽/低频）
  div=2  → 10 kHz 采样，Nyquist 5 kHz，分辨率 9.8 Hz（看音频段 200 Hz~4 kHz）

用法: python spec_scan.py COM27 [rpm]
"""
import sys, time, struct, math
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_CW, X_CW = 30054, 108
MSG_MODE, X_MODE = 30056, 26
MSG_PERM, X_PERM = 30068, 134
MSG_PS, X_PS = 30019, 8
MSG_LP, X_LP = 30091, 91
MSG_REC, X_REC = 30094, 94

KNOWN = {MSG_CW: X_CW, MSG_MODE: X_MODE, MSG_PERM: X_PERM,
         MSG_PS: X_PS, MSG_LP: X_LP, MSG_REC: X_REC}

ENC_COUNTS = 2097152.0
POLE_PAIRS = 14
TS = 1.0 / 20000.0
REC_N = 1024


def x25(data, extra):
    crc = 0xFFFF
    for b in list(data) + [extra]:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class Link:
    def __init__(self, port, baud=921600):
        self.s = serial.Serial(port, baud, timeout=0)
        self.seq = 0
        self.buf = bytearray()
        self.w_cs = 0.0

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
            time.sleep(0.002)
        return out

    def keepalive(self):
        body = bytearray(44)
        struct.pack_into('<q', body, 0, 0)
        struct.pack_into('<f', body, 28, self.w_cs)
        self.send(MSG_PS, bytes(body), X_PS)

    def wait(self, msgid, dur, keep=False):
        t = time.time()
        while time.time() - t < dur:
            if keep:
                self.keepalive()
            for mid, pl in self.poll(0.04):
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
    n = len(x)
    if n < 8 or f <= 0:
        return 0.0
    w = 2.0 * math.pi * f / fs
    cw, sw = math.cos(w), math.sin(w)
    coeff = 2.0 * cw
    s0 = s1 = s2 = 0.0
    for i, v in enumerate(x):
        hann = 0.5 - 0.5 * math.cos(2.0 * math.pi * i / (n - 1))
        s0 = v * hann + coeff * s1 - s2
        s2 = s1
        s1 = s0
    re = s1 - s2 * cw
    im = s2 * sw
    return 2.0 * math.sqrt(re * re + im * im) / n / 0.5


def capture(L, div):
    L.keepalive()
    r = L.rec(1, 0, div, dur=0.8)
    if r is None or r[12] != 0:
        return None
    t0 = time.time()
    while time.time() - t0 < 2.5:
        L.keepalive()
        time.sleep(0.04)
        s = L.rec(0, dur=0.25)
        if s and s[4] == 2 and (s[6] | (s[7] << 8)) >= REC_N:
            break
    iq = []
    th = []
    for b in range((REC_N + 15) // 16):
        r = L.rec(3, b, dur=0.5)
        if r is None or r[12] != 0:
            return None
        for k in range(r[5]):
            iq.append(struct.unpack_from('<f', r, 16 + k * 4)[0])
            th.append(struct.unpack_from('<f', r, 80 + k * 4)[0])
    return iq if len(iq) >= REC_N else None


def scan(L, div, f_lo, f_hi, step, tag):
    iq = capture(L, div)
    if iq is None:
        print('  %s: 录波失败' % tag)
        return
    dt = div * TS
    fs = 1.0 / dt
    n = len(iq)
    mean = sum(iq) / n
    ac = [v - mean for v in iq]
    rms = math.sqrt(sum(v * v for v in ac) / n)

    peaks = []
    f = f_lo
    while f < min(f_hi, fs * 0.45):
        peaks.append((goertzel(ac, fs, f), f))
        f += step
    peaks.sort(reverse=True)

    print('  %s：采样 %.1f kHz  时长 %.0f ms  分辨率 %.1f Hz  RMS %.4f A'
          % (tag, fs / 1000, n * dt * 1000, fs / n, rms))
    for a, fr in peaks[:8]:
        print('    %6.0f Hz   %7.4f A   (%4.1f%% RMS)' % (fr, a, 100 * a / rms if rms else 0))


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0

    L = Link(port)
    print('端口 %s，电机将 %.0f rpm 旋转' % (port, rpm))

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.poll(0.15)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.15)
    L.send(MSG_MODE, bytes([3]), X_MODE)
    L.poll(0.15)

    L.w_cs = rpm * 2.0 * math.pi / 60.0 * ENC_COUNTS / (2.0 * math.pi)
    L.keepalive()
    L.poll(0.08)
    L.send(MSG_CW, struct.pack('<HB', 1, 0), X_CW)
    L.wait(MSG_CW, 1.0, keep=True)
    L.keepalive()
    time.sleep(1.2)
    g = L.wait(MSG_LP, 0.6, keep=True)
    if g:
        print('  wmeas=%.1f rpm' % struct.unpack_from('<f', g, 8)[0])

    print('\n=== 低频段（div=10，0.3~900 Hz）===')
    scan(L, 10, 20.0, 900.0, 2.0, 'div10')

    print('\n=== 音频段（div=2，100~4200 Hz）===')
    scan(L, 2, 100.0, 4200.0, 10.0, 'div2')

    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.2)
    print('\n已封锁')
    L.s.close()


if __name__ == '__main__':
    main()
