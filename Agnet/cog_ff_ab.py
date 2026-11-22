# -*- coding: utf-8 -*-
"""齿槽前馈 A/B：量化前馈到底补掉了多少齿槽

原理：前馈开/关各录一次 iq，比较齿槽频率（转速 × 24 齿槽/转）处的幅值。
  开 = 残差，关 = 原始激励；补偿率 = (关 - 开) / 关。
cog_en 是 30082 int[3]，只改 RAM 不写 Flash，测完恢复。

用法: python cog_ff_ab.py COM27 [rpm]
"""
import sys, time, struct, math
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_CW, X_CW = 30054, 108
MSG_MODE, X_MODE = 30056, 26
MSG_PERM, X_PERM = 30068, 134
MSG_PS, X_PS = 30019, 8
MSG_LP, X_LP = 30091, 91
MSG_ST, X_ST = 30090, 90
MSG_REC, X_REC = 30094, 94
MSG_DBG, X_DBG = 30082, 218

KNOWN = {MSG_CW: X_CW, MSG_MODE: X_MODE, MSG_PERM: X_PERM, MSG_PS: X_PS,
         MSG_LP: X_LP, MSG_ST: X_ST, MSG_REC: X_REC, MSG_DBG: X_DBG}

ENC_COUNTS = 2097152.0
TS = 1.0 / 20000.0
REC_N = 1024
IDX_COG_EN = 3
COG_PER_REV = 24.0          # 40 Hz @ 100 rpm 反推（100/60 × 24 = 40）


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

    def set_cog(self, en):
        body = bytearray(192)
        struct.pack_into('<i', body, 128 + IDX_COG_EN * 4, int(en))
        struct.pack_into('<I', body, 124, 1 << IDX_COG_EN)
        self.send(MSG_DBG, bytes(body), X_DBG)
        return self.wait(MSG_DBG, 0.8, keep=True)


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


def capture(L, div=10):
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
    for b in range((REC_N + 15) // 16):
        r = L.rec(3, b, dur=0.5)
        if r is None or r[12] != 0:
            return None
        for k in range(r[5]):
            iq.append(struct.unpack_from('<f', r, 16 + k * 4)[0])
    return iq if len(iq) >= REC_N else None


def analyse(L, rpm, tag):
    iq = capture(L)
    if iq is None:
        print('  %s: 录波失败' % tag)
        return None
    n = len(iq)
    fs = 1.0 / (10 * TS)
    mean = sum(iq) / n
    ac = [v - mean for v in iq]
    rms = math.sqrt(sum(v * v for v in ac) / n)
    f_cog = abs(rpm) / 60.0 * COG_PER_REV
    a_cog = goertzel(ac, fs, f_cog)
    a_best = max(goertzel(ac, fs, f_cog + d) for d in (-3, -2, -1, 0, 1, 2, 3))
    f_e = abs(rpm) / 60.0 * 14.0
    a1 = goertzel(ac, fs, f_e)
    print('  %-16s 齿槽 %.1f Hz: %.4f A（邻域峰 %.4f）  RMS %.4f  1次 %.4f'
          % (tag, f_cog, a_cog, a_best, rms, a1))
    return {'a_cog': a_cog, 'a_best': a_best, 'rms': rms, 'f_cog': f_cog, 'a1': a1}


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0

    L = Link(port)
    print('端口 %s，电机 %.0f rpm（齿槽约 %.0f Hz）' % (port, rpm, rpm / 60.0 * COG_PER_REV))

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.poll(0.15)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.15)
    L.send(MSG_MODE, bytes([3]), X_MODE)
    L.poll(0.15)

    st = L.wait(MSG_ST, 0.6)
    if st:
        print('  flags=0x%02X（bit7 COG=%d）' % (st[3], (st[3] >> 7) & 1))

    L.w_cs = rpm * 2.0 * math.pi / 60.0 * ENC_COUNTS / (2.0 * math.pi)
    L.keepalive()
    L.poll(0.08)
    L.send(MSG_CW, struct.pack('<HB', 1, 0), X_CW)
    L.wait(MSG_CW, 1.0, keep=True)
    L.keepalive()
    time.sleep(1.5)
    g = L.wait(MSG_LP, 0.6, keep=True)
    w = rpm
    if g:
        w = abs(struct.unpack_from('<f', g, 8)[0])
        print('  wmeas=%.1f rpm' % w)

    print('\n=== A/B（同一转速，只切 cog_en）===')
    r = L.set_cog(1)
    print('  cog_en=1 应答 %s' % ('?' if r is None else r[0]))
    time.sleep(0.3)
    on = analyse(L, w, 'cog_en=1（开）')

    r = L.set_cog(0)
    print('  cog_en=0 应答 %s' % ('?' if r is None else r[0]))
    time.sleep(0.3)
    off = analyse(L, w, 'cog_en=0（关）')

    L.set_cog(1)                     # 恢复
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.2)
    print('\n已恢复 cog_en=1 并封锁')

    if on and off and off['a_best'] > 1e-6:
        comp = 100.0 * (off['a_best'] - on['a_best']) / off['a_best']
        print('\n=== 结论 ===')
        print('  齿槽 %.0f Hz：关 %.4f A → 开 %.4f A' % (on['f_cog'], off['a_best'], on['a_best']))
        print('  前馈补偿率 %.1f%%   残差 %.4f A（占关时 %.0f%%）'
              % (comp, on['a_best'], 100 * on['a_best'] / off['a_best']))
        print('  交流 RMS：关 %.4f → 开 %.4f A' % (off['rms'], on['rms']))
    L.s.close()


if __name__ == '__main__':
    main()
