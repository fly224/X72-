# -*- coding: utf-8 -*-
"""位置环 Kp 扫描：量化静止保持时的振荡，找不振的 Kp

方法：位置模式保持 → 逐个写 30082 float[7]=pos_kp → 录波 iq →
      统计峰峰 / 交流 RMS / 主峰频率与幅值。

判据：嗡嗡声对应 iq 的周期性振荡，峰峰和主峰幅值随 Kp 变化的拐点就是可用区。
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
MSG_DBG,  X_DBG  = 30082, 218

KNOWN = {MSG_CW: X_CW, MSG_MODE: X_MODE, MSG_PERM: X_PERM, MSG_PS: X_PS,
         MSG_ST: X_ST, MSG_LP: X_LP, MSG_REC: X_REC, MSG_DBG: X_DBG}
ENC_COUNTS = 2097152.0
TWO_PI = 6.283185307179586
REC_N = 1024
REC_DIV = 10
IDX_POS_KP = 7


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

    def set_pos_kp(self, kp):
        """写 30082 float[7]。返回 (status, n_ok)；失败自动重试 3 次。
        30082 与保活共用 s_pending 单槽，保活太密会把应答挤掉。"""
        body = bytearray(192)
        struct.pack_into('<f', body, IDX_POS_KP * 4, kp)
        struct.pack_into('<I', body, 64, 1 << IDX_POS_KP)
        for _ in range(3):
            self.send(MSG_DBG, bytes(body), X_DBG)
            r = self.wait(MSG_DBG, 0.8)
            if r is not None:
                return (r[0], r[1] if len(r) > 1 else 0xFF)
            time.sleep(0.05)
        return (None, 0)

    def get_pos_kp(self):
        """从 30100 读回实际生效的 kp"""
        pd = self.wait(30100, 0.5, keep=True)
        if pd and len(pd) >= 24:
            return struct.unpack_from('<f', pd, 20)[0]
        return None


def goertzel(x, fs, f):
    n = len(x)
    if n < 8 or f <= 0 or f >= fs / 2:
        return 0.0
    w = 2.0 * math.pi * f / fs
    cw, sw = math.cos(w), math.sin(w)
    coeff = 2.0 * cw
    s1 = s2 = 0.0
    for i, v in enumerate(x):
        wv = v * 0.5 * (1.0 - math.cos(2.0 * math.pi * i / (n - 1)))
        s0 = wv + coeff * s1 - s2
        s2, s1 = s1, s0
    real = s1 - s2 * cw
    imag = s2 * sw
    return math.sqrt(real * real + imag * imag) * 2.0 / n * 2.0


def capture_iq(L):
    r = L.rec(1, 0, REC_DIV)
    if r is None or r[12] != 0:
        return None
    t0 = time.time()
    while time.time() - t0 < 2.0:
        L.keepalive()
        time.sleep(0.03)
        s = L.rec(0)
        if s is None:
            continue
        if s[4] == 2 and (s[6] | (s[7] << 8)) >= REC_N:
            break
    iq = []
    for b in range((REC_N + 15) // 16):
        r = L.rec(3, b)
        if r is None or r[12] != 0:
            return None
        for k in range(r[5]):
            iq.append(struct.unpack_from('<f', r, 16 + k * 4)[0])
    return iq


def stats(iq):
    n = len(iq)
    fs = 20000.0 / REC_DIV
    mean = sum(iq) / n
    ac = [v - mean for v in iq]
    pp = max(iq) - min(iq)
    rms = math.sqrt(sum(v * v for v in ac) / n)
    best_f, best_a = 0.0, 0.0
    f = 15.0
    while f < 900.0:
        a = goertzel(ac, fs, f)
        if a > best_a:
            best_a, best_f = a, f
        f += 5.0
    return pp, rms, best_f, best_a


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    kp_list = [100.0, 80.0, 60.0, 40.0, 25.0]
    if len(sys.argv) > 2:
        kp_list = [float(x) for x in sys.argv[2].split(',')]

    L = Link(port)
    time.sleep(0.3)
    L.poll(0.3)

    st = L.wait(MSG_ST, 0.6)
    theta_m = struct.unpack_from('<f', st, 16)[0]
    L.pos_cs = int(round(theta_m * ENC_COUNTS / TWO_PI))
    print('保持角 %.4f rad，扫描 pos_kp: %s' % (theta_m, kp_list))

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.poll(0.1)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.15)
    L.send(MSG_MODE, bytes([1]), X_MODE)
    L.poll(0.15)
    L.keepalive()
    L.poll(0.1)
    L.send(MSG_CW, struct.pack('<HB', 1, 0), X_CW)
    ack = L.wait(MSG_CW, 1.0, keep=True)
    print('使能 ack=%s\n' % ('无' if ack is None else ack[0]))

    print('  pos_kp  st n_ok   iq峰峰   iq_RMS  主峰Hz  主峰幅值   嗡嗡声')
    print('  ' + '-' * 70)
    results = []
    for kp in kp_list:
        stt, nok = L.set_pos_kp(kp)
        if stt != 0 or nok == 0:
            print('  %6.1f  %s  %3d   —— 写入失败（st=%s n_ok=%s）'
                  % (kp, '-' if stt is None else stt, nok, stt, nok))
            continue
        time.sleep(0.8)
        iq = capture_iq(L)
        if iq is None:
            print('  %6.1f   0  %3d   录波失败' % (kp, nok))
            continue
        pp, rms, pf, pa = stats(iq)
        # 嗡嗡声判据用**主峰幅值**（周期脉动），不用峰峰（含随机尖峰）
        buzz = '无' if pa < 0.05 else ('轻' if pa < 0.12 else ('明显' if pa < 0.2 else '强'))
        print('  %6.1f  %d  %3d  %7.4f  %7.4f  %5.0f  %8.4f   %s'
              % (kp, stt, nok, pp, rms, pf, pa, buzz))
        results.append((kp, pp, rms, pf, pa))
        L.keepalive()
        L.poll(0.08)

    # 恢复 100
    L.set_pos_kp(100.0)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.2)
    print('\n已恢复 pos_kp=100 并失能')

    if results:
        best = min(results, key=lambda x: x[1])
        print('最佳（按 iq 峰峰）：pos_kp=%.0f  峰峰 %.4f A' % (best[0], best[1]))
    L.s.close()


if __name__ == '__main__':
    main()
