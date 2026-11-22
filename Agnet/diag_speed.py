# -*- coding: utf-8 -*-
"""速度模式使能逐拍诊断：定位"下发没反应"卡在哪一层

读 30090（flags/mode）+ 30091（speed_mode/src/wref/wmeas/iqc），
使能后连续 3 秒逐拍打印，一眼看出速度环有没有起来、给定有没有进去。
"""
import sys, time, struct
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_CW,   X_CW   = 30054, 108
MSG_MODE, X_MODE = 30056, 26
MSG_PERM, X_PERM = 30068, 134
MSG_PS,   X_PS   = 30019, 8
MSG_ST,   X_ST   = 30090, 90
MSG_LP,   X_LP   = 30091, 91

KNOWN = {MSG_CW: X_CW, MSG_MODE: X_MODE, MSG_PERM: X_PERM,
         MSG_PS: X_PS, MSG_ST: X_ST, MSG_LP: X_LP}
ENC_COUNTS = 2097152.0

SRC_NAME = {0: 'NONE', 1: 'POS', 2: 'SPEED?', 3: 'VEL', 4: 'TORQUE', 12: 'MIT'}


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
            time.sleep(0.001)
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
            for mid, pl in self.poll(0.03):
                if mid == msgid:
                    return pl
        return None


def snap(L):
    st = L.wait(MSG_ST, 0.4, keep=True)
    lp = L.wait(MSG_LP, 0.4, keep=True)
    r = {}
    if st is not None and len(st) >= 40:
        r['flags'] = st[3]
        r['mode'] = st[2]
        r['fault'] = struct.unpack_from('<H', st, 36)[0]   # fault @ offset 36
    if lp is not None and len(lp) >= 16:
        r['spd_mode'] = lp[0]          # IsSpeedMode
        r['src'] = lp[1]               # Servo_GetSrc
        r['why'] = lp[2]               # 最近一次 do_disable 原因 why[0]
        r['wref'] = struct.unpack_from('<f', lp, 4)[0]
        r['wmeas'] = struct.unpack_from('<f', lp, 8)[0]
        r['iqc'] = struct.unpack_from('<f', lp, 12)[0]
    return r


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
    L = Link(port)
    time.sleep(0.3)
    L.poll(0.3)

    print('初始:')
    r = snap(L)
    print('  flags=0x%02X(armed=%d) mode=%s fault=0x%04X'
          % (r.get('flags', 0), 1 if r.get('flags', 0) & 4 else 0,
             r.get('mode', '?'), r.get('fault', 0)))
    print('  spd_mode=%s src=%s(%s) wref=%.1f wmeas=%.1f iqc=%.3f'
          % (r.get('spd_mode', '?'), r.get('src', '?'),
             SRC_NAME.get(r.get('src', -1), '?'), r.get('wref', 0),
             r.get('wmeas', 0), r.get('iqc', 0)))

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.poll(0.1)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.15)
    L.send(MSG_MODE, bytes([3]), X_MODE)
    L.poll(0.15)
    L.w_cs = rpm * ENC_COUNTS / 60.0
    L.keepalive()
    L.poll(0.1)

    print('\n使能 (cw=1):')
    L.send(MSG_CW, struct.pack('<HB', 1, 0), X_CW)
    ack = L.wait(MSG_CW, 1.0, keep=True)
    if ack is None:
        print('  ack 无应答')
    else:
        print('  ack status=%s  控制字回读=%s' % (ack[0], ack[1] if len(ack) > 1 else '?'))

    print('\n逐拍 (时间 armed/mode/fault/why  spd/src/wref/wmeas/iqc):')
    t = time.time()
    for k in range(8):
        L.keepalive()
        r = snap(L)
        print('  %4.1fs  arm=%d mode=%s fault=0x%04X why=%s  spd=%s src=%s  wref=%6.1f wmeas=%6.1f iqc=%6.3f'
              % (time.time() - t, 1 if r.get('flags', 0) & 4 else 0,
                 r.get('mode', '?'), r.get('fault', 0),
                 repr(chr(r['why'])) if 'why' in r and 32 <= r['why'] < 127 else r.get('why', '?'),
                 r.get('spd_mode', '?'),
                 SRC_NAME.get(r.get('src', -1), str(r.get('src', '?'))),
                 r.get('wref', 0), r.get('wmeas', 0), r.get('iqc', 0)))
        time.sleep(0.25)

    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.2)
    print('\n已失能')
    L.s.close()


if __name__ == '__main__':
    main()
