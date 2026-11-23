# -*- coding: utf-8 -*-
"""欠压阈值实测：低母线电压下使能，看是否按预期报 0x0080

欠压只在 MOE 使能时判（bsp_servo.c:286），所以必须真的使能才能验证。
速度给 0 rpm（不出力），只为把 MOE 打开。
"""
import sys, time, struct
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_CW,   X_CW   = 30054, 108
MSG_MODE, X_MODE = 30056, 26
MSG_PERM, X_PERM = 30068, 134
MSG_PS,   X_PS   = 30019, 8
MSG_ST,   X_ST   = 30090, 90

KNOWN = {MSG_CW: X_CW, MSG_MODE: X_MODE, MSG_PERM: X_PERM,
         MSG_PS: X_PS, MSG_ST: X_ST}

FAULT_NAME = [(0, '过流'), (2, '过压'), (3, '编码器'), (4, '缺相'),
              (5, '超速'), (6, '母线过压'), (7, '母线欠压'),
              (8, '位置跟随'), (9, '辨识')]


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
        struct.pack_into('<f', body, 28, 0.0)     # 速度 0，不出力
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


def read_st(L, keep=False):
    st = L.wait(MSG_ST, 0.4, keep=keep)
    if st is None or len(st) < 40:
        return None
    return {'vbus': struct.unpack_from('<f', st, 20)[0],
            'fault': struct.unpack_from('<H', st, 36)[0],
            'flags': st[3], 'cw': st[0] | (st[1] << 8)}


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    L = Link(port)
    time.sleep(0.3)
    L.poll(0.3)

    s = read_st(L)
    print('使能前: 母线 %.2f V  fault=0x%04X  cw=0x%04X'
          % (s['vbus'], s['fault'], s['cw']))

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.poll(0.1)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)   # 先封锁
    L.poll(0.15)
    L.send(MSG_MODE, bytes([3]), X_MODE)             # VEL
    L.poll(0.15)
    L.keepalive()                                    # 速度 0
    L.poll(0.1)
    L.send(MSG_CW, struct.pack('<HB', 1, 0), X_CW)   # 使能
    ack = L.wait(MSG_CW, 1.0, keep=True)
    print('使能 ack=%s' % ('无' if ack is None else ack[0]))

    print('\n使能后逐拍（欠压 20 ms 消抖，应很快报 0x0080）:')
    t = time.time()
    for k in range(8):
        L.keepalive()
        s = read_st(L, keep=True)
        if s:
            names = ' '.join(n for b, n in FAULT_NAME if s['fault'] & (1 << b))
            print('  %5.2fs  母线 %6.2f V  fault=0x%04X %-8s  armed=%d'
                  % (time.time() - t, s['vbus'], s['fault'],
                     names or '-', 1 if s['flags'] & 4 else 0))
        time.sleep(0.12)

    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.2)
    print('\n已封锁')
    L.s.close()


if __name__ == '__main__':
    main()
