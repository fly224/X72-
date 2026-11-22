# -*- coding: utf-8 -*-
"""CAN FD 内环自检：30069 action=20 -> 30092 kind=10

用法: python can_selftest.py COM27
"""
import sys, time, struct
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_PERM, X_PERM = 30068, 134
MSG_CW,   X_CW   = 30054, 108
MSG_SYS,  X_SYS  = 30069, 169
MSG_ST,   X_ST   = 30090, 90
MSG_ID,   X_ID   = 30092, 92

KNOWN = {MSG_PERM: X_PERM, MSG_CW: X_CW, MSG_SYS: X_SYS,
         MSG_ST: X_ST, MSG_ID: X_ID}

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

    def send(self, msgid, payload, extra):
        hdr = bytes([len(payload) & 0xFF, 0, 0, self.seq & 0xFF, 1, 1,
                     msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
        body = hdr + payload
        c = x25(body, extra)
        self.s.write(b'\xFD' + body + bytes([c & 0xFF, c >> 8]))
        self.seq += 1

    def _parse(self):
        out = []
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
            body = self.buf[i+1:i+10+ln]
            crc = self.buf[i+10+ln] | (self.buf[i+11+ln] << 8)
            if x25(body, ex) != crc:
                i += 1
                continue
            out.append((mid, bytes(self.buf[i+10:i+10+ln])))
            i += 12 + ln
        del self.buf[:i]
        return out

    def poll(self, dur):
        out = []
        t = time.time()
        while time.time() - t < dur:
            d = self.s.read(self.s.in_waiting or 1)
            if d:
                self.buf += d
            out += self._parse()
            time.sleep(0.001)
        return out

    def wait(self, msgid, dur=1.0):
        t = time.time()
        while time.time() - t < dur:
            for mid, pl in self.poll(0.02):
                if mid == msgid:
                    return pl
        return None

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    L = Link(port)
    print('端口', port, '已打开')
    time.sleep(0.3)

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    a = L.wait(MSG_PERM, 0.5)
    print('30068 权限 status =', None if a is None else a[0])

    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.08)
    print('已 halt（自检要求 disarmed）')

    # 触发前先清空 30092 队列里可能残留的帧
    L.poll(0.05)

    t0 = time.time()
    L.send(MSG_SYS, bytes([20]), X_SYS)
    ack = L.wait(MSG_SYS, 1.0)
    print('30069 action=20 status =', None if ack is None else ack[0])

    res = None
    while time.time() - t0 < 3.0:
        pl = L.wait(MSG_ID, 0.2)
        if pl and pl[0] == 10:
            res = pl
            break

    if res is None:
        print('!! 3 s 内没有收到 30092 kind=10')
        L.s.close()
        return

    ok = res[1]
    n  = res[2] | (res[3] << 8)
    v  = list(struct.unpack_from('<8f', res, 4))
    print('\n=== CAN FD 内环自检结果 ===')
    print('ok =', ok, '  rx_n =', n)
    print('tx        =', int(v[0]))
    print('rx        =', int(v[1]))
    print('err       =', int(v[2]))
    print('TEC       =', int(v[3]))
    print('bitrate_n =', int(v[4]))
    print('bitrate_d =', int(v[5]))
    print('dlc_mask  = 0x%02X  (bit0=8B ... bit5=64B)' % int(v[6]))
    print('REC       =', int(v[7]))
    print('判定:', 'PASS' if ok == 1 and v[2] == 0 else 'FAIL')

    st = L.wait(MSG_ST, 0.5)
    if st:
        print('30090: fault=0x%04X mode=%d flags=0x%02X'
              % (st[36] | (st[37] << 8), st[2], st[3]))
    L.s.close()

if __name__ == '__main__':
    main()
