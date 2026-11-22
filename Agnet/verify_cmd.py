# -*- coding: utf-8 -*-
"""逐命令 ACK 验证：权限/模式/给定/使能 每步读应答，确认哪一步被拒

重点：30056 回读 s_op_mode 是否为 3；30019 在 VEL 下是否返回 SUCCESS。
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

    def wait(self, msgid, dur):
        t = time.time()
        while time.time() - t < dur:
            for mid, pl in self.poll(0.03):
                if mid == msgid:
                    return pl
        return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
    L = Link(port)
    time.sleep(0.3)
    L.poll(0.3)

    # 30068 权限
    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    r = L.wait(MSG_PERM, 0.8)
    print('30068 权限    ack=%s' % ('无' if r is None else list(r[:3])))

    # 30054 封锁
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    r = L.wait(MSG_CW, 0.8)
    print('30054 封锁    ack=%s' % ('无' if r is None else list(r[:2])))

    # 30056 选模式 VEL=3
    L.send(MSG_MODE, bytes([3]), X_MODE)
    r = L.wait(MSG_MODE, 0.8)
    print('30056 模式=3   ack=%s  (resp[1]=op_mode 回读)' % ('无' if r is None else list(r[:2])))

    # 30019 速度给定（未使能）
    L.w_cs = rpm * ENC_COUNTS / 60.0
    L.keepalive()
    r = L.wait(MSG_PS, 0.8)
    print('30019 速度给定 ack=%s' % ('无' if r is None else list(r[:4])))

    # 读当前 s_op_mode / armed / src
    st = L.wait(MSG_ST, 0.5)
    lp = L.wait(MSG_LP, 0.5)
    if st is not None and len(st) >= 8:
        print('  当前: flags=0x%02X(armed=%d) mode=%s'
              % (st[3], 1 if st[3] & 4 else 0, st[2]))
    if lp is not None and len(lp) >= 8:
        print('  当前: spd_mode=%s src=%s wref=%.1f'
              % (lp[0], lp[1], struct.unpack_from('<f', lp, 4)[0]))

    # 30054 使能
    L.send(MSG_CW, struct.pack('<HB', 1, 0), X_CW)
    r = L.wait(MSG_CW, 0.8)
    print('30054 使能    ack=%s' % ('无' if r is None else list(r[:2])))

    # 立即读状态
    time.sleep(0.1)
    st = L.wait(MSG_ST, 0.5)
    lp = L.wait(MSG_LP, 0.5)
    if st is not None and len(st) >= 8:
        print('  使能后: flags=0x%02X(armed=%d) mode=%s fault=0x%04X'
              % (st[3], 1 if st[3] & 4 else 0, st[2],
                 struct.unpack_from('<H', st, 8)[0]))
    if lp is not None and len(lp) >= 12:
        print('  使能后: spd_mode=%s src=%s wref=%.1f wmeas=%.1f iqc=%.3f'
              % (lp[0], lp[1], struct.unpack_from('<f', lp, 4)[0],
                 struct.unpack_from('<f', lp, 8)[0],
                 struct.unpack_from('<f', lp, 12)[0]))

    # 收尾封锁
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.2)
    L.s.close()


if __name__ == '__main__':
    main()
