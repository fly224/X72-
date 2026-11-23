# -*- coding: utf-8 -*-
"""母线电压保护压测：急加速 / 急减速，看 UV / OV 阈值会不会误触发

方法：速度模式使能后，给定在 0 ↔ 目标之间阶跃（加速度用 30019 off32 拉大），
      每 100 ms 读 30090 的 vbus + fault，统计最低/最高母线。
      加速时电流大 → 母线跌落 → 考验 UV；减速时回馈泵升 → 考验 OV。

用法: python vbus_stress.py COM27 [rpm]
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

    def keepalive(self, acc=None):
        """30019：off28=速度(counts/s)，off32=加速度(counts/s²，0=不改)"""
        body = bytearray(44)
        struct.pack_into('<q', body, 0, 0)
        struct.pack_into('<f', body, 28, self.w_cs)
        if acc is not None:
            struct.pack_into('<f', body, 32, acc)
        self.send(MSG_PS, bytes(body), X_PS)

    def wait(self, msgid, dur, keep=False, acc=None):
        t = time.time()
        while time.time() - t < dur:
            if keep:
                self.keepalive(acc)
            for mid, pl in self.poll(0.04):
                if mid == msgid:
                    return pl
        return None


def read_vbus(L, acc=None):
    st = L.wait(MSG_ST, 0.35, keep=True, acc=acc)
    if st is None or len(st) < 40:
        return None, None
    return (struct.unpack_from('<f', st, 20)[0],
            struct.unpack_from('<H', st, 36)[0])


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 200.0
    L = Link(port)
    time.sleep(0.3)
    L.poll(0.3)

    v0, f0 = read_vbus(L)
    print('静止母线 %.2f V  fault=0x%04X' % (v0 or 0, f0 or 0))
    print('目标 %.0f rpm（空载），加速度拉到 2000 rpm/s 制造大电流' % rpm)

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.poll(0.1)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.15)
    L.send(MSG_MODE, bytes([3]), X_MODE)
    L.poll(0.15)

    # 加速度：2000 rpm/s 换算成 counts/s²
    acc_cs = 2000.0 * ENC_COUNTS / 60.0
    L.w_cs = 0.0
    L.keepalive(acc_cs)
    L.poll(0.1)
    L.send(MSG_CW, struct.pack('<HB', 1, 0), X_CW)
    ack = L.wait(MSG_CW, 1.0, keep=True, acc=acc_cs)
    print('使能 ack=%s' % ('无' if ack is None else ack[0]))

    lo, hi = 999.0, 0.0
    faults = set()
    rows = []

    for phase, target in (('急加速→%.0f' % rpm, rpm),
                          ('保持', rpm),
                          ('急减速→0', 0.0),
                          ('停稳', 0.0)):
        L.w_cs = target * ENC_COUNTS / 60.0
        t = time.time()
        dur = 1.0 if phase in ('急加速→%.0f' % rpm, '急减速→0') else 0.8
        while time.time() - t < dur:
            L.keepalive(acc_cs)
            v, f = read_vbus(L, acc_cs)
            if v is not None:
                lo = min(lo, v)
                hi = max(hi, v)
                if f:
                    faults.add(f)
                rows.append((phase, v, f))
        print('  %-12s 末段母线 %.2f V  fault=0x%04X' % (phase, rows[-1][1], rows[-1][2]))

    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.2)

    print('\n=== 压测结果 ===')
    print('  母线最低 %.2f V   最高 %.2f V' % (lo, hi))
    print('  欠压阈值 30.00 V  过压阈值 60.00 V')
    print('  故障: %s' % ('无' if not faults else
                          ' '.join('0x%04X' % f for f in faults)))
    if lo < 30.0:
        print('  ⚠ 母线最低值已低于 30 V —— 欠压阈值贴着工作点，需要下调')
    if hi > 60.0:
        print('  ⚠ 母线最高值超过 60 V —— 回馈泵升需硬件泄放')
    L.s.close()


if __name__ == '__main__':
    main()
