# -*- coding: utf-8 -*-
"""位置保持逐拍诊断：读 30100（p_now/p_tgt/err/st）看位置环实际行为

判据：
  p_now 稳定 + err→0 + st=DONE   → 正常保持
  p_now 持续变化                  → 电机在转（目标没对上 / 方向反 / 跨 2π）
  err 很大                        → 目标折算有问题
  st 在 MOVE 长期不变             → 规划器没收敛
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
MSG_PD,   X_PD   = 30100, 100

KNOWN = {MSG_CW: X_CW, MSG_MODE: X_MODE, MSG_PERM: X_PERM,
         MSG_PS: X_PS, MSG_ST: X_ST, MSG_LP: X_LP, MSG_PD: X_PD}
ENC_COUNTS = 2097152.0
TWO_PI = 6.283185307179586

ST_NAME = {0: 'IDLE', 1: 'MOVE', 2: 'HOLD', 3: 'DONE', 4: 'ERR'}


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


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    L = Link(port)
    time.sleep(0.3)
    L.poll(0.3)

    st = L.wait(MSG_ST, 0.6)
    theta_m = struct.unpack_from('<f', st, 16)[0]
    print('当前单圈角 %.4f rad' % theta_m)
    L.pos_cs = int(round(theta_m * ENC_COUNTS / TWO_PI))
    print('目标 counts = %d（= %.4f rad）' % (L.pos_cs, L.pos_cs * TWO_PI / ENC_COUNTS))

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
    print('使能 ack=%s' % ('无' if ack is None else ack[0]))

    print('\n 时间   p_now    p_tgt   p_tgt_c   err      travel   st     wmeas   iqc')
    t0 = time.time()
    last_p = None
    for k in range(20):
        L.keepalive()
        pd = L.wait(MSG_PD, 0.1, keep=True)
        lp = L.wait(MSG_LP, 0.1, keep=True)
        if pd and len(pd) >= 46:
            p_now = struct.unpack_from('<f', pd, 0)[0]
            p_tgt = struct.unpack_from('<f', pd, 4)[0]
            p_tc = struct.unpack_from('<f', pd, 8)[0]
            err = struct.unpack_from('<f', pd, 16)[0]
            trav = struct.unpack_from('<f', pd, 36)[0]
            stt = pd[44]
            wm = struct.unpack_from('<f', lp, 8)[0] if (lp and len(lp) >= 16) else 0
            iqc = struct.unpack_from('<f', lp, 12)[0] if (lp and len(lp) >= 16) else 0
            dp = '' if last_p is None else ' (%+.4f)' % (p_now - last_p)
            last_p = p_now
            print(' %5.2fs %8.4f %8.4f %8.4f %8.4f %7.4f  %-5s %8.2f %7.4f%s'
                  % (time.time() - t0, p_now, p_tgt, p_tc, err, trav,
                     ST_NAME.get(stt, str(stt)), wm, iqc, dp))
        time.sleep(0.1)

    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.2)
    print('\n已失能')
    L.s.close()


if __name__ == '__main__':
    main()
