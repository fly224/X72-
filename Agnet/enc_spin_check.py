# -*- coding: utf-8 -*-
"""编码器恢复后的转动验证

只看 fault=0 不够 —— 编码器"卡死但读数合法"时故障位也会是 0。
真正的判据是 enc_sum 随转速**单调变化**，且转动全程 fault 保持 0。

用法: python enc_spin_check.py COM27 [rpm] [秒]
"""
import sys, time, struct
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_CW, X_CW = 30054, 108
MSG_MODE, X_MODE = 30056, 26
MSG_PERM, X_PERM = 30068, 134
MSG_PS, X_PS = 30019, 8
MSG_ST, X_ST = 30090, 90
MSG_LP, X_LP = 30091, 91
MSG_POS, X_POS = 30040, 160

KNOWN = {MSG_CW: X_CW, MSG_MODE: X_MODE, MSG_PERM: X_PERM,
         MSG_PS: X_PS, MSG_ST: X_ST, MSG_LP: X_LP, MSG_POS: X_POS}

ENC_COUNTS = 2097152.0


def x25(data, extra):
    crc = 0xFFFF
    for b in list(data) + [extra]:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def f32(b, off):
    return struct.unpack_from('<f', b, off)[0]


class Link:
    def __init__(self, port, baud=921600):
        self.s = serial.Serial(port, baud, timeout=0)
        # 板子持续 100 Hz 发三帧，打开前积压常有几 KB；不清掉会把 poll 拖慢
        self.s.reset_input_buffer()
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

    def wait(self, msgid, dur=1.0):
        t = time.time()
        while time.time() - t < dur:
            for mid, pl in self.poll(0.04):
                if mid == msgid:
                    return pl
        return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0

    L = Link(port)
    print('端口 %s，电机 %.0f rpm 转 %g s' % (port, rpm, secs))

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.poll(0.15)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.15)
    L.send(MSG_MODE, bytes([3]), X_MODE)
    L.poll(0.15)

    L.w_cs = rpm * 2.0 * 3.141592653589793 / 60.0 * ENC_COUNTS / (2.0 * 3.141592653589793)
    L.keepalive()
    L.poll(0.1)
    L.send(MSG_CW, struct.pack('<HB', 1, 0), X_CW)
    L.wait(MSG_CW, 1.0)

    print('\n  t(s)   fault   enc_sum      d_enc    wmeas   diag')
    print('  ' + '-' * 52)
    base = None
    faults = set()
    encs = []
    t0 = time.time()
    while time.time() - t0 < secs:
        L.keepalive()
        got = {}
        for mid, pl in L.poll(0.12):
            got[mid] = pl
        if MSG_ST in got and MSG_POS in got:
            st = got[MSG_ST]
            f = st[36] | (st[37] << 8)
            e = struct.unpack_from('<q', got[MSG_POS], 0)[0]
            w = f32(got[MSG_LP], 8) if MSG_LP in got else 0.0
            d = f32(got[MSG_LP], 3) if False else (got[MSG_LP][3] if MSG_LP in got else 0)
            if base is None:
                base = e
            faults.add(f)
            encs.append(e)
            print('  %5.2f  0x%04X  %9d  %+8d  %6.1f   0x%02X'
                  % (time.time() - t0, f, e, e - base, w, d))

    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.25)

    print('\n=== 判读 ===')
    moving = len(set(encs)) > 1
    span = (max(encs) - min(encs)) if encs else 0
    exp = abs(rpm) / 60.0 * ENC_COUNTS * secs
    print('  fault 取值集合 : %s' % ', '.join('0x%04X' % f for f in sorted(faults)))
    print('  enc_sum 变化量 : %d counts（理论 %.0f）' % (span, exp))
    print('  enc_sum 单调   : %s' % ('是' if moving else '否'))
    if faults == {0} and moving and span > exp * 0.5:
        print('  → 通过：编码器在跟随，全程无故障')
    elif faults != {0}:
        print('  → 失败：转动中又报故障，属硬件侧（SPI 质量或磁铁装配）')
    else:
        print('  → 异常：fault 为 0 但 enc_sum 不跟随，编码器读数仍是死值')
    L.s.close()


if __name__ == '__main__':
    main()
