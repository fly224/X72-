# -*- coding: utf-8 -*-
"""Z 电角度对齐（30069 action=5）实测脚本

关注两件事：
  1. v[0]（上位机显示的"极对数"）必须是**整数**
  2. v[7]=resid（|ratio-round(ratio)|）反映测量质量；停稳修正后应显著变小

用法: python z_ident.py COM27
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

ACT_ALIGN = 5


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

    def wait(self, msgid, dur):
        t = time.time()
        while time.time() - t < dur:
            for mid, pl in self.poll(0.05):
                if mid == msgid:
                    return pl
        return None


def read_state(L):
    pl = L.wait(MSG_ST, 1.0)
    if pl is None:
        return None
    # 30090: 前面是 float 区，flags/fault 在固定偏移
    d = {}
    d['pos'] = struct.unpack_from('<f', pl, 16)[0] if len(pl) >= 20 else 0.0
    d['flags'] = pl[3] if len(pl) > 3 else 0   # 30090: flags 在 p_st[3]
    return d, pl


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    L = Link(port)
    print('端口', port, '已打开')

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.wait(MSG_PERM, 0.5)
    L.send(MSG_CW, struct.pack('<HB', 0, 0), X_CW)   # cw=0, halt=0 → 确保未使能
    L.poll(0.1)

    got = read_state(L)
    if got is None:
        print('读不到 30090，检查串口/波特率')
        return
    st, pl = got
    print('30090: flags=0x%02X  fault=%s' %
          (st['flags'], ('0x%04X' % (pl[36] | (pl[37] << 8))) if len(pl) > 37 else '?'))
    if (st['flags'] & 0x01) == 0:
        print('!! 电流零点未标定（flags.bit0=0），Z 会被拒。先做 C（action=4）')
        return

    if len(sys.argv) > 2 and sys.argv[2] == 'status':
        print('仅查状态，不触发辨识')
        L.s.close()
        return

    print('触发 Z（action=5），约 22~27 s ...')
    t0 = time.time()
    L.send(MSG_SYS, bytes([ACT_ALIGN]), X_SYS)
    ack = L.wait(MSG_SYS, 1.0)
    print('  ack status =', None if ack is None else ack[0])

    res = None
    while time.time() - t0 < 40.0:
        for mid, pl in L.poll(0.2):
            if mid == MSG_ID and len(pl) >= 40 and pl[0] == 8:
                res = pl
                break
        if res is not None:
            break
    if res is None:
        print('超时：没收到 30092 kind=8')
        return

    ok = res[1]
    v = list(struct.unpack_from('<8f', res, 4))
    print('\n30092 kind=8  ok=%d  用时 %.1f s' % (ok, time.time() - t0))
    if ok:
        print('  极对数 pp      = %d      <- 必须是整数' % int(v[0]))
        print('  dir            = %+d' % int(v[1]))
        print('  offset         = %.3f deg' % v[2])
        print('  zero           = %.3f deg' % v[3])
        print('  n_step/n_sweep = %d / %d' % (int(v[4]), int(v[5])))
        print('  n_fail         = %d' % int(v[6]))
        print('  resid          = %.3f      <- 应 < 0.40' % v[7])
    else:
        names = {1: 'BUSY', 2: 'CAL(先做C)', 3: 'FAULT(先做X)', 4: 'PWM',
                 5: 'TIMEOUT', 6: 'STUCK(没动/卡死)', 7: 'RATIO(极对数不合理)',
                 8: 'ENC', 9: 'OTHER'}
        print('  错误码 v[0] = %d (%s)' % (int(v[0]), names.get(int(v[0]), '?')))
        print('  dmech avg/min/max = %.0f / %.0f / %.0f mdeg' % (v[1], v[2], v[3]))
        print('  raw ratio         = %.3f' % v[4])
    L.s.close()


if __name__ == '__main__':
    main()
