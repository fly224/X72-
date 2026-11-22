# -*- coding: utf-8 -*-
"""CAN FD 双节点链路测试：action 21 主发 / 22 从收 / 23 停止并取结果

单节点也可以跑（只会看到 tx 失败/TEC 涨，用来确认错误检测路径有效）。

用法:
    python can_link_test.py COM27 tx 3     # 主发 3 秒后停止并读结果
    python can_link_test.py COM27 rx 5     # 从收 5 秒后停止并读结果
    python can_link_test.py COM27 stop     # 只停止并读结果
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

ACT = {'tx': 21, 'rx': 22, 'stop': 23}

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

def show(res):
    ok = res[1]
    n = res[2] | (res[3] << 8)
    v = list(struct.unpack_from('<8f', res, 4))
    role = int(v[0])
    print('\n=== CAN 链路测试结果（%s）==='
          % ('主发' if role == 1 else '从收' if role == 2 else '未知'))
    print('ok = %d   n = %d' % (ok, n))
    if role == 1:
        print('tx_ok   =', int(v[1]))
        print('tx_fail =', int(v[2]))
    else:
        print('rx_ok   =', int(v[3]))
        print('lost    =', int(v[4]))
        print('dup     =', int(v[5]))
    print('TEC     =', int(v[6]))
    print('REC     =', int(v[7]))
    if role == 1:
        if int(v[2]) > 0 or int(v[6]) > 0:
            print('判定: FAIL —— 有帧没被 ACK（对端没接/没启动接收/线束或端接有问题）')
        else:
            print('判定: PASS —— 每一帧都被 ACK，TEC 未增长')
    else:
        if int(v[4]) == 0 and int(v[5]) == 0 and int(v[3]) > 0:
            print('判定: PASS —— 无丢帧、无重复')
        else:
            print('判定: FAIL —— 有丢帧或重复')

def main():
    args = [a for a in sys.argv[1:] if not a.startswith('-')]
    no_stop = ('-n' in sys.argv) or ('--no-stop' in sys.argv)

    port = args[0] if len(args) > 0 else 'COM27'
    cmd = args[1].lower() if len(args) > 1 else 'tx'
    secs = float(args[2]) if len(args) > 2 else 3.0

    if cmd not in ACT:
        print('用法: python can_link_test.py COM27 [tx|rx|stop] [秒] [-n]')
        return

    L = Link(port)
    print('端口', port, '已打开')

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.wait(MSG_PERM, 0.5)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.08)

    if cmd != 'stop':
        L.poll(0.05)
        L.send(MSG_SYS, bytes([ACT[cmd]]), X_SYS)
        ack = L.wait(MSG_SYS, 1.0)
        print('action=%d status = %s' % (ACT[cmd], None if ack is None else ack[0]))
        if ack is None or ack[0] != 0:
            L.s.close()
            return
        if no_stop:
            print('已启动（不停止），固件会持续 120 s 或直到 action=23')
            print('现在可以拔掉这根串口线，去操作另一个节点')
            L.s.close()
            return
        print('已启动，持续 %g s ...' % secs)
        time.sleep(secs)

    L.poll(0.05)
    L.send(MSG_SYS, bytes([ACT['stop']]), X_SYS)
    ack = L.wait(MSG_SYS, 1.0)
    print('action=23 status = %s' % (None if ack is None else ack[0]))

    res = None
    t0 = time.time()
    while time.time() - t0 < 3.0:
        pl = L.wait(MSG_ID, 0.2)
        if pl and pl[0] == 11:
            res = pl
            break

    if res is None:
        print('!! 没收到 30092 kind=11')
    else:
        show(res)

    st = L.wait(MSG_ST, 0.5)
    if st:
        print('30090: fault=0x%04X mode=%d flags=0x%02X'
              % (st[36] | (st[37] << 8), st[2], st[3]))
    L.s.close()

if __name__ == '__main__':
    main()
