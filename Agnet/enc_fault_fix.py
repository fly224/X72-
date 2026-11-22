# -*- coding: utf-8 -*-
"""编码器故障（0x0002）诊断与恢复

判据来自 bsp_servo.c:874-907 与 BspServo_GetEncDiag 的位定义：
  b0..2  MT6835 的 3 个状态位（帧内容坏）
  b3..5  连续错误计数（饱和 7，阈值 5 即置故障）
  b6..7  连续跳变计数（饱和 3，阈值 3 即置故障）

诊断思路：0x0002 + enc_sum 恒定 + elec_angle 冻结 = SPI 状态机卡死（非控制参数问题）。
这类卡死清故障会立即复发，必须软复位（30069 action=2）。

用法:
  python enc_fault_fix.py COM27          只诊断 + 清故障
  python enc_fault_fix.py COM27 reset    诊断 + 清故障，若复发则软复位再清
"""
import sys, time, struct
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_CW, X_CW = 30054, 108
MSG_PERM, X_PERM = 30068, 134
MSG_SYS, X_SYS = 30069, 169
MSG_ST, X_ST = 30090, 90
MSG_LP, X_LP = 30091, 91
MSG_POS, X_POS = 30040, 160

KNOWN = {MSG_CW: X_CW, MSG_PERM: X_PERM, MSG_SYS: X_SYS,
         MSG_ST: X_ST, MSG_LP: X_LP, MSG_POS: X_POS}

ACT_RESET = 2
ACT_CLEAR_FAULT = 3

FAULT_BITS = [(0, 'OC 过流'), (1, 'ENCODER 编码器'), (2, 'ADC_SYNC'), (3, 'BSP_INIT'),
              (4, 'SPI'), (5, 'OVERSPEED'), (6, 'OVERVOLT'), (7, 'UNDERVOLT'),
              (8, 'FOLLOW 跟随'), (9, 'PHASE_LOSS 缺相'), (10, 'CUR_SENSE 电流采样')]


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

    def wait(self, msgid, dur=1.0):
        t = time.time()
        while time.time() - t < dur:
            for mid, pl in self.poll(0.04):
                if mid == msgid:
                    return pl
        return None

    def snap(self, want=(MSG_ST, MSG_LP, MSG_POS), dur=1.0):
        got = {}
        t = time.time()
        while time.time() - t < dur and len(got) < len(want):
            for mid, pl in self.poll(0.04):
                if mid in want:
                    got[mid] = pl
        return got


def decode_enc_diag(v):
    status = v & 0x07
    err = (v >> 3) & 0x07
    jump = (v >> 6) & 0x03
    return status, err, jump


def report(g, tag):
    print('\n--- %s ---' % tag)
    if MSG_ST not in g:
        print('  读不到 30090')
        return None
    st = g[MSG_ST]
    fault = st[36] | (st[37] << 8)
    flags = st[3]
    names = [n for b, n in FAULT_BITS if fault & (1 << b)]
    print('  fault = 0x%04X  %s' % (fault, ' / '.join(names) if names else '无'))
    print('  flags = 0x%02X（CAL=%d ZERO=%d COG=%d）'
          % (flags, flags & 1, (flags >> 1) & 1, (flags >> 7) & 1))
    print('  mode=%d  iq=%.3f A' % (st[2], f32(st, 4)))

    if MSG_LP in g:
        lp = g[MSG_LP]
        d = lp[3]
        status, err, jump = decode_enc_diag(d)
        print('  编码器诊断 0x%02X → status=0x%X  连续错误=%d  连续跳变=%d'
              % (d, status, err, jump))
        if status:
            print('    ！status≠0：帧内容坏（SPI 质量 / MT6835 状态位）')
        if err >= 3:
            print('    ！连续错误计数高：正在持续读坏帧')
        if jump:
            print('    ！存在跳变计数：角度连续性检查触发过')

    if MSG_POS in g:
        enc = struct.unpack_from('<q', g[MSG_POS], 0)[0]
        el = f32(g[MSG_POS], 8)
        print('  enc_sum=%d  elec_angle=%.3f rad' % (enc, el))
        return {'fault': fault, 'flags': flags, 'enc': enc, 'elec': el,
                'diag': g[MSG_LP][3] if MSG_LP in g else 0}
    return {'fault': fault, 'flags': flags, 'enc': None, 'elec': None,
            'diag': g[MSG_LP][3] if MSG_LP in g else 0}


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    do_reset = len(sys.argv) > 2 and sys.argv[2] == 'reset'

    L = Link(port)
    print('端口 %s 已打开' % port)

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.poll(0.15)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)     # 确保封锁
    L.poll(0.15)

    # 连读 3 次 enc_sum 判是否卡死
    seq = []
    for i in range(3):
        g = L.snap((MSG_ST, MSG_POS), 0.8)
        if MSG_POS in g:
            seq.append(struct.unpack_from('<q', g[MSG_POS], 0)[0])
        if i < 2:
            time.sleep(0.25)

    a = report(L.snap(), '当前状态')
    if a is None:
        L.s.close()
        return

    print('\n  enc_sum 三次采样：%s' % (seq if seq else '读不到'))
    frozen = (len(seq) == 3 and seq[0] == seq[1] == seq[2])
    if frozen:
        print('  ！三次完全相同 → 编码器读数疑似卡死（静止时会正常重复，需轻转电机确认）')

    if a['fault'] == 0:
        print('\n当前无故障，无需处理')
        L.s.close()
        return

    print('\n清故障（30069 action=3）...')
    L.send(MSG_SYS, bytes([ACT_CLEAR_FAULT]), X_SYS)
    ack = L.wait(MSG_SYS, 1.2)
    print('  ack status = %s  (0=成功 3=需先封锁)' % (None if ack is None else ack[0]))
    time.sleep(0.3)
    b = report(L.snap(), '清故障后')

    if b and b['fault'] == 0:
        print('\n结论：清故障后已恢复，未立即复发')
        print('  → 若是偶发，观察是否再现；若再现请再跑本脚本加 reset')
        L.s.close()
        return

    print('\n！清故障后立即复发（0x%04X）' % (b['fault'] if b else -1))
    if not do_reset:
        print('  下一步：python enc_fault_fix.py %s reset  （软复位，保留标定）' % port)
        L.s.close()
        return

    print('\n软复位（30069 action=2），标定与齿槽表保留 ...')
    L.send(MSG_SYS, bytes([ACT_RESET]), X_SYS)
    L.wait(MSG_SYS, 1.0)
    L.s.close()
    time.sleep(2.5)

    L2 = Link(port)
    L2.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L2.poll(0.15)
    c = report(L2.snap(), '软复位后')
    if c:
        if c['fault'] == 0 and (c['flags'] & 0x03) == 0x03:
            print('\n结论：复位后无故障，CAL/ZERO 保留 → 属于 SPI 状态机卡死，已恢复')
        elif c['fault'] == 0:
            print('\n结论：复位后无故障，但 CAL/ZERO 不全（flags=0x%02X）→ 需重做标定' % c['flags'])
        else:
            print('\n结论：复位后仍报 0x%04X → 不是状态机卡死，按硬件方向排查：' % c['fault'])
            print('       1) MT6835 供电与 SPI 线（SCK/MISO/CS）是否有松动/干扰')
            print('       2) 磁铁是否偏心或松动（跳变计数高指向这个）')
            print('       3) 30091 的编码器诊断位：status≠0 → SPI；jump≠0 → 磁铁/装配')
    L2.s.close()


if __name__ == '__main__':
    main()
