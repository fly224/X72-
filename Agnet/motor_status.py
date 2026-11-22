# -*- coding: utf-8 -*-
"""电机当前状态总览：解析 30090 / 30091 / 30040，打印人看的结论。

用法: python motor_status.py COM27
"""
import sys, time, struct
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_POS, X_POS = 30040, 160
MSG_ST,  X_ST  = 30090, 90
MSG_LP,  X_LP  = 30091, 91
KNOWN = {MSG_POS: X_POS, MSG_ST: X_ST, MSG_LP: X_LP}

MODE_NAME = {0: 'IDLE', 1: 'POS', 3: 'VEL', 4: 'TORQUE', 12: 'MIT', 64: 'IMP'}
SRC_NAME = {0: 'NONE', 1: 'TORQUE', 2: 'SPEED', 3: 'POS', 4: 'MIT', 5: 'VOLT', 6: 'OL'}
FLAG_BITS = [(0, 'CAL'), (1, 'ZERO'), (2, 'ARMED'), (3, 'FF'),
             (4, 'DEADBAND'), (5, 'HARM'), (6, 'KTCOMP'), (7, 'COG')]
FAULT_BITS = [(0, 'OC'), (1, 'ENCODER'), (2, 'OVERVOLT'), (3, 'UNDERVOLT'),
              (4, 'OVERTEMP'), (5, 'OVERSPEED'), (6, 'FOLLOW_ERR'),
              (10, 'CUR_SENSE')]


def x25(data, extra):
    crc = 0xFFFF
    for b in list(data) + [extra]:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def f32(b, off):
    return struct.unpack_from('<f', b, off)[0]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    s = serial.Serial(port, 921600, timeout=0)
    buf = bytearray()
    got = {}
    t0 = time.time()

    while time.time() - t0 < 2.5 and len(got) < 3:
        d = s.read(s.in_waiting or 1)
        if d:
            buf += d
        i, n = 0, len(buf)
        while i + 12 <= n:
            if buf[i] != 0xFD:
                i += 1
                continue
            ln = buf[i + 1]
            if i + 12 + ln > n:
                break
            mid = buf[i+7] | (buf[i+8] << 8) | (buf[i+9] << 16)
            ex = KNOWN.get(mid)
            if ex is None or ln != {MSG_POS: 12, MSG_ST: 40, MSG_LP: 48}[mid]:
                i += 12 + ln
                continue
            body = bytes(buf[i+1:i+10+ln])
            crc = buf[i+10+ln] | (buf[i+11+ln] << 8)
            if x25(body, ex) != crc:
                i += 1
                continue
            got[mid] = bytes(buf[i+10:i+10+ln])
            i += 12 + ln
        del buf[:i]
        time.sleep(0.002)
    s.close()

    if MSG_ST not in got:
        print('读不到 30090，检查串口/波特率/是否被 Keil 占用')
        return

    st = got[MSG_ST]
    cw = st[0] | (st[1] << 8)
    mode = st[2]
    flags = st[3]
    iq = f32(st, 4)
    iq_ref = f32(st, 8)
    rpm = f32(st, 12)
    pos = f32(st, 16)
    vbus = f32(st, 20)
    t_mot = f32(st, 24)
    t_gan = f32(st, 28)
    ktc = f32(st, 32)
    fault = st[36] | (st[37] << 8)
    drop = st[38] | (st[39] << 8)

    print('=== 30090 控制状态 ===')
    print('  控制字     0x%04X   (使能=%d halt=%d)' % (cw, cw & 1, (cw >> 2) & 1))
    print('  模式       %d (%s)' % (mode, MODE_NAME.get(mode, '?')))
    fl = [nm for b, nm in FLAG_BITS if flags & (1 << b)]
    print('  标志       0x%02X  %s' % (flags, ' '.join(fl) if fl else '-'))
    print('  iq 实测/给定  %.3f / %.3f A' % (iq, iq_ref))
    print('  转速       %.1f rpm' % rpm)
    print('  位置       %.4f rad  (%.1f deg)' % (pos, pos * 57.2957795))
    print('  母线电压   %.2f V' % vbus)
    print('  温度       电机 %.1f / 驱动 %.1f °C' % (t_mot, t_gan))
    print('  Kt 补偿    %.4f' % ktc)
    if fault:
        fb = [nm for b, nm in FAULT_BITS if fault & (1 << b)]
        print('  故障       0x%04X  %s' % (fault, ' '.join(fb) if fb else '未命名位'))
    else:
        print('  故障       无')
    print('  发送丢弃   %d' % drop)

    if MSG_LP in got:
        lp = got[MSG_LP]
        print('\n=== 30091 环路状态 ===')
        print('  速度模式/源 %d / %d (%s)' % (lp[0], lp[1], SRC_NAME.get(lp[1], '?')))
        print('  增益调度   %d      编码器诊断 0x%02X' % (lp[2], lp[3]))
        print('  wref/wmeas %.1f / %.1f rpm' % (f32(lp, 4), f32(lp, 8)))
        print('  iqc/iterm  %.3f / %.3f A' % (f32(lp, 12), f32(lp, 16)))
        print('  速度 kp    %.4f    iqmax %.2f A' % (f32(lp, 20), f32(lp, 28)))
        print('  加速度     %.1f rad/s²' % f32(lp, 24))
        print('  电角度偏置 %.2f deg' % f32(lp, 32))
        print('  磁链       %.3f mWb' % f32(lp, 36))
        print('  过流现场   峰值 %.2f A / 原始码 %d' % (f32(lp, 40), int(f32(lp, 44))))

    if MSG_POS in got:
        p = got[MSG_POS]
        enc = struct.unpack_from('<q', p, 0)[0]
        el = f32(p, 8)
        print('\n=== 30040 位置传感器 ===')
        print('  enc_sum    %d counts  (%.4f 圈)' % (enc, enc / 2097152.0))
        print('  电角度     %.4f rad  (%.1f deg)' % (el, el * 57.2957795))

    print('\n=== 判读 ===')
    if fault:
        print('  ！有故障：先清故障（30069 action=3）再动作')
    if not (flags & 1):
        print('  ！电流零点未标定：先做 C（action=4），否则力矩方向不可信')
    if not (flags & 2):
        print('  ！电角度零点未标定：先做 Z（action=5）')
    if not (cw & 1):
        print('  当前未使能（正常待机状态）')
    if not fault and (flags & 3) == 3:
        print('  标定齐全、无故障 → 可以起环')


if __name__ == '__main__':
    main()
