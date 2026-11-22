# -*- coding: utf-8 -*-
"""读节点指纹，用于区分 A/B 并贴标签。

指纹取 30090 的 theta（编码器当前角）+ vbus + 温度。
两个模组停在不同位置，theta 到 0.001 rad 就足够区分；
万一巧合相同，再看温度和母线。

用法: python can_node_id.py COM27
"""
import sys, time, struct
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_ST, X_ST = 30090, 90

def x25(data, extra):
    crc = 0xFFFF
    for b in list(data) + [extra]:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    s = serial.Serial(port, 921600, timeout=0)
    buf = bytearray()
    st = None
    t0 = time.time()
    while time.time() - t0 < 1.5 and st is None:
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
            if mid != MSG_ST or ln != 40:
                i += 12 + ln
                continue
            body = bytes(buf[i+1:i+10+ln])
            crc = buf[i+10+ln] | (buf[i+11+ln] << 8)
            if x25(body, X_ST) != crc:
                i += 1
                continue
            st = bytes(buf[i+10:i+10+ln])
            break
        del buf[:i]
        time.sleep(0.002)
    s.close()

    if st is None:
        print('!! 没收到 30090')
        return

    theta = struct.unpack_from('<f', st, 16)[0]
    vbus  = struct.unpack_from('<f', st, 20)[0]
    tm    = struct.unpack_from('<f', st, 24)[0]
    tg    = struct.unpack_from('<f', st, 28)[0]
    rpm   = struct.unpack_from('<f', st, 12)[0]
    fault = st[36] | (st[37] << 8)

    print('--- 节点指纹 ---')
    print('theta      = %.4f rad' % theta)
    print('vbus       = %.2f V' % vbus)
    print('temp_motor = %.1f C' % tm)
    print('temp_gan   = %.1f C' % tg)
    print('rpm        = %.1f' % rpm)
    print('fault      = 0x%04X' % fault)
    print()
    print('指纹串: %.3f | %.2f | %.1f' % (theta, vbus, tm))

if __name__ == '__main__':
    main()
