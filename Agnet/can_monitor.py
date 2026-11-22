# -*- coding: utf-8 -*-
"""CAN 链路监控：持续读 30092 kind=11 的周期上报（1 s 一次）

字段口径（固件 can_proto.c can_push_periodic 第 2 版）：
  v8[0]=role  v8[1]=rx_ok  v8[2]=lost  v8[3]=dup+resync  v8[4]=uptime_s
  v8[5]=TEC*1000+REC      v8[6]=PSR 包（LEC + DLEC*16 + EP*256 + BO*512）
  v8[7]=tx_ok             resid=tx_fail

看这几个数：
  uptime      连续递增 —— 归零说明节点重启
  d_rx/s      接近 1000 说明 1 kHz 正常；几十说明总线上只有这么多帧
  d_tx/s      入队速率（≈1000 只代表"塞进 FIFO"，不代表上了总线）
  tec / lec   发送侧才是关键：TEC 不涨、LEC=7 说明每帧都被 ACK
  bo          Bus-off 标志置位说明 TEC 曾打到 255

用法: python can_monitor.py COM27 20
"""
import sys, time, struct
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_ID, X_ID = 30092, 92

LEC_NAME = {0: '无错', 1: '位填充', 2: '格式', 3: 'ACK', 4: '位',
            5: 'CRC', 6: '显性序列', 7: '无'}


def x25(data, extra):
    crc = 0xFFFF
    for b in list(data) + [extra]:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

    s = serial.Serial(port, 921600, timeout=0)
    buf = bytearray()
    last_up = None
    last_rx = None
    last_tx = None
    t0 = time.time()

    print('监控 %g s（Ctrl-C 退出）' % dur)
    print(' uptime rol  rx_ok d_rx/s   tx_ok d_tx/s  lost rsync  tec  rec  lec      bo# txfail  zc')
    print('-' * 86)

    while time.time() - t0 < dur:
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
            if mid != MSG_ID or ln != 40:
                i += 12 + ln
                continue
            body = bytes(buf[i+1:i+10+ln])
            crc = buf[i+10+ln] | (buf[i+11+ln] << 8)
            if x25(body, X_ID) != crc:
                i += 1
                continue
            pl = bytes(buf[i+10:i+10+ln])
            i += 12 + ln

            if pl[0] == 12:
                r = list(struct.unpack_from('<8f', pl, 4))
                print('   [REG] TXBC=0x%08X TXFQS=0x%08X TXBRP=0x%08X TXBTO=0x%08X'
                      % (int(r[0]), int(r[1]), int(r[2]), int(r[3])))
                print('         PSR =0x%08X ECR  =0x%08X CCCR =0x%08X  free=%d TFQF=%d pend=%d rc=%d'
                      % (int(r[4]), int(r[5]), int(r[6]),
                         int(r[1]) & 0x7, (int(r[1]) >> 21) & 1, int(r[2]) & 0x7,
                         int(r[7])))
                continue
            if pl[0] != 11:
                continue
            v = list(struct.unpack_from('<8f', pl, 4))
            txfail = struct.unpack_from('<f', pl, 36)[0]
            up = int(v[4])
            if up == last_up:
                continue
            rx = int(v[1])
            tx = int(v[7])
            drx = 0 if last_rx is None else rx - last_rx
            dtx = 0 if last_tx is None else tx - last_tx
            tec = int(v[5]) // 1000
            rec = int(v[5]) % 1000
            psr = int(v[6])
            zc = psr // 1000            # 对端重启次数（收到 seq==0 的次数）
            rest = psr % 1000
            lec = rest & 0xF
            bo = rest >> 4
            print(' %6d %3d  %6d %6d   %6d %6d  %4d %5d  %3d  %3d  %-8s %2d %6d  %3d'
                  % (up, int(v[0]), rx, drx, tx, dtx, int(v[2]), int(v[3]),
                     tec, rec, LEC_NAME.get(lec, '?'), bo, int(txfail), zc))
            last_up, last_rx, last_tx = up, rx, tx
        del buf[:i]
        time.sleep(0.002)

    s.close()
    print('-' * 86)
    print('判读:  d_rx/s≈1000 且 tec=0/lec=无  → 双向 1 kHz 干净')
    print('       d_tx/s≈1000 但 d_rx/s 很小  → 帧没上总线，看 lec')
    print('       lec=ACK                      → 总线上没人应答（对端没上电/没 Start）')
    print('       lec=位填充/CRC               → 位定时或 5M 信号完整性')
    print('       bo=1                         → 曾进 Bus-off，TEC 打到 255')


if __name__ == '__main__':
    main()
