# -*- coding: utf-8 -*-
"""抽读 30093 齿槽表：表头 + 前两块数据，确认表非全零

坑：30093 的 meta len = 48，固件按 meta->len 严格校验，
请求必须发满 48 B（前 2 字节为 op / 块索引），发 2 B 会被当坏帧静默丢弃。
"""
import sys, time, struct
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')
MSG_COG, X_COG = 30093, 93
MSG_ST, X_ST = 30090, 90
KNOWN = {MSG_COG: X_COG, MSG_ST: X_ST, 30040: 160, 30091: 91}


def x25(data, extra):
    crc = 0xFFFF
    for b in list(data) + [extra]:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
s = serial.Serial(port, 921600, timeout=0)
seq = 0
buf = bytearray()


def send(msgid, payload, extra):
    global seq
    hdr = bytes([len(payload) & 0xFF, 0, 0, seq & 0xFF, 1, 1,
                 msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    body = hdr + payload
    c = x25(body, extra)
    s.write(b'\xFD' + body + bytes([c & 0xFF, c >> 8]))
    seq += 1


def wait_for(msgid, dur=1.2):
    global buf
    t = time.time()
    while time.time() - t < dur:
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
            if ex is None:
                i += 1
                continue
            body = bytes(buf[i+1:i+10+ln])
            crc = buf[i+10+ln] | (buf[i+11+ln] << 8)
            if x25(body, ex) != crc:
                i += 1
                continue
            pl = bytes(buf[i+10:i+10+ln])
            i += 12 + ln
            if mid == msgid:
                del buf[:i]
                return pl
        del buf[:i]
        time.sleep(0.002)
    return None


# 表头
send(MSG_COG, bytes([0, 0]) + bytes(46), X_COG)
h = wait_for(MSG_COG)
if h is None:
    print('表头读取失败')
    s.close()
    sys.exit(1)

n_blk = h[2]
flags = h[3]
pp = struct.unpack_from('<f', h, 4)[0]
phase = struct.unpack_from('<f', h, 8)[0]
zdev = struct.unpack_from('<f', h, 12)[0]
print('=== 30093 表头 ===')
print('  块数 %d（%d 箱）  valid=%d aligned=%d'
      % (n_blk, n_blk * 8, flags & 1, (flags >> 1) & 1))
print('  pp=%.3f A   phase=%.2f deg   zero_dev=%.3f deg' % (pp, phase, zdev))

print('\n=== 前两块数据（mA → A）===')
peak = 0.0
for idx in (0, 1, n_blk // 2):
    send(MSG_COG, bytes([1, idx]) + bytes(46), X_COG)
    r = wait_for(MSG_COG)
    if r is None:
        print('  块 %d 读取失败' % idx)
        continue
    if r[1] == 0xFF:
        print('  块 %d 越界（表只有 %d 块）' % (idx, n_blk))
        continue
    vals = list(struct.unpack_from('<8f', r, 16))
    peak = max(peak, max(abs(v) for v in vals))
    print('  块 %2d: %s' % (idx, ' '.join('%+.3f' % v for v in vals)))

print('\n判定：%s' % ('表有效且非全零' if peak > 1e-4 else '！表看起来全零'))
s.close()
