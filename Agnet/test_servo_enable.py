# -*- coding: utf-8 -*-
"""速度模式一键测试：权限 → 选模式 → 下发转速 → 使能 → 保活 → 读回 → 失能

目的：验证"上位机下发指令电机没反应"是页面问题还是固件问题。
      脚本直连串口，与上位机页面走同一条物理通道、同一套 MAVLink 协议。

用法: python test_servo_enable.py COM27 [rpm]
  rpm 默认 50（电机侧）。电机必须空载可自由转动。
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
    def __init__(self, port, baud=921600):
        self.s = serial.Serial(port, baud, timeout=0)
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

    def wait(self, msgid, dur, keep=False):
        t = time.time()
        while time.time() - t < dur:
            if keep:
                self.keepalive()
            for mid, pl in self.poll(0.04):
                if mid == msgid:
                    return pl
        return None


def read_state(L):
    """读 30090 + 30091，返回关键字段 dict"""
    st = L.wait(MSG_ST, 0.6)
    lp = L.wait(MSG_LP, 0.6)
    out = {}
    if st is not None and len(st) >= 40:
        out['fault'] = struct.unpack_from('<H', st, 36)[0]   # fault @ offset 36
        out['flags'] = st[3]
        out['mode'] = st[2]
    if lp is not None and len(lp) >= 32:
        out['wmeas'] = struct.unpack_from('<f', lp, 8)[0]
        out['wref'] = struct.unpack_from('<f', lp, 4)[0]
        out['iqc'] = struct.unpack_from('<f', lp, 12)[0]
        out['spd_kp'] = struct.unpack_from('<f', lp, 20)[0]
    return out


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0

    L = Link(port)
    print('端口 %s  目标 %.0f rpm（电机侧，空载）' % (port, rpm))
    print('=' * 60)

    # 1. 清空积压缓冲，读初始状态
    time.sleep(0.3)
    L.poll(0.3)
    s0 = read_state(L)
    print('[初始状态] fault=0x%04X flags=0x%02X mode=%s wmeas=%.1f rpm'
          % (s0.get('fault', 0), s0.get('flags', 0), s0.get('mode', '?'),
             s0.get('wmeas', 0.0)))

    if s0.get('fault', 0xFFFF) != 0:
        print('!! 有故障，先清故障（30069 action=3）')
        # 这里不自动清，提示即可
        print('   请先处理故障再测。')
        L.s.close()
        return

    # 2. 权限（确保 comm_auth=0 有权限）
    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.poll(0.1)

    # 3. 先封锁，确保干净起点
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.15)
    print('[封锁] cw=0 halt=1 已下发')

    # 4. 选模式 VEL=3
    L.send(MSG_MODE, bytes([3]), X_MODE)
    L.poll(0.15)
    print('[选模式] 30056 mode=3 (VEL)')

    # 5. 下发转速给定（未使能，先写好目标）
    L.w_cs = rpm * ENC_COUNTS / 60.0
    L.keepalive()
    L.poll(0.1)
    print('[下发给定] 30019 speed=%.0f counts/s (=%.0f rpm)' % (L.w_cs, rpm))

    # 6. 使能
    L.send(MSG_CW, struct.pack('<HB', 1, 0), X_CW)
    ack = L.wait(MSG_CW, 1.0, keep=True)
    print('[使能] cw=1 halt=0, 应答 status=%s'
          % ('?' if ack is None else ack[0]))

    # 7. 保活 3 秒，读转速
    print('[保活] 持续发 30019，3 秒 ...')
    t = time.time()
    rows = []
    while time.time() - t < 3.0:
        L.keepalive()
        st = read_state(L)
        rows.append((time.time() - t, st.get('wmeas', 0.0), st.get('iqc', 0.0)))
        time.sleep(0.4)

    print('  时间    wmeas(rpm)   iqc(A)')
    for dt, w, iq in rows:
        print('  %5.1fs   %7.1f    %6.3f' % (dt, w, iq))

    final = read_state(L)
    w_final = final.get('wmeas', 0.0)

    # 8. 失能
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    L.poll(0.2)
    print('[失能] cw=0 halt=1 已下发')

    # 9. 判读
    print('=' * 60)
    print('最终 wmeas = %.1f rpm' % w_final)
    if abs(w_final) > rpm * 0.7:
        print('结论：✓ 固件正常响应，电机已转动 → "没反应"是上位机页面问题')
    elif abs(w_final) > 1.0:
        print('结论：△ 电机在转但转速异常（%.1f vs %.0f）→ 查速度环/限幅' % (w_final, rpm))
    else:
        print('结论：✗ 电机没转 → 固件/硬件问题，需进一步查')
        if final.get('fault', 0) != 0:
            print('      fault=0x%04X（使能后出现故障）' % final['fault'])

    L.s.close()


if __name__ == '__main__':
    main()
