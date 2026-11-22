# -*- coding: utf-8 -*-
"""MIT 上刚度修复验证（2026-11-22）

验证两件事：
  A. 起环(p=0,kp=0) 后**直接**发 30066 上刚度，不再触发 0x0020
     —— 这是旧固件必炸的路径（用了缓存的 p=0 刷新给定）
  B. 完整序列（先 p=p_now 贴住，再上刚度）能正常跑小幅正弦

用法: python mit_fix_verify.py COM27
"""
import sys, time, struct, math
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

ENC    = 2097152.0
TWO_PI = 6.283185307179586
K_CNT  = ENC / TWO_PI

MSG_PERM, X_PERM = 30068, 134
MSG_MODE, X_MODE = 30056, 26
MSG_CW,   X_CW   = 30054, 108
MSG_SYS,  X_SYS  = 30069, 169
MSG_MIT,  X_MIT  = 30065, 17
MSG_CFG,  X_CFG  = 30066, 147
MSG_ST,   X_ST   = 30090, 90
MSG_LP,   X_LP   = 30091, 91

KNOWN = {MSG_PERM: X_PERM, MSG_MODE: X_MODE, MSG_CW: X_CW, MSG_SYS: X_SYS,
         MSG_MIT: X_MIT, MSG_CFG: X_CFG, MSG_ST: X_ST, MSG_LP: X_LP}

def x25(data, extra):
    """MAVLink X25：crc_extra 不是简单异或，要再走一遍累加器
    （第一版写成 (crc ^ extra) 导致所有帧 CRC 判错、一帧都收不到）。"""
    crc = 0xFFFF
    for b in list(data) + [extra]:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

class Link:
    def __init__(self, port, baud=921600):
        # timeout=0 + 按 in_waiting 读：非阻塞。第一版用 timeout=0.02 且
        # in_waiting=0 时只读 1 字节，21 B 的 ack 要 420 ms 才凑齐 → 全超时。
        self.s = serial.Serial(port, baud, timeout=0)
        self.seq = 0
        self.buf = bytearray()

    def send(self, msgid, payload, crc_extra):
        hdr = bytes([len(payload) & 0xFF, 0, 0, self.seq & 0xFF, 1, 1,
                     msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
        body = hdr + payload
        c = x25(body, crc_extra)
        self.s.write(b'\xFD' + body + bytes([c & 0xFF, c >> 8]))
        self.seq += 1

    def _parse(self):
        out = []
        i = 0
        n = len(self.buf)
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

    def drain(self):
        d = self.s.read(self.s.in_waiting)
        if d:
            self.buf += d
        return self._parse()

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

    def wait(self, msgid, dur=0.5):
        t = time.time()
        while time.time() - t < dur:
            for mid, pl in self.poll(0.02):
                if mid == msgid:
                    return pl
        return None

    def close(self):
        self.s.close()

def mit_pack(p, v=0.0, tau=0.0):
    return struct.pack('<qqffff', int(round(p * K_CNT)), 0,
                       float(tau), 0.0, float(v * K_CNT), 0.0)

def cfg_pack(kp, kd, kt=0.091):
    return struct.pack('<ffff', float(kp), float(kd), 0.0, float(kt))

def st_read(pl):
    return {
        'cw':   pl[0] | (pl[1] << 8),
        'mode': pl[2],
        'flags': pl[3],
        'iq':   struct.unpack_from('<f', pl, 4)[0],
        'iqr':  struct.unpack_from('<f', pl, 8)[0],
        'rpm':  struct.unpack_from('<f', pl, 12)[0],
        'fault': pl[36] | (pl[37] << 8),
    }

def read_st(L, tries=10):
    for _ in range(tries):
        pl = L.wait(MSG_ST, 0.12)
        if pl:
            return st_read(pl)
    return None

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    L = Link(port)
    print('端口', port, '已打开')
    time.sleep(0.3)
    L.drain()

    # 0) 封锁 + 清故障
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW); L.poll(0.06)
    L.send(MSG_SYS, bytes([3]), X_SYS); L.poll(0.06)
    time.sleep(0.2)

    # 1) 权限 + 模式
    ack = None
    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    ack = L.wait(MSG_PERM, 0.4)
    print('30068 权限 status =', None if ack is None else ack[0])
    L.send(MSG_MODE, bytes([12]), X_MODE)
    ack = L.wait(MSG_MODE, 0.4)
    print('30056 mode=12 status =', None if ack is None else ack[0])

    # 2) 状态自检
    st = read_st(L)
    if not st:
        print('!! 收不到 30090'); L.close(); return
    print('30090: fault=0x%04X mode=%d flags=0x%02X (CAL=%d ZERO=%d) iq=%.3f rpm=%.1f'
          % (st['fault'], st['mode'], st['flags'],
             st['flags'] & 1, (st['flags'] >> 1) & 1, st['iq'], st['rpm']))
    if (st['flags'] & 3) != 3:
        print('!! CAL/ZERO 不全（烧录可能擦了参数页），请先做 C 与 Z')
        L.close(); return

    # 3) kp=0
    L.send(MSG_CFG, cfg_pack(0.0, 0.05), X_CFG)
    ack = L.wait(MSG_CFG, 0.4)
    print('30066 kp=0 status =', None if ack is None else ack[0])

    # 4) 起环 p=0
    L.send(MSG_MIT, mit_pack(0.0), X_MIT)
    ack = L.wait(MSG_MIT, 0.5)
    if ack is None:
        print('!! 起环无应答'); L.close(); return
    p_des, p_now = struct.unpack_from('<f', ack, 1)[0], struct.unpack_from('<f', ack, 5)[0]
    print('30065 起环 status=%d  p_des=%.4f  p_now=%.4f' % (ack[0], p_des, p_now))
    if ack[0] != 0:
        print('!! 起环被拒，检查 fault/CAL/ZERO'); L.close(); return
    p_hold = p_now

    # 5) 【A】跳过"贴当前角"，直接上刚度
    print('\n--- A: 跳过 p=p_now，直接上刚度 kp=30（旧固件必炸路径）---')
    L.send(MSG_CFG, cfg_pack(30.0, 0.05), X_CFG)
    ack = L.wait(MSG_CFG, 0.4)
    print('30066 kp=30 status =', None if ack is None else ack[0])
    time.sleep(0.25)
    st = read_st(L)
    ok_a = False
    if st:
        print('等 250 ms: fault=0x%04X mode=%d iq=%.3f iq_ref=%.3f rpm=%.1f'
              % (st['fault'], st['mode'], st['iq'], st['iqr'], st['rpm']))
        if st['fault'] & 0x20:
            print('!! 仍然 0x0020 —— 修复未生效')
        elif st['fault'] == 0:
            print('OK: 未触发超速（旧固件这里必然 0x0020）')
            ok_a = True
    else:
        print('!! 收不到 30090')

    if not ok_a:
        L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW); L.poll(0.1)
        L.close(); return

    # 6) 贴回当前角（A 段 kp 只在 AppParam 缓存里，Servo 层仍是 0，电机应没动）
    L.send(MSG_MIT, mit_pack(p_hold), X_MIT)
    ack = L.wait(MSG_MIT, 0.4)
    print('30065 贴 p_now status =', None if ack is None else ack[0])
    if ack:
        print('  回读 p_des=%.4f p_now=%.4f'
              % (struct.unpack_from('<f', ack, 1)[0], struct.unpack_from('<f', ack, 5)[0]))

    # 7) 【B】小幅正弦
    print('\n--- B: 小幅正弦 0.02 rad 0.5 Hz 1.5 s @200 Hz ---')
    amp, f, dur, rate = 0.02, 0.5, 1.5, 200.0
    w = TWO_PI * f
    t0 = time.time()
    nxt = t0
    max_iq, max_rpm, faults, frames = 0.0, 0.0, set(), 0
    while time.time() - t0 < dur:
        t = time.time() - t0
        L.send(MSG_MIT, mit_pack(p_hold + amp * math.sin(w * t),
                                 amp * w * math.cos(w * t)), X_MIT)
        frames += 1
        for mid, pl in L.drain():
            if mid == MSG_ST:
                d = st_read(pl)
                max_iq = max(max_iq, abs(d['iq']))
                max_rpm = max(max_rpm, abs(d['rpm']))
                if d['fault']:
                    faults.add(d['fault'])
        nxt += 1.0 / rate
        dt = nxt - time.time()
        if dt > 0:
            time.sleep(dt)
    print('发帧 %d (%.0f Hz)，|iq|峰=%.3f A，|rpm|峰=%.1f，故障=%s'
          % (frames, frames / dur, max_iq, max_rpm,
             ','.join('0x%04X' % x for x in faults) if faults else '无'))

    # 8) 【C】3.09 rad 大阶跃 —— 10:59 那次 0x0020 的复现场景
    print('\n--- C: 3.09 rad 阶跃（10:59 事故场景），目标整形应把它变成斜坡 ---')
    p_from = p_hold
    p_to   = p_hold + 3.09
    t0 = time.time()
    dur_c = 2.0
    max_iq_c, max_rpm_c, faults_c = 0.0, 0.0, set()
    p_hist = []
    while time.time() - t0 < dur_c:
        L.send(MSG_MIT, mit_pack(p_to, 0.0), X_MIT)
        for mid, pl in L.drain():
            if mid == MSG_ST:
                d = st_read(pl)
                max_iq_c = max(max_iq_c, abs(d['iq']))
                max_rpm_c = max(max_rpm_c, abs(d['rpm']))
                if d['fault']:
                    faults_c.add(d['fault'])
            elif mid == MSG_MIT and pl[0] == 0:
                p_hist.append(struct.unpack_from('<f', pl, 5)[0])
        time.sleep(1.0 / 200.0)

    st = read_st(L, 6)
    print('2 s 内：|iq|峰=%.3f A，|rpm|峰=%.1f，故障=%s'
          % (max_iq_c, max_rpm_c,
             ','.join('0x%04X' % x for x in faults_c) if faults_c else '无'))
    if st:
        print('末态: fault=0x%04X mode=%d iq=%.3f rpm=%.1f'
              % (st['fault'], st['mode'], st['iq'], st['rpm']))
    # 用最后一帧 ack 的 p_now 看是否到位
    last = None
    for _ in range(6):
        L.send(MSG_MIT, mit_pack(p_to), X_MIT)
        a = L.wait(MSG_MIT, 0.15)
        if a:
            last = struct.unpack_from('<f', a, 5)[0]
    if last is not None:
        print('到位: p_now=%.4f  target=%.4f  残差=%.1f mrad'
              % (last, p_to, (p_to - last) * 1000.0))
    if p_hist:
        print('p_now 轨迹采样 %d 点，首=%.4f 末=%.4f'
              % (len(p_hist), p_hist[0], p_hist[-1]))

    # 8) 收尾
    L.send(MSG_MIT, mit_pack(p_hold, 0.0), X_MIT); L.poll(0.06)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW); L.poll(0.12)
    st = read_st(L, 4)
    if st:
        print('收尾: fault=0x%04X mode=%d iq=%.3f rpm=%.1f'
              % (st['fault'], st['mode'], st['iq'], st['rpm']))
    L.close()
    print('\n完成')

if __name__ == '__main__':
    main()
