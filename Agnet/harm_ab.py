# -*- coding: utf-8 -*-
"""5/7 次谐波抑制：开/关对比测试

方法：速度模式匀速转动 → 录 iq（1024 点 @ 2 kHz，512 ms）→
      用录波同时带回的电角度反算真实电频率 → 在该频率的 6 倍处测幅值。
      关 HARM 录一次，开 HARM 录一次，对比 6 次分量。

为什么必须转动：电频率为 0 时 5/7 次的 6 倍频也是 0 Hz，与直流无法区分。
为什么用 6 倍频：5 次负序(−5θe)、7 次正序(+7θe)，投到 dq 都落在 6ω。

用法: python harm_ab.py COM27 [rpm] [--keep]
  rpm 默认 100（电机侧）。--keep 测完不关机，便于接着手动看。
"""
import sys, time, struct, math
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_READ, X_READ = 30000, 43
MSG_CW,   X_CW   = 30054, 108
MSG_MODE, X_MODE = 30056, 26
MSG_PERM, X_PERM = 30068, 134
MSG_SYS,  X_SYS  = 30069, 169
MSG_PS,   X_PS   = 30019, 8
MSG_ST,   X_ST   = 30090, 90
MSG_LP,   X_LP   = 30091, 91
MSG_REC,  X_REC  = 30094, 94
MSG_DBG,  X_DBG  = 30082, 218

KNOWN = {MSG_CW: X_CW, MSG_MODE: X_MODE, MSG_PERM: X_PERM, MSG_SYS: X_SYS,
         MSG_PS: X_PS, MSG_ST: X_ST, MSG_LP: X_LP, MSG_REC: X_REC, MSG_DBG: X_DBG}

ENC_COUNTS = 2097152.0
POLE_PAIRS = 14
TS = 1.0 / 20000.0        # 电流环 20 kHz
REC_N = 1024
REC_DIV = 10              # 2 kHz 采样，512 ms，频率分辨率 1.95 Hz
IDX_HARM_EN = 1           # 30082 int[1] = harm_en


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
        self.w_cs = 0.0          # 保活用的速度给定（counts/s）

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
        """速度/力矩给定有 2 s deadman，录波前必须持续保活"""
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

    def rec(self, op, arg=0, div=0, retry=3, dur=0.6, keep=False):
        """30094：应答只有一条通道，keepalive 太密会把它挤掉（s_pending 单槽），
        所以默认不保活、失败重试。"""
        body = bytes([op & 0xFF, arg & 0xFF, div & 0xFF]) + bytes(141)
        for _ in range(retry):
            self.send(MSG_REC, body, X_REC)
            r = self.wait(MSG_REC, dur, keep=keep)
            if r is not None:
                return r
        return None

    def set_harm(self, en):
        body = bytearray(192)
        struct.pack_into('<i', body, 128 + IDX_HARM_EN * 4, int(en))
        struct.pack_into('<I', body, 124, 1 << IDX_HARM_EN)   # int 掩码 @124
        self.send(MSG_DBG, bytes(body), X_DBG)
        return self.wait(MSG_DBG, 1.0, keep=True)


def goertzel(x, fs, f):
    """单频幅值（峰值）。加 Hanning 抑制泄漏。"""
    n = len(x)
    if n < 8 or f <= 0:
        return 0.0
    w = 2.0 * math.pi * f / fs
    cw, sw = math.cos(w), math.sin(w)
    coeff = 2.0 * cw
    s0 = s1 = s2 = 0.0
    for i, v in enumerate(x):
        hann = 0.5 - 0.5 * math.cos(2.0 * math.pi * i / (n - 1))
        s0 = v * hann + coeff * s1 - s2
        s2 = s1
        s1 = s0
    re = s1 - s2 * cw
    im = s2 * sw
    mag = 2.0 * math.sqrt(re * re + im * im) / n
    return mag / 0.5        # Hanning 相干增益 0.5


def capture(L):
    """录一次 iq，返回 (iq[A], theta[rad])

    时序要点：
      1) 采满只要 512 ms，deadman 是 2 s —— 采集阶段保活即可；
      2) 采满后的数据留在 s_rec_buf 里，**封锁了也能读**（op=3 只读缓冲，
         不要求 PWM 使能），所以读块阶段不保活，避免抢掉 30094 的应答槽；
      3) 读块期间若 deadman 到期封锁，下一次 measure 前重起环即可。
    """
    L.keepalive()
    r = L.rec(1, 0, REC_DIV, keep=True)
    if r is None or r[12] != 0:
        print('    录波启动失败 st=%s' % ('无应答' if r is None else r[12]))
        return None, None

    t0 = time.time()
    while time.time() - t0 < 2.5:
        L.keepalive()
        time.sleep(0.04)
        s = L.rec(0, dur=0.25)
        if s is None:
            continue
        n_pts = s[6] | (s[7] << 8)
        if s[4] == 2 and n_pts >= REC_N:      # 采满自动停
            break

    iq = []
    th = []
    n_blk = (REC_N + 15) // 16
    for b in range(n_blk):
        r = L.rec(3, b, dur=0.5)
        if r is None or r[12] != 0:
            print('    块 %d 读取失败' % b)
            return None, None
        n_this = r[5]
        for k in range(n_this):
            iq.append(struct.unpack_from('<f', r, 16 + k * 4)[0])
            th.append(struct.unpack_from('<f', r, 80 + k * 4)[0])
    if len(iq) < REC_N:
        print('    点数不足（%d/%d）' % (len(iq), REC_N))
        return None, None
    return iq, th


def measure(L, rpm, label):
    print('\n--- %s（电机侧 %.0f rpm）---' % (label, rpm))
    iq, th = capture(L)
    if iq is None:
        return None

    n = len(iq)
    dt = REC_DIV * TS
    fs = 1.0 / dt
    mean = sum(iq) / n
    ac = [v - mean for v in iq]
    pp = max(iq) - min(iq)
    rms = math.sqrt(sum(v * v for v in ac) / n)

    # 录波通道 0 只存 iq，**不存电角度**（θ 只在 ch1/ch3 存），
    # 所以频率只能由实测转速推算：f_e = rpm/60 × 极对数。
    f_e = abs(rpm) / 60.0 * POLE_PAIRS
    f6 = 6.0 * f_e

    a6 = goertzel(ac, fs, f6)
    a1 = goertzel(ac, fs, f_e)
    a3 = goertzel(ac, fs, 3.0 * f_e)

    # 扫频找主峰（20~400 Hz，2 Hz 步长）
    peaks = []
    f = 20.0
    while f < 400.0:
        peaks.append((goertzel(ac, fs, f), f))
        f += 2.0
    peaks.sort(reverse=True)
    best_a, best_f = peaks[0]

    print('    点数 %d  采样 %.1f kHz  时长 %.0f ms' % (n, fs / 1000, n * dt * 1000))
    print('    iq 均值 %.3f A  峰峰 %.3f A  交流 RMS %.4f A' % (mean, pp, rms))
    print('    推算电频率 %.2f Hz → 6 次 %.1f Hz' % (f_e, f6))
    print('    6 次幅值 %.4f A   占交流 RMS %.1f%%' % (a6, 100 * a6 / rms if rms else 0))
    print('    参考：1 次 %.4f  3 次 %.4f A' % (a1, a3))
    print('    扫频前三个峰：' + '  '.join('%.0fHz/%.3fA' % (f, a) for a, f in peaks[:3]))
    return {'f_e': f_e, 'f6': f6, 'a6': a6, 'rms': rms, 'mean': mean,
            'pp': pp, 'peak_f': best_f, 'peak_a': best_a, 'peaks': peaks[:3]}


def arm_speed(L, rpm, settle=1.2):
    """起速度环并稳速。读块阶段可能已经 deadman 封锁，所以每次录波前都重来一遍。
    返回实测转速（rpm），供频率推算用。"""
    L.w_cs = rpm * 2.0 * math.pi / 60.0 * ENC_COUNTS / (2.0 * math.pi)
    L.keepalive()
    L.poll(0.08)
    L.send(MSG_CW, struct.pack('<HB', 1, 0), X_CW)
    L.wait(MSG_CW, 1.0, keep=True)
    L.keepalive()
    time.sleep(settle)
    g = L.wait(MSG_LP, 0.6, keep=True)
    w_meas = abs(rpm)
    if g:
        w_meas = struct.unpack_from('<f', g, 8)[0]
        print('  wmeas=%.1f rpm  iqc=%.3f A  iqmax=%.2f A'
              % (w_meas, struct.unpack_from('<f', g, 12)[0],
                 struct.unpack_from('<f', g, 28)[0]))
    return abs(w_meas)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
    keep = '--keep' in sys.argv

    L = Link(port)
    print('端口 %s' % port)
    print('！电机将以 %.0f rpm 旋转，请确认空载可自由转动' % rpm)

    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.poll(0.15)
    L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)     # 先封锁
    L.poll(0.15)

    # 确保 HARM 关闭作为基线
    L.set_harm(0)
    print('HARM 已置 0（基线）')

    L.send(MSG_MODE, bytes([3]), X_MODE)               # VEL
    L.poll(0.15)

    print('起环，等稳速 ...')
    w_base = arm_speed(L, rpm)
    base = measure(L, w_base, 'HARM 关闭（基线）')

    if '--base' in sys.argv:
        L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
        L.poll(0.2)
        print('\n（--base：只测基线，不开 HARM）已封锁')
        L.s.close()
        return

    print('\n开启 HARM（30082 int[1]=1）...')
    r = L.set_harm(1)
    print('  应答 status=%s n_ok=%s' % ('?' if r is None else r[0],
                                        '?' if r is None or len(r) < 2 else r[1]))
    print('重起环 ...')
    w_on = arm_speed(L, rpm, settle=1.8)
    on = measure(L, w_on, 'HARM 开启')

    # 收尾：关谐波、封锁
    L.set_harm(0)
    if not keep:
        L.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
        L.poll(0.2)
        print('\n已封锁')

    if base and on:
        print('\n=== 对比 ===')
        print('  iq 均值      %.3f → %.3f A' % (base['mean'], on['mean']))
        print('  峰峰         %.3f → %.3f A' % (base['pp'], on['pp']))
        print('  交流 RMS     %.4f → %.4f A' % (base['rms'], on['rms']))
        print('  6 次幅值     %.4f → %.4f A' % (base['a6'], on['a6']))
        if base['a6'] > 1e-6:
            print('  6 次变化     %+.1f%%' % (100 * (on['a6'] - base['a6']) / base['a6']))
        print('  主峰         %.0f Hz/%.4f A → %.0f Hz/%.4f A'
              % (base['peak_f'], base['peak_a'], on['peak_f'], on['peak_a']))
        ratio = 100 * base['a6'] / base['rms'] if base['rms'] else 0
        print('\n判读：基线 6 次占交流 RMS %.1f%%' % ratio)
        if ratio < 3:
            print('  → 6 次成分本来就很小，开谐波抑制收益有限')
        elif ratio < 8:
            print('  → 6 次成分可见，值得开；看上面"6 次变化"确认实际效果')
        else:
            print('  → 6 次成分明显，应该开')
    L.s.close()


if __name__ == '__main__':
    main()
