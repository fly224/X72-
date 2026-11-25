# -*- coding: utf-8 -*-
"""转速纹波诊断：定位"恒速时转速波动大"的来源

做三件事：
  1) 只读快照：30090/30091，确认模式、wref、wmeas、iqc、kp、iqmax
  2) 15 s 连续采集（30090 rpm/iq/vbus + 30091 wref/wmeas/iqc/iterm/kp），存 CSV
  3) 频谱：rpm 与 iqc 的主频、幅值、互相关系数 / 相位差

判据（关键）：
  · rpm 与 iqc 同频反相（rpm 高时 iq 低? 不对，转速高时 iq 应低）→ 电磁极限环
  · iqc 平稳而 rpm 波动           → 测速链路 / 机械侧
  · 主频 ≈ 每转整数倍            → 齿槽/偏心
  · 主频 与转速无关(≈5 Hz)      → 环路极限环 或 外部周期扰动（电源/上位机保活）

用法:
  python spd_ripple.py                # 只读 + 采集 15 s
  python spd_ripple.py --port COM27 --sec 15
  python spd_ripple.py --rpm 1560     # 采集前先下发速度(需已使能)
"""
import sys, time, struct, argparse, math, csv
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_ST, X_ST = 30090, 90
MSG_LP, X_LP = 30091, 91
MSG_MODE, X_MODE = 30056, 26
MSG_PS, X_PS = 30019, 8
MSG_CW, X_CW = 30054, 108
MSG_TRAJ, X_TRAJ = 30097, 97

KNOWN = {MSG_ST: X_ST, MSG_LP: X_LP, MSG_TRAJ: X_TRAJ}
RPM2PS = 2097152.0 / 60.0          # rpm -> 计数/s
PS2RAD = 2.0 * math.pi / 2097152.0


def pack_traj_t(t_s):
    """30097 AppSpeedTraj（32 B）：cmd@0=1 写入，t_ramp_s@4；t<=0 关闭时间优先"""
    p = bytearray(32)
    p[0] = 1
    struct.pack_into('<f', p, 4, t_s)
    return bytes(p)


def pack_speed(v_rpm, acc_rpm_s=0.0):
    """30019 PosSpeedCtlInput（44 B）：int64 pos_tar_p@0 | f32 speed_tar_p_s@28 | f32 acc_tar_p_ss@32

    ★ 长度必须是 44，不是 8。早先按 8 B 发，帧被长度门卫丢弃，
    现象是"30019 发了但电机不动、2 s 后 deadman 停机"。
    pos_tar_p 必须填 0（固件对 p==0 不置 s_p_valid，才不会把位置目标带跑）。
    """
    p = bytearray(44)
    struct.pack_into('<q', p, 0, 0)
    struct.pack_into('<f', p, 28, v_rpm * RPM2PS)
    struct.pack_into('<f', p, 32, acc_rpm_s * RPM2PS)
    return bytes(p)


def x25(data, extra):
    crc = 0xFFFF
    for b in list(data) + [extra]:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class Link:
    def __init__(self, port):
        self.s = serial.Serial(port, 921600, timeout=0)
        self.seq = 0
        self.buf = bytearray()

    def send(self, msgid, payload, extra):
        hdr = bytes([len(payload) & 0xFF, 0, 0, self.seq & 0xFF, 1, 1,
                     msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
        body = hdr + payload
        c = x25(body, extra)
        self.s.write(b'\xFD' + body + bytes([c & 0xFF, c >> 8]))
        self.seq += 1

    def poll(self, dur, sink=None, ka=None, ka_period=0.4):
        out = []
        t0 = time.time()
        t_next = t0
        while time.time() - t0 < dur:
            if ka is not None and time.time() >= t_next:
                ka()
                t_next = time.time() + ka_period
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
                pl = bytes(self.buf[i+10:i+10+ln])
                if sink is not None:
                    sink.append((time.time(), mid, pl))
                else:
                    out.append((mid, pl))
                i += 12 + ln
            del self.buf[:i]
            time.sleep(0.0005)
        return out


def f32(p, off):
    return struct.unpack_from('<f', p, off)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM27')
    ap.add_argument('--sec', type=float, default=15.0)
    ap.add_argument('--rpm', type=float, default=None, help='采集前下发该转速（需已使能速度模式）')
    ap.add_argument('--run', action='store_true', help='自动：清给定 → 速度模式 → 使能 → 下发 --rpm')
    ap.add_argument('--ka', type=float, default=0.4, help='30019 保活周期 s（默认 0.4）')
    a = ap.parse_args()

    lk = Link(a.port)
    lk.poll(0.3)

    print('--- 快照 ---')
    snap = lk.poll(0.5)
    st = lp = None
    for mid, pl in snap:
        if mid == MSG_ST:
            st = pl
        elif mid == MSG_LP:
            lp = pl
    if st:
        print('30090 cw=0x%04X mode=%d flags=0x%02X(armed=%d) iq=%.3f iqref=%.3f rpm=%.2f theta=%.4f vbus=%.2f km=%s fault=0x%04X'
              % (struct.unpack_from('<H', st, 0)[0], st[2], st[3], (st[3] >> 2) & 1,
                 f32(st, 4), f32(st, 8), f32(st, 12), f32(st, 16),
                 f32(st, 20), ('%.3f' % f32(st, 32)) if len(st) >= 36 else '?',
                 struct.unpack_from('<H', st, 36)[0] if len(st) >= 38 else 0))
    else:
        print('30090 无数据：检查串口占用/链路')
    if lp:
        print('30091 spdmode=%d src=%d sched=%d encdiag=%d wref=%.2f wmeas=%.2f iqc=%.3f iterm=%.3f kp=%.3f acc=%.1f iqmax=%.2f'
              % (lp[0], lp[1], lp[2], lp[3], f32(lp, 4), f32(lp, 8), f32(lp, 12),
                 f32(lp, 16), f32(lp, 20), f32(lp, 24), f32(lp, 28)))
    if lp is None and st is None:
        return

    if a.run:
        rpm = a.rpm if a.rpm is not None else 1560.0

        def cmd_speed(v):
            lk.send(MSG_PS, pack_speed(v), X_PS)

        cmd_speed(0.0)
        time.sleep(0.05)
        lk.send(MSG_MODE, bytes([3]), X_MODE)          # mode=3 速度模式
        time.sleep(0.05)
        lk.send(MSG_TRAJ, pack_traj_t(0.0), X_TRAJ)     # 关掉"到达时间优先"，改用配置加速度
        time.sleep(0.05)
        lk.send(MSG_CW, struct.pack('<HB', 0x000F, 0), X_CW)   # cw=0x0F halt=0 使能
        time.sleep(0.30)
        lk.send(MSG_PS, pack_speed(rpm, 500.0), X_PS)   # 500 rpm/s 爬坡
        print('已使能并下发 %.0f rpm（500 rpm/s），等待到速 ...' % rpm)
        lk.poll(4.0, ka=lambda: cmd_speed(rpm), ka_period=a.ka)
        chk = lk.poll(0.3)
        for mid, pl in chk[-4:]:
            if mid == MSG_ST:
                print('  使能后 30090 cw=0x%04X mode=%d flags=0x%02X iqref=%.3f rpm=%.1f fault=0x%04X'
                      % (struct.unpack_from('<H', pl, 0)[0], pl[2], pl[3],
                         f32(pl, 8), f32(pl, 12),
                         struct.unpack_from('<H', pl, 36)[0] if len(pl) >= 38 else 0))
            elif mid == MSG_LP:
                print('  使能后 30091 spdmode=%d src=%d wref=%.1f wmeas=%.1f iqc=%.3f iterm=%.3f'
                      % (pl[0], pl[1], f32(pl, 4), f32(pl, 8), f32(pl, 12), f32(pl, 16)))
    elif a.rpm is not None:
        lk.send(MSG_PS, pack_speed(a.rpm), X_PS)
        print('已下发 %.0f rpm' % a.rpm)
        time.sleep(2.0)

    keep = (a.rpm if a.rpm is not None else 1560.0)
    ka_fn = None
    if a.run:
        ka_fn = lambda: lk.send(MSG_PS, pack_speed(keep), X_PS)
    elif a.rpm is not None:
        ka_fn = lambda: lk.send(MSG_PS, pack_speed(keep), X_PS)

    print('\n--- 采集 %.0f s ---' % a.sec)
    raw = []
    lk.poll(a.sec, sink=raw, ka=ka_fn, ka_period=a.ka)

    rows = []
    for ts, mid, pl in raw:
        if mid == MSG_ST and len(pl) >= 38:
            rows.append([ts, f32(pl, 12), f32(pl, 8), f32(pl, 20)])
        elif mid == MSG_LP and len(pl) >= 32:
            rows.append([ts, None, None, None])
    # 两个消息交错存在 raw 里，单独抽
    rpm_s, iq_s, vb_s, wm_s, wref_s, iqc_s = [], [], [], [], [], []
    for ts, mid, pl in raw:
        if mid == MSG_ST and len(pl) >= 38:
            rpm_s.append((ts, f32(pl, 12)))
            iq_s.append((ts, f32(pl, 8)))
            vb_s.append((ts, f32(pl, 20)))
        elif mid == MSG_LP and len(pl) >= 32:
            wm_s.append((ts, f32(pl, 8)))
            wref_s.append((ts, f32(pl, 4)))
            iqc_s.append((ts, f32(pl, 12)))

    if len(rpm_s) < 100:
        print('样本不足: rpm=%d lp=%d' % (len(rpm_s), len(lp or [])))
        return

    t0 = rpm_s[0][0]
    path = 'spd_ripple_%s.csv' % time.strftime('%Y%m%d_%H%M%S')
    n = max(len(rpm_s), len(wm_s))
    with open(path, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['t', 'rpm', 'iqref_A', 'vbus_V', 'wmeas_rpm', 'wref_rpm', 'iqc_A'])
        for k in range(n):
            w.writerow([
                '%.4f' % (rpm_s[k][0] - t0) if k < len(rpm_s) else '',
                '%.3f' % rpm_s[k][1] if k < len(rpm_s) else '',
                '%.4f' % iq_s[k][1] if k < len(iq_s) else '',
                '%.3f' % vb_s[k][1] if k < len(vb_s) else '',
                '%.3f' % wm_s[k][1] if k < len(wm_s) else '',
                '%.3f' % wref_s[k][1] if k < len(wref_s) else '',
                '%.4f' % iqc_s[k][1] if k < len(iqc_s) else '',
            ])
    print('CSV: %s' % path)

    def stats(name, ser):
        v = [x[1] for x in ser]
        m = sum(v) / len(v)
        pp = max(v) - min(v)
        rms = math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
        print('  %-8s mean=%9.3f  pp=%8.3f  rms=%7.3f  min=%9.3f  max=%9.3f  n=%d'
              % (name, m, pp, rms, min(v), max(v), len(v)))
        return m, pp, rms, v

    print('--- 统计 ---')
    stats('rpm', rpm_s)
    stats('iqref', iq_s)
    stats('vbus', vb_s)
    if wm_s:
        stats('wmeas', wm_s)
        stats('iqc', iqc_s)
    # 采样率
    dur = rpm_s[-1][0] - rpm_s[0][0]
    print('  30090 帧率 ≈ %.0f Hz，30091 帧率 ≈ %.0f Hz'
          % (len(rpm_s) / dur if dur > 0 else 0, len(wm_s) / dur if dur > 0 else 0))

    # 频谱（去均值 + Hann，频率分辨率限制在采样率上）
    def spectrum(ser, name):
        v = [x[1] for x in ser]
        ts = [x[0] for x in ser]
        # 重采样到均匀网格
        fs = 200.0
        dt = 1.0 / fs
        t_start, t_end = ts[0], ts[-1]
        m = int((t_end - t_start) / dt)
        if m < 64:
            return
        import bisect
        idx = [bisect.bisect_left(ts, t_start + k * dt) for k in range(m)]
        y = [v[min(i, len(v) - 1)] for i in idx]
        mean = sum(y) / m
        y = [x - mean for x in y]
        # 简单 DFT（只扫 0.5~50 Hz，够用）
        best = (0.0, 0.0)
        out = []
        f = 0.5
        while f <= 50.0:
            wr = 2 * math.pi * f * dt
            re = im = 0.0
            for k in range(m):
                w = 0.5 - 0.5 * math.cos(2 * math.pi * k / (m - 1))
                re += y[k] * w * math.cos(wr * k)
                im -= y[k] * w * math.sin(wr * k)
            amp = 2.0 * math.hypot(re, im) / m
            out.append((f, amp))
            if amp > best[1]:
                best = (f, amp)
            f += 0.05
        # 前 5 峰
        out.sort(key=lambda x: -x[1])
        print('  %-6s 主频 %.2f Hz  幅值 %.3f  | 前5峰: %s'
              % (name, best[0], best[1],
                 ', '.join('%.2fHz@%.2f' % (a1, a2) for a1, a2 in out[:5])))
        return out

    print('--- 频谱 ---')
    spectrum(rpm_s, 'rpm')
    spectrum(iqc_s, 'iqc')
    spectrum(vb_s, 'vbus')


if __name__ == '__main__':
    main()
