# -*- coding: utf-8 -*-
"""电机三步处置：1) iqmax 恢复 5 A  2) 重做齿槽表  3) 清故障并核对过流现场

用法: python motor_fix.py COM27 [step1|step2|step3|all]
默认 all。每步都回读真值，不靠应答"成功"下结论 —— 30082 的 ack 只报写入项数，
iqmax 是否真的生效要看 30091 的 offset 28。
"""
import sys, time, struct
import serial

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

MSG_READ, X_READ = 30000, 43
MSG_DBG,  X_DBG  = 30082, 218
MSG_SYS,  X_SYS  = 30069, 169
MSG_PERM, X_PERM = 30068, 134
MSG_CW,   X_CW   = 30054, 108
MSG_ST,   X_ST   = 30090, 90
MSG_LP,   X_LP   = 30091, 91
MSG_ID,   X_ID   = 30092, 92
MSG_COG,  X_COG  = 30093, 93

KNOWN = {MSG_READ: X_READ, MSG_DBG: X_DBG, MSG_SYS: X_SYS, MSG_PERM: X_PERM,
         MSG_CW: X_CW, MSG_ST: X_ST, MSG_LP: X_LP, MSG_ID: X_ID, MSG_COG: X_COG}

ACT_CLEAR_FAULT = 3
ACT_COG = 13

IDX_IQMAX = 12          # 30082 float[12] = cur_iqmax


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
                mid = self.buf[i + 7] | (self.buf[i + 8] << 8) | (self.buf[i + 9] << 16)
                ex = KNOWN.get(mid)
                if ex is None:
                    i += 1
                    continue
                body = bytes(self.buf[i + 1:i + 10 + ln])
                crc = self.buf[i + 10 + ln] | (self.buf[i + 11 + ln] << 8)
                if x25(body, ex) != crc:
                    i += 1
                    continue
                out.append((mid, bytes(self.buf[i + 10:i + 10 + ln])))
                i += 12 + ln
            del self.buf[:i]
            time.sleep(0.002)
        return out

    def read_param(self, struct_id, times=3):
        """30000 READ_PARAM：发 4 B struct_id，收同 id 的 192 B 应答"""
        for _ in range(times):
            self.send(MSG_READ, struct.pack('<I', struct_id), X_READ)
            t = time.time()
            while time.time() - t < 1.0:
                for mid, pl in self.poll(0.05):
                    if mid == MSG_READ and len(pl) >= 68:
                        return pl
        return None

    def snapshot(self):
        """收一轮遥测，返回 {msgid: payload}"""
        got = {}
        t = time.time()
        while time.time() - t < 1.0 and len(got) < 3:
            for mid, pl in self.poll(0.05):
                got[mid] = pl
        return got


def show_state(L, title):
    g = L.snapshot()
    print('  [%s]' % title)
    if MSG_ST in g:
        st = g[MSG_ST]
        print('    flags=0x%02X (CAL=%d ZERO=%d COG=%d)  fault=0x%04X  mode=%d'
              % (st[3], st[3] & 1, (st[3] >> 1) & 1, (st[3] >> 7) & 1,
                 st[36] | (st[37] << 8), st[2]))
    if MSG_LP in g:
        lp = g[MSG_LP]
        print('    iqmax=%.3f A   过流现场: 峰值 %.2f A / 码 %d'
              % (f32(lp, 28), f32(lp, 40), int(f32(lp, 44))))
    return g


def step1(L):
    print('\n=== 步骤 1：iqmax 恢复 5.0 A ===')
    print('  注：30091[28] 报的是**速度环** spd_iqmax（另一个量），不是 cur_iqmax。')
    print('      cur_iqmax 没有遥测出口，判据只能用 30082 写入应答的 n_ok ——')
    print('      dbg_set_cur_iqmax 内部会 GetCurIqMax() 回读比对，n_ok=1 即等于写入值。')
    g = L.snapshot()
    if MSG_LP in g:
        print('  当前 spd_iqmax(30091) = %.3f A（不动它）' % f32(g[MSG_LP], 28))

    body = bytearray(192)
    struct.pack_into('<f', body, IDX_IQMAX * 4, 5.0)
    struct.pack_into('<I', body, 64, 1 << IDX_IQMAX)   # 掩码只置这一位
    L.send(MSG_DBG, bytes(body), X_DBG)
    ack = None
    t = time.time()
    while time.time() - t < 1.2:
        for mid, p in L.poll(0.05):
            if mid == MSG_DBG:
                ack = p
                break
        if ack:
            break
    if ack is None:
        print('  没收到 30082 应答')
        return False
    st = ack[0]
    n_ok = ack[1] if len(ack) >= 2 else 0
    print('  写入应答 status=%d  n_ok=%d' % (st, n_ok))
    ok = (st == 0) and (n_ok >= 1)
    print('  判定：%s' % ('通过（cur_iqmax 已置 5.0 A 且固件回读一致）'
                        if ok else '失败（写入被拒或回读不一致）'))
    if ok:
        print('  力矩上限随之变为 5.0×0.091×kt_comp×10 ≈ 4.5 N·m（输出侧）')
    return ok


def step2(L):
    print('\n=== 步骤 2：重做齿槽表（action=%d）===' % ACT_COG)
    print('  电机将以 10 rpm 正反各转 1 圈（约 15~20 s），需要空载可自由转动')
    L.send(MSG_SYS, bytes([ACT_COG]), X_SYS)
    ack = None
    t = time.time()
    while time.time() - t < 1.5:
        for mid, p in L.poll(0.05):
            if mid == MSG_SYS:
                ack = p
                break
        if ack:
            break
    st = None if ack is None else ack[0]
    print('  ack status = %s  (0=已接受 3=需先封锁 2=超范围)' % st)
    if st != 0:
        return False

    print('  等待 30092 kind=5 ...')
    t0 = time.time()
    res = None
    while time.time() - t0 < 60.0:
        for mid, pl in L.poll(0.2):
            if mid == MSG_ID and len(pl) >= 40 and pl[0] == 5:
                res = pl
                break
        if res is not None:
            break
        if int(time.time() - t0) % 5 == 0:
            pass
    if res is None:
        print('  超时：没收到 30092 kind=5')
        return False

    ok = res[1]
    v = list(struct.unpack_from('<8f', res, 4))
    print('  30092 kind=5  ok=%d  用时 %.1f s' % (ok, time.time() - t0))
    print('    pp_A=%.3f  pp_fwd=%.3f  pp_rev=%.3f  dc=%.3f' % (v[0], v[1], v[2], v[3]))
    if not ok:
        print('  齿槽辨识失败')
        return False

    g = L.snapshot()
    cog_bit = (g[MSG_ST][3] >> 7) & 1 if MSG_ST in g else -1
    print('  30090 flags.bit7(COG) = %d  %s'
          % (cog_bit, '← 表已装载' if cog_bit == 1 else '← 仍未生效！'))

    # 回读表头确认非全零
    L.send(MSG_COG, bytes([0, 0]), X_COG)
    hdr = None
    t = time.time()
    while time.time() - t < 1.0:
        for mid, p in L.poll(0.05):
            if mid == MSG_COG:
                hdr = p
                break
        if hdr:
            break
    if hdr:
        bins = hdr[2] | (hdr[3] << 8)
        print('  30093 表头：status=%d  箱数=%d' % (hdr[0] & 0xFF, bins))
    return cog_bit == 1


def step3(L):
    print('\n=== 步骤 3：清故障 + 核对过流现场 ===')
    show_state(L, '清除前')
    L.send(MSG_SYS, bytes([ACT_CLEAR_FAULT]), X_SYS)
    ack = None
    t = time.time()
    while time.time() - t < 1.5:
        for mid, p in L.poll(0.05):
            if mid == MSG_SYS:
                ack = p
                break
        if ack:
            break
    print('  ack status = %s  (0=成功)' % (None if ack is None else ack[0]))
    time.sleep(0.2)
    g = show_state(L, '清除后')
    if MSG_LP not in g:
        return False
    peak = abs(f32(g[MSG_LP], 40))
    raw = int(f32(g[MSG_LP], 44))
    # 判据不能要求 "raw == 0"：ClearOvcInfo 之后 PWM 中断立刻继续记录
    # |iu| 的全程最大值，所以读到的必然是"清除后这几百毫秒里的最大采样"。
    # 健康状态 = 电流在零附近（<0.5 A）且码值在偏置附近（2048±250）。
    # 之前那次 -54.95 A / 码 2 才是异常：码 2 表示 ADC 读回接近 0。
    ok = (peak < 0.5) and (1750 < raw < 2350)
    print('  判据：|峰值| < 0.5 A 且 码值在 1750~2350（偏置附近）')
    print('  判定：%s' % ('通过（现场已干净，电流在零附近）' if ok else '失败（仍有异常采样）'))
    if not ok:
        print('  ！码值 2 或峰值 >0.5 A 说明电流采样链又读到了异常值')
    return ok


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM27'
    step = sys.argv[2] if len(sys.argv) > 2 else 'all'
    L = Link(port)
    print('端口 %s 已打开' % port)

    # 申请权限 + 确保未使能（辨识与清故障都要求封锁）
    L.send(MSG_PERM, bytes([0, 0]), X_PERM)
    L.poll(0.1)
    L.send(MSG_CW, struct.pack('<HB', 0, 0), X_CW)
    L.poll(0.15)
    show_state(L, '起始状态')

    r = {}
    if step in ('1', 'all'):
        r['步骤1 iqmax'] = step1(L)
    if step in ('2', 'all'):
        r['步骤2 齿槽表'] = step2(L)
    if step in ('3', 'all'):
        r['步骤3 清故障'] = step3(L)

    print('\n=== 汇总 ===')
    for k, v in r.items():
        print('  %s: %s' % (k, '通过' if v else '未通过'))
    L.s.close()


if __name__ == '__main__':
    main()
