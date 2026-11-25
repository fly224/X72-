# -*- coding: utf-8 -*-
"""MIT 静止嗡嗡声定位 v3：正确提取 theta（offset 80）+ iq（offset 16）。
死区极限环判据：theta 在死区里来回摆（pp 数 mrad、频率几十 Hz）。
用法：python mit_hold_test.py [kp ...]   默认 30
"""
import sys, time, struct, math
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import Link, f32

TWO_PI = 6.283185307179586
RAD2CNT = 2097152.0 / TWO_PI
PS2RADS = 2097152.0 / 60.0

MSG_MODE, X_MODE = 30056, 26
MSG_CW,   X_CW   = 30054, 108
MSG_SYS,  X_SYS  = 30069, 169
MSG_REC,  X_REC  = 30094, 94
MSG_MITC, X_MITC = 30066, 147
MSG_MIT,  X_MIT  = 30065, 17
MSG_ENC,  X_ENC  = 30040, 160
spd_ripple.KNOWN[MSG_REC] = X_REC
spd_ripple.KNOWN[MSG_MITC] = X_MITC
spd_ripple.KNOWN[MSG_MIT] = X_MIT
spd_ripple.KNOWN[MSG_ENC] = X_ENC

DIV = 4          # 5 kHz，1024 点 = 204.8 ms
QUANT = TWO_PI / 65536.0   # theta 量化台阶 0.0959 mrad


def pack_cfg(kp, kd):
    p = bytearray(16)
    struct.pack_into('<f', p, 0, kp)
    struct.pack_into('<f', p, 4, kd)
    return bytes(p)


def pack_mit(pos_out, tau=0.0, v_out=0.0):
    p = bytearray(32)
    struct.pack_into('<q', p, 0, int(pos_out * RAD2CNT))
    struct.pack_into('<f', p, 16, tau)
    struct.pack_into('<f', p, 24, v_out * PS2RADS)
    return bytes(p)


def rec(lk, op, arg=0, div=0, timeout=1.2):
    lk.send(MSG_REC, bytes([op, arg, div]) + bytes(141), X_REC)
    t = time.time()
    while time.time() - t < timeout:
        for mid, pl in lk.poll(0.02):
            if mid == MSG_REC and len(pl) >= 16:
                return pl
    return None


def read_theta_out(lk):
    """读当前输出端多圈角：30040 enc_sum（电机端 2^21 脉冲/圈）÷9"""
    for mid, pl in lk.poll(0.3)[-4:]:
        if mid == MSG_ENC and len(pl) >= 8:
            enc = struct.unpack_from('<q', pl, 0)[0]
            return (enc / 2097152.0) * TWO_PI / 9.0
    return None


def record(lk, kp, ch):
    """起环到 kp，录 ch 通道，返回 (val, th)"""
    cur = read_theta_out(lk)
    if cur is None:
        return None, None
    # 起环：kp=0 贴住（30065 目标=当前位置，无力矩）
    lk.send(MSG_MITC, pack_cfg(0.0, 0.0), X_MITC)
    time.sleep(0.05)
    lk.send(MSG_MIT, pack_mit(cur), X_MIT)
    time.sleep(0.4)
    # 上刚度：30066 写 kp，然后【重发 30065 目标】让新 kp 生效
    lk.send(MSG_MITC, pack_cfg(kp, 0.05), X_MITC)
    time.sleep(0.05)
    lk.send(MSG_MIT, pack_mit(cur), X_MIT)
    time.sleep(0.6)

    rec(lk, 2)
    r = rec(lk, 1, ch, DIV)
    if r is None:
        return None, None
    # 录波期间保活 30065（目标=当前位置），避免 deadman 失能
    need = DIV * 1024 / 20000.0
    for _ in range(int(need / 0.4) + 1):
        lk.send(MSG_MIT, pack_mit(cur), X_MIT)
        lk.poll(0.4)
    rec(lk, 2)
    time.sleep(0.05)
    val, th = [], []
    for blk in range(64):
        r = rec(lk, 3, blk)
        if r is None:
            break
        nt = r[5]
        np_ = r[6] | (r[7] << 8)
        if nt == 0:
            break
        for k in range(nt):
            val.append(f32(r, 16 + k * 4))
            th.append(f32(r, 80 + k * 4))
        if len(val) >= np_:
            break
        time.sleep(0.004)
    return val, th


def spectrum(v, fs, fmin=5.0, fmax=1000.0):
    """返回 [(f, amp), ...] 按幅值降序的前 8 个峰（对数扫频）"""
    m = sum(v) / len(v)
    n = len(v)
    out = []
    f = fmin
    while f < min(fmax, fs / 2):
        wr = TWO_PI * f / fs
        re = im = 0.0
        for j in range(n):
            wnd = 0.5 - 0.5 * math.cos(TWO_PI * j / (n - 1))
            re += (v[j] - m) * wnd * math.cos(wr * j)
            im -= (v[j] - m) * wnd * math.sin(wr * j)
        amp = 2 * math.hypot(re, im) / n
        out.append((f, amp))
        f *= 1.01
    out.sort(key=lambda x: -x[1])
    return out[:8]


def main():
    kps = [float(x) for x in sys.argv[1:]] or [30.0]
    lk = Link('COM27')
    lk.poll(0.3)
    lk.send(MSG_SYS, bytes([3]), X_SYS)
    time.sleep(0.05)
    lk.send(MSG_MODE, bytes([12]), X_MODE)
    time.sleep(0.05)
    lk.send(MSG_CW, struct.pack('<HB', 0x000F, 0), X_CW)
    time.sleep(0.4)

    fs = 20000.0 / DIV
    for kp in kps:
        print('=== MIT kp=%.0f（div=%d，%.1f kHz）===' % (kp, DIV, fs / 1000))
        # ch=3 录 theta + vd
        val, th = record(lk, kp, 3)
        if th and len(th) >= 100:
            m = sum(th) / len(th)
            d = [x - m for x in th]
            pp = max(d) - min(d)
            rms = math.sqrt(sum(x * x for x in d) / len(d))
            print('theta 摆幅: pp=%.3f mrad, rms=%.3f mrad（量化台阶 %.3f mrad）'
                  % (pp * 1000, rms * 1000, QUANT * 1000))
            print('theta 频谱峰: ' + ', '.join('%.0fHz/%.3fmrad' % (f, a * 1000) for f, a in spectrum(th, fs)))
        # ch=0 录 iq
        val, _ = record(lk, kp, 0)
        if val and len(val) >= 100:
            m = sum(val) / len(val)
            pp = max(val) - min(val)
            rms = math.sqrt(sum((x - m) ** 2 for x in val) / len(val))
            print('iq 摆幅: pp=%.3f A, rms=%.3f A' % (pp, rms))
            print('iq 频谱峰: ' + ', '.join('%.0fHz/%.3fA' % (f, a) for f, a in spectrum(val, fs)))
        time.sleep(0.3)

    lk.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    print('done')


if __name__ == '__main__':
    main()
