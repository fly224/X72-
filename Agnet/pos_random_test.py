# -*- coding: utf-8 -*-
"""随机位置控制测试：验证位置环输出端重构后定位正确 + 就近折算修复。

两类测试：
  A. 随机相对定位（p_now + Δ，Δ 随机 ±0.3~0.8 rad 输出端）：测到位精度 + st=DONE
  B. 本圈角定位（给 [0, 2π) 内的绝对角，含 [0, 40°) 边界）：测就近折算 bug 是否修复

安全边界：目标行程 ≤ 0.8 rad 输出端（约 46°），空载自由转动。
"""
import sys, time, struct, random, math
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import Link, MSG_PS, MSG_MODE, MSG_CW, X_PS, X_MODE, X_CW, pack_speed, f32

TWO_PI = 6.283185307179586
RAD2CNT = 2097152.0 / TWO_PI
spd_ripple.KNOWN[30100] = 100
spd_ripple.KNOWN[30082] = 218


def pack_pos(out_rad):
    p = bytearray(44)
    struct.pack_into('<q', p, 0, int(out_rad * RAD2CNT))
    return bytes(p)


def get_pos(lk, tag=""):
    """读 30100，返回 (p_now, p_tgt, err, st)"""
    for mid, pl in lk.poll(0.3)[-4:]:
        if mid == 30100:
            return (f32(pl, 0), f32(pl, 4), f32(pl, 16), pl[44] if len(pl) >= 45 else -1)
    return None


def read_kp(lk):
    lk.send(30000, struct.pack('<I', 30082), 43)   # READ_PARAM 30082
    for mid, pl in lk.poll(0.3)[-3:]:
        if mid == 30082:
            return f32(pl, 7 * 4)   # float 槽 7 = pos_kp
    return None


def wrap_pi(d):
    while d > math.pi: d -= TWO_PI
    while d < -math.pi: d += TWO_PI
    return d


def wait_done(lk, tgt, timeout=4.0):
    """稳健到位判断：st==DONE 且 |err|<2mrad 连续 3 次（~90ms），避免读到旧 DONE 残留。"""
    t0 = time.time()
    ok = 0
    last = None
    while time.time() - t0 < timeout:
        lk.send(MSG_PS, pack_pos(tgt), X_PS)   # 保活（相同目标固件会忽略）
        for mid, pl in lk.poll(0.04):
            if mid == 30100:
                st = pl[44] if len(pl) >= 45 else -1
                err = f32(pl, 16)
                last = (f32(pl, 0), err, st)
                if st == 3 and abs(err) < 0.002:
                    ok += 1
                    if ok >= 3:
                        return last
                else:
                    ok = 0
        time.sleep(0.01)
    return last if last else (float('nan'), float('nan'), -1)


def main():
    lk = Link('COM27')
    lk.poll(0.3)
    kp = read_kp(lk)
    print('pos_kp（30082 float7）= %.1f' % (kp if kp is not None else -1))

    # 清故障 + 使能位置模式
    lk.send(30069, bytes([3]), 169); time.sleep(0.05)
    lk.send(MSG_MODE, bytes([1]), X_MODE); time.sleep(0.05)
    lk.send(MSG_CW, struct.pack('<HB', 0x000F, 0), X_CW); time.sleep(0.4)
    r0 = get_pos(lk)
    if r0 is None:
        print('使能失败，读不到 30100')
        return
    p0 = r0[0]
    print('使能后 p_now = %.4f rad (%.1f°)' % (p0, p0 * 57.2958))

    # ---- A. 随机相对定位 ----
    print('\n=== A. 随机相对定位（20 次，Δ ±0.3~0.8 rad 输出端）===')
    errs = []
    rng = random.Random(20261124)
    p_cur = p0
    for i in range(20):
        d = rng.uniform(0.3, 0.8) * (1 if rng.random() < 0.5 else -1)
        tgt = p_cur + d
        lk.send(MSG_PS, pack_pos(tgt), X_PS)
        p_now, err, st = wait_done(lk, tgt)
        if st == 3:
            errs.append(abs(err) * 1000)   # mrad
            p_cur = p_now
            print('  #%2d  Δ=%+.3f → p_tgt=%.4f p_now=%.4f err=%+.3f mrad DONE' %
                  (i + 1, d, tgt, p_now, err * 1000))
        else:
            print('  #%2d  Δ=%+.3f 超时 st=%d err=%+.3f mrad' % (i + 1, d, st, err * 1000))
    if errs:
        errs.sort()
        print('  到位误差：中位 %.3f mrad / P90 %.3f / 最大 %.3f mrad' %
              (errs[len(errs) // 2], errs[int(len(errs) * 0.9)], errs[-1]))

    # ---- B. 本圈角定位（含 [0,40°) 边界，测就近折算）----
    print('\n=== B. 本圈角定位（[0,2π) 绝对角，含 [0,40°) 边界）===')
    # 先定位到一个"远离 0 点"的位置，让电机端 now 落在"不同电机圈"
    lk.send(MSG_PS, pack_pos(p_cur + 1.5), X_PS); wait_done(lk, p_cur + 1.5)
    p_cur = get_pos(lk)[0]
    for tgt in [0.05, 0.2, 0.4, 0.698, 1.0, 2.0]:
        lk.send(MSG_PS, pack_pos(tgt), X_PS)
        p_now, err, st = wait_done(lk, tgt)
        # 目标应被就近折算到 tgt 的某个 2π 等价点；到位后 p_now 应与 tgt 同余 2π
        real_err = wrap_pi(p_now - tgt) * 1000
        print('  目标 %.4f rad(%.1f°) → p_now=%.4f err=%+.3f mrad st=%d %s' %
              (tgt, tgt * 57.2958, p_now, real_err, st, 'DONE' if st == 3 else '(超时)'))
        p_cur = p_now

    # 收尾：失能
    lk.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    print('\n完成，已失能')


if __name__ == '__main__':
    main()
