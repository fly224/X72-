# -*- coding: utf-8 -*-
"""验证 F2（位置环余速门禁）/ F3（MIT 余速门禁）—— 长测 2026-11-23 修复落实。

原理：起速度模式到目标转速 -> 失能（电机自由滑行，余速还在）-> 立即热切目标模式 + 使能，
读 30090 的 armed 与 fault：
  - 修复前：F2 会多跑 4~6 圈锁 0x0100；F3 会锁 0x0020 且 mode 回 NONE。
  - 修复后：被入口门禁拒绝，armed=0、fault=0x0000、mode 不进入目标模式。
"""
import sys, time, struct
sys.path.insert(0, 'G:/FOC/X72/Agnet')
import spd_ripple
from spd_ripple import Link, MSG_PS, MSG_MODE, MSG_CW, X_PS, X_MODE, X_CW, pack_speed, f32

MSG_TRAJ, X_TRAJ = 30097, 97
spd_ripple.KNOWN[MSG_TRAJ] = X_TRAJ


def snap(lk, tag):
    for mid, pl in lk.poll(0.35)[-3:]:
        if mid == 30090:
            fault = struct.unpack_from('<H', pl, 36)[0] if len(pl) >= 38 else 0
            armed = (pl[3] >> 2) & 1
            print('%s mode=%d flags=0x%02X armed=%d rpm=%.1f fault=0x%04X'
                  % (tag, pl[2], pl[3], armed, f32(pl, 12), fault))
            return (pl[2], armed, fault, f32(pl, 12))
    print('%s 无 30090' % tag)
    return None


def run_case(rpm, want_mode, mode_name):
    lk = Link('COM27')
    lk.poll(0.3)
    # 清故障 + 关时间优先
    lk.send(30069, bytes([3]), 169); time.sleep(0.05)
    lk.send(MSG_TRAJ, bytes([1]) + bytes(31), X_TRAJ); time.sleep(0.05)  # t=0 关时间优先
    # 起速度模式到 rpm
    lk.send(MSG_MODE, bytes([3]), X_MODE); time.sleep(0.05)
    lk.send(MSG_CW, struct.pack('<HB', 0x000F, 0), X_CW); time.sleep(0.30)
    lk.send(MSG_PS, pack_speed(rpm, 1000.0), X_PS)
    lk.poll(3.0, ka=lambda: lk.send(MSG_PS, pack_speed(rpm), X_PS), ka_period=0.4)
    snap(lk, '到速 %d rpm' % rpm)
    # 失能（不停稳，让电机自由滑行，余速还在）
    lk.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW); time.sleep(0.05)
    # 立即热切目标模式 + 使能
    lk.send(MSG_MODE, bytes([want_mode]), X_MODE); time.sleep(0.05)
    lk.send(MSG_CW, struct.pack('<HB', 0x000F, 0), X_CW); time.sleep(0.30)
    r = snap(lk, '热切%s后' % mode_name)
    time.sleep(0.5)
    r2 = snap(lk, '+0.5s')
    # 收尾：停稳（给速度 0 刹车）+ 失能
    lk.send(MSG_MODE, bytes([3]), X_MODE); time.sleep(0.05)
    lk.send(MSG_PS, pack_speed(0.0), X_PS)
    for _ in range(5):
        lk.send(MSG_PS, pack_speed(0.0), X_PS); lk.poll(0.3)
    lk.send(MSG_CW, struct.pack('<HB', 0, 1), X_CW)
    print('---')
    return r, r2


if __name__ == '__main__':
    print('=== F2：位置环余速门禁（>215 rpm 应拒绝，armed=0 fault=0）===')
    run_case(400, 1, 'POS')
    print('=== F3：MIT 余速门禁（>270 rpm 应拒绝，armed=0 fault=0）===')
    run_case(400, 12, 'MIT')
    print('done')
