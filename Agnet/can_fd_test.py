#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CAN FD 互连测试：两台 X72 用 CAN 线互连，一台发 FD 帧、一台收。

原理：
  - 台 A 发 30069 action=21（TX 角色）：每 1ms 发一帧 FD 帧（BRS_ON，数据段 4.25M），
    带序列号 buf[0..1]，固定图样 buf[2]=0x5A / buf[last]=0xA5。
  - 台 B 发 30069 action=22（RX 角色）：收帧，检测序列号连续性，统计丢帧/重复。
  - 停止（action=23）时各自回传 30092 kind=11 结果：
      v[1]=tx_ok v[2]=tx_fail v[3]=rx_ok v[4]=rx_lost v[6]=TEC v[7]=REC
  - TX 侧 ok 判定 = tx_fail==0 且 TEC==0（每一帧都被对端 ACK）
  - RX 侧 ok 判定 = rx_ok>0 且 rx_lost==0 且 rx_dup==0

用法：
  python can_fd_test.py --tx COM8 --rx COM11 --seconds 5
"""
import argparse
import sys
import time

sys.path.insert(0, '.')
import x72_hmi as H

ACT_CAN_TX = 21
ACT_CAN_RX = 22
ACT_CAN_STOP = 23
KIND_CAN_TEST = 11


def collect_result(lk, timeout=3.0):
    """等 30092 kind=11 的结果帧，返回 v 数组或 None"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        for f in list(lk.seq.get(30092) or []):
            if f.get('kind') == KIND_CAN_TEST:
                return f
        time.sleep(0.05)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tx', required=True, help='发送侧串口（TX 角色）')
    ap.add_argument('--rx', required=True, help='接收侧串口（RX 角色）')
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--baud', type=int, default=921600)
    a = ap.parse_args()

    lkA = H.Link(a.tx, a.baud)
    lkB = H.Link(a.rx, a.baud)
    time.sleep(1.5)

    # 确认两台都在线、disarmed
    for lk, name in ((lkA, 'TX'), (lkB, 'RX')):
        lk.get(30090, 1.0)
        s = lk.last.get(30090) or {}
        print('[%s] %s: armed=%s fault=0x%04X' % (name, a.tx if name == 'TX' else a.rx,
              s.get('armed'), s.get('fault', 0)))
        if s.get('armed'):
            print('  ⚠ %s 已使能，CAN 测试需 disarmed，先复位或 Disarm' % name)
            return 2

    # 触发测试角色
    lkA.sys_cmd(ACT_CAN_TX)
    lkB.sys_cmd(ACT_CAN_RX)
    print('\n已触发：%s → TX 角色（发 FD 帧），%s → RX 角色（收 FD 帧）'
          % (a.tx, a.rx))
    print('测试 %g 秒...' % a.seconds)
    time.sleep(a.seconds)

    # 停止，回传结果
    lkA.sys_cmd(ACT_CAN_STOP)
    lkB.sys_cmd(ACT_CAN_STOP)
    time.sleep(0.5)

    rA = collect_result(lkA)
    rB = collect_result(lkB)

    print('\n===== CAN FD 测试结果 =====')
    for lk, name, port in ((lkA, 'TX 发送侧', a.tx), (lkB, 'RX 接收侧', a.rx)):
        r = rA if name.startswith('TX') else rB
        if r is None:
            print('[%s] %s: 未收到结果帧（超时）' % (name, port))
            continue
        v = r.get('v') or []
        print('[%s] %s:' % (name, port))
        print('  ok=%s  role=%d' % (r.get('ok'), (v[0] if v else -1)))
        if len(v) >= 8:
            print('  tx_ok=%d  tx_fail=%d  rx_ok=%d  rx_lost=%d  dup/resync=%d'
                  % (v[1], v[2], v[3], v[4], v[5]))
            print('  TEC=%d  REC=%d' % (v[6], v[7]))

    # 判定
    okA = (rA and rA.get('ok'))
    okB = (rB and rB.get('ok'))
    print('\n判定：TX=%s RX=%s' % ('✓' if okA else '✗', '✓' if okB else '✗'))
    if okA and okB:
        print('→ CAN FD 双向链路通（FD 帧 + BRS 数据段 4.25M，TEC/REC=0）')
    elif okA and not okB:
        print('→ TX 侧帧被 ACK（TEC=0），但 RX 侧有丢帧/重复 —— 数据段时序/收发器问题')
    else:
        print('→ 链路未通，查接线/终端电阻/TDC')
    lkA.close()
    lkB.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
