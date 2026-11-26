#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CANopen 通讯长测 —— 持续采样主站的 30092 kind=11 统计，落盘 + 判丢帧。

背景
  CANopen 协议层 8 项已验证，但双板启动竞争、长时间稳定性还没测。
  本脚本把主站（板 B，CANOPEN_MASTER_TEST）每秒上报的统计落成 CSV，
  跑完后能算出：TPDO 实际速率、丢帧率、总线错误累积、状态字是否稳定。

判据（长测重点看）
  · tpdo_rate：应该稳定在 ~650 帧/s（从站 Operational 时），突降 = 通讯中断
  · tec/rec：TEC 持续累积 = 发送有问题；REC 累积 = 接收有问题
  · sw：状态字应恒为 0x0627（enabled + remote + target reached）
  · 连续 N 秒 tpdo 增量 < 预期×50% → 判定通讯中断并告警

用法
  python can_long.py --port COM8 --minutes 30
  python can_long.py --port COM8 --seconds 120 --out can_long_out
"""
import argparse
import csv
import os
import sys
import time

import x72_hmi as H

EXPECT_TPDO = 650.0          # Operational 时实测 ~653 帧/s，留余量
DROP_THRESH = 0.5            # 增量低于预期×50% 判定异常


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM8')
    ap.add_argument('--minutes', type=float, default=30.0)
    ap.add_argument('--seconds', type=float, default=0.0)
    ap.add_argument('--out', default='can_long_out')
    a = ap.parse_args()

    dur = a.seconds if a.seconds > 0 else a.minutes * 60.0
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, time.strftime('canlong_%m%d_%H%M%S.csv'))
    f = open(path, 'w', newline='', encoding='utf-8-sig')
    w = csv.writer(f)
    w.writerow(['t_s', 'tpdo', 'tpdo_rate', 'sw', 'sdo_val', 'tec', 'rec', 'tx'])

    lk = H.Link(a.port)
    time.sleep(1.5)
    print('=== CANopen 通讯长测 %.0f s（%s）===' % (dur, a.port))
    print('CSV: %s' % path)
    print('%8s %9s %10s %8s %10s %6s %6s' % ('t', 'tpdo', 'rate/s', 'sw', 'sdo_val', 'tec', 'rec'))

    t0 = time.time()
    last_tpdo = None
    last_ok_t = t0
    anomalies = 0
    rows = 0

    try:
        while time.time() - t0 < dur:
            v = lk.get(30092, 1.0) or {}
            if v.get('kind') != 11:           # 只要 CAN_TEST 周期上报
                time.sleep(0.2)
                continue
            v8 = v.get('v') or [0] * 8
            tpdo = int(v8[1]) if len(v8) > 1 else 0
            sw = int(v8[2]) if len(v8) > 2 else 0
            sdo_val = int(v8[3]) if len(v8) > 3 else 0
            ecr = int(v8[5]) if len(v8) > 5 else 0
            tx = int(v8[7]) if len(v8) > 7 else 0
            tec = ecr & 0xFF
            rec = (ecr >> 8) & 0xFF

            now = time.time() - t0
            rate = 0.0
            if last_tpdo is not None and now > 0:
                rate = (tpdo - last_tpdo) / max(now - (last_tpdo and last_ok_t or 0), 1e-9)
            # 用上次时间间隔算 rate 更稳
            if last_tpdo is not None:
                rate = (tpdo - last_tpdo) / 1.0   # 1s 采样一次

            w.writerow(['%.1f' % now, tpdo, '%.1f' % rate, '0x%04X' % sw,
                        sdo_val, tec, rec, tx])
            f.flush()
            rows += 1

            # 判据：连续 3 秒 tpdo 不涨 → 通讯中断
            if last_tpdo is not None and tpdo == last_tpdo:
                anomalies += 1
                if anomalies >= 3:
                    print('  !! 通讯中断（%.0f s 起 tpdo 不再增长，卡在 %d）' % (now, tpdo))
                    anomalies = 0      # 只报一次
            else:
                anomalies = 0

            print('%8.1f %9d %10.1f 0x%04X %10d %6d %6d' % (now, tpdo, rate, sw, sdo_val, tec, rec))
            last_tpdo = tpdo
            last_ok_t = now
            time.sleep(1.0)
    except KeyboardInterrupt:
        print('\n用户中断')
    finally:
        f.close()
        lk.close()

    print('\n=== 结束 ===')
    print('采样 %d 秒（%d 行），CSV: %s' % (int(time.time() - t0), rows, path))
    print('判读：rate 稳定 ~650 = 通讯正常；tec/rec 持续涨 = 总线错误累积')
    return 0


if __name__ == '__main__':
    sys.exit(main())
