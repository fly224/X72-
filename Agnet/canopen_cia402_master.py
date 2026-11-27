#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CANopen 主站（PC 侧）：推进 CiA402 八态机 + 读写对象字典。

为什么先用 SDO 而不是 PDO：SDO 有应答、能报 abort code，能立刻区分
"帧没出去"和"从站拒绝了"。PDO 配错的典型现象是发了毫无反应，很难定位。
先 SDO 跑通状态机，再验 PDO 映射。

★ 硬约束：从站 CANOPEN_RX_TMO_MS = 500 ms
  Operational 且已使能后，从站 500 ms 收不到任何 SDO 就强制停机
  （canopen.c:603）。所以使能后的保活周期必须 <500 ms，本脚本用 100 ms。

用法
----
  python canopen_cia402_master.py --list

  python canopen_cia402_master.py -i pcan -c PCAN_USBBUS1 -n 1
  python canopen_cia402_master.py -i slcan -c COM9 -n 1
  python canopen_cia402_master.py -i canalystii -c 0 -n 1
  python canopen_cia402_master.py -i zlgcan -c 0 -n 1

  python canopen_cia402_master.py -i pcan -n 1 --read-only
  python canopen_cia402_master.py -i pcan -n 1 --pp 10000
"""
import argparse
import struct
import sys
import time

try:
    import can
except ImportError:
    print('缺少 python-can：python -m pip install python-can')
    raise SystemExit(2)

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

BITRATE = 1_000_000          # 与固件 fdcan.c 一致：170 MHz/(17×10 TQ)
SDO_TMO = 0.05

SW_MASK = 0x006F
SW_TEXT = {
    0x0000: 'Not ready to switch on',
    0x0040: 'Switch on disabled',
    0x0021: 'Ready to switch on',
    0x0023: 'Switched on',
    0x0027: 'Operation enabled',
    0x0007: 'Quick stop active',
    0x000F: 'Fault reaction active',
    0x0008: 'Fault',
}

CW_SHUTDOWN = 0x0006
CW_SWITCH_ON = 0x0007
CW_ENABLE = 0x000F
CW_DISABLE = 0x0000
CW_QUICK_STOP = 0x0002
CW_FAULT_RESET = 0x0080


def sw_name(sw):
    if sw is None:
        return '(读超时)'
    return SW_TEXT.get(sw & SW_MASK, '0x%04X' % sw)


class Master(object):
    def __init__(self, bus, node):
        self.bus = bus
        self.node = node
        self.cob_rx = 0x600 + node
        self.cob_tx = 0x580 + node
        self.abort = None

    def _send(self, cob, data):
        self.bus.send(can.Message(arbitration_id=cob, data=data,
                                  is_extended_id=False, is_fd=False))

    def _wait(self, cob, tmo=SDO_TMO):
        t_end = time.time() + tmo
        while time.time() < t_end:
            m = self.bus.recv(timeout=max(0.0, t_end - time.time()))
            if m is None:
                break
            if m.arbitration_id == cob and not m.is_extended_id:
                return bytes(m.data)
        return None

    def write(self, idx, sub, value, size):
        cs = {1: 0x2F, 2: 0x2B, 4: 0x27}[size]
        pack = {1: '<B', 2: '<H', 4: '<I'}[size]
        body = struct.pack(pack, value & 0xFFFFFFFF)
        frame = bytes([cs, idx & 0xFF, (idx >> 8) & 0xFF, sub]) + body
        frame += bytes(8 - len(frame))
        for _ in range(3):
            self._send(self.cob_rx, frame)
            r = self._wait(self.cob_tx)
            if r is None:
                continue
            if r[0] == 0x80:
                self.abort = struct.unpack_from('<I', r, 4)[0]
                return 'abort 0x%08X' % self.abort
            if r[0] == 0x60:
                return None
        return 'timeout'

    def read(self, idx, sub):
        for _ in range(3):
            frame = bytes([0x40, idx & 0xFF, (idx >> 8) & 0xFF, sub, 0, 0, 0, 0])
            self._send(self.cob_rx, frame)
            r = self._wait(self.cob_tx)
            if r is None:
                continue
            if r[0] == 0x80:
                return None, 'abort 0x%08X' % struct.unpack_from('<I', r, 4)[0]
            n = {0x4F: 1, 0x4B: 2, 0x43: 4}.get(r[0])
            if n is None:
                continue
            return int.from_bytes(r[4:4 + n], 'little', signed=(n == 4)), None
        return None, 'timeout'

    def nmt_start(self):
        self._send(0x000, bytes([0x01, self.node]))

    def status_word(self):
        return self.read(0x6041, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-i', '--interface', default='pcan')
    ap.add_argument('-c', '--channel', default=None)
    ap.add_argument('-n', '--node', type=int, default=1)
    ap.add_argument('-b', '--bitrate', type=int, default=BITRATE)
    ap.add_argument('--list', action='store_true', help='列出可用接口后退出')
    ap.add_argument('--read-only', action='store_true')
    ap.add_argument('--pp', type=int, default=None, help='使能后写 0x607A（pulse）')
    ap.add_argument('--hold', type=float, default=3.0, help='使能后保活秒数')
    ap.add_argument('--no-fault-reset', action='store_true')
    a = ap.parse_args()

    if a.list:
        cfgs = can.detect_available_configs()
        print('可用配置 %d 个：' % len(cfgs))
        for c in cfgs:
            print('  ', c)
        return 0

    kw = {'interface': a.interface, 'bitrate': a.bitrate}
    if a.channel is not None:
        kw['channel'] = a.channel
    try:
        bus = can.Bus(**kw)
    except Exception as e:
        print('打开 %s 失败：%s' % (a.interface, e))
        print('提示：python canopen_cia402_master.py --list 看可用接口')
        return 2

    m = Master(bus, a.node)
    print('=== CANopen 主站 ===')
    print('  接口 %s 通道 %s  %d bit/s  目标节点 %d  (SDO 0x%03X / 0x%03X)'
          % (a.interface, a.channel, a.bitrate, a.node, m.cob_rx, m.cob_tx))

    t0 = time.time()
    ok_all = True
    try:
        print('\n[1] NMT Start → 节点 %d' % a.node)
        m.nmt_start()
        time.sleep(0.1)

        dev, err = m.read(0x1000, 0)
        if err:
            print('    读 0x1000 失败：%s' % err)
            print('    → 检查：总线 120Ω、CAN_H/CAN_L 是否反、波特率 1 Mbit/s、'
                  '从站是否烧 CANopen 固件、节点号是否对')
            return 3
        print('    0x1000 设备类型 = 0x%08X  ✓ 节点在线（%.0f ms）'
              % (dev, (time.time() - t0) * 1000))

        sw, _ = m.status_word()
        print('    0x6041 状态字 = 0x%04X (%s)' % (sw, sw_name(sw)))
        mode, _ = m.read(0x6061, 0)
        print('    0x6061 模式 = %s' % mode)

        if a.read_only:
            print('\n--read-only：结束')
            return 0

        seq = []
        if (sw & SW_MASK) == 0x0008 and not a.no_fault_reset:
            seq.append(('FaultReset', CW_FAULT_RESET, 0x0040))
        seq += [('Shutdown', CW_SHUTDOWN, 0x0021),
                ('Switch on', CW_SWITCH_ON, 0x0023),
                ('Enable operation', CW_ENABLE, 0x0027)]

        print('\n[2] CiA402 使能序列（每步读 0x6041 校验）')
        for name, cw, expect in seq:
            e = m.write(0x6040, 0, cw, 2)
            if e:
                print('    写 0x6040=0x%04X 失败：%s' % (cw, e))
                ok_all = False
                break
            time.sleep(0.06)
            got, err = m.status_word()
            if err:
                print('    %-18s → 读状态失败 %s' % (name, err))
                ok_all = False
                break
            mark = '✓' if (got & SW_MASK) == expect else '✗'
            print('    %-18s cw=0x%04X  sw=0x%04X %s  期望 0x%04X (%s)'
                  % (name, cw, got, mark, expect, sw_name(got)))
            if mark == '✗':
                ok_all = False
                break

        cur, _ = m.status_word()
        if ok_all and (cur & SW_MASK) == 0x0027:
            print('\n[3] 已使能，保活 %.1f s（周期 100 ms < 从站 500 ms 超时）' % a.hold)
            if a.pp is not None:
                e = m.write(0x6060, 0, 1, 1)
                print('    写 0x6060 = 1 (PP) : %s' % (e or 'ok'))
                e = m.write(0x607A, 0, a.pp, 4)
                print('    写 0x607A = %d : %s' % (a.pp, e or 'ok'))
            t_end = time.time() + a.hold
            n = 0
            while time.time() < t_end:
                m.write(0x6040, 0, CW_ENABLE, 2)
                n += 1
                if n % 5 == 0:
                    sw2, _ = m.status_word()
                    pos, _ = m.read(0x6064, 0)
                    print('    sw=0x%04X (%s)  0x6064=%s'
                          % (sw2, sw_name(sw2), pos))
                time.sleep(0.1)

        print('\n[4] 收尾：Disable voltage')
        m.write(0x6040, 0, CW_DISABLE, 2)
        time.sleep(0.05)
        sw, _ = m.status_word()
        print('    0x6041 = 0x%04X (%s)' % (sw, sw_name(sw)))
        print('\n总用时 %.2f s，结论：%s' % (time.time() - t0,
              '通过' if ok_all else '未通过'))
        return 0 if ok_all else 4
    finally:
        try:
            m.write(0x6040, 0, CW_DISABLE, 2)
        except Exception:
            pass
        bus.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
