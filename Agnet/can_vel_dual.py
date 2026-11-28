#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CANopen 双节点速度测试：两台同时跑不同转速，周期保活 + 监控。

口径（与 can_vel_test.py 一致）：
  0x60FF 单位输出端 pulse/s，ppr=10000，gear=9
  电机端 rpm -> 0x60FF = rpm/60/gear*ppr

用法：
  python can_vel_dual.py -n1 3 --rpm1 3000 -n2 4 --rpm2 1000 -b 1000000 --hold 5
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

GEAR = 9
PPR = 10000
SW_MASK = 0x006F


def sw_name(sw):
    if sw is None:
        return '(超时)'
    return {0x0040: 'SwitchOnDisabled', 0x0021: 'Ready', 0x0023: 'SwitchedOn',
            0x0027: 'OpEnabled', 0x0008: 'Fault'}.get(sw & SW_MASK, '0x%04X' % sw)


class Master:
    def __init__(self, bus, node):
        self.bus = bus
        self.node = node
        self.cob_rx = 0x600 + node
        self.cob_tx = 0x580 + node

    def _send(self, cob, data):
        self.bus.send(can.Message(arbitration_id=cob, data=data,
                                  is_extended_id=False))

    def _wait(self, cob, tmo=0.05):
        t_end = time.time() + tmo
        while time.time() < t_end:
            m = self.bus.recv(timeout=max(0.0, t_end - time.time()))
            if m is None:
                break
            if m.arbitration_id == cob and not m.is_extended_id:
                return bytes(m.data)
        return None

    def write(self, idx, sub, value, size, tries=4):
        cs = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]
        pack = {1: '<B', 2: '<H', 4: '<I'}[size]
        body = struct.pack(pack, value & 0xFFFFFFFF)
        frame = bytes([cs, idx & 0xFF, (idx >> 8) & 0xFF, sub]) + body
        frame += bytes(8 - len(frame))
        for _ in range(tries):
            self._send(self.cob_rx, frame)
            r = self._wait(self.cob_tx)
            if r is None:
                continue
            if r[0] == 0x80:
                return 'abort 0x%08X' % struct.unpack_from('<I', r, 4)[0]
            if r[0] == 0x60:
                return None
        return 'timeout'

    def read(self, idx, sub, signed=False, tries=3):
        frame = bytes([0x40, idx & 0xFF, (idx >> 8) & 0xFF, sub, 0, 0, 0, 0])
        for _ in range(tries):
            self._send(self.cob_rx, frame)
            r = self._wait(self.cob_tx)
            if r is None:
                continue
            if r[0] == 0x80:
                return None, 'abort 0x%08X' % struct.unpack_from('<I', r, 4)[0]
            n = {0x4F: 1, 0x4B: 2, 0x43: 4}.get(r[0])
            if n is None:
                continue
            return int.from_bytes(r[4:4 + n], 'little', signed=signed or n == 4), None
        return None, 'timeout'


def rpm_to_pulse(rpm):
    return int(round(rpm / 60.0 / GEAR * PPR))


def pulse_to_rpm(pulse):
    return pulse * 60.0 / PPR * GEAR


def enable_node(m, rpm):
    """切 PV 模式 + 清故障 + 使能，返回 0 成功"""
    m._send(0x000, bytes([0x01, m.node]))
    time.sleep(0.05)
    sw, _ = m.read(0x6041, 0)
    if (sw is not None) and (sw & SW_MASK) == 0x0008:
        m.write(0x6040, 0, 0x0080, 2); time.sleep(0.03)
        m.write(0x6040, 0, 0x0000, 2); time.sleep(0.08)
    m.write(0x6060, 0, 3, 1)   # MODE_PV
    for cw in (0x0006, 0x0007, 0x000F):
        m.write(0x6040, 0, cw, 2)
        time.sleep(0.08)
    sw, _ = m.read(0x6041, 0)
    return (sw & SW_MASK) == 0x0027


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-i', '--interface', default='pcan')
    ap.add_argument('-c', '--channel', default='PCAN_USBBUS1')
    ap.add_argument('-b', '--bitrate', type=int, default=1000000)
    ap.add_argument('-n1', '--node1', type=int, default=3)
    ap.add_argument('--rpm1', type=float, default=3000.0)
    ap.add_argument('-n2', '--node2', type=int, default=4)
    ap.add_argument('--rpm2', type=float, default=1000.0)
    ap.add_argument('--hold', type=float, default=5.0)
    a = ap.parse_args()

    bus = can.Bus(interface=a.interface, channel=a.channel, bitrate=a.bitrate)
    m1 = Master(bus, a.node1)
    m2 = Master(bus, a.node2)
    p1 = rpm_to_pulse(a.rpm1)
    p2 = rpm_to_pulse(a.rpm2)
    print('节点%d: %g rpm -> 0x60FF=%d | 节点%d: %g rpm -> 0x60FF=%d'
          % (a.node1, a.rpm1, p1, a.node2, a.rpm2, p2))

    print('\n[使能]')
    ok1 = enable_node(m1, a.rpm1)
    print('  节点%d: %s' % (a.node1, 'Enabled ✓' if ok1 else '✗'))
    ok2 = enable_node(m2, a.rpm2)
    print('  节点%d: %s' % (a.node2, 'Enabled ✓' if ok2 else '✗'))
    if not (ok1 and ok2):
        print('使能失败，终止')
        bus.shutdown()
        return 2

    e1 = m1.write(0x60FF, 0, p1, 4)
    e2 = m2.write(0x60FF, 0, p2, 4)
    print('\n[下发速度] 节点%d: %s | 节点%d: %s'
          % (a.node1, e1 or 'ok', a.node2, e2 or 'ok'))

    print('\n[监控]（每 0.2s 保活两个节点）')
    t0 = time.time()
    r1 = []
    r2 = []
    while time.time() - t0 < a.hold:
        sw1, _ = m1.read(0x6041, 0)   # 保活
        sw2, _ = m2.read(0x6041, 0)
        fb1, _ = m1.read(0x606C, 0, signed=True)
        fb2, _ = m2.read(0x606C, 0, signed=True)
        v1 = pulse_to_rpm(fb1) if fb1 is not None else float('nan')
        v2 = pulse_to_rpm(fb2) if fb2 is not None else float('nan')
        r1.append(v1); r2.append(v2)
        print('  t=%4.1fs  节点%d: %8.1f rpm | 节点%d: %8.1f rpm'
              % (time.time() - t0, a.node1, v1, a.node2, v2))
        time.sleep(0.2)

    print('\n[停止]')
    m1.write(0x60FF, 0, 0, 4)
    m2.write(0x60FF, 0, 0, 4)
    time.sleep(0.6)
    m1.write(0x6040, 0, 0x0000, 2)
    m2.write(0x6040, 0, 0x0000, 2)
    time.sleep(0.1)

    r1 = [v for v in r1 if v == v]
    r2 = [v for v in r2 if v == v]
    print('\n节点%d 峰速 %.1f rpm（目标 %g） | 节点%d 峰速 %.1f rpm（目标 %g）'
          % (a.node1, max(map(abs, r1)) if r1 else 0, a.rpm1,
             a.node2, max(map(abs, r2)) if r2 else 0, a.rpm2))
    bus.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
