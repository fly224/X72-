#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CANopen SDO 协议一致性测试（软件在环，不需要 CAN 硬件）。

为什么需要它
  手上没有 USB-CAN 分析仪时，唯一能系统性验证 SDO 编解码的办法是：
  把固件里的逻辑**逐行复刻**成 Python，然后用手册附录 D 的**真实报文**去打，
  看应答是否合理。本文件就是那个复刻 + 测试台。

  ⚠ 它是"逻辑等价模型"，不是固件本身。能抓出编码/查表/长度类的错误
  （已经抓到 2 个：下载长度算成 3-n、上传应答用减法算 scs），
  但验证不了物理层和真实主站兼容性。

复刻对应关系
  固件 Board/canopen.c:  s_od[]          ↔ 本文件 OD
                         sdo_handle()    ↔ 本文件 sdo()
                         od_write/od_read↔ 本文件读写字宽逻辑

用法
  python canopen_sdo_test.py
"""
import struct

NODE = 1
OD = {}


def add(idx, sub, typ, val, access='rw', name=''):
    OD[(idx, sub)] = {'type': typ, 'val': val, 'acc': access, 'name': name}


W = {'u8': 1, 'i8': 1, 'u16': 2, 'i16': 2, 'u32': 4, 'i32': 4}

# ---- 与固件 s_od[] 一一对应 ----
add(0x1000, 0, 'u32', 0x00020192, 'ro', 'Device type')
add(0x1001, 0, 'u8',  0x00,       'ro', 'Error register')
add(0x1005, 0, 'u32', 0x080,      'rw', 'COB-ID SYNC')
add(0x1006, 0, 'u32', 1000,       'rw', 'Comm cycle')
add(0x1014, 0, 'u32', 0x080 + NODE, 'rw', 'COB-ID EMCY')
add(0x1017, 0, 'u16', 1000,       'rw', 'Heartbeat ms')
add(0x2000, 0, 'u32', 10000,      'rw', 'Pulse per rev')
add(0x2001, 0, 'u32', 9,          'rw', 'Gear ratio')
add(0x2002, 0, 'u32', 15000,      'rw', 'MIT kp x1000')
add(0x2003, 0, 'u32', 300,        'rw', 'MIT kd x1000')
add(0x2004, 0, 'i32', 0,          'rw', 'MIT tau mNm')
add(0x6040, 0, 'u16', 0,          'rw', 'Controlword')
add(0x6041, 0, 'u16', 0x0040,     'ro', 'Statusword')
add(0x6060, 0, 'i8',  0,          'rw', 'Modes of operation')
add(0x6061, 0, 'i8',  0,          'ro', 'Modes display')
add(0x6064, 0, 'i32', 0,          'ro', 'Position actual')
add(0x606C, 0, 'i32', 0,          'ro', 'Velocity actual')
add(0x607A, 0, 'i32', 0,          'rw', 'Target position')
add(0x6081, 0, 'u32', 0,          'rw', 'Profile velocity')
add(0x6083, 0, 'u32', 0,          'rw', 'Profile accel')
add(0x6084, 0, 'u32', 0,          'rw', 'Profile decel')
add(0x6085, 0, 'u32', 0,          'rw', 'Quick stop decel')
add(0x60FF, 0, 'i32', 0,          'rw', 'Target velocity')
add(0x1400, 0, 'u8',  3,          'ro', 'RPDO1 nsub')
add(0x1400, 1, 'u32', 0x200 + NODE, 'rw', 'RPDO1 COB-ID')
add(0x1400, 2, 'u8',  255,        'rw', 'RPDO1 type')
add(0x1400, 3, 'u16', 0,          'rw', 'RPDO1 event timer')
add(0x1600, 0, 'u8',  2,          'rw', 'RPDO1 nmap')
add(0x1600, 1, 'u32', 0x60400010, 'rw', 'RPDO1 map1')
add(0x1600, 2, 'u32', 0x607A0020, 'rw', 'RPDO1 map2')
add(0x1600, 3, 'u32', 0,          'rw', 'RPDO1 map3')
add(0x1600, 4, 'u32', 0,          'rw', 'RPDO1 map4')
add(0x1401, 0, 'u8',  3,          'ro', 'RPDO2 nsub')
add(0x1401, 1, 'u32', 0x300 + 1,  'rw', 'RPDO2 COB-ID')
add(0x1401, 2, 'u8',  255,        'rw', 'RPDO2 type')
add(0x1401, 3, 'u16', 0,          'rw', 'RPDO2 event timer')
add(0x1601, 0, 'u8',  2,          'rw', 'RPDO2 nmap')
add(0x1601, 1, 'u32', 0x60FF0020, 'rw', 'RPDO2 map1 = 60FF')
add(0x1601, 2, 'u32', 0x20040020, 'rw', 'RPDO2 map2 = tau')
add(0x1601, 3, 'u32', 0,          'rw', 'RPDO2 map3')
add(0x1601, 4, 'u32', 0,          'rw', 'RPDO2 map4')
add(0x1800, 0, 'u8',  3,          'ro', 'TPDO1 nsub')
add(0x1800, 1, 'u32', 0x180 + NODE, 'rw', 'TPDO1 COB-ID')
add(0x1800, 2, 'u8',  255,        'rw', 'TPDO1 type')
add(0x1800, 3, 'u16', 0,          'rw', 'TPDO1 event timer')
add(0x1A00, 0, 'u8',  2,          'rw', 'TPDO1 nmap')
add(0x1A00, 1, 'u32', 0x60410010, 'rw', 'TPDO1 map1')
add(0x1A00, 2, 'u32', 0x60640020, 'rw', 'TPDO1 map2')
add(0x1A00, 3, 'u32', 0,          'rw', 'TPDO1 map3')
add(0x1A00, 4, 'u32', 0,          'rw', 'TPDO1 map4')

ABORT = {0x06020000: 'NO_OBJECT', 0x06010002: 'READ_ONLY',
         0x06070010: 'TYPE_MISMATCH', 0x08000000: 'GENERAL'}


def pack_val(typ, v):
    if typ == 'u8':  return struct.pack('<B', v & 0xFF)
    if typ == 'i8':  return struct.pack('<b', v)
    if typ == 'u16': return struct.pack('<H', v & 0xFFFF)
    if typ == 'i16': return struct.pack('<h', v)
    if typ == 'u32': return struct.pack('<I', v & 0xFFFFFFFF)
    return struct.pack('<i', v)


def unpack_val(typ, b):
    n = W[typ]
    buf = bytes(b[:n]) + b'\0' * (n - len(b))
    if typ == 'u8':  return buf[0]
    if typ == 'i8':  return struct.unpack('<b', buf)[0]
    if typ == 'u16': return struct.unpack('<H', buf)[0]
    if typ == 'i16': return struct.unpack('<h', buf)[0]
    if typ == 'u32': return struct.unpack('<I', buf)[0]
    return struct.unpack('<i', buf)[0]


def sdo(req):
    """复刻固件 sdo_handle()。req 是 8 字节（不足补零）。"""
    buf = bytes(req) + b'\0' * (8 - len(req))
    dlc = len(req)
    ccs = buf[0] >> 5
    idx = buf[1] | (buf[2] << 8)
    sub = buf[3]
    it = OD.get((idx, sub))

    if dlc < 4:
        return ('abort', idx, sub, 0x08000000)
    if it is None:
        return ('abort', idx, sub, 0x06020000)

    if ccs == 1:                                    # 下载
        if ((buf[0] >> 1) & 1) == 0:
            return ('abort', idx, sub, 0x08000000)  # 不支持分段
        n = 4 - ((buf[0] >> 2) & 3)                 # ★ 修正过的长度算法
        need = W[it['type']]
        if it['acc'] == 'ro':
            return ('abort', idx, sub, 0x06010002)
        if n != need:
            return ('abort', idx, sub, 0x06070010)
        if dlc < 4 + n:
            return ('abort', idx, sub, 0x08000000)
        it['val'] = unpack_val(it['type'], buf[4:4 + n])
        return ('dl-ok', idx, sub, None)

    if ccs == 2:                                    # 上传
        if it['acc'] == 'wo':
            return ('abort', idx, sub, 0x06010001)
        need = W[it['type']]
        scs = 0x43 | ((4 - need) << 2)              # ★ 修正过的 scs 编码
        return ('ul', idx, sub, (scs, pack_val(it['type'], it['val'])))

    return ('abort', idx, sub, 0x08000000)


def show(r):
    kind, idx, sub, extra = r
    if kind == 'abort':
        return 'ABORT %02X %04X:%02X -> %08X (%s)' % (
            0x80, idx, sub, extra, ABORT.get(extra, '?'))
    if kind == 'dl-ok':
        return 'OK 60 %04X:%02X' % (idx, sub)
    scs, data = extra
    return 'OK %02X %04X:%02X = %s' % (scs, idx, sub, data.hex(' ').upper())


# ---- 手册附录 D 的 RPDO1 配置序列 + 基本读写 ----
CASES = [
    ('读设备类型 0x1000',        [0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0]),
    ('禁止 RPDO1 (0x1400:1)',    [0x23, 0x00, 0x14, 0x01, 0x00, 0x00, 0x00, 0x80]),
    ('清除 RPDO1 映射 0x1600:0', [0x2F, 0x00, 0x16, 0x00, 0x00]),
    ('RPDO1 同步型 0x1400:2',    [0x2F, 0x00, 0x14, 0x02, 0x01]),
    ('RPDO1 事件定时 0x1400:3',  [0x2B, 0x00, 0x14, 0x03, 0x90, 0x01]),
    ('映射 0x60FF 到 0x1600:1',  [0x23, 0x00, 0x16, 0x01, 0x20, 0x00, 0xFF, 0x60]),
    ('映射 0x6040 到 0x1600:2',  [0x23, 0x00, 0x16, 0x02, 0x10, 0x00, 0x40, 0x60]),
    ('映射个数 0x1600:0 = 2',    [0x2F, 0x00, 0x16, 0x00, 0x02]),
    ('开放 RPDO1 (0x1400:1)',    [0x23, 0x00, 0x14, 0x01, 0x01, 0x02, 0x00, 0x40]),
    ('读回 RPDO1 COB-ID',        [0x40, 0x00, 0x14, 0x01, 0, 0, 0, 0]),
    ('读回 0x1600:1 映射',       [0x40, 0x00, 0x16, 0x01, 0, 0, 0, 0]),
    ('写控制字 0x6040 = 0x0F',   [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00]),
    ('读回控制字',               [0x40, 0x40, 0x60, 0x00, 0, 0, 0, 0]),
    ('写目标位置 0x607A=10000',  [0x23, 0x7A, 0x60, 0x00, 0x10, 0x27, 0x00, 0x00]),
    ('读回目标位置',             [0x40, 0x7A, 0x60, 0x00, 0, 0, 0, 0]),
    ('写只读对象 0x6041（应拒）',[0x2B, 0x41, 0x60, 0x00, 0x27, 0x00]),
    ('写不存在 0x9999（应拒）',  [0x2B, 0x99, 0x99, 0x00, 0x00, 0x00]),
    ('宽度不符 0x6040 写4B(应拒)',[0x23, 0x40, 0x60, 0x00, 1, 2, 3, 4]),
    ('读速度反馈 0x606C',        [0x40, 0x6C, 0x60, 0x00, 0, 0, 0, 0]),
    # ---- MIT 模式相关 ----
    ('切 MIT 模式 6060=12',      [0x2F, 0x60, 0x60, 0x00, 0x0C]),
    ('读回模式',                 [0x40, 0x60, 0x60, 0x00, 0, 0, 0, 0]),
    ('写 MIT kp = 15000',        [0x23, 0x02, 0x20, 0x00, 0x98, 0x3A, 0x00, 0x00]),
    ('读回 kp',                  [0x40, 0x02, 0x20, 0x00, 0, 0, 0, 0]),
    ('写 MIT tau = -500 mNm',    [0x23, 0x04, 0x20, 0x00, 0x0C, 0xFE, 0xFF, 0xFF]),
    ('读回 tau',                 [0x40, 0x04, 0x20, 0x00, 0, 0, 0, 0]),
    ('读 RPDO2 COB-ID',          [0x40, 0x01, 0x14, 0x01, 0, 0, 0, 0]),
    ('读 RPDO2 映射2(tau)',      [0x40, 0x01, 0x16, 0x02, 0, 0, 0, 0]),
]


def main():
    print('=== CANopen SDO 一致性测试（手册附录 D 报文序列）===\n')
    print('%-28s %-8s %s' % ('用例', '请求', '模型应答'))
    print('-' * 78)
    bad = 0
    for name, req in CASES:
        r = sdo(req)
        print('%-28s %-24s %s' % (name, bytes(req).hex(' ').upper(), show(r)))
        if r[0] == 'abort' and '应拒' not in name:
            print('   !! 非预期 ABORT')
            bad += 1
    print('\n结论：%d 条用例，%d 条非预期失败' % (len(CASES), bad))

    # 写后读回一致性
    sdo([0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00])
    got = sdo([0x40, 0x40, 0x60, 0x00, 0, 0, 0, 0])[3][1]
    print('写 0x6040=0x000F → 读回 0x%04X  %s'
          % (struct.unpack('<H', got)[0], '一致' if got == b'\x0f\x00' else '**不一致**'))
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
