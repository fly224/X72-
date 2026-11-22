#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
X72_FOC 串口联通检查 —— USART1 / RS-485 / 921600 8N1

用途
----
在接电机、开闭环、标定之前，先确认"板子能持续把帧发出来、主机能正确解出来"。
本工具只读不写：不向板子下发任何字节，因此可以在任何状态下安全运行。

它同时看两条流
--------------
1) ASCII 横幅（uart_link.c 发出）：任何串口助手都能看到，用来确认 TX 物理层通。
2) 二进制遥测（telemetry_tx.c 发出）：MAVLink v2 帧，100 Hz 成组发
   30025 CurrentCtlInput + 30027 CurrentCtlOutput + 30040 MotorPosSensorOutput。
   本工具做 CRC-16/X.25 校验、帧率统计、字段解码，最后给 PASS / FAIL 判定。

判定标准（全部满足才 PASS）
--------------------------
  A. 30025 实测帧率 >= 期望帧率的 80%
  B. CRC 错误帧数 == 0
  C. 30040、30027 相对 30025 的帧数比值都落在 0.90 ~ 1.10（三条报文成组发出）
  D. 最大帧间隔 < 100 ms（无长时间断流）
  E. 母线电压 0 <= V <= 60，三相电流 |I| <= 20 A（排除解错位的可能）

用法
----
  python serial_link_check.py                  # 自动列串口并选第一个
  python serial_link_check.py -p COM5
  python serial_link_check.py -p COM5 -t 10    # 观察 10 秒
  python serial_link_check.py --list           # 只列串口
  python serial_link_check.py -p COM5 --ascii  # 只看文本横幅（等同串口助手）
  python serial_link_check.py --selftest       # 用合成帧验证解析器（不接硬件）

字段偏移与 telemetry_tx.c 的 put_f32() 一一对应；改动固件组帧时必须同步改本文件，
并跑 --selftest 确认。
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass, field

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    sys.stderr.write("缺少 pyserial：请执行  python -m pip install pyserial\n")
    raise SystemExit(2)


# --------------------------------------------------------------------------
# 协议常量（与 telemetry_tx.c 保持一致）
# --------------------------------------------------------------------------
STX = 0xFD
HDR_LEN = 10          # STX + LEN + incompat + compat + SEQ + SYSID + COMPID + msgid(3)

MSG_CUR_IN = 30025
MSG_CUR_OUT = 30027
MSG_POS = 30040
CRC_EXTRA = {MSG_CUR_IN: 191, MSG_CUR_OUT: 58, MSG_POS: 160}
PAYLOAD_LEN = {MSG_CUR_IN: 57, MSG_CUR_OUT: 32, MSG_POS: 12}   # 30025：53→57（+53 motor_temp）
EXPECTED_FPS = 100.0
FRAME_BYTES = (10 + 57 + 2) + (10 + 32 + 2) + (10 + 12 + 2)   # 一组的字节数 = 137
ENC_COUNTS = 2097152.0          # MT6835 2^21 脉冲/圈

# 30025 CurrentCtlInput 字段表：(偏移, 名称, 单位)
FIELDS_IN = [
    (0,  "idq_tar_Id",        "A"),
    (4,  "idq_tar_Iq",        "A"),
    (8,  "iabc_iu",           "A"),
    (12, "iabc_iv",           "A"),
    (16, "iabc_iw",           "A"),
    (20, "elec_theta",        "rad"),
    (24, "dc_bus",            "V"),
    (28, "uabc_ua",           "V"),
    (32, "uabc_ub",           "V"),
    (36, "uabc_uc",           "V"),
    (40, "driver_temp",       "C"),
    (44, "elec_speed",        "rad/s"),
    (48, "iq_offset",         "A"),
]

# 30027 CurrentCtlOutput 字段表
FIELDS_OUT = [
    (0,  "uabc_comp_ua",      "V"),
    (4,  "uabc_comp_ub",      "V"),
    (8,  "uabc_comp_uc",      "V"),
    (12, "idq_now_Id",        "A"),
    (16, "idq_now_Iq",        "A"),
    (20, "uabc_org_ua",       "V"),
    (24, "uabc_org_ub",       "V"),
    (28, "uabc_org_uc",       "V"),
]


# --------------------------------------------------------------------------
# CRC-16/X.25（MAVLink 标准，含 crc_extra）
# --------------------------------------------------------------------------
def crc_x25(data: bytes, extra: int) -> int:
    crc = 0xFFFF
    for b in data:
        t = (b ^ (crc & 0xFF)) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    t = (extra ^ (crc & 0xFF)) & 0xFF
    t = (t ^ (t << 4)) & 0xFF
    crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


# --------------------------------------------------------------------------
# 帧解析
# --------------------------------------------------------------------------
@dataclass
class Stats:
    ok: dict = field(default_factory=lambda: {MSG_CUR_IN: 0, MSG_CUR_OUT: 0, MSG_POS: 0})
    crc_err: int = 0
    crc_err_head: int = 0       # 抓包起点未对齐的残帧（第一个合法帧之前的错），不算链路误码
    crc_legacy: int = 0         # 旧固件口径（CRC 含 0xFD 魔数）命中数，仅供兼容统计
    unknown_id: int = 0
    padding_bytes: int = 0      # 帧间被丢弃的非帧头字节（横幅、噪声、半帧）
    total_bytes: int = 0
    max_gap_ms: float = 0.0
    last_frame_ts: float | None = None
    last_payload: dict = field(default_factory=dict)

    @property
    def frames(self) -> int:
        return sum(self.ok.values())


class FrameParser:
    """滑动窗口 MAVLink v2 解析器，容忍 ASCII 横幅与半帧。"""

    def __init__(self, stats: Stats):
        self.buf = bytearray()
        self.stats = stats
        self.synced = False     # 是否已锁定帧边界：锁定前的 CRC 错属于抓包起点残帧

    def feed(self, chunk: bytes) -> None:
        self.buf.extend(chunk)
        self.stats.total_bytes += len(chunk)
        self._drain()

    def _drain(self) -> None:
        buf = self.buf
        while True:
            k = buf.find(STX)
            if k < 0:
                # 整段没有帧头：全部算填充（横幅文本走这里）
                self.stats.padding_bytes += len(buf)
                buf.clear()
                return
            if k > 0:
                self.stats.padding_bytes += k
                del buf[:k]

            if len(buf) < HDR_LEN:
                return

            ln = buf[1]
            msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
            if ln > 64:
                # 长度非法，说明这个 0xFD 是数据里的巧合，跳过它
                self.stats.padding_bytes += 1
                del buf[:1]
                continue

            total = 12 + ln
            if len(buf) < total:
                return

            raw = bytes(buf[:total])
            if msgid in CRC_EXTRA:
                # MAVLink v2 标准口径：从 len 字节起（不含 0xFD 魔数）
                body = raw[1:HDR_LEN + ln]
                got = raw[HDR_LEN + ln] | (raw[HDR_LEN + ln + 1] << 8)
                want = crc_x25(body, CRC_EXTRA[msgid])
                if got != want:
                    # 兼容 2026-11-15 之前的旧固件：CRC 含帧首 0xFD
                    if crc_x25(raw[:HDR_LEN + ln], CRC_EXTRA[msgid]) == got:
                        self.stats.crc_legacy += 1
                    else:
                        # 第一个合法帧之前的错：抓包从数据流中间切入留下的残帧，
                        # 不是链路误码，单独计数（否则每次抓包都会误报 B 项 FAIL）
                        if self.synced:
                            self.stats.crc_err += 1
                        else:
                            self.stats.crc_err_head += 1
                        del buf[:1]
                        continue
                self.synced = True
                if ln != PAYLOAD_LEN[msgid]:
                    # CRC 对但长度不符：方言版本不匹配，按未知处理
                    self.stats.unknown_id += 1
                    del buf[:total]
                    continue

                now = time.monotonic()
                if self.stats.last_frame_ts is not None:
                    gap = (now - self.stats.last_frame_ts) * 1000.0
                    if gap > self.stats.max_gap_ms:
                        self.stats.max_gap_ms = gap
                self.stats.last_frame_ts = now
                self.stats.ok[msgid] += 1
                self.stats.last_payload[msgid] = raw[HDR_LEN:HDR_LEN + ln]
                del buf[:total]
            else:
                self.stats.unknown_id += 1
                del buf[:total]


def decode(payload: bytes, fields) -> dict:
    out = {}
    for off, name, unit in fields:
        if off + 4 <= len(payload):
            (v,) = struct.unpack_from("<f", payload, off)
            out[name] = (v, unit)
    return out


# --------------------------------------------------------------------------
# selftest：不接硬件也能验证解析器
# --------------------------------------------------------------------------
def build_frame(msgid: int, seq: int, payload: bytes, legacy_crc: bool = False) -> bytes:
    """legacy_crc=True 复现 2026-11-15 之前固件的错误口径（CRC 含 0xFD），仅用于兼容性自测。"""
    hdr = bytes([STX, len(payload), 0, 0, seq & 0xFF, 1, 1,
                 msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    body = hdr + payload
    crc = crc_x25(body if legacy_crc else body[1:], CRC_EXTRA[msgid])
    return body + bytes([crc & 0xFF, crc >> 8])


def selftest() -> int:
    st = Stats()
    pr = FrameParser(st)

    # 造一条 30025（LEN=57），vbus=48.3，Iq=1.25，GAN=41.2°C，MOTOR=36.8°C
    p_in = bytearray(57)
    struct.pack_into("<f", p_in, 4, 1.25)
    struct.pack_into("<f", p_in, 24, 48.3)
    struct.pack_into("<f", p_in, 40, 41.2)
    p_in[52] = 2
    struct.pack_into("<f", p_in, 53, 36.8)

    p_out = bytearray(32)
    struct.pack_into("<f", p_out, 12, 0.003)
    struct.pack_into("<f", p_out, 16, 1.24)

    # 30040：enc_sum = 1.25 圈，电角度 3.5
    p_pos = bytearray(12)
    struct.pack_into("<q", p_pos, 0, 2621440)
    struct.pack_into("<f", p_pos, 8, 3.5)

    stream = (b"\r\n[BOOT] banner line\r\n"
              + build_frame(MSG_CUR_IN, 0, bytes(p_in))
              + build_frame(MSG_CUR_OUT, 1, bytes(p_out))
              + build_frame(MSG_POS, 2, bytes(p_pos))
              + b"garbage\xfd\xff"
              + build_frame(MSG_CUR_IN, 3, bytes(p_in)))

    for i in range(0, len(stream), 7):          # 故意小块喂，模拟串口碎片
        pr.feed(stream[i:i + 7])

    d_in = decode(st.last_payload[MSG_CUR_IN], FIELDS_IN)
    (enc_sum,) = struct.unpack_from("<q", st.last_payload[MSG_POS], 0)
    pos_ok = (enc_sum == 2621440) and (st.ok[MSG_POS] == 1)
    ok = (st.ok[MSG_CUR_IN] == 2 and st.ok[MSG_CUR_OUT] == 1 and pos_ok
          and st.crc_err == 0
          and abs(d_in["dc_bus"][0] - 48.3) < 1e-3
          and abs(d_in["idq_tar_Iq"][0] - 1.25) < 1e-6)

    print("=== 解析器自检 ===")
    print(f"  30025 帧数     : {st.ok[MSG_CUR_IN]}  (期望 2)")
    print(f"  30027 帧数     : {st.ok[MSG_CUR_OUT]}  (期望 1)")
    print(f"  30040 帧数     : {st.ok[MSG_POS]}  (期望 1)")
    print(f"  CRC 错误       : {st.crc_err}  (期望 0)")
    print(f"  旧口径帧       : {st.crc_legacy}  (期望 0；>0 说明固件未换成标准口径)")
    print(f"  解出 dc_bus    : {d_in['dc_bus'][0]:.3f} V  (期望 48.300)")
    print(f"  解出 Iq 给定   : {d_in['idq_tar_Iq'][0]:.3f} A  (期望 1.250)")
    print(f"  解出 enc_sum   : {enc_sum}  (期望 2621440 = 1.25 圈)")

    # 旧口径兼容：2026-11-15 之前固件发出的帧（CRC 含 0xFD）仍要能解析，
    # 只计入 crc_legacy，不判为 CRC 错 —— 历史抓包因此可以继续回放。
    st_l = Stats()
    pr_l = FrameParser(st_l)
    pr_l.feed(build_frame(MSG_CUR_IN, 0, bytes(p_in), legacy_crc=True))
    legacy_ok = (st_l.crc_legacy == 1 and st_l.crc_err == 0 and st_l.ok[MSG_CUR_IN] == 1)
    print(f"  旧口径兼容     : {'PASS' if legacy_ok else 'FAIL'}  "
          f"(legacy={st_l.crc_legacy} err={st_l.crc_err} ok={st_l.ok[MSG_CUR_IN]})")
    ok = ok and legacy_ok

    print(f"  结论           : {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


# --------------------------------------------------------------------------
# 串口枚举
# --------------------------------------------------------------------------
def pick_port(hint: str | None) -> str | None:
    ports = list(list_ports.comports())
    if hint:
        return hint
    if not ports:
        print("未发现任何串口。检查 USB-485 转换器是否插好、驱动是否装好。")
        return None

    print("可用串口：")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device:<8} {p.description}")
    # 优先选带 USB / 485 / CH340 / CP210 / FTDI 字样的
    keys = ("usb", "485", "ch340", "cp210", "ftdi", "silicon")
    for p in ports:
        txt = f"{p.device} {p.description}".lower()
        if any(k in txt for k in keys):
            print(f"自动选择: {p.device}")
            return p.device
    print(f"自动选择: {ports[0].device}")
    return ports[0].device


# --------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------
def run(port: str, baud: int, secs: float, ascii_only: bool) -> int:
    print("=" * 62)
    print(" X72_FOC 串口联通检查")
    print("=" * 62)
    print(f" 端口 : {port}   {baud} 8N1   观察 {secs:.1f} s")
    print()

    try:
        ser = serial.Serial(port, baud, bytesize=8, parity="N",
                            stopbits=1, timeout=0.05)
    except Exception as e:                       # noqa: BLE001
        print(f"[FAIL] 打不开 {port}: {e}")
        print("       常见原因：端口被串口助手占用 / 驱动没装 / 端口号写错")
        return 2

    # 不主动拉 DTR/RTS：板端是 RS-485 半双工，避免转换器把总线拉死
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:                            # noqa: BLE001
        pass

    st = Stats()
    pr = FrameParser(st)
    text_buf = bytearray()
    ascii_lines: list[str] = []

    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < secs:
            chunk = ser.read(4096)
            if not chunk:
                continue
            if ascii_only:
                text_buf.extend(chunk)
                while b"\n" in text_buf:
                    line, _, rest = text_buf.partition(b"\n")
                    text_buf = bytearray(rest)
                    s = line.decode("ascii", "replace").rstrip("\r")
                    if s:
                        ascii_lines.append(s)
                        print("  " + s)
            else:
                pr.feed(chunk)
    except KeyboardInterrupt:
        print("\n（用户中断，按已收数据判定）")
    finally:
        elapsed = max(time.monotonic() - t0, 1e-6)
        ser.close()

    if ascii_only:
        print()
        print(f"收到 {len(ascii_lines)} 行文本，{st.total_bytes} 字节")
        if ascii_lines:
            print("结论 : PASS —— TX 已通（能看到板子主动输出的文本）")
            return 0
        print("结论 : FAIL —— 一个字都没收到，见下方排查顺序")
        return 1

    return report(st, elapsed, secs)


def report(st: Stats, elapsed: float, secs: float) -> int:
    n_in = st.ok[MSG_CUR_IN]
    n_out = st.ok[MSG_CUR_OUT]
    n_pos = st.ok[MSG_POS]
    fps_in = n_in / elapsed
    fps_out = n_out / elapsed
    fps_pos = n_pos / elapsed
    expect = EXPECTED_FPS * elapsed
    throughput = st.total_bytes / elapsed

    print()
    print("-" * 62)
    print(" 帧统计")
    print("-" * 62)
    print(f"  30025 CurrentCtlInput    : {n_in:>5} 帧   {fps_in:6.1f} fps  (期望 {EXPECTED_FPS:.0f})")
    print(f"  30027 CurrentCtlOutput   : {n_out:>5} 帧   {fps_out:6.1f} fps  (期望 {EXPECTED_FPS:.0f})")
    print(f"  30040 MotorPosSensorOut  : {n_pos:>5} 帧   {fps_pos:6.1f} fps  (期望 {EXPECTED_FPS:.0f})")
    print(f"  CRC 错误 / 未知 ID      : {st.crc_err} / {st.unknown_id}  "
          f"(起点残帧 {st.crc_err_head}，旧口径帧 {st.crc_legacy})")
    print(f"  帧间丢弃字节            : {st.padding_bytes}")
    print(f"  最大帧间隔              : {st.max_gap_ms:.1f} ms")
    print(f"  总字节 / 吞吐           : {st.total_bytes} B / {throughput:.0f} B/s  (理论 {FRAME_BYTES*EXPECTED_FPS:.0f})")

    # ---- 字段解码 ----
    d_in = decode(st.last_payload[MSG_CUR_IN], FIELDS_IN) if MSG_CUR_IN in st.last_payload else {}
    d_out = decode(st.last_payload[MSG_CUR_OUT], FIELDS_OUT) if MSG_CUR_OUT in st.last_payload else {}
    d_pos = {}
    if MSG_POS in st.last_payload and len(st.last_payload[MSG_POS]) >= 12:
        p = st.last_payload[MSG_POS]
        (enc_sum,) = struct.unpack_from("<q", p, 0)
        (elec_now,) = struct.unpack_from("<f", p, 8)
        frac = enc_sum % int(ENC_COUNTS)
        d_pos = {
            "enc_sum": enc_sum,
            "turns": enc_sum / ENC_COUNTS,
            "theta_m": frac / ENC_COUNTS * 2.0 * math.pi,
            "elec": elec_now,
        }

    if d_in or d_out or d_pos:
        print()
        print("-" * 62)
        print(" 最新一帧")
        print("-" * 62)
        if d_in:
            v = d_in
            print(f"  给定 Id / Iq        : {v['idq_tar_Id'][0]:+8.3f} / {v['idq_tar_Iq'][0]:+8.3f} A")
            print(f"  实测三相 iu/iv/iw   : {v['iabc_iu'][0]:+8.3f} / {v['iabc_iv'][0]:+8.3f} / {v['iabc_iw'][0]:+8.3f} A")
            print(f"  母线电压 dc_bus     : {v['dc_bus'][0]:8.2f} V")
            print(f"  电角度 / 电角速度   : {v['elec_theta'][0]:+8.3f} rad / {v['elec_speed'][0]:+8.2f} rad/s")
            print(f"  相电压 ua/ub/uc     : {v['uabc_ua'][0]:+8.2f} / {v['uabc_ub'][0]:+8.2f} / {v['uabc_uc'][0]:+8.2f} V")
            t, _ = v["driver_temp"]
            print(f"  驱动温度            : {'-- (NaN，NTC 未接 ADC)' if t != t else f'{t:8.2f} C'}")
        if d_out:
            v = d_out
            print(f"  实测 Id / Iq        : {v['idq_now_Id'][0]:+8.3f} / {v['idq_now_Iq'][0]:+8.3f} A")
        if d_pos:
            deg = d_pos["theta_m"] * 180.0 / math.pi
            print(f"  累积脉冲 enc_sum    : {d_pos['enc_sum']:>12d}  ({d_pos['turns']:+.4f} 圈)")
            print(f"  单圈机械角          : {d_pos['theta_m']:6.4f} rad  ({deg:7.2f} °)")
            print(f"  电角度(30040)       : {d_pos['elec']:+8.3f} rad")

    # ---- 判定 ----
    print()
    print("=" * 62)
    checks = []

    checks.append((f"A 帧率达标", fps_in >= 0.8 * EXPECTED_FPS,
                   f"{fps_in:.1f} / {EXPECTED_FPS:.0f} fps"))
    checks.append((f"B CRC 全对", st.crc_err == 0,
                   f"{st.crc_err} 错" + (f"，旧口径 {st.crc_legacy} 帧" if st.crc_legacy else "")
                   + (f"，起点残帧 {st.crc_err_head}（不计）" if st.crc_err_head else "")))
    ratio_out = (n_out / n_in) if n_in else 0.0
    ratio_pos = (n_pos / n_in) if n_in else 0.0
    trio_ok = (0.90 <= ratio_out <= 1.10) and (0.90 <= ratio_pos <= 1.10)
    checks.append((f"C 三报文成对", trio_ok,
                   f"30027/30025={ratio_out:.3f} 30040/30025={ratio_pos:.3f}"))
    checks.append((f"D 无断流", st.max_gap_ms < 100.0, f"最大间隔 {st.max_gap_ms:.1f} ms"))

    vbus_ok = True
    cur_ok = True
    if d_in:
        vb = d_in["dc_bus"][0]
        vbus_ok = (vb == vb) and (0.0 <= vb <= 60.0)
        for k in ("iabc_iu", "iabc_iv", "iabc_iw"):
            x = d_in[k][0]
            if not (x == x) or abs(x) > 20.0:
                cur_ok = False
    checks.append(("E 量纲/偏移合理", vbus_ok and cur_ok,
                   f"vbus={d_in['dc_bus'][0]:.2f} V" if d_in else "无数据"))

    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<16} {detail}")

    all_ok = all(c[1] for c in checks)
    print("=" * 62)

    if all_ok:
        print(" 结论 : PASS —— 串口链路已通，二进制遥测稳定，可进入下一步")
        print(" 下一步：1) 网页上位机接同一串口看 Iq/Id/母线")
        print("         2) SWD 读回故障字与 Servo_GetMaxStepCycles()")
        print("         3) 低压空载验证 MOE 封锁，再做编码器零点/电流偏置标定")
        return 0

    print(" 结论 : FAIL —— 链路未打通，按下列顺序排查")
    print("  1) 一个字节都收不到")
    print("     - 确认接的是 RS-485 的 A/B（板上 U1 的 GH1.25 第 2/3 脚），不是 CN8 的 TTL 脚")
    print("     - 板端 THVD1426 的 RO 一直使能，若把 USB-TTL 的 TX 直接并到 PB7 会双端驱动打架")
    print("     - 确认 A/B 极性没反（反了通常收到 0x00/0xFF 或全无）")
    print("     - 确认波特率 921600（不是 115200），板子供电正常")
    print("  2) 只有 ASCII 横幅、没有二进制帧")
    print("     - 说明 TX 通但 BSP 或 TIM6 失败；看横幅里 [BOOT] bsp= / tim6= / fault=")
    print("     - fault bit: 0=OVC 1=ENC 2=ADC_SYNC 3=BSP_INIT 4=SPI")
    print("  3) 有帧但 CRC 全错")
    print("     - 波特率偏差、线太长无终端、或固件组帧与本工具偏移不一致")
    print("     - 用 --hexdump 看原始字节，帧头应为 FD 35 00 00 .. 29 75 00（30025）")
    print("  4) 帧率远低于 100 fps")
    print("     - 说明 DMA 发送被反复丢弃：检查是否有人在 1 ms 任务里长阻塞")
    return 1


def main() -> int:
    ap = argparse.ArgumentParser(
        description="X72_FOC 串口联通检查（只读，不下发）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="串口号，如 COM5；不填则自动选")
    ap.add_argument("-b", "--baud", type=int, default=921600, help="波特率，默认 921600")
    ap.add_argument("-t", "--secs", type=float, default=5.0, help="观察时长（秒），默认 5")
    ap.add_argument("--list", action="store_true", help="只列出串口")
    ap.add_argument("--ascii", action="store_true", help="只按文本显示（等同串口助手）")
    ap.add_argument("--selftest", action="store_true", help="用合成帧验证解析器，不接硬件")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.list:
        for p in list_ports.comports():
            print(f"{p.device:<8} {p.description}")
        return 0

    port = pick_port(args.port)
    if not port:
        return 2

    return run(port, args.baud, args.secs, args.ascii)


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:                            # noqa: BLE001
        pass
    raise SystemExit(main())
