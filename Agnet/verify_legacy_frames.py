# -*- coding: utf-8 -*-
"""验证 ASCII 关闭后两个私有二进制小帧确实不再生效（2026-11-21）。

背景：0x04（PCMP 位置补偿参数帧）与 0x02（MIT 8 字节帧）是 ASCII 时代的
两个"第二入口"，现在与 ASCII 一起被 `#if X72_ASCII_DEBUG` 关掉：
  - 能力没丢：PCMP 三个参数已迁到 30082 的整数槽 6/7/8，而且能读回；
  - 入口已关：0x04 帧不再改任何状态（以前会直接 PosLoop_SetComp）。

判据（必须能区分"关了"和"还在"）：
  1) 用 30082 把补偿量设成 20 mrad，读回确认 20
  2) 发一帧 0x04 要求 50 mrad
  3) 再读回：仍是 20 → 帧被忽略（若还能用，会变成 50 并回显 [CMD] PCMP）
  4) 再发一帧 0x02 MIT（全量程 kp=0），确认 30090 的 mode 不变

用法：python verify_legacy_frames.py COM27
"""
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"

passed = 0
failed = []


def chk(name, cond, detail=""):
    global passed
    if cond:
        passed += 1
        print("  PASS  %-38s %s" % (name, detail))
    else:
        failed.append(name)
        print("  FAIL  %-38s %s" % (name, detail))


def posf_frame(cmd, val):
    """0x04 + cmd + f32 LE + 前 6 字节字节和"""
    b = bytearray([0x04, cmd]) + struct.pack("<f", val)
    b.append(sum(b) & 0xFF)
    return bytes(b)


def mit_frame(p=0.0, v=0.0, kp=0.0, kd=0.0, t=0.0):
    """0x02 + 8 B（Mini Cheetah 位序）+ 字节和。这里全给 0，等价"零力矩"。"""
    def u2f(x, lo, hi, bits):
        return int(round((x - lo) / (hi - lo) * ((1 << bits) - 1)))
    pi = u2f(p, -12.5, 12.5, 16)
    vi = u2f(v, -65.0, 65.0, 12)
    kpi = u2f(kp, 0.0, 5000.0, 12)
    kdi = u2f(kd, 0.0, 100.0, 12)
    ti = u2f(t, -18.0, 18.0, 12)
    b = bytearray(9)
    b[0] = 0x02
    b[1] = (pi >> 8) & 0xFF
    b[2] = pi & 0xFF
    b[3] = ((vi >> 4) & 0xF0) | ((kpi >> 8) & 0x0F)
    b[4] = vi & 0xFF
    b[5] = kpi & 0xFF
    b[6] = ((kdi >> 4) & 0xF0) | ((ti >> 8) & 0x0F)
    b[7] = ((kdi & 0x0F) << 4) | 0
    b[8] = ti & 0xFF
    out = bytearray([0x02]) + b[1:]
    out.append(sum(out[:9]) & 0xFF)
    return bytes(out)


lk = SerialLink()
r = lk.connect(PORT, 921600)
print("[连接] " + ("OK" if r.get("ok") else str(r)))
if not r.get("ok"):
    sys.exit(1)
time.sleep(1.2)
lk.set_permission(0, 0)
time.sleep(0.3)

print("\n[1] 用 30082 把补偿量设为 20 mrad")
lk.dbg_write(ints={"comp_mrad": 20})
time.sleep(0.35)
d = lk.dbg_read()
base = d["ints"]["comp_mrad"] if d.get("ok") else None
chk("写入并读回 20", base == 20, "回读=%s" % base)

print("\n[2] 发 0x04 帧要求 50 mrad（ASCII 打开时这会生效）")
with lk.wlock:
    lk.ser.write(posf_frame(0, 0.050))
    lk.ser.flush()
time.sleep(0.5)
d = lk.dbg_read()
now = d["ints"]["comp_mrad"] if d.get("ok") else None
chk("补偿量未被 0x04 改动", now == 20, "回读=%s（应仍为 20）" % now)

print("\n[3] 再发一帧 0x02 MIT（10 B），只作冒烟：不打扰链路、不改状态")
# 说明：MIT 入口与 POSF 是同一个 #if 里的两条 if，POSF 被证伪即两条都关。
# 这一节不再声称"能证明 MIT 被拒" —— 当前未使能，无论入口开没开，
# mit_engage() 都会因联锁返回 0，从外部看不出来。这里只验证"小帧不会
# 把 MAVLink 解析器带偏"，这是它真正的风险。
st0 = lk.ctrl_status()
with lk.wlock:
    lk.ser.write(mit_frame())
    lk.ser.flush()
time.sleep(0.5)
st1 = lk.ctrl_status()
chk("在环模式未变", st1["mode"] == st0["mode"],
    "%d -> %d" % (st0["mode"], st1["mode"]))
chk("使能状态未变", st1["en"] == st0["en"], "%d -> %d" % (st0["en"], st1["en"]))

print("\n[4] 链路仍然健康（小帧不会把解析器带偏）")
st = lk._parser_stats()
chk("无 CRC 错", st["crc_errors"] == 0, "crc=%d" % st["crc_errors"])
chk("命令队列无溢出", lk.dbg_read()["ints"]["q_ovf"] == 0,
    "q_ovf=%d" % lk.dbg_read()["ints"]["q_ovf"])
q = lk.rec_status()
chk("30094 仍可应答", q.get("ok"), str(q.get("error") or ""))

print("\n=== 结果：%d 项通过，%d 项失败 ===" % (passed, len(failed)))
if failed:
    print("失败项：" + ", ".join(failed))
lk.disconnect()
sys.exit(0 if not failed else 1)
