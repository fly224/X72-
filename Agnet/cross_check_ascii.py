# -*- coding: utf-8 -*-
"""交叉验证：ASCII 单字符命令与 30082 整数通道必须操作同一份真值。

为什么这条最要紧：迁移期两套接口并存，如果各自维护一份状态，
"MAVLink 说关、ASCII 说开"这种分歧会静默存在，而且只在某一条路径上复现。
判据：用 ASCII 切换，再用 30082 读回，必须看到同一个变化。

用法：python cross_check_ascii.py COM27
"""
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
sys.path.insert(0, r"G:\FOC\X72_Agnet")
import mavlink_proto as MV  # noqa: E402
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
CRC_EXTRA_30082 = 218          # 与固件 s_meta 一致

rig = Rig(PORT)
time.sleep(1.3)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)


def xfer(data, wait=1.0):
    rig.rx.pause_evt.set()
    time.sleep(0.05)
    try:
        rig.s.reset_input_buffer()
        rig.s.write(data)
        rig.s.flush()
        b = bytearray()
        t0 = time.time()
        while time.time() - t0 < wait:
            d = rig.s.read(8192)
            if d:
                b += d
        return bytes(b)
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def read_ints():
    """读 30082 并返回 16 个 int32。"""
    raw = xfer(MV.encode(30000, struct.pack("<I", 30082), seq=31), 1.2)
    i = 0
    while i + 12 <= len(raw):
        if raw[i] == 0xFD:
            ln = raw[i + 1]
            if i + 12 + ln + 2 > len(raw):
                break
            mid = raw[i + 7] | (raw[i + 8] << 8) | (raw[i + 9] << 16)
            if mid == 30082 and ln == 192:
                pay = raw[i + 10:i + 202]
                return struct.unpack_from("<16i", pay, 128)
            i += 12 + ln
        else:
            i += 1
    return None


def ascii_cmd(ch, wait=1.2):
    txt = strip_frames(xfer(ch, wait)).decode("ascii", "replace")
    return [l.strip() for l in txt.splitlines() if "[CMD]" in l]


passed = 0
failed = []


def chk(name, cond, detail=""):
    global passed
    if cond:
        passed += 1
        print("  PASS  %-34s %s" % (name, detail))
    else:
        failed.append(name)
        print("  FAIL  %-34s %s" % (name, detail))


print("[0] 基线")
a = read_ints()
if a is None:
    print("  读 30082 失败，退出")
    rig.close()
    sys.exit(1)
print("  dead_en=%d harm_en=%d" % (a[0], a[1]))

print("\n[1] ASCII 'B' 切换 → 30082 读 dead_en")
before = read_ints()[0]
lines = ascii_cmd(b"B")
mid = read_ints()[0]
chk("B 后 dead_en 翻转", mid != before, "%d -> %d  （%s）" % (before, mid, lines[0][:46] if lines else ""))
ascii_cmd(b"B")
after = read_ints()[0]
chk("再按 B 恢复", after == before, "%d -> %d" % (mid, after))

print("\n[2] ASCII '%%' 切换 → 30082 读 harm_en")
before = read_ints()[1]
lines = ascii_cmd(b"%")
mid = read_ints()[1]
chk("%% 后 harm_en 翻转", mid != before, "%d -> %d  （%s）" % (before, mid, lines[0][:46] if lines else ""))
ascii_cmd(b"%")
after = read_ints()[1]
chk("再按 %% 恢复", after == before, "%d -> %d" % (mid, after))

print("\n[3] 30082 写 dead_en → ASCII 回显必须跟着变")
b0 = read_ints()[0]
p = bytearray(192)
struct.pack_into("<i", p, 128 + 0 * 4, 1 - b0)      # 取反
struct.pack_into("<I", p, 124, 1 << 0)
xfer(MV.encode(30082, bytes(p), seq=33), 0.5)
time.sleep(0.2)
lines = ascii_cmd(b"B")     # 注意：B 是"切换"，所以这里应看到它切回 b0
b1 = read_ints()[0]
chk("30082 写后 ASCII 回显一致", b1 == b0,
    "写 %d → 按 B 后回读 %d（回显 %s）" % (1 - b0, b1, lines[0][:46] if lines else ""))

print("\n=== 结果：%d 项通过，%d 项失败 ===" % (passed, len(failed)))
if failed:
    print("失败项：" + ", ".join(failed))
rig.close()
sys.exit(0 if not failed else 1)
