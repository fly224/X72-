# -*- coding: utf-8 -*-
"""验证 30095 AppTorqueCmd：力矩模式的 id 给定与 iq 斜坡时间常数。

替代 ASCII 的 `/id`（id 给定）与 `"ms`（iq 斜坡时间常数）。
判据不是"发出去没报错"，而是**固件回读的生效值等于写入值**，
以及掩码为 0 的项必须原样保持（不能顺手清零）。

用法：python verify_torque_cmd.py COM27
"""
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"

lk = SerialLink()
r = lk.connect(PORT, 921600)
print("[连接] " + ("OK" if r.get("ok") else str(r)))
time.sleep(1.3)
lk.set_permission(0, 0)
time.sleep(0.3)

passed = 0
failed = []


def chk(name, cond, detail=""):
    global passed
    if cond:
        passed += 1
        print("  PASS  %-32s %s" % (name, detail))
    else:
        failed.append(name)
        print("  FAIL  %-32s %s" % (name, detail))


def raw_cmd(mask, id_a=None, ramp_ms=None, timeout=1.0):
    """直接发 30095 并解析应答：+0 状态 +1 n_ok +2 id +6 ramp。"""
    p = bytearray(12)
    p[0] = mask
    if id_a is not None:
        struct.pack_into("<f", p, 4, float(id_a))
    if ramp_ms is not None:
        struct.pack_into("<f", p, 8, float(ramp_ms))
    t0 = time.time()
    lk.send(30095, bytes(p), label="30095 mask=0x%02X" % mask)
    deadline = time.time() + timeout
    while time.time() < deadline:
        m = lk.last_msg.get(30095)
        if m and m["ts"] >= t0:
            pay = bytes.fromhex(m["hex"].replace(" ", ""))
            if len(pay) >= 10:
                return {"status": pay[0], "n_ok": pay[1],
                        "id_a": struct.unpack_from("<f", pay, 2)[0],
                        "ramp_ms": struct.unpack_from("<f", pay, 6)[0]}
        time.sleep(0.02)
    return None


print("\n[0] 基线（mask=0 只读）")
b = raw_cmd(0x00)
if b is None:
    print("  无应答，退出")
    lk.disconnect()
    sys.exit(1)
print("  id=%.4f A  ramp=%.1f ms  status=%d" % (b["id_a"], b["ramp_ms"], b["status"]))
base_id, base_ramp = b["id_a"], b["ramp_ms"]

print("\n[1] 只写 id，ramp 必须保持")
r1 = raw_cmd(0x01, id_a=0.5)
chk("id <- 0.5", r1 and abs(r1["id_a"] - 0.5) < 1e-4,
    "回读 %.4f A（status=%s n_ok=%s）" % (r1["id_a"] if r1 else -9,
                                          r1["status"] if r1 else "?", r1["n_ok"] if r1 else "?"))
chk("ramp 未被顺手改动", r1 and abs(r1["ramp_ms"] - base_ramp) < 1e-3,
    "回读 %.1f ms（基线 %.1f）" % (r1["ramp_ms"] if r1 else -9, base_ramp))

print("\n[2] 只写 ramp，id 必须保持")
r2 = raw_cmd(0x02, ramp_ms=100.0)
chk("ramp <- 100 ms", r2 and abs(r2["ramp_ms"] - 100.0) < 1e-2,
    "回读 %.2f ms" % (r2["ramp_ms"] if r2 else -9))
chk("id 未被顺手清零", r2 and abs(r2["id_a"] - 0.5) < 1e-4,
    "回读 %.4f A（上一步写的 0.5）" % (r2["id_a"] if r2 else -9))

print("\n[3] 超范围应被底层钳位，回读必须是钳位后的真值")
r3 = raw_cmd(0x03, id_a=10.0, ramp_ms=5000.0)
chk("id 钳到 ±iq_max", r3 and abs(r3["id_a"]) <= 5.0001,
    "写 10 A → 回读 %.3f A（应 ≤5）" % (r3["id_a"] if r3 else -9))
chk("ramp 钳到 2000 ms", r3 and r3["ramp_ms"] <= 2000.01,
    "写 5000 ms → 回读 %.1f ms（应 ≤2000）" % (r3["ramp_ms"] if r3 else -9))

print("\n[4] 恢复基线")
r4 = raw_cmd(0x03, id_a=base_id, ramp_ms=base_ramp)
chk("恢复 id / ramp", r4 and abs(r4["id_a"] - base_id) < 1e-4
    and abs(r4["ramp_ms"] - base_ramp) < 1e-2,
    "id=%.4f ramp=%.1f" % (r4["id_a"] if r4 else -9, r4["ramp_ms"] if r4 else -9))

print("\n[5] 掩码都不置时不得改动任何值（再读一次确认）")
r5 = raw_cmd(0x00)
chk("mask=0 纯读不改值", r5 and abs(r5["id_a"] - base_id) < 1e-4
    and abs(r5["ramp_ms"] - base_ramp) < 1e-2,
    "id=%.4f ramp=%.1f" % (r5["id_a"] if r5 else -9, r5["ramp_ms"] if r5 else -9))

print("\n=== 结果：%d 项通过，%d 项失败 ===" % (passed, len(failed)))
if failed:
    print("失败项：" + ", ".join(failed))
lk.disconnect()
sys.exit(0 if not failed else 1)
