# -*- coding: utf-8 -*-
"""验证 30082 扩展后的整数通道（开关/档位）与新增两个 float 槽。

判据不是"写成功"，而是**写后读回等于写入值**，并且保留槽位写不进去。
因为该通道历史上踩过最坑的一次就是"读完全正常、写完全不生效"
（s_meta 未登记 30082 → crc_extra=0 → 写帧被静默丢弃）。

用法：python verify_dbg_int.py COM27
"""
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"

lk = SerialLink()
r = lk.connect(PORT, 921600)
print("[连接] " + ("OK" if r.get("ok") else str(r)))
time.sleep(1.2)
lk.set_permission(0, 0)
time.sleep(0.3)

base = lk.dbg_read()
if not base.get("ok"):
    print("[读全表] 失败：" + str(base.get("error")))
    lk.disconnect()
    sys.exit(1)

print("[读全表] n_float=%d n_int=%d" % (base["n_float"], base["n_int"]))
print("  floats:")
for k, v in base["floats"].items():
    print("    %-11s = %.6g" % (k, v))
print("  ints:")
for k, v in base["ints"].items():
    print("    %-11s = %d" % (k, v))

passed = 0
failed = []


def chk(name, cond, detail=""):
    global passed
    if cond:
        passed += 1
        print("  PASS  %-28s %s" % (name, detail))
    else:
        failed.append(name)
        print("  FAIL  %-28s %s" % (name, detail))


def set_int(name, val, expect=None):
    """写一个开关，读回比对，然后恢复原值。"""
    old = base["ints"][name]
    w = lk.dbg_write(ints={name: val})
    if not w.get("ok"):
        return None, old
    time.sleep(0.35)
    now = lk.dbg_read()
    if not now.get("ok"):
        return None, old
    got = now["ints"][name]
    # 恢复
    lk.dbg_write(ints={name: old})
    time.sleep(0.3)
    return got, old


print("\n[1] 逐个开关 写→读回→恢复")
for name, target in (("harm_en", 1), ("harm_en", 0),
                     ("dead_en", 0), ("ff_en", 0),
                     ("cog_en", 0), ("ktc_en", 0), ("sched_en", 1)):
    got, old = set_int(name, target)
    chk("%s <- %d" % (name, target), got == target,
        "回读=%s 原值=%d" % (got, old))

print("\n[2] 新增 float 槽")
for name, val in (("eleoff_deg", 0.5), ("ff_boost", 1.5)):
    old = base["floats"][name]
    lk.dbg_write(floats={name: val})
    time.sleep(0.35)
    now = lk.dbg_read()
    got = now["floats"][name] if now.get("ok") else None
    lk.dbg_write(floats={name: old})
    time.sleep(0.3)
    chk("%s <- %.3f" % (name, val),
        got is not None and abs(got - val) < 1e-4,
        "回读=%s 原值=%.6g" % (got, old))

print("\n[2b] 位置补偿三参数（原 ASCII 0x04 PCMP 帧，2026-11-21 迁到这里）")
# 为什么单列一节：这三项换了通道（0x04 帧已随 ASCII 一起关闭），
# 必须证明"能力没丢"——写进去、读回来、边界挡得住。
for name, val in (("comp_mrad", 20), ("comp_mrad", 0),
                  ("comp_min_mrad", 5000), ("comp_min_mrad", 0),
                  ("comp_en", 1), ("comp_en", 0)):
    got, old = set_int(name, val)
    chk("%s <- %d" % (name, val), got == val, "回读=%s 原值=%d" % (got, old))

# 超限必须被拒（-1 与 100001 mrad 都在 DBG_COMP_MRAD_MAX 之外）
for name, bad_val, why in (("comp_mrad", -1, "负"), ("comp_mrad", 100001, ">100mrad"),
                           ("comp_min_mrad", 100001, ">100rad")):
    old = base["ints"][name]
    lk.dbg_write(ints={name: bad_val})
    time.sleep(0.35)
    now = lk.dbg_read()
    got_b = now["ints"][name] if now.get("ok") else None
    lk.dbg_write(ints={name: old})
    time.sleep(0.25)
    chk("%s <- %d 被拒（%s）" % (name, bad_val, why),
        got_b == old, "回读=%s（应仍为 %d）" % (got_b, old))

print("\n[3] 越界应被拒（回读保持原值）")
old_e = base["floats"]["eleoff_deg"]
lk.dbg_write(floats={"eleoff_deg": 40.0})
time.sleep(0.35)
now = lk.dbg_read()
got_e = now["floats"]["eleoff_deg"] if now.get("ok") else None
chk("eleoff_deg <- 40（超 ±30）", got_e is not None and abs(got_e - old_e) < 1e-4,
    "回读=%.5g（应仍为 %.5g）" % (got_e if got_e is not None else float("nan"), old_e))

print("\n[4] 保留槽位写不进去")
lk.dbg_write(ints={"rsv10": 123})   # 名字不在表里 → 上位机层就该拒
time.sleep(0.2)
# 直接用原始掩码打保留槽，绕过上位机名字检查
# 注意：槽 6/7/8 已分配给 comp_mrad / comp_min_mrad / comp_en，槽 9 是只读
# link_diag（有 get 无 set）。原来打 slot 7 的写法会真的把 comp_min 写进去。
import struct  # noqa: E402

p = bytearray(192)
struct.pack_into("<i", p, 128 + 10 * 4, 123)
struct.pack_into("<I", p, 124, 1 << 10)
lk.send(30082, bytes(p), label="写保留槽 rsv10（应被拒）")
time.sleep(0.4)
now = lk.dbg_read()
chk("int 槽 10 保持 0", now.get("ok") and now["raw_int"][10] == 0,
    "回读=%s" % (now["raw_int"][10] if now.get("ok") else "?"))

# 槽 9 是只读诊断：写必须被拒，值必须保持（不是 0 也可能，所以比"前后相等"）
before9 = now["raw_int"][9] if now.get("ok") else None
p = bytearray(192)
struct.pack_into("<i", p, 128 + 9 * 4, 999)
struct.pack_into("<I", p, 124, 1 << 9)
lk.send(30082, bytes(p), label="写只读槽 link_diag（应被拒）")
time.sleep(0.4)
now9 = lk.dbg_read()
chk("link_diag 只读", now9.get("ok") and now9["raw_int"][9] != 999,
    "回读=%s 前后=%s" % (now9["raw_int"][9] if now9.get("ok") else "?", before9))

print("\n[5] 恢复后的一致性")
# 只读计数器（link_diag / q_ovf / uart_err / rx_ok）本来就会随通信增长，
# 不能参与"恢复原值"比对 —— 否则这条判据永远失败，反而掩盖真的没恢复。
RO_KEYS = ("link_diag", "q_ovf", "uart_err", "rx_ok")
fin = lk.dbg_read()
same_int = all(fin["ints"][k] == base["ints"][k] for k in base["ints"] if k not in RO_KEYS)
same_flt = all(abs(fin["floats"][k] - base["floats"][k]) < 1e-6 for k in base["floats"])
chk("所有可写项已恢复原值", same_int and same_flt,
    "ints=%s floats=%s" % ("同" if same_int else "异", "同" if same_flt else "异"))

print("\n=== 结果：%d 项通过，%d 项失败 ===" % (passed, len(failed)))
if failed:
    print("失败项：" + ", ".join(failed))
lk.disconnect()
sys.exit(0 if not failed else 1)
