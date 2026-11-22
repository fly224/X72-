# -*- coding: utf-8 -*-
"""验证 30093 AppCogTable：齿槽表分块回读。

判据三层：
  1) 头部与 ASCII `:` 行逐项对账（同源）
  2) 点数 == n_bins，且块索引不重叠不漏
  3) 表的峰峰值与头部 pp 自洽（pp 是辨识值，表可能被平滑，容差放宽）

用法：python verify_cog_table.py COM27
"""
import re
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
sys.path.insert(0, r"G:\FOC\X72_Agnet")
import mavlink_proto as MV  # noqa: E402
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"

rig = Rig(PORT)
time.sleep(1.3)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)

passed = 0
failed = []


def chk(name, cond, detail=""):
    global passed
    if cond:
        passed += 1
        print("  PASS  %-30s %s" % (name, detail))
    else:
        failed.append(name)
        print("  FAIL  %-30s %s" % (name, detail))


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


# ---- 1) ASCII ':' 状态行 ----
txt = strip_frames(xfer(b':', 1.2)).decode("ascii", "replace")
cog_line = ""
for l in txt.splitlines():
    if "[CMD] cog " in l:
        cog_line = l.strip()
        break
print("[1] ASCII ':' 行")
print("  " + (cog_line or "(无)"))

A = {}
for m in re.finditer(r"([a-z]+)=(-?[0-9.]+)", cog_line):
    A[m.group(1)] = float(m.group(2))

# ---- 2) 30093 拉全表 ----
print("\n[2] 30093 分块拉表")
N_BLOCK = 16
head = None
table = []
for idx in range(N_BLOCK):
    p = bytearray(48)
    p[0] = 1
    p[1] = idx
    raw = xfer(MV.encode(30093, bytes(p), seq=110 + idx), 0.9)
    got = None
    i = 0
    while i + 12 <= len(raw):
        if raw[i] == 0xFD:
            n = raw[i + 1]
            if i + 12 + n + 2 > len(raw):
                break
            mid = raw[i + 7] | (raw[i + 8] << 8) | (raw[i + 9] << 16)
            if mid == 30093 and n == 48:
                pay = raw[i + 10:i + 58]
                crc = raw[i + 58] | (raw[i + 59] << 8)
                if MV.crc_calculate(bytes(raw[i + 1:i + 10]) + pay, 93) == crc:
                    got = pay
                break
            i += 12 + n
        else:
            i += 1
    if got is None:
        print("  !! 块 %d 无有效应答" % idx)
        break
    if head is None:
        head = {"n_bins": got[2], "valid": got[3],
                "pp": struct.unpack_from("<f", got, 4)[0],
                "phase_deg": struct.unpack_from("<f", got, 8)[0],
                "zero_dev_deg": struct.unpack_from("<f", got, 12)[0]}
        print("  头部: n_bins=%d valid=%d pp=%.3fA phase=%.2fdeg zd=%.2fdeg"
              % (head["n_bins"], head["valid"], head["pp"],
                 head["phase_deg"], head["zero_dev_deg"]))
    vals = struct.unpack_from("<8f", got, 16)
    table.extend(vals)
    if head["n_bins"] and len(table) >= head["n_bins"]:
        break

chk("头部已解析", head is not None)
if head:
    chk("n_bins=128", head["n_bins"] == 128, "实际 %d" % head["n_bins"])
    chk("valid=1", head["valid"] == 1, "实际 %d" % head["valid"])
    chk("点数够 n_bins", len(table) >= head["n_bins"],
        "%d / %d" % (len(table), head["n_bins"]))

# ---- 3) 与 ASCII ':' 对账 ----
print("\n[3] 30093 头部 vs ASCII ':' 行")
if head and A:
    chk("n_bins ↔ cog.n", head["n_bins"] == int(A.get("n", -1)),
        "%d vs %s" % (head["n_bins"], A.get("n")))
    chk("valid ↔ cog.v", head["valid"] == int(A.get("v", -1)),
        "%d vs %s" % (head["valid"], A.get("v")))
    chk("pp ↔ cog.pp", abs(head["pp"] - A.get("pp", -9)) < 0.02,
        "%.3f vs %s" % (head["pp"], A.get("pp")))
    chk("phase ↔ cog.ph", abs(head["phase_deg"] - A.get("ph", -999)) < 0.05,
        "%.2f vs %s" % (head["phase_deg"], A.get("ph")))
    chk("zero_dev ↔ cog.zd", abs(head["zero_dev_deg"] - A.get("zd", -999)) < 0.05,
        "%.2f vs %s" % (head["zero_dev_deg"], A.get("zd")))
else:
    failed.append("ASCII 对账")
    print("  FAIL  缺少 ASCII 行或头部，无法对账")

# ---- 4) 表自洽 ----
print("\n[4] 表自洽")
if head and len(table) >= head["n_bins"]:
    n = head["n_bins"]
    t = table[:n]
    pp_meas = max(t) - min(t)
    chk("峰峰值与 pp 同量级", abs(pp_meas - head["pp"]) <= max(0.15, 0.25 * head["pp"]),
        "表 %.3fA vs pp %.3fA（差 %.1f%%）"
        % (pp_meas, head["pp"],
           100.0 * (pp_meas - head["pp"]) / head["pp"] if head["pp"] else -1))
    chk("与 ASCII ':' 的 pp 也不矛盾",
        abs(pp_meas - A.get("pp", -9)) <= max(0.15, 0.25 * A.get("pp", 1)),
        "表 %.3fA vs ASCII %.3fA" % (pp_meas, A.get("pp", -9)))
    # 周期性：128 箱一圈，齿槽主要谐波应是 12 或 24 的整数倍（14 极对×槽数）
    # 这里只查"不是常数"，具体谐波由上位机画图看
    chk("表不是常数", pp_meas > 0.01, "pp=%.4f A" % pp_meas)
    print("  前 8 点: " + " ".join("%.3f" % x for x in t[:8]))
    print("  后 8 点: " + " ".join("%.3f" % x for x in t[-8:]))

print("\n=== 结果：%d 项通过，%d 项失败 ===" % (passed, len(failed)))
if failed:
    print("失败项：" + ", ".join(failed))
rig.close()
sys.exit(0 if not failed else 1)
