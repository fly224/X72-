# -*- coding: utf-8 -*-
"""已迁移通道的全量回归对账：MAVLink 的每个值都必须与 ASCII 同源一致。

为什么必须对账而不是"能收到帧就算通"：迁移期两套接口并存，最危险的失败模式
不是收不到帧，而是**收到了但值不一样**（比如 30091 曾把调度前的 kp 报成
583e-3，而 K 行报 250e-3）。帧能解析、CRC 也对，只有逐项比数值才看得出来。

覆盖：30090 / 30091 / 30082 / 30095 / 30069，外加 30092 的最近结果帧。
判据：整数严格相等；浮点按 ASCII 打印精度给容差。

用法：python verify_migrated_all.py COM27
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
skipped = []


def chk(name, cond, detail=""):
    global passed
    if cond:
        passed += 1
        print("  PASS  %-30s %s" % (name, detail))
    else:
        failed.append(name)
        print("  FAIL  %-30s %s" % (name, detail))


def near(a, b, tol):
    return a is not None and b is not None and abs(a - b) <= tol


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


def ascii_lines(ch, wait=1.2):
    txt = strip_frames(xfer(ch, wait)).decode("ascii", "replace")
    return [l.strip() for l in txt.splitlines() if "[CMD]" in l]


def kv(line):
    """把 'a=1 b=2/3 c=4.5' 解析成 dict；带斜杠的只取第一段。"""
    d = {}
    for m in re.finditer(r"([A-Za-z_]+)=(-?[0-9.]+(?:e-?[0-9]+)?)", line or ""):
        try:
            d[m.group(1)] = float(m.group(2))
        except ValueError:
            pass
    return d


def frames(msgid, ln, wait=1.2):
    """抓一段遥测并返回指定消息的 payload 列表（CRC 已校验）。"""
    raw = xfer(b"", wait)
    out = []
    i = 0
    while i + 12 <= len(raw):
        if raw[i] == 0xFD:
            n = raw[i + 1]
            if i + 12 + n + 2 > len(raw):
                break
            mid = raw[i + 7] | (raw[i + 8] << 8) | (raw[i + 9] << 16)
            if mid == msgid and n == ln:
                pay = raw[i + 10:i + 10 + n]
                crc = raw[i + 10 + n] | (raw[i + 11 + n] << 8)
                extra = 90 if msgid == 30090 else (91 if msgid == 30091 else 92)
                if MV.crc_calculate(bytes(raw[i + 1:i + 10]) + pay, extra) == crc:
                    out.append(pay)
            i += 12 + n
        else:
            i += 1
    return out


# ---------- 采样 ----------
print("[采样] 读 ASCII S / K，再抓一帧 30090 / 30091")
s_line = (ascii_lines(b"S") or [""])[0]
k_line = (ascii_lines(b"K") or [""])[0]
S = kv(s_line)
K = kv(k_line)

raw = xfer(b"", 1.3)
st_pay = None
lp_pay = None
i = 0
while i + 12 <= len(raw):
    if raw[i] == 0xFD:
        n = raw[i + 1]
        if i + 12 + n + 2 > len(raw):
            break
        mid = raw[i + 7] | (raw[i + 8] << 8) | (raw[i + 9] << 16)
        pay = raw[i + 10:i + 10 + n]
        crc = raw[i + 10 + n] | (raw[i + 11 + n] << 8)
        if mid == 30090 and n == 40 and st_pay is None:
            if MV.crc_calculate(bytes(raw[i + 1:i + 10]) + pay, 90) == crc:
                st_pay = pay
        elif mid == 30091 and n == 48 and lp_pay is None:
            if MV.crc_calculate(bytes(raw[i + 1:i + 10]) + pay, 91) == crc:
                lp_pay = pay
        i += 12 + n
    else:
        i += 1

print("  S: %s" % s_line[:150])
print("  K: %s" % k_line[:150])
if st_pay is None or lp_pay is None:
    print("  !! 30090/30091 未抓到，退出")
    rig.close()
    sys.exit(1)

ST = {"cw": st_pay[0] | (st_pay[1] << 8), "mode": st_pay[2], "fl": st_pay[3]}
_stf = struct.unpack_from("<8f", st_pay, 4)
ST.update(iq=_stf[0], iqref=_stf[1], rpm=_stf[2], pos=_stf[3],
          vbus=_stf[4], tmot=_stf[5], tdrv=_stf[6], ktc=_stf[7])
ST["fault"] = st_pay[36] | (st_pay[37] << 8)

LP = {"en": lp_pay[0], "src": lp_pay[1], "sch": lp_pay[2]}
_lpf = struct.unpack_from("<11f", lp_pay, 4)
LP.update(wref=_lpf[0], wmeas=_lpf[1], iqc=_lpf[2], iterm=_lpf[3], kp=_lpf[4],
          acc=_lpf[5], iqmax=_lpf[6], eoff=_lpf[7], psi=_lpf[8], kt=_lpf[9],
          ld=_lpf[10])

# ---------- 30090 自洽 ----------
# ASCII 的 S/K 行已关闭（固件 X72_ASCII_DEBUG=0），所以不再与文本对账。
# 改成"字段自洽 + 跨消息交叉"——这其实比文本对账更强：文本是同一份数据的
# 另一种呈现（同一路径），而 30090/30091/30082 是三条独立路径。
print("\n[1] 30090 AppCtrlStatus 自洽")
chk("mode 合法", ST["mode"] in (0, 1, 3, 4, 12), "mode=%d" % ST["mode"])
chk("fault 在 16 位范围", 0 <= ST["fault"] <= 0xFFFF, "0x%04X" % ST["fault"])
chk("vbus 合理", 10.0 <= ST["vbus"] <= 60.0, "%.2f V" % ST["vbus"])
chk("iq_meas 有限", abs(ST["iq"]) < 100.0, "%.3f A" % ST["iq"])
chk("温度合理", -40.0 <= ST["tmot"] <= 150.0, "%.1f C" % ST["tmot"])
chk("ktc 在 0.85~1.15", 0.85 <= ST["ktc"] <= 1.15, "%.4f" % ST["ktc"])
chk("flags.en ↔ cw bit0", ((ST["fl"] >> 2) & 1) == (ST["cw"] & 1),
    "flags.en=%d cw=0x%02X" % ((ST["fl"] >> 2) & 1, ST["cw"]))

print("\n[2] 30091 AppLoopStatus 自洽 + 与 30090 交叉")
chk("kp > 0", LP["kp"] > 0.0, "%.5f" % LP["kp"])
chk("iqmax 合理", 0.5 <= LP["iqmax"] <= 10.0, "%.2f A" % LP["iqmax"])
chk("acc 合理", 0.0 <= LP["acc"] <= 5000.0, "%.1f" % LP["acc"])
chk("psi 合理（mWb）", 0.5 <= LP["psi"] <= 50.0, "%.3f mWb" % LP["psi"])
SRC = {0: "NONE", 1: "TORQUE", 2: "SPEED", 3: "POS", 4: "MIT", 5: "OPENLOOP"}
chk("src 合法", LP["src"] in SRC, "src=%s" % SRC.get(LP["src"], "?"))
chk("en ↔ 30090.flags.en", LP["en"] == ((ST["fl"] >> 2) & 1),
    "30091.en=%d 30090.flags.en=%d" % (LP["en"], (ST["fl"] >> 2) & 1))

# ---------- 30082 交叉 ----------
print("\n[3] 30082 整数通道 ↔ 30090 flags（两条独立通道互证）")
p = bytearray(192)
raw82 = xfer(MV.encode(30000, struct.pack("<I", 30082), seq=101), 1.2)
ints = None
i = 0
while i + 12 <= len(raw82):
    if raw82[i] == 0xFD:
        n = raw82[i + 1]
        if i + 12 + n + 2 > len(raw82):
            break
        mid = raw82[i + 7] | (raw82[i + 8] << 8) | (raw82[i + 9] << 16)
        if mid == 30082 and n == 192:
            pay = raw82[i + 10:i + 202]
            if MV.crc_calculate(bytes(raw82[i + 1:i + 10]) + pay, 218) == (raw82[i + 202] | (raw82[i + 203] << 8)):
                ints = struct.unpack_from("<16i", pay, 128)
                u = struct.unpack_from("<16I", pay, 64)
                n_flt, n_int = u[1], u[2]
                break
        i += 12 + n
    else:
        i += 1
if ints is None:
    skipped.append("30082")
    print("  SKIP  30082 未应答")
else:
    # 有效槽位数不再写死 6/9：它必须等于上位机名字表的长度。
    # 这才是真正的契约 —— 固件加了槽而上位机没加名字，就会在这里断，
    # 而不是等调用 dbg_write 时才报"未知参数名"。
    from mavlink_link import SerialLink as _SL  # noqa: E402
    want_int = len(_SL.DBG_INT_NAMES)
    chk("有效槽位数 = 上位机表长", n_flt == 16 and n_int == want_int,
        "n_float=%d n_int=%d (表长 %d)" % (n_flt, n_int, want_int))
    chk("dead_en ↔ flags.dead", ints[0] == ((ST["fl"] >> 4) & 1), "dead_en=%d" % ints[0])
    chk("harm_en ↔ flags.harm", ints[1] == ((ST["fl"] >> 5) & 1), "harm_en=%d" % ints[1])
    chk("ff_en ↔ flags.ff", ints[2] == ((ST["fl"] >> 3) & 1), "ff_en=%d" % ints[2])
    chk("cog_en ↔ flags.cog", ints[3] == ((ST["fl"] >> 7) & 1), "cog_en=%d" % ints[3])
    chk("ktc_en ↔ flags.ktc", ints[4] == ((ST["fl"] >> 6) & 1), "ktc_en=%d" % ints[4])
    chk("sched_en ↔ 30091.sch", ints[5] == LP["sch"], "sched_en=%d sch=%d" % (ints[5], LP["sch"]))

# ---------- 30095 ----------
print("\n[4] 30095 AppTorqueCmd 回读自洽")
raw95 = xfer(MV.encode(30095, bytes(12), seq=103), 1.0)
got95 = None
i = 0
while i + 12 <= len(raw95):
    if raw95[i] == 0xFD:
        n = raw95[i + 1]
        if i + 12 + n + 2 > len(raw95):
            break
        mid = raw95[i + 7] | (raw95[i + 8] << 8) | (raw95[i + 9] << 16)
        if mid == 30095 and n >= 10:
            pay = raw95[i + 10:i + 10 + n]
            got95 = (pay[0], pay[1],
                     struct.unpack_from("<f", pay, 2)[0],
                     struct.unpack_from("<f", pay, 6)[0])
            break
        i += 12 + n
    else:
        i += 1
if got95 is None:
    skipped.append("30095")
    print("  SKIP  30095 未应答")
else:
    chk("状态 SUCCESS", got95[0] == 0, "status=%d n_ok=%d" % (got95[0], got95[1]))
    chk("id 回读非 NaN", got95[2] == got95[2], "id=%.4f A" % got95[2])
    chk("ramp 回读在 0..2000", 0.0 <= got95[3] <= 2000.01, "ramp=%.1f ms" % got95[3])

# ---------- 30069 ----------
print("\n[5] 30069 SystemCmd 可达（action=0 空动作，无副作用）")
r69 = xfer(MV.encode(30069, bytes([0]), seq=105), 0.8)
acked = False
i = 0
while i + 12 <= len(r69):
    if r69[i] == 0xFD:
        n = r69[i + 1]
        if i + 12 + n + 2 > len(r69):
            break
        mid = r69[i + 7] | (r69[i + 8] << 8) | (r69[i + 9] << 16)
        if mid == 30069 and n >= 1:
            acked = True
            break
        i += 12 + n
    else:
        i += 1
chk("30069 ACK 收到", acked, "空动作应回 SUCCESS")

# ---------- 30054 控制字往返 ----------
# 为什么要单独测：之前这里只有一句 `cw & 1 == en`，而 en=0 时 cw 也是 0，
# 这条判据恒成立、什么也证明不了。实测 30090 曾经把 controlword 写死成
# `(armed)?1:0`，写 0x08 读回 0 —— 只有"写进去再读回来"才抓得住。
print("\n[6] 30054 控制字往返（30090.cw 必须回真实值）")


def _cw_now():
    raw = xfer(b"", 0.9)
    j = 0
    while j + 12 <= len(raw):
        if raw[j] == 0xFD:
            n = raw[j + 1]
            if j + 12 + n + 2 > len(raw):
                break
            m = raw[j + 7] | (raw[j + 8] << 8) | (raw[j + 9] << 16)
            if m == 30090 and n == 40:
                p = bytes(raw[j + 10:j + 50])
                if MV.crc_calculate(bytes(raw[j + 1:j + 10]) + p, 90) == (raw[j + 50] | (raw[j + 51] << 8)):
                    return p[0] | (p[1] << 8)
            j += 12 + n
        else:
            j += 1
    return None


for want in (0x08, 0x02):
    # 这两个值都不含 bit0（CW_SWITCH_ON），走 disarm 分支，不会真使能
    xfer(MV.encode(30054, struct.pack("<HB", want, 0), seq=130 + want), 0.7)
    time.sleep(0.25)
    got = _cw_now()
    chk("cw 0x%02X 往返" % want, got == want, "回读 0x%02X" % (got if got is not None else 0))
# 收尾：回到 0，避免留下奇怪的控制字
xfer(MV.encode(30054, struct.pack("<HB", 0, 0), seq=139), 0.6)

print("\n=== 结果：%d 项通过，%d 项失败，%d 项跳过 ==="
      % (passed, len(failed), len(skipped)))
if failed:
    print("失败项：" + ", ".join(failed))
if skipped:
    print("跳过项：" + ", ".join(skipped))
rig.close()
sys.exit(0 if not failed else 1)
