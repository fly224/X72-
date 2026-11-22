# -*- coding: utf-8 -*-
"""验证缺相检测判据：用真实电流波形验证固件结论，并验证三条分支都能翻转。

为什么这么验证而不是"等它真断一次"：
  缺相是偶发硬件故障（本机今年已两次），不可能为了验证去拔线。但判据本身是
  纯代数（三相电流的 RMS 关系），可以拿**真实稳态波形**做代换：
      V 缺 → 物理上 iu ≡ -iw（电流只在 U-W 之间流），直接把 iw 换成 -iu
      U 缺 → iu ≡ 0；W 缺 → iw ≡ 0
  这样三条分支都在真实波形上跑过，而不是在合成正弦上跑。

同时与固件对账：固件 30082 槽 14 报的 ratio_v/结论，应与离线用同一段数据
算出来的一致（同一物理量、两条独立实现）。健康时固件必须判"正常"。

用法：python verify_phase_mon.py COM27 [rpm]
"""
import math
import re
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
RPM = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0

# 与固件 servo_isr.c 的宏逐一对应，改一处必须改两处（脚本头已注明原因）
PM_MIN_RMS_A = 0.15
PM_OPEN_RATIO = 0.30
PM_OPEN_GAIN = 0.20
PM_MIN_SWEEP = 1.5
POLE_PAIRS = 14

passed = 0
failed = []


def chk(name, cond, detail=""):
    global passed
    if cond:
        passed += 1
        print("  PASS  %-36s %s" % (name, detail))
    else:
        failed.append(name)
        print("  FAIL  %-36s %s" % (name, detail))


def judge(iu, iw, sweep):
    """复刻固件判据，返回 (code, ratio_v, balance)。code: 0/1 U/2 V/3 W/4 不可判"""
    n = float(len(iu))
    ru = math.sqrt(sum(x * x for x in iu) / n)
    rw = math.sqrt(sum(x * x for x in iw) / n)
    rs = math.sqrt(sum((iu[k] + iw[k]) ** 2 for k in range(len(iu))) / n)
    rmax, rmin = (ru, rw) if ru > rw else (rw, ru)
    ratio = rs / rmax if rmax > 1e-6 else 0.0
    bal = rmin / rmax if rmax > 1e-6 else 0.0
    if rmax < PM_MIN_RMS_A:
        return 4, ratio, bal
    if sweep < PM_MIN_SWEEP:
        return 4, ratio, bal
    if rmin < PM_OPEN_GAIN * rmax:
        return (1 if ru < rw else 3), ratio, bal
    if rs < PM_OPEN_RATIO * rmax:
        return 2, ratio, bal
    return 0, ratio, bal


def unwrap_sweep(th):
    """mod-2π 序列的角度总行程（rad）。与固件一样：单步 >90° 视为跳变忽略。"""
    tot, prev = 0.0, th[0]
    for v in th[1:]:
        d = v - prev
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        if -math.pi / 2 < d < math.pi / 2:
            tot += abs(d)
        prev = v
    return tot


def collect(lk, t_from):
    """从 frame_log 取 t_from 之后的 30025 帧，解析 iu / iw / 电角度。

    布局（mavlink_layouts.json）：iabc_now_A 在 offset 8（3×f32），
    elec_theta_rad 在 offset 20。**角度从同一条 30025 取**，不要另抓 30040 ——
    两路帧不同步，扫角会算错（第一版就踩了：30040 的 elec_angle_now_rad
    在 offset 8，不是 0，读成 0 导致扫角恒为 0，全部判成"不可判"）。
    """
    iu, iw, th = [], [], []
    for e in lk.frame_log(400):
        if e.get("ts", 0) < t_from or e.get("msgid") != 30025:
            continue
        try:
            raw = bytes.fromhex(e["hex"].replace(" ", ""))
        except (ValueError, KeyError):
            continue
        if len(raw) < 24:
            continue
        a, _b, c = struct.unpack_from("<3f", raw, 8)
        ang = struct.unpack_from("<f", raw, 20)[0]
        if any(x != x for x in (a, c, ang)):    # NaN
            continue
        iu.append(a)
        iw.append(c)
        th.append(ang)
    return iu, iw, th


lk = SerialLink()
r = lk.connect(PORT, 921600)
print("[连接] " + ("OK" if r.get("ok") else str(r)))
if not r.get("ok"):
    sys.exit(1)
time.sleep(1.2)
lk.set_permission(0, 0)
time.sleep(0.2)

st = lk.ctrl_status()
print("[状态] cal=%d z=%d en=%d fault=0x%04X vbus=%.2f"
      % (st["cal"], st["z"], st["en"], st["fault"], st["vbus"]))
if st["cal"] != 1 or st["z"] != 1:
    print("[ERR] 需要 cal=1 且 z=1 才能起速度环（先跑 ident_elec 的 C/Z）")
    lk.disconnect()
    sys.exit(1)
if st["fault"]:
    lk.action(3)
    time.sleep(0.3)

print("\n[1] 起速度环 %.0f rpm，采集真实波形" % RPM)
lk.set_mode(3)
lk.set_speed_rpm(RPM)
lk.set_keepalive(True, 3, 5.0)
time.sleep(0.3)
lk.set_controlword(1, 0)
time.sleep(2.0)                     # 等稳态
st = lk.ctrl_status()
print("    en=%d rpm=%.1f iq=%.3f" % (st["en"], st["rpm"], st["iq"]))

t0 = time.time()
time.sleep(1.0)
iu, iw, th = collect(lk, t0)
pm1 = lk.phase_mon()

print("    采到 30025 帧 %d 个（含 iu/iw/电角度）" % len(iu))
chk("采样点足够", len(iu) >= 40, "%d 点" % len(iu))

if len(iu) >= 40:
    sweep = unwrap_sweep(th)
    # 100 Hz 遥测 + 100 rpm（电频率 23 Hz）→ 每点约 1.46 rad，unwrap 有效

    code, ratio, bal = judge(iu, iw, sweep)
    print("    离线：扫角 %.1f rad  ratio_v=%.3f  balance=%.3f  → code=%d"
          % (sweep, ratio, bal, code))
    print("    固件：code=%d(%s)  ratio_v=%.2f  balance=%.2f"
          % (pm1["code"], pm1["name"], pm1["ratio_v"], pm1["balance"]))

    print("\n[2] 健康波形：固件与离线判据都必须判正常")
    chk("固件判正常", pm1["code"] == 0, pm1["name"])
    chk("离线判正常", code == 0, "code=%d" % code)
    chk("ratio_v 固件≈离线", abs(pm1["ratio_v"] - ratio) < 0.12,
        "%.2f vs %.2f" % (pm1["ratio_v"], ratio))
    chk("ratio_v 健康≈1", ratio > 0.75, "%.3f" % ratio)

    print("\n[3] 用同一段真实波形做代换，验证三条分支都能翻转")
    codes = {}
    for name, u, w in (("U 缺（iu≡0）", [0.0] * len(iu), iw),
                       ("W 缺（iw≡0）", iu, [0.0] * len(iw)),
                       ("V 缺（iw≡-iu）", iu, [-x for x in iu])):
        c, rr, bb = judge(u, w, sweep)
        codes[name] = c
        print("    %-16s ratio_v=%.3f balance=%.3f → code=%d" % (name, rr, bb, c))
    chk("U 缺 → 判 U", codes["U 缺（iu≡0）"] == 1)
    chk("V 缺 → 判 V", codes["V 缺（iw≡-iu）"] == 2)
    chk("W 缺 → 判 W", codes["W 缺（iw≡0）"] == 3)
    chk("三条分支互不串", len(set(codes.values())) == 3, str(sorted(codes.values())))

print("\n[4] 收尾：封锁并关保活")
lk.set_keepalive(False)
lk.set_controlword(0, halt=1)
time.sleep(0.6)
st = lk.ctrl_status()
chk("已失能", st["en"] == 0, "en=%d" % st["en"])

print("\n=== 结果：%d 项通过，%d 项失败 ===" % (passed, len(failed)))
if failed:
    print("失败项：" + ", ".join(failed))
lk.disconnect()
sys.exit(0 if not failed else 1)
