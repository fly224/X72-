# -*- coding: utf-8 -*-
"""speed_run.py —— 速度模式定速跑（任意转速），输出稳定性指标与判定。

为什么单独一个脚本：`bringup_1000.py` 是 ASCII 时代的（写 `E`/`=rpm`），
ASCII 关闭后发不出有效命令；而每次只想看"某个转速稳不稳"时不该重跑整套辨识。
这里只做一件事：起速度环 → 稳态采样 → 给均值/波动/电流 → 收尾失能。

判据（沿用 1000 rpm 验收时定的口径）：
  |均值 − 给定| ≤ 0.5%      —— 稳态精度
  CV（标准差/均值） ≤ 1.0%   —— 稳态平稳度
  稳态 iterm 很小            —— 前馈把 BEMF/摩擦补掉了，积分不用硬顶
  无故障、缺相判"正常"        —— 顺带回归新加的缺相检测

用法：
    python speed_run.py COM27            # 默认 100 rpm / 5 s
    python speed_run.py COM27 1000 8
"""
import argparse
import math
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("port", nargs="?", default="COM27")
ap.add_argument("rpm", nargs="?", type=float, default=100.0)
ap.add_argument("secs", nargs="?", type=float, default=5.0)
ap.add_argument("--acc", type=float, default=0.0, help="加速度 rad/s²（0=不改）")
args = ap.parse_args()

passed, failed = 0, []


def chk(name, cond, detail=""):
    global passed
    if cond:
        passed += 1
        print("  PASS  %-30s %s" % (name, detail))
    else:
        failed.append(name)
        print("  FAIL  %-30s %s" % (name, detail))


lk = SerialLink()
r = lk.connect(args.port, 921600)
print("[连接] " + ("OK" if r.get("ok") else str(r)))
if not r.get("ok"):
    sys.exit(1)
time.sleep(1.2)
lk.set_permission(0, 0)
time.sleep(0.2)

st = lk.ctrl_status()
print("[状态] cal=%d z=%d cog=%d en=%d fault=0x%04X vbus=%.2f"
      % (st["cal"], st["z"], st["cog"], st["en"], st["fault"], st["vbus"]))
if st["cal"] != 1 or st["z"] != 1:
    print("[ERR] 需要 cal=1 且 z=1（先跑 ident_elec 里的 C / Z）")
    lk.disconnect()
    sys.exit(1)
if st["fault"]:
    print("[清故障] " + str(lk.action(3).get("ok")))
    time.sleep(0.4)
if st["en"]:
    lk.set_controlword(0, halt=1)
    time.sleep(0.8)

print("\n[1] 起速度环 %.0f rpm" % args.rpm)
lk.set_mode(3)
if args.acc > 0:
    lk.set_speed_accel(args.acc)
lk.set_speed_rpm(args.rpm)
lk.set_keepalive(True, 3, 5.0)
time.sleep(0.3)
t_en = time.time()
lk.set_controlword(1, 0)

# 升速时间：进环后到 |w| 进入 ±1% 用多久
t_reach, rpm_now = None, 0.0
while time.time() - t_en < 3.0:
    time.sleep(0.02)
    s = lk.ctrl_status()
    rpm_now = s["rpm"]
    if t_reach is None and abs(abs(rpm_now) - abs(args.rpm)) <= 0.01 * abs(args.rpm):
        t_reach = time.time() - t_en
    if t_reach is not None and time.time() - t_en > (t_reach + 0.3):
        break
print("    升速到位 %s，当前 %.1f rpm"
      % (("%.0f ms" % (t_reach * 1000)) if t_reach else "未在 3 s 内", rpm_now))

time.sleep(1.0)                        # 再稳一拍才开始统计

print("\n[2] 稳态采样 %.1f s" % args.secs)
t0 = time.time()
spd, iq, it, kp = [], [], [], []
while time.time() - t0 < args.secs:
    s = lk.ctrl_status()
    l = lk.loop_status()
    if s and l:
        spd.append(s["rpm"])
        iq.append(s["iq"])
        it.append(l["iterm"])
        kp.append(l["kp"])
    time.sleep(0.01)

n = len(spd)
mean = sum(spd) / n
var = sum((x - mean) ** 2 for x in spd) / n
sd = math.sqrt(var)
cv = sd / abs(mean) * 100.0 if abs(mean) > 1e-9 else float("inf")
err = (mean - args.rpm) / args.rpm * 100.0
print("    样本 %d  均值 %.2f rpm  标准差 %.3f  极差 %.2f..%.2f"
      % (n, mean, sd, min(spd), max(spd)))

print("\n[3] 判定")
chk("稳态误差 ≤0.5%%", abs(err) <= 0.5, "%+.3f%%（%.2f vs %.2f）" % (err, mean, args.rpm))
chk("波动 CV ≤1.0%%", cv <= 1.0, "%.3f%%" % cv)
chk("稳态 iq 有界（≤3 A）", max(abs(x) for x in iq) <= 3.0,
    "均值 %.3f A，峰 %.3f A" % (sum(iq) / n, max(abs(x) for x in iq)))
chk("积分项未被顶死", abs(sum(it) / n) < 1.0,
    "iterm 均值 %.3f A（前馈生效时应接近 0）" % (sum(it) / n))

pm = lk.phase_mon()
print("    缺相：%s（ratio_v=%.2f bal=%.2f raw=%s）"
      % (pm["name"], pm["ratio_v"], pm["balance"], pm["raw_name"]))
chk("缺相未误报", pm["code"] == 0, pm["name"])
chk("无故障位", lk.ctrl_status()["fault"] == 0,
    "fault=0x%04X" % lk.ctrl_status()["fault"])

print("\n[4] 收尾")
lk.set_keepalive(False)
lk.set_controlword(0, halt=1)
time.sleep(0.8)
st = lk.ctrl_status()
print("    en=%d mode=%d rpm=%.2f" % (st["en"], st["mode"], st["rpm"]))
chk("已失能", st["en"] == 0)

print("\n=== 结果：%d 项通过，%d 项失败 ===" % (passed, len(failed)))
if failed:
    print("失败项：" + ", ".join(failed))
lk.disconnect()
sys.exit(0 if not failed else 1)
