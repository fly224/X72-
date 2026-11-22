# -*- coding: utf-8 -*-
"""验证 30094 AppRecord：触发 / 配置 / 停止 / 分块回传（2026-11-21 重写）。

为什么重写：原版判据是"ASCII `&` 的 [REC] 十六进制行 ↔ 30094 逐点对账"，
ASCII 关闭后这条基准消失了（同 verify_migrated_all 的教训）。新版不依赖任何
文本回显，改为**接口自洽 + 物理量对账**：

  结构层   op 语义（START/STOP/DATA/QUERY）、忙互斥、块边界、越界块
  物理层   ch1（iu + 电角度）的电角度累计增量 ↔ 给定转速 × 采样窗口
           —— 这一条同时校验 div 时基、通道选择、角度编码三件事，
              任何一项错都会让增量差几倍（div 错 5 倍、通道错给的是电流、
              角度按 uint16 误当 int16 会得到完全不同的斜率）

用法：python verify_record.py COM27 [rpm]
"""
import math
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
RPM = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
POLE_PAIRS = 14
FS_HZ = 20000.0          # 录波采样基频（div=1 时的 20 kHz）
REC_N = 1024

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


def unwrap(seq, period=2.0 * math.pi):
    """把 mod-2π 的角度序列还原成连续量，返回累计增量（rad）。"""
    if len(seq) < 2:
        return 0.0
    total = 0.0
    prev = seq[0]
    for v in seq[1:]:
        d = v - prev
        while d > period / 2.0:
            d -= period
        while d < -period / 2.0:
            d += period
        total += d
        prev = v
    return total


lk = SerialLink()
r = lk.connect(PORT, 921600)
print("[连接] " + ("OK" if r.get("ok") else str(r)))
if not r.get("ok"):
    sys.exit(1)
time.sleep(1.2)
lk.set_permission(0, 0)
time.sleep(0.3)

st = lk.ctrl_status()
if st is None:
    print("[ERR] 读不到 30090，检查固件/端口")
    lk.disconnect()
    sys.exit(1)
print("[状态] en=%d mode=%d cal=%d z=%d fault=0x%04X vbus=%.2f"
      % (st["en"], st["mode"], st["cal"], st["z"], st["fault"], st["vbus"]))

if st["fault"]:
    a = lk.action(3)
    print("[清故障] " + ("OK" if a.get("ok") else str(a.get("error"))))
    time.sleep(0.3)

# 先确保封锁（STOP 之前不能有 PWM，否则 [2] 那条判据没有意义）
lk.set_keepalive(False)
lk.set_controlword(0, halt=1)
time.sleep(0.9)                     # 执行器 grace 500 ms

print("\n[1] QUERY 结构与默认配置")
q = lk.rec_status()
if not q.get("ok"):
    print("  !! " + str(q.get("error")))
    lk.disconnect()
    sys.exit(1)
print("  div=%d ch=%d state=%d n_pts=%d n_blk=%d"
      % (q["div"], q["ch"], q["state"], q["n_pts"], q["n_blk"]))
chk("QUERY ok", q["ok"])
chk("div 在 1..100", 1 <= q["div"] <= 100, "div=%d" % q["div"])
chk("ch 在 0..2", 0 <= q["ch"] <= 2, "ch=%d" % q["ch"])

print("\n[2] PWM 未开时 START 必须被拒（status=1）")
st = lk.ctrl_status()
r2 = lk.rec_start(ch=0, div=5)
print("  en=%d  rec_start -> %s" % (st["en"], r2.get("error") or "ok"))
chk("无输出被拒", (not r2.get("ok")) and ("无输出" in str(r2.get("error"))),
    "err=%s" % r2.get("error"))

print("\n[3] 起速度环给输出（%.0f rpm）" % RPM)
lk.set_mode(3)
lk.set_speed_rpm(RPM)
lk.set_keepalive(True, 3, 5.0)
time.sleep(0.3)
lk.set_controlword(1, 0)
time.sleep(0.8)
st = lk.ctrl_status()
print("  en=%d mode=%d rpm=%.1f iq=%.3f" % (st["en"], st["mode"], st["rpm"], st["iq"]))
chk("已使能", st["en"] == 1)
chk("实际在环=速度", st["mode"] == 3, "mode=%d" % st["mode"])

print("\n[4] START(div=100) → 采集中 → 重复 START 返回忙")
r4 = lk.rec_start(ch=0, div=100)
chk("START 接受", r4.get("ok"), str(r4.get("error") or ""))
time.sleep(0.6)
q4 = lk.rec_status()
print("  state=%d n_pts=%d" % (q4["state"], q4["n_pts"]))
chk("state=1 采集中", q4["state"] == 1, "state=%d" % q4["state"])
r4b = lk.rec_start(ch=0, div=100)
print("  再 START -> %s" % (r4b.get("error") or "ok"))
chk("采集中拒绝重复 START", (not r4b.get("ok")) and ("正在录" in str(r4b.get("error"))),
    "err=%s" % r4b.get("error"))

print("\n[5] op=2 STOP：提前停止且保留已采点数（新功能）")
n_before = lk.rec_status()["n_pts"]
r5 = lk.rec_stop()
print("  rec_stop -> %s  n_pts=%s" % ("ok" if r5.get("ok") else r5.get("error"),
                                      r5.get("n_pts")))
chk("STOP 接受", r5.get("ok"), str(r5.get("error") or ""))
chk("停在 1024 点之前", 0 < (r5.get("n_pts") or 0) < REC_N,
    "n=%s（div=100 窗口 5.12 s，停得早才有余量）" % r5.get("n_pts"))

time.sleep(0.6)
q5 = lk.rec_status()
print("  0.6 s 后 n_pts=%d state=%d" % (q5["n_pts"], q5["state"]))
chk("停止后不再增长", q5["n_pts"] == r5.get("n_pts"),
    "%d vs %d" % (q5["n_pts"], r5.get("n_pts")))
chk("state=2 已有数据", q5["state"] == 2, "state=%d" % q5["state"])
chk("n_pts 与 STOP 应答一致", q5["n_pts"] == r5.get("n_pts"))
r5b = lk.rec_stop()
chk("重复 STOP 幂等", r5b.get("ok"), str(r5b.get("error") or ""))

print("\n[6] 停止后再 START 可重触发（不需要复位）")
r6 = lk.rec_start(ch=0, div=1)
chk("重触发接受", r6.get("ok"), str(r6.get("error") or ""))
time.sleep(0.4)                      # div=1 → 51 ms 采满
q6 = lk.rec_status()
print("  state=%d n_pts=%d n_blk=%d" % (q6["state"], q6["n_pts"], q6["n_blk"]))
chk("采满 1024", q6["n_pts"] == REC_N, "n=%d" % q6["n_pts"])
chk("n_blk=64", q6["n_blk"] == 64, "n_blk=%d" % q6["n_blk"])

print("\n[7] DATA 分块：末块余数 + 越界块自描述")
p = lk._rec_req(3, arg=0)
chk("块 0 满 16 点", p and p["n_this"] == 16, "n_this=%s" % (p and p["n_this"]))
pl = lk._rec_req(3, arg=q6["n_blk"] - 1)
chk("末块 16 点", pl and pl["n_this"] == 16, "n_this=%s" % (pl and pl["n_this"]))
po = lk._rec_req(3, arg=q6["n_blk"])
chk("越界块 n_this=0", po and po["n_this"] == 0,
    "n_this=%s status=%s" % (po and po["n_this"], po and po["status"]))

print("\n[8] ch1 物理对账：电角度增量 ↔ 转速 × 采样窗口")
r8 = lk.rec_start(ch=1, div=5)
chk("ch1 START 接受", r8.get("ok"), str(r8.get("error") or ""))
time.sleep(0.5)                      # div=5 → 256 ms 采满
q8 = lk.rec_status()
chk("ch1 采满 1024", q8["n_pts"] == REC_N, "n=%d" % q8["n_pts"])

d = lk.rec_read(timeout=1.0)
if d.get("ok"):
    th = d["th"][:d["n_pts"]]
    dth = unwrap(th)
    t_win = REC_N * 5.0 / FS_HZ                     # 0.256 s
    expect = 2.0 * math.pi * (RPM / 60.0) * t_win * POLE_PAIRS
    err = abs(abs(dth) - expect) / expect
    print("  电角度累计 %.2f rad（期望 %.2f，误差 %.1f%%）"
          % (dth, expect, err * 100.0))
    print("  前 5 点角度(rad): " + " ".join("%.3f" % v for v in th[:5]))
    chk("角度增量正确（±25%）", err < 0.25, "%.1f%%" % (err * 100.0))
    chk("角度落在 0..2π", all(-0.01 <= v <= 6.30 for v in th),
        "%.3f..%.3f" % (min(th), max(th)))
    iu = [v for v in d["val"][:d["n_pts"]]]
    chk("iu 有实际电流", max(abs(v) for v in iu) > 0.05,
        "max|iu|=%.3f A rms≈%.3f A"
        % (max(abs(v) for v in iu),
           math.sqrt(sum(v * v for v in iu) / len(iu))))
else:
    failed.append("ch1 rec_read")
    print("  FAIL  " + str(d.get("error")))

print("\n[9] 收尾：封锁并确认失能")
lk.set_keepalive(False)
lk.set_controlword(0, halt=1)
time.sleep(0.5)
st = lk.ctrl_status()
print("  en=%d mode=%d rpm=%.1f" % (st["en"], st["mode"], st["rpm"]))
chk("已失能", st["en"] == 0)

print("\n=== 结果：%d 项通过，%d 项失败 ===" % (passed, len(failed)))
if failed:
    print("失败项：" + ", ".join(failed))
lk.disconnect()
sys.exit(0 if not failed else 1)
