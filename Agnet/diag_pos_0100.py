# -*- coding: utf-8 -*-
"""diag_pos_0100.py —— 位置控制 0x0100 跟随误差的逐拍诊断

用法：
    python diag_pos_0100.py            # 目标 0.0000（复现用户场景）
    python diag_pos_0100.py 3.1416     # 指定目标角（0~2π 语义）

输出：每 20 ms 一行 p_now / p_tgt / p_ref / err / st / wref / wmeas / iq，
      报错时把**报错前 10 拍**单独打出来，并汇总 |err| 峰值。

判读（见 位置控制_0x0100复发_分析_20261122.md §5）：
  · p_tgt 是 6.28 而 err 涨到 6      -> p_meas 在反向跑，查 ds / 编码器
  · p_tgt 随 err 一起变大            -> 目标被跨圈重设
  · p_ref 一路涨而 p_meas 不动        -> 堵转 / 机械限位
"""
import sys
import time
import math

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink

TARGET = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
TWO = 6.283185307179586
DT = 0.02
DUR = 8.0

lk = SerialLink()
lk.connect("COM27", 921600)
time.sleep(4.0)


def wait_status(tmo=8.0):
    t0 = time.time()
    while time.time() - t0 < tmo:
        s = lk.ctrl_status()
        if s:
            return s
        time.sleep(0.2)
    return None


def fault_names(f):
    n = [("过流", 1), ("编码器", 2), ("ADC同步", 4), ("BSPINIT", 8), ("SPI", 16),
         ("超速", 32), ("过压", 64), ("欠压", 128), ("跟随", 256), ("缺相", 512),
         ("电流采样", 1024)]
    return ", ".join(x for x, b in n if f & b) or "无"


s = wait_status()
if s is None:
    print("链路异常：读不到 30090")
    raise SystemExit(1)
lk.set_permission(0, 0)
time.sleep(0.3)

print("===== 当前生效参数 =====")
d = lk.dbg_read()
if d:
    f = d["floats"]
    print("  pos_kp   = %.2f 1/s      (默认 100)" % f.get("pos_kp", -1))
    print("  cur_iqmax= %.2f A        (力矩限幅，默认 5)" % f.get("cur_iqmax", -1))
    print("  spd_iqmax= %.2f A        (速度/位置环限幅)" % f.get("spd_iqmax", -1))
    print("  psi      = %.4f mWb" % (f.get("psi", 0) * 1000.0))
st = lk.ctrl_status()
print("  cal=%d z=%d fault=0x%04X [%s] en=%d rpm=%.0f"
      % (st["cal"], st["z"], st["fault"], fault_names(st["fault"]), st["en"], st["rpm"]))

try:
    if st["fault"]:
        print("\n先清故障...")
        lk.action(3)
        time.sleep(0.8)

    # 停稳 -> 位置模式 -> 使能
    lk.set_keepalive(False)
    lk.set_controlword(0x06, halt=1)
    time.sleep(1.0)
    lk.set_mode(1)
    time.sleep(0.4)
    lk.set_controlword(0x0F, 0)
    time.sleep(1.0)

    p0 = lk.pos_status()
    if p0 is None:
        print("读不到 30100")
        raise SystemExit(1)
    print("\n===== 起环基准 =====")
    print("  p_now=%.4f  p_tgt=%.4f  p_ref=%.4f  err=%+.5f  st=%s"
          % (p0["p_now"], p0["p_tgt"], p0["p_ref"], p0["err"], p0["st_name"]))
    if abs(p0["err"]) > 0.05:
        print("  ★ 起环 err 偏大，说明目标不是当前位置")

    print("\n===== 下发目标 %.4f，逐拍记录（每 %.0f ms）====="
          % (TARGET, DT * 1000))
    lk.set_pos_speed(pos_rad=TARGET)
    time.sleep(0.05)
    lk.set_keepalive(True, 1, 20.0)

    rows = []
    t0 = time.time()
    fault = 0
    while time.time() - t0 < DUR:
        q = lk.pos_status()
        s2 = lk.ctrl_status()
        lp = lk.loop_status()
        if q and s2 and lp:
            rows.append((time.time() - t0, q["p_now"], q["p_tgt"], q["p_ref"],
                         q["err"], q["st_name"], lp["wref"], lp["wmeas"], s2["iq"],
                         s2["fault"]))
            if s2["fault"]:
                fault = s2["fault"]
                break
            if q["settled"]:
                break
        time.sleep(DT)

    print("  t(s)     p_now     p_tgt     p_ref      err     st     wref   wmeas    iq")
    if fault or len(rows) <= 24:
        show = rows
    else:
        show = rows[:12] + [None] + rows[-8:]
    for r in show:
        if r is None:
            print("   ...")
            continue
        print("  %5.2f  %8.4f  %8.4f  %8.4f  %+8.4f  %-5s %+7.1f %+7.1f %+6.3f"
              % (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]))

    if fault:
        print("\n★ 故障 0x%04X [%s]  在 t=%.3f s" % (fault, fault_names(fault), rows[-1][0]))
        print("  报错前 10 拍：")
        for r in rows[-10:]:
            print("  %5.2f  p_now=%8.4f  p_tgt=%8.4f  p_ref=%8.4f  err=%+8.4f  st=%-5s"
                  % (r[0], r[1], r[2], r[3], r[4], r[5]))
    else:
        q = lk.pos_status()
        d_err = q["p_now"] - TARGET
        while d_err > math.pi:
            d_err -= TWO
        while d_err < -math.pi:
            d_err += TWO
        print("\n未报故障：末点 p_now=%.4f  st=%s  环形误差 %+.2f mrad"
              % (q["p_now"], q["st_name"], d_err * 1000))

    if rows:
        peak = max(rows, key=lambda r: abs(r[4]))
        print("\n|err| 峰值 = %.4f rad  (errmax=6.0，行程限制 %.3f)"
              % (abs(peak[4]), abs(rows[-1][2] - rows[0][1])))
        print("  峰值出现在 t=%.2f s: p_now=%.4f p_tgt=%.4f p_ref=%.4f"
              % (peak[0], peak[1], peak[2], peak[3]))

finally:
    lk.set_keepalive(False)
    try:
        lk.set_controlword(0x00, 0)
    except Exception:
        pass
    time.sleep(0.6)
    st = lk.ctrl_status()
    if st:
        print("\n收尾 en=%d fault=0x%04X [%s] cal=%d z=%d"
              % (st["en"], st["fault"], fault_names(st["fault"]), st["cal"], st["z"]))
    lk.disconnect()
