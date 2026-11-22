# -*- coding: utf-8 -*-
"""repro_high2zero.py —— 复现"高速切零速报 30019 状态错误"，并抓故障现场

用法：
    python repro_high2zero.py COM27 1000          # 用当前 acc（默认 400）减速
    python repro_high2zero.py COM27 1000 1.5      # 用 30097 设 1.5 s 减速（对比用）

它做四件事：
  1. 起转到指定转速并稳住
  2. 切 0，全程 100 Hz 记录 rpm / vbus / iq / fault / en
  3. 打印故障首次出现的时刻与前后各 6 点（看 vbus 冲到哪里）
  4. 切零后 0.6 s 主动发一帧 30019（等价于保活的下一帧），复现"状态错误"

判据：30019 返回 status=3 只可能发生在 s_armed==0 时（servo_cmd.c 的
CtrlSpeed 用 `s_armed && IsSpeedMode` 判定），所以一定是"先被故障封锁、
下一帧给定才被拒"。这个脚本就是要把那个"先"抓出来。
"""
import math
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
from mavlink_link import SerialLink   # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
RPM_HI = float(sys.argv[2]) if len(sys.argv) > 2 else 1000.0
T_RAMP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

FAULTS = [(0, "过流"), (1, "编码器"), (2, "ADC 同步"), (3, "BSP 初始化"),
          (4, "SPI"), (5, "超速"), (6, "母线过压"), (7, "母线欠压"),
          (8, "跟随误差"), (9, "缺相")]


def parse90(raw):
    fl = raw[3]
    f = struct.unpack_from("<8f", raw, 4)
    return {"cw": raw[0] | (raw[1] << 8), "mode": raw[2],
            "en": (fl >> 2) & 1, "cal": fl & 1, "z": (fl >> 1) & 1,
            "rpm": f[2], "iq": f[0], "vbus": f[4],
            "fault": raw[36] | (raw[37] << 8)}


class Snap:
    def __init__(self, lk):
        self.lk = lk
        self.last = None
        self.rows = []

    def pump(self, t0):
        m = self.lk.last_msg.get(30090)
        if m is None or m["ts"] < t0 or m["ts"] == self.last:
            return
        raw = self.lk._payload_of(m)
        if not raw or len(raw) < 40:
            return
        self.last = m["ts"]
        d = parse90(raw)
        d["t"] = m["ts"] - t0
        self.rows.append(d)


def show(rows, i0, i1, mark=None):
    for i in range(max(0, i0), min(len(rows), i1)):
        r = rows[i]
        flag = "  <<<" if mark is not None and i == mark else ""
        print("    t=%5.3f  rpm=%8.2f  vbus=%6.2f  iq=%6.3f  en=%d  fault=0x%04X%s"
              % (r["t"], r["rpm"], r["vbus"], r["iq"], r["en"], r["fault"], flag))


def main():
    lk = SerialLink()
    c = lk.connect(PORT, 921600)
    print("连接:", c.get("ok"), c.get("error", ""))
    if not c.get("ok"):
        return 2
    time.sleep(1.2)
    lk.set_permission(0, 0)
    st = lk.ctrl_status()
    if st is None:
        print("读不到 30090，终止")
        lk.disconnect()
        return 2
    if st["fault"]:
        print("先清故障 0x%04X" % st["fault"])
        lk.action(3)
        time.sleep(0.6)
    if T_RAMP > 0:
        r = lk.set_speed_traj(T_RAMP)
        print("轨迹时间 %.2f s -> a_eff=%.1f" % (T_RAMP, r.get("a_eff", 0)))
    else:
        lk.set_speed_traj(0.0)
        lk.set_speed_accel(400.0)
        print("用 acc=400 rad/s² 减速（约 %.0f ms 到 0）"
              % (RPM_HI * 2 * math.pi / 60 / 400 * 1000))

    lk.set_speed_profile(0, 20000.0)
    lk.set_mode(3)
    lk.set_keepalive(True, 3, 5.0)
    lk.set_speed_rpm(RPM_HI)
    time.sleep(0.3)
    lk.set_controlword(0x0F, 0)
    time.sleep(0.2)

    s = Snap(lk)
    t0 = time.time()
    while time.time() - t0 < 4.0:
        s.pump(t0)
        st = lk.ctrl_status()
        if st and abs(st["rpm"] - RPM_HI) < RPM_HI * 0.02:
            break
        time.sleep(0.05)
    print("已到速: %.1f rpm（目标 %.0f）" % (st["rpm"] if st else -1, RPM_HI))
    time.sleep(0.8)

    print("\n[1] 切 0")
    s2 = Snap(lk)
    t1 = time.time()
    lk.set_speed_rpm(0.0)
    dur = max(3.0, (RPM_HI * 2 * math.pi / 60 / 400.0) * 3.0)
    if T_RAMP > 0:
        dur = max(dur, T_RAMP * 2.5)
    while time.time() - t1 < dur:
        s2.pump(t1)
        time.sleep(0.004)

    rows = s2.rows
    print("    采到 %d 点（%.1f Hz 有效）" % (len(rows), len(rows) / dur))
    if not rows:
        print("    没有 30090，终止")
        lk.disconnect()
        return 2

    vmax = max(rows, key=lambda r: r["vbus"])
    print("    母线峰值 %.2f V @ t=%.3f（阈值 58 V）" % (vmax["vbus"], vmax["t"]))
    fi = next((i for i, r in enumerate(rows) if r["fault"] != 0), None)
    if fi is None:
        print("    全程 fault=0 —— 不是故障封锁，问题在别处")
    else:
        r = rows[fi]
        print("    故障首次出现 t=%.3f  fault=0x%04X  rpm=%.1f  vbus=%.2f  iq=%.3f"
              % (r["t"], r["fault"], r["rpm"], r["vbus"], r["iq"]))
        names = [n for b, n in FAULTS if r["fault"] & (1 << b)]
        print("    故障位: %s" % (" + ".join(names) if names else "?"))
        show(rows, fi - 6, fi + 7, fi)

    print("\n[2] 复现 30019 的应答（等价于保活下一帧）")
    a = lk.action(0)                     # 先确认链路
    time.sleep(0.1)
    r = lk.set_speed_rpm(120.0)
    print("    30019 -> status=%s ok=%s err=%s" % (r.get("status"), r.get("ok"), r.get("error")))
    st = lk.ctrl_status()
    print("    之后: en=%d cw=0x%02X fault=0x%04X rpm=%.1f"
          % (st["en"], st["cw"], st["fault"], st["rpm"]))

    print("\n[3] 收尾")
    lk.set_speed_traj(0.0)
    lk.set_speed_rpm(0.0)
    time.sleep(0.5)
    lk.set_keepalive(False)
    lk.set_controlword(0x06, halt=1)
    time.sleep(0.5)
    st = lk.ctrl_status()
    print("    en=%d fault=0x%04X" % (st["en"], st["fault"]))
    lk.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
