# -*- coding: utf-8 -*-
"""电气 + 机械参数辨识（一次跑完，原始回显全记录）。

顺序（避免已知冲突）：
  +  ψ/R/L（需转动，9~12 s）
  @  R/L（注入法）
  <  惯量 J（iq 1A/2A）
  )  摩擦 Tc/Bv（8 点）
  (  扫频（20..1000 Hz，约 150 s）
  }  齿槽（2 rpm 1 转）

已知坑：
  · @ 后紧跟 + 会 FI FAIL，所以先 + 再 @（每步之间 D+X 复位）
  · 全部要求 MOE=0（D 状态）
"""
import re
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402
from combo_sweep import cmd, fault, toggle_expect  # noqa: E402

RE_FI = re.compile(r"FI (?:ok )?R=([\d.]+)mohm L=([\d.]+)uH psi=([\d.]+)mWb")
RE_RL = re.compile(r"RL ok R=([\d.]+)mohm L=([\d.]+)uH")


def run(rig, ch, wait, tag):
    print("\n--- %s（cmd '%s'，等待 %.0f s）---" % (tag, ch, wait))
    t0 = time.time()
    txt = cmd(rig, ch, wait)
    lines = [l.strip() for l in txt.splitlines() if "[CMD]" in l]
    for l in lines:
        print("  " + l[:180])
    print("  耗时 %.1f s，回显 %d 行" % (time.time() - t0, len(lines)))
    return txt


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    rig = Rig(port)
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.25)
        cmd(rig, "D", 0.6)
        cmd(rig, "X", 0.7)
        cmd(rig, "Y", 0.4)
        toggle_expect(rig, "%", False)
        toggle_expect(rig, "B", True)
        cmd(rig, "\\59\r", 0.4)

        snap = rig.rx.get_snap()[0]
        t_mot = snap.get("temp_mot") if snap else None
        t_drv = snap.get("temp_drv") if snap else None
        print("起点温度: 电机 %s °C / 驱动 %s °C" %
              ("%.2f" % t_mot if t_mot else "?", "%.2f" % t_drv if t_drv else "?"))

        for l in cmd(rig, "S", 0.7).splitlines():
            if "st en=" in l:
                print("状态: " + l.strip()[:150])

        # 1) 电气：+ (psi/R/L)
        txt = run(rig, "+", 20.0, "电气辨识 + ：psi / R / L（多点回归）")
        m = RE_FI.search(txt)
        if m:
            print("  => R=%.2f mΩ  L=%.2f µH  psi=%.4f mWb"
                  % (float(m.group(1)), float(m.group(2)), float(m.group(3))))
        cmd(rig, "D", 0.5)
        cmd(rig, "X", 0.6)
        time.sleep(0.8)

        # 2) 电气：@ (R/L 注入法)
        txt = run(rig, "@", 10.0, "电气辨识 @ ：R / L（阶跃注入）")
        m = RE_RL.search(txt)
        if m:
            print("  => R=%.2f mΩ  L=%.2f µH" % (float(m.group(1)), float(m.group(2))))
        cmd(rig, "D", 0.5)
        cmd(rig, "X", 0.6)
        time.sleep(0.8)

        # 3) 机械：惯量
        run(rig, "<", 40.0, "机械辨识 < ：转动惯量 J")
        cmd(rig, "D", 0.5)
        cmd(rig, "X", 0.6)
        time.sleep(0.8)

        # 4) 机械：摩擦
        run(rig, ")", 70.0, "机械辨识 ) ：摩擦 Tc / Bv")
        cmd(rig, "D", 0.5)
        cmd(rig, "X", 0.6)
        time.sleep(0.8)

        # 5) 机械：齿槽 / 扫频 —— 本轮跳过（齿槽是前馈标定表、扫频是频响曲线，
        #    都不属于温升测试要跟踪的"参数"，且合计约 4 分钟）
        print("\n--- 跳过 } 齿槽 / ( 扫频（如需另跑：python ident_extra.py）---")

        cmd(rig, "D", 0.6)
        cmd(rig, "X", 0.6)
        f = fault(rig)
        print("\n收尾 f=0x%s" % ("--" if f is None else "%02x" % f))
        for l in cmd(rig, "K", 0.6).splitlines():
            if "psi=" in l:
                print("控制律当前: " + l.strip()[:190])
        return 0
    finally:
        try:
            rig.s.write(b"D")
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    sys.exit(main())
