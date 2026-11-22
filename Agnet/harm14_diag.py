"""
harm14_diag.py —— 判定 14 阶（117 Hz @500rpm）iq 纹波是不是"速度环自己打出来的"

背景
----
速度闭环稳态下，iq 是"负载/扰动"的像：扰动需要多少转矩，iq 就得给多少。
所以在速度模式里**往 iq 上注入同频分量不会降低 iq 纹波**（前馈给多少，
PI 就少给多少，总和不变）。真正能改变 iq 纹波的只有两件事：
  ① 扰动本身变小（磁钢/机械）；
  ② 速度环在该频率上不再"较劲"（降低环路增益）。

本脚本用现成的 30096 陷波做第 ② 种验证：
  - 若在 117 Hz 陷波后 **iq 的 14 阶明显下降** → 纹波里有相当一部分是速度环
    相位不对"打出来"的，陷波（或相位修正）是有效手段；
  - 若几乎不变 → 纹波就是真实扰动，固件侧没有对冲手段，只能从磁钢/机械
    或电流环带宽入手。

用法：python harm14_diag.py COM27 500
"""
import sys
import time
import math

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
import enc_comp_compare as E  # noqa: E402


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 500.0
    f14 = rpm / 60.0 * 14.0

    lk = E.connect(port)
    try:
        st = lk.ctrl_status()
        if st["fault"]:
            lk.action(3)
            time.sleep(0.6)
        lk.enc_comp_off()
        lk.notch(en=0)
        lk.set_controlword(0x06, halt=1)
        time.sleep(0.4)

        lk.set_mode(3)
        lk.set_speed_accel(400.0)
        lk.set_speed_traj(0.0)
        lk.set_speed_rpm(rpm)
        lk.set_keepalive(True, 3, 5.0)
        time.sleep(0.3)
        lk.set_controlword(0x0F, 0)
        time.sleep(2.6)

        cases = [
            ("基线（陷波关）",        0, 0.0, 0.0),
            ("117Hz 12dB Q3",        1, f14, 12.0),
            ("117Hz 24dB Q8",        1, f14, 24.0),
            ("117Hz 40dB Q8",        1, f14, 40.0),
        ]
        rows = []
        for name, en, f0, dep in cases:
            lk.notch(en=en, f0_hz=(f0 if en else 150.0), depth_db=dep, q=8.0 if en else 2.0)
            time.sleep(1.0)
            iq = E.iq_metric(lk, rpm)
            if iq is None:
                print("  %-16s 采集失败" % name)
                continue
            rows.append((name, iq))
            print("  %-16s iq AC %.4f  h14 %.5f  h4 %.5f  DC %.4f"
                  % (name, iq["ac_rms"], iq["h14"], iq["h4"], iq["dc"]))

        lk.notch(en=0)
        print("\n%-18s %9s %9s" % ("工况", "iq_AC", "h14"))
        for name, iq in rows:
            print("%-18s %9.4f %9.5f" % (name, iq["ac_rms"], iq["h14"]))
        if len(rows) >= 2:
            b = rows[0][1]["h14"]
            for name, iq in rows[1:]:
                print("  %s：h14 %+.1f%%" % (name, (iq["h14"] / b - 1.0) * 100.0))
    finally:
        lk.notch(en=0)
        lk.set_keepalive(False)
        lk.set_speed_rpm(0)
        time.sleep(0.8)
        lk.set_controlword(0x06, halt=1)
        time.sleep(0.3)
        print("收尾 en=%d" % lk.ctrl_status()["en"])
        lk.disconnect()


if __name__ == "__main__":
    main()
