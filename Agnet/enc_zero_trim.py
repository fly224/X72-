"""
enc_zero_trim.py —— 把 Z 之后残留的常值电角度偏置补掉（方案 A）

原理：Z 用 60 步 ×120° 求平均零点，残余是一个常值电角度误差（实测约 0.8°）。
在 id=0 稳态下它表现为 vd 积分的直流分量，用同一套测量算出均值，
写进 30082 的 eleoff_deg 即可，不需要新代码、不掉参数、随时可回退。

用法：
    python enc_zero_trim.py COM27 500          # 只测，打印建议值
    python enc_zero_trim.py COM27 500 --apply  # 测完写入并复测
"""
import sys
import time
import math

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
import enc_comp_compare as E  # noqa: E402


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    rpm = float(sys.argv[2]) if len(sys.argv) > 2 else 500.0
    apply_it = "--apply" in sys.argv

    lk = E.connect(port)
    try:
        st = lk.ctrl_status()
        if st["fault"]:
            lk.action(3)
            time.sleep(0.6)
        if not st["cal"] or not st["z"]:
            raise SystemExit("cal=%d z=%d：先做 C/Z" % (st["cal"], st["z"]))

        lk.enc_comp_off()                     # 谐波补偿必须关，否则均值里混入补偿的直流
        e0 = lk.dbg_read()["floats"]["eleoff_deg"]
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

        c, mean, rms, psi = E.measure(lk, segs=5)
        print("当前 eleoff=%.4f°  实测残余均值 %.4f°  δRMS %.4f°"
              % (e0, mean * 180 / math.pi, rms * 180 / math.pi))

        target = e0 - mean * 180 / math.pi
        print("建议 eleoff = %.4f°" % target)
        if not apply_it:
            return

        lk.dbg_write(floats={"eleoff_deg": target})
        time.sleep(0.8)
        c, mean2, rms2, _ = E.measure(lk, segs=5)
        print("写入后：eleoff=%.4f°  残余均值 %.4f°  δRMS %.4f°"
              % (lk.dbg_read()["floats"]["eleoff_deg"],
                 mean2 * 180 / math.pi, rms2 * 180 / math.pi))
        print("判定：%s" % ("通过（|均值| < 0.1°）"
                          if abs(mean2 * 180 / math.pi) < 0.1 else "未收敛，检查转速是否稳定"))
    finally:
        lk.set_keepalive(False)
        lk.set_speed_rpm(0)
        time.sleep(0.8)
        lk.set_controlword(0x06, halt=1)
        time.sleep(0.3)
        print("收尾 en=%d" % lk.ctrl_status()["en"])
        lk.disconnect()


if __name__ == "__main__":
    main()
