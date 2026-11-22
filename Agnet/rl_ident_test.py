# -*- coding: utf-8 -*-
"""rl_ident_test.py —— 跑 @ 在线辨识（R / L）并对照模型值

判读标准（来自 three_loop_params.m 与之前的实测）：
  R 模型 0.3785 ohm，万用表 0.42 ohm
  L 模型 113.5 uH（从未独立验证过）
  所以 @ 的结果如果落在 R∈[0.30,0.50] ohm、L∈[60,200] uH，就算与既有认知一致；
  偏离到这个区间外，要么辨识有问题，要么模型参数有问题 —— 两种都有价值。

用法：
  python rl_ident_test.py            # COM27
  python rl_ident_test.py COM5 2.0   # 指定口与测试电流
"""
import re
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
AMP = sys.argv[2] if len(sys.argv) > 2 else "2.0"
BAUD = 921600


def cmd(ser, c, wait=0.3):
    ser.reset_input_buffer()
    ser.write(c.encode())
    ser.flush()
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < wait:
        d = ser.read(256)
        if d:
            buf += d
            i = buf.rfind(b"[CMD]")
            if i >= 0 and b"\r\n" in buf[i:]:
                return bytes(buf[i:]).split(b"\r\n")[0].decode("ascii", "replace")
    i = buf.rfind(b"[CMD]")
    return bytes(buf[i:]).split(b"\r\n")[0].decode("ascii", "replace") if i >= 0 else "(no echo)"


def field(line, key):
    m = re.search(r"\b" + key + r"=(-?[\w.]+)", line)
    return m.group(1) if m else None


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print("=" * 68)
    print("在线辨识 R / L   port=%s  test current=%s A" % (PORT, AMP))
    print("=" * 68)

    print("\n[1] 复位到已知状态")
    print("    " + cmd(ser, "D"))
    print("    " + cmd(ser, "X"))
    st = cmd(ser, "S")
    print("    " + st)
    if field(st, "cal") != "1":
        print("\n[2] C 标定")
        print("    " + cmd(ser, "C", 0.5))
        time.sleep(0.6)
        st = cmd(ser, "S")
        print("    " + st)
    if field(st, "z") != "1":
        print("\n[3] Z 辨识（约 9 s）")
        print("    " + cmd(ser, "Z", 0.3))
        for _ in range(15):
            time.sleep(1.0)
            line = cmd(ser, "S")
            if "z=1" in line:
                print("    " + line)
                break
        else:
            print("    [FAIL] Z 未完成")
            ser.close()
            return 3

    print("\n[4] @ 启动 R/L 辨识（约 0.4 s，转子会被磁场锁住）")
    t0 = time.time()
    echo = cmd(ser, "@", 0.35)
    print("    " + echo)
    if "refused" in echo:
        ser.close()
        return 4

    # 等结果行。注意：**不能在这期间发别的命令** —— s_out 是单缓冲，
    # 一条 S 命令的回显会把还没发出去的 RL 结果行顶掉（实测就是这么丢的）。
    res = None
    buf = bytearray()
    while time.time() - t0 < 3.0:
        d = ser.read(256)
        if d:
            buf += d
        if b"RL ok" in buf or b"RL FAIL" in buf:
            break
    txt = bytes(buf).decode("ascii", "replace")
    for ln in txt.splitlines():
        if "RL ok" in ln or "RL FAIL" in ln:
            res = ln.strip()
            break

    print("\n" + "=" * 68)
    if res and "RL ok" in res:
        print(res)
        r = re.search(r"\bR=([\d.]+)", res)
        l = re.search(r"\bL=([\d.]+)", res)
        r = r.group(1) if r else None
        l = l.group(1) if l else None
        pts = field(res, "pts")
        sp = field(res, "spread")
        print("\n  辨识值：R = %s mohm   L = %s uH   有效点 %s  离散度 %s%%" % (r, l, pts, sp))
        print("  模型值：R = 378.5 mohm  L = 113.5 uH")
        if r and l:
            rv, lv = float(r), float(l)
            print("  偏差  ：R %+.1f%%      L %+.1f%%"
                  % ((rv / 378.5 - 1) * 100, (lv / 113.5 - 1) * 100))
        if sp and float(sp) > 20:
            print("  ⚠ 离散度偏大：L 拟合点散布宽，建议降低测试电流或检查电流采样噪声")
    else:
        print("辨识未成功：%s" % (res or "(无结果)"))

    print("\n  说明：")
    print("  · R 走 VESC 的稳态 |v|/|i| 口径（虚拟角锁死，ωe=0）")
    print("  · L 走 d 轴阶跃瞬态拟合 v = R·i + L·di/dt（VESC 原版用 HFI+FFT，X72 没有那套模块）")
    print("  · 结果是**额定电流下的等效值**，不是小信号 Ld；饱和会让它偏小")
    print("=" * 68)

    cmd(ser, "D")
    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
