# -*- coding: utf-8 -*-
"""param_flash_test.py —— 验证"辨识结果进 Flash、冷启动自动恢复"的完整链路

验证的是**行为**而不是"代码看起来对"，所以每一步都要拿到板上回显作为证据：
  1) 烧录后首次上电：fl=0 cal=0 z=0（Flash 空）
  2) C 标定 -> 自动保存
  3) Z 辨识 -> 自动保存
  4) A 手动保存 -> 回显 PS saved
  5) ! 软复位 -> 冷启动
  6) 复位后 S：必须是 cal=1 z=1 fl=1，且不需要再发 C/Z

第 6 步是关键：如果复位后 cal/z 又变 0，说明 param_load_and_apply 没生效
（常见原因：CRC 算法两边不一致、bank/page 算错、或者擦除后没真正编程）。

用法：
  python param_flash_test.py            # COM27
  python param_flash_test.py COM5
"""
import re
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
BAUD = 921600


def open_port():
    return serial.Serial(PORT, BAUD, timeout=0.1)


def cmd(ser, c, wait=0.25):
    """发一个字符，返回 [CMD] 回显行（从原始字节流里截）"""
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
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
    return bytes(buf[i:]).split(b"\r\n")[0].decode("ascii", "replace") if i >= 0 else ""


def field(line, key):
    m = re.search(r"\b" + key + r"=(-?[\w.]+)", line)
    return m.group(1) if m else None


def main():
    ser = open_port()
    print("=" * 66)
    print("Flash 参数持久化验证  port=%s" % PORT)
    print("=" * 66)

    print("\n[1] 清故障并读初始状态（烧录后 Flash 应为空）")
    print("    " + cmd(ser, "D"))
    print("    " + cmd(ser, "X"))
    st = cmd(ser, "S")
    print("    " + st)
    fl0, cal0, z0 = field(st, "fl"), field(st, "cal"), field(st, "z")
    print("    -> fl=%s cal=%s z=%s" % (fl0, cal0, z0))

    if cal0 != "1":
        print("\n[2] C 电流零点标定（200 ms，保持静止）")
        print("    " + cmd(ser, "C", 0.5))
        time.sleep(0.6)
        st = cmd(ser, "S")
        print("    " + st)
        print("    -> cal=%s" % field(st, "cal"))

    if field(cmd(ser, "S"), "z") != "1":
        print("\n[3] Z 电角度/极对数/方向辨识（约 9 s，转子会小幅步进）")
        print("    " + cmd(ser, "Z", 0.3))
        z_done = False
        for _ in range(15):
            time.sleep(1.0)
            line = cmd(ser, "S")
            if "z=1" in line:
                print("    " + line)
                z_done = True
                break
        if not z_done:
            print("    [FAIL] Z 未完成")
            ser.close()
            return 3

    print("\n[4] 等自动保存落地（1 ms 任务每 500 ms 检查一次），再手动 A 确认")
    time.sleep(1.0)
    print("    " + cmd(ser, "A"))
    st = cmd(ser, "S")
    print("    " + st)
    print("    -> fl=%s" % field(st, "fl"))

    if field(st, "fl") != "1":
        print("\n    [FAIL] Flash 里没有有效参数，后续复位验证无意义")
        ser.close()
        return 4

    print("\n[5] ! 软复位（模拟冷启动）")
    ser.write(b"!")
    ser.flush()
    time.sleep(0.3)
    try:
        ser.close()
    except Exception:
        pass
    time.sleep(2.5)                     # 等 ST-Link VCP 重新枚举

    ser = open_port()
    time.sleep(0.5)
    boot = bytearray()
    t0 = time.time()
    while time.time() - t0 < 1.5:
        d = ser.read(512)
        if d:
            boot += d
    txt = bytes(boot).decode("ascii", "replace")
    ps_line = [ln for ln in txt.splitlines() if "ps:" in ln]
    print("    启动横幅里的持久化行：%s" % (ps_line[0] if ps_line else "(未捕获)"))

    print("\n[6] 复位后状态（这是本测试的核心判据）")
    st = cmd(ser, "S", 0.4)
    print("    " + st)
    cal, z, fl = field(st, "cal"), field(st, "z"), field(st, "fl")

    print("\n" + "=" * 66)
    ok = (cal == "1") and (z == "1") and (fl == "1")
    print("结果：cal=%s  z=%s  fl=%s  -> %s" % (cal, z, fl, "PASS" if ok else "FAIL"))
    if ok:
        print("冷启动直接可用，不需要再发 C / Z。")
    else:
        print("冷启动未恢复：检查 param_store 的 CRC 计算、bank/page 计算，")
        print("以及 Keil 的擦除设置是否把最后一页也擦掉了（Erase Full Chip）。")
    print("=" * 66)

    cmd(ser, "D")
    ser.close()
    return 0 if ok else 5


if __name__ == "__main__":
    sys.exit(main())
