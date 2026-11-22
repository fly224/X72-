# -*- coding: utf-8 -*-
"""cog_status.py —— 只读查询齿槽前馈当前状态（不发任何运动命令）

用法: python cog_status.py COM27
"""
import sys
import time

import serial


def ask(s, c, wait=0.5):
    s.reset_input_buffer()
    s.write(c.encode())
    s.flush()
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < wait:
        d = s.read(512)
        if d:
            buf.extend(d)
    # 串口里混着 30025 二进制遥测：先把不可打印字节变成换行，
    # 再只挑 [CMD] 开头的行，否则一行会被乱码冲掉。
    txt = "".join(ch if 32 <= ord(ch) < 127 else "\n"
                  for ch in buf.decode("ascii", "replace"))
    return [ln.strip() for ln in txt.split("\n") if ln.strip().startswith("[CMD]")]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    s = serial.Serial(port, 921600, timeout=0.05)
    try:
        for c, name in (("S", "全局状态"), (":", "齿槽前馈状态"), ("=", "调度段")):
            print("--- %s (%s) ---" % (name, c))
            for ln in ask(s, c):
                print(ln)
    finally:
        s.close()


if __name__ == "__main__":
    main()
