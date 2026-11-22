# -*- coding: utf-8 -*-
"""debug_setrpm.py —— 单独验证 `=` 数值调速的收发时序"""
import time

from cog_ff_test import Rig


def drain(rig, secs):
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < secs:
        d = rig.s.read(512)
        if d:
            buf += d
    txt = "".join(ch if 32 <= ord(ch) < 127 else "\n"
                  for ch in buf.decode("ascii", "replace"))
    return [ln.strip() for ln in txt.split("\n") if ln.strip().startswith("[CMD]")]


def try_send(rig, tag, chunks, gaps):
    rig.s.reset_input_buffer()
    for c, g in zip(chunks, gaps):
        rig.s.write(c if isinstance(c, bytes) else c.encode())
        rig.s.flush()
        time.sleep(g)
    return tag, drain(rig, 0.6)


def main():
    rig = Rig("COM27")
    try:
        print("S:", rig.cmd("S", 0.5))
        print("D:", rig.cmd("D", 0.4))

        for tag, chunks, gaps in (
            ("A 一次发 =160\\r", [b"=160\r"], [0.0]),
            ("B 分两段 =|160\\r", [b"=", b"160\r"], [0.05, 0.0]),
            ("C 逐字符", [b"=", b"1", b"6", b"0", b"\r"], [0.04, 0.006, 0.006, 0.006, 0.0]),
        ):
            print("\n--- %s ---" % tag)
            for ln in try_send(rig, tag, chunks, gaps)[1]:
                print("   ", ln)
            print("   S:", rig.cmd("S", 0.4)[:90])
            rig.cmd("D", 0.3)
    finally:
        rig.cmd("D", 0.3)
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
