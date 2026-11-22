# -*- coding: utf-8 -*-
"""重试 Z 并把原始回显打全（链路约 2/3 成功，必须多重试）。

用法：python retry_z.py COM27 [注入电流A]
"""
import re
import sys
import time

sys.path.insert(0, r"G:\FOC\X72_Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"

rig = Rig(PORT)
time.sleep(1.0)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)


def acmd(c, wait=1.0, tries=5):
    for k in range(tries):
        rig.rx.pause_evt.set()
        time.sleep(0.04)
        try:
            rig.s.reset_input_buffer()
            rig.s.write(c.encode())
            rig.s.flush()
            raw = b""
            t0 = time.time()
            while time.time() - t0 < wait:
                d = rig.s.read(8192)
                if d:
                    raw += d
        finally:
            rig.rx.buf = bytearray()
            rig.rx.pause_evt.clear()
        txt = strip_frames(raw).decode("ascii", "replace")
        if "[CMD]" in txt:
            return txt
        time.sleep(0.25)
    return ""


def state():
    for _ in range(6):
        s = acmd("S", 1.0, tries=2)
        m = re.search(r"cal=(\d) z=(\d).*?f=(0x[0-9a-fA-F]+)", s)
        if m:
            return m.group(1), m.group(2), m.group(3), s
        time.sleep(0.3)
    return "?", "?", "?", ""


def passive(stops, timeout):
    rig.rx.pause_evt.set()
    time.sleep(0.04)
    try:
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < timeout:
            d = rig.s.read(4096)
            if d:
                buf += d
                txt = strip_frames(bytes(buf)).decode("ascii", "replace")
                for w in stops:
                    if w in txt:
                        return txt, time.time() - t0
            else:
                time.sleep(0.01)
        return strip_frames(bytes(buf)).decode("ascii", "replace"), time.time() - t0
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


c, z, f, line = state()
print("前: cal=%s z=%s f=%s" % (c, z, f))
if f not in ("0x00", "?"):
    print("  清故障:", acmd("X", 1.0).strip()[:60])
    time.sleep(0.5)
    c, z, f, line = state()
    print("  清后: cal=%s z=%s f=%s" % (c, z, f))

print("发 Z ...")
txt, el = passive(["[CMD] IDENT ok", "[CMD] IDENT FAIL"], 50.0)
hits = [l.strip() for l in txt.splitlines() if "IDENT" in l or "ident" in l]
if hits:
    for h in hits:
        print("  %.1fs  %s" % (el, h))
else:
    print("  无 IDENT 行；原始尾部:")
    print("  " + repr(txt[-260:]))

time.sleep(0.8)
c, z, f, line = state()
print("后: cal=%s z=%s f=%s" % (c, z, f))
if line:
    print("  " + line.strip()[:190])
print("[%s] z=%s" % ("OK" if z == "1" else "NG", z))

rig.close()
