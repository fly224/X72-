# -*- coding: utf-8 -*-
"""_probe_iterr.py —— 验证位置环 Ki=0 时抗饱和是否写坏 s_i_term

假说：POS_KI_DFLT=0，但 pos_loop.c:265 的饱和回灌分支无条件执行，
      s_i_term 被拉到 -POS_I_MAX(-2.0 rad/s)，而 Ki=0 无法消除它。
      位置偏差 = 2.0/Kp = 2.0/100 = 0.020 rad = 4125"。
"""
import re
import sys
import time

import serial

sys.path.insert(0, ".")
from cog_ff_test import Rig
from pos_test import ask, collect


def soft_reset(port="COM27"):
    """! 软复位把多圈累积归零。位置跑到几千 rad 后 float ulp 会让 tgt=nan。"""
    s = serial.Serial(port, 921600, timeout=0.05)
    s.write(b"!")
    s.flush()
    time.sleep(2.5)
    try:
        s.close()
    except Exception:
        pass
    time.sleep(0.3)


def pos(r):
    pts, _ = collect(r, 0.35)
    return sum(pts[-4:]) / 4 if pts else float("nan")


def read_p(r):
    return ask(r, b"P\r", 1.0)


soft_reset()
r = Rig("COM27")
print("S  :", ask(r, b"S", 0.7)[:90])
print("D  :", ask(r, b"D", 0.4))
line = read_p(r)
print("hold:", line)
m = re.search(r"tgt=(-?\d+\.\d+)", line)
a = float(m.group(1))
print("A_ctrl=%.4f" % a)
p0 = pos(r)
print("enc0=%.4f  iv 初值见上" % p0)

# 大行程触发饱和：目标 +10 rad（VMAX=25 rad/s 下约 1.4 s 饱和段）
tgt = a + 10.0
c_t = ("%.4f" % tgt).encode()
print("\n[1] P%.4f（会经历约 1 s 的速度饱和）" % tgt)
r.s.write(b"P" + c_t + b"\r")
r.s.flush()


def poke(r, c_t):
    """重发同一目标：SetTarget 对同目标直接 return，不改状态，
    但回显会带 iv，且顺带续 deadman。绝不能用 P\\r（空回车 = 保持）。"""
    r.s.write(b"P" + c_t + b"\r")
    r.s.flush()
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < 0.25:
        d = r.s.read(512)
        if d:
            buf += d
            if b"\r\n" in buf and b"[CMD]" in buf:
                break
    i = buf.rfind(b"[CMD]")
    return bytes(buf[i:]).split(b"\r\n")[0].decode("ascii", "replace") if i >= 0 else ""


for i in range(9):
    time.sleep(0.28)
    ln = poke(r, c_t)
    iv = re.search(r"iv=(-?[\d.]+)", ln)
    st = re.search(r"st=(\d)", ln)
    ee = re.search(r"e=(-?[\d.]+)", ln)
    p = pos(r)
    print("  t=%4.1f  iv=%-9s st=%s e=%-9s enc=%+9.4f  err=%+7.4f"
          % ((i + 1) * 0.28, iv.group(1) if iv else "?", st.group(1) if st else "?",
             ee.group(1) if ee else "?", p, p - p0 - 10.0))

print("\n[2] 回原点，看 iv 是否被带回")
c_a = ("%.4f" % a).encode()
r.s.write(b"P" + c_a + b"\r")
r.s.flush()
for i in range(5):
    time.sleep(0.32)
    ln = poke(r, c_a)
    iv = re.search(r"iv=(-?[\d.]+)", ln)
    p = pos(r)
    print("  iv=%-9s enc=%+9.4f err=%+7.4f" % (iv.group(1) if iv else "?", p, p - p0))

r.s.write(b"D")
r.s.flush()
time.sleep(0.2)
r.close()
