# -*- coding: utf-8 -*-
"""电角度校准（C 电流零点 → Z 对齐），只在必要时发命令。

链路抖动时（S 查询约 2/3 成功）多一次往返就多一次失败机会，所以：
  1) C 发出去后**被动等**完成（标定期间输出会顶掉别的命令）
  2) Z 同理，只被动收 "[CMD] IDENT ok/FAIL"
  3) 每一步之后用 S 复核 cal / z / f，不靠回显猜

用法：python do_align.py COM27
"""
import re
import sys
import time

sys.path.insert(0, r"G:\FOC\X72_Agnet")
sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402


def cmd(rig, c, wait=0.8):
    rig.rx.pause_evt.set()
    time.sleep(0.03)
    try:
        rig.s.reset_input_buffer()
        rig.s.write(c.encode() if isinstance(c, str) else c)
        rig.s.flush()
        raw = b""
        t0 = time.time()
        while time.time() - t0 < wait:
            d = rig.s.read(8192)
            if d:
                raw += d
        return strip_frames(raw).decode("ascii", "replace")
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def passive(rig, stops, timeout):
    rig.rx.pause_evt.set()
    time.sleep(0.03)
    try:
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < timeout:
            d = rig.s.read(4096)
            if d:
                buf += d
                txt = bytes(buf).decode("ascii", "replace")
                for w in stops:
                    if w in txt:
                        return txt, time.time() - t0
            else:
                time.sleep(0.01)
        return bytes(buf).decode("ascii", "replace"), time.time() - t0
    finally:
        rig.rx.buf = bytearray()
        rig.rx.pause_evt.clear()


def fld(s, k):
    m = re.search(k + r"=(0x[0-9a-fA-F]+|\d+)", s or "")
    return m.group(1) if m else "?"


def st(rig, tries=5):
    for _ in range(tries):
        s = cmd(rig, "S", 0.9)
        if s and "[CMD] st" in s:
            return s
        time.sleep(0.3)
    return ""


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    rig = Rig(port)
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.4)
        cmd(rig, "D", 0.7)                       # 必须在封锁态

        s = st(rig)
        print("初始: cal=%s z=%s f=%s" % (fld(s, "cal"), fld(s, "z"), fld(s, "f")))

        # ---- 1) 电流零点 ----
        if fld(s, "cal") != "1":
            print("C 电流零点标定 ...")
            rig.rx.pause_evt.set(); time.sleep(0.03)
            rig.s.reset_input_buffer(); rig.s.write(b"C"); rig.s.flush()
            rig.rx.buf = bytearray(); rig.rx.pause_evt.clear()
            txt, el = passive(rig, ["[CMD] CAL done", "[CMD] CAL ok"], 6.0)
            time.sleep(0.8)
            s = st(rig)
            print("  %.1f s -> cal=%s bias=%s" % (el, fld(s, "cal"),
                  re.search(r"bias=([\d.]+/[\d.]+)", s or "").group(1)
                  if re.search(r"bias=", s or "") else "?"))
            if fld(s, "cal") != "1":
                print("  [!] C 未成功，终止"); return 1

        # ---- 2) 电角度对齐 ----
        if fld(s, "z") != "1":
            print("Z 电角度对齐（1.5 A，转子转一圈，约 10 s）...")
            rig.rx.pause_evt.set(); time.sleep(0.03)
            rig.s.reset_input_buffer(); rig.s.write(b"Z"); rig.s.flush()
            rig.rx.buf = bytearray(); rig.rx.pause_evt.clear()
            txt, el = passive(rig, ["[CMD] IDENT ok", "[CMD] IDENT FAIL"], 45.0)
            done = [l.strip() for l in txt.splitlines()
                    if "[CMD] IDENT ok" in l or "[CMD] IDENT FAIL" in l]
            print("  %.1f s  %s" % (el, done[-1] if done else "(无完成行)"))
            time.sleep(0.5)
            s = st(rig)
            print("  -> cal=%s z=%s f=%s" % (fld(s, "cal"), fld(s, "z"), fld(s, "f")))
            if fld(s, "z") != "1":
                print("  [!] Z 未成功"); return 1
        else:
            print("电角度已对齐（z=1），跳过")

        print("\n=== 校准完成 ===")
        print("S: " + " ".join(x for x in re.findall(r"(?:en|moe|cal|z|f)=[0-9a-fx]+", s) if x))
        k = cmd(rig, "K", 0.8)
        m = re.search(r"psi=([\d.]+)m kt=([\d.]+)m", k or "")
        if m:
            print("K: psi=%s mWb  kt=%s mN·m/A" % (m.group(1), m.group(2)))
    finally:
        try:
            cmd(rig, "D", 0.6)
        except Exception:
            pass
        rig.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
