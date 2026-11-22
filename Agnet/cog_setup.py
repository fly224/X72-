# -*- coding: utf-8 -*-
"""cog_setup.py —— 烧固件后的齿槽前馈一键重建 + 开关持久化验证

流程：C（电流零点）→ Z（电角度）→ `}`（齿槽标定，自动装表）
      → `;`（开前馈）→ A（落盘，含 en）→ `!`（软复位）→ 复查

判据：
  1. `}` 结束必须出现 "->cog ok"（装表成功）
  2. A 之后 `:` 为 en=1 v=1
  3. 复位后 `:` 仍为 en=1 v=1（en 真的进了 Flash）
  4. 复位后 S 为 cal=1 z=1（其余参数也在）

用法: python cog_setup.py COM27
"""
import sys
import time

from cog_ff_test import Rig


def drain(rig, token, tmo):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < tmo:
        d = rig.s.read(1024)
        if d:
            buf += d
        if token.encode() in buf:
            break
    return bytes(buf).decode("ascii", "replace")


def cmd_lines(txt, keep=135):
    out = []
    for ln in txt.splitlines():
        j = ln.find("[CMD]")
        if j >= 0:
            out.append(ln[j:j + keep].strip())
    return out


def show(txt):
    for ln in cmd_lines(txt):
        print("   ", ln)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    rig = Rig(port)
    ok = True
    try:
        print("[0] 初始状态")
        show(rig.cmd("S", 0.5))
        show(rig.cmd(":", 0.4))

        print("[1] C 电流零点")
        rig.s.reset_input_buffer()
        rig.s.write(b"C")
        rig.s.flush()
        t = drain(rig, "\x00", 4.0)     # 无匹配 token，读满 4 s
        show(t)

        print("[2] Z 电角度辨识（约 9~25 s）")
        rig.s.reset_input_buffer()
        rig.s.write(b"Z")
        rig.s.flush()
        t = drain(rig, "IDENT ok", 30.0)
        show(t)          # 被拒时是 "[CMD] Z refused: ..."，也要看到
        if "IDENT ok" not in t:
            print("   !! Z 未成功")
            ok = False

        print("[3] `}` 齿槽标定（10 rpm 正反各一圈，约 15 s）")
        rig.s.reset_input_buffer()
        rig.s.write(b"}")
        rig.s.flush()
        t = drain(rig, "MI end", 50.0)
        show(t)
        if "->cog ok" not in t:
            print("   !! 装表失败")
            ok = False

        st = rig.cmd(":", 0.5)
        print("[4] 标定后状态 ", st)
        if " v=1" not in st:
            ok = False

        print("[5] `;` 开前馈")
        print("   ", rig.cmd(";", 0.5))

        print("[6] A 落盘")
        print("   ", rig.cmd("A", 2.0))
        st = rig.cmd(":", 0.5)
        print("   ", st)
        if " en=1" not in st:
            ok = False

        print("[7] `!` 软复位")
        rig.s.reset_input_buffer()
        rig.s.write(b"!")
        rig.s.flush()
        time.sleep(4.0)         # 启动横幅 + trace 有几十行，3 s 会抢跑
        rig.s.reset_input_buffer()

        print("[8] 复位后复查")
        s_st = ""
        for _ in range(3):
            s_st = rig.cmd("S", 0.6)
            if "cal=" in s_st:
                break
        print("   ", s_st)
        st = rig.cmd(":", 0.6)
        print("   ", st)
        if " en=1" not in st:
            print("   !! 复位后 en 没保持")
            ok = False
        if " v=1" not in st:
            print("   !! 复位后表失效")
            ok = False
        if "cal=1" not in s_st or "z=1" not in s_st:
            print("   !! 复位后 cal/z 丢失")
            ok = False

        print("\n=== %s ===" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1
    finally:
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
