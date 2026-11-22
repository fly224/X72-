# -*- coding: utf-8 -*-
"""MIT 模式测试：使能 / 保持 / 阶跃 / 力矩限幅 / 校验拒绝 / 退出。

帧格式（与 MIT Mini Cheetah 一致，共 10 字节）：
    0x02 | 8 字节数据 | 1 字节校验和(数据字节算术和)

数据位序：
    [0..1] 位置 16bit [-12.5,12.5]rad
    [2..3] 速度 12bit [-65,65]rad/s
    [3..4] kp   12bit [0,5000]N·m/rad
    [5..6] kd   12bit [0,100]N·m·s/rad
    [6..7] 力矩 12bit [-18,18]N·m

用法: python mit_test.py COM27
"""
import argparse
import re
import time

from cog_ff_test import Rig

RE_MIT = re.compile(r"mit T=(-?[\d.]+) p=(-?[\d.]+)")


def cmd_line(txt):
    """从混杂二进制遥测的原始流里只取 [CMD] 行"""
    s = "".join(c if 32 <= ord(c) < 127 else "\n" for c in txt)
    ls = [l.strip() for l in s.split("\n") if l.strip().startswith("[CMD]")]
    return ls[-1] if ls else "(no echo)"


def f2u(x, lo, hi, bits):
    x = max(lo, min(hi, x))
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo) + 0.5)


def pack(p, v, kp, kd, t, bad=False):
    p_i = f2u(p, -12.5, 12.5, 16)
    v_i = f2u(v, -65.0, 65.0, 12)
    kp_i = f2u(kp, 0.0, 5000.0, 12)
    kd_i = f2u(kd, 0.0, 100.0, 12)
    t_i = f2u(t, -18.0, 18.0, 12)
    b = [p_i >> 8, p_i & 0xFF,
         v_i >> 4, ((v_i & 0xF) << 4) | (kp_i >> 8), kp_i & 0xFF,
         kd_i >> 4, ((kd_i & 0xF) << 4) | (t_i >> 8), t_i & 0xFF]
    chk = sum(b) & 0xFF
    if bad:
        chk ^= 0x5A
    return bytes([0x02] + b + [chk])


def send_frames(rig, secs, p, v, kp, kd, t, rate=200.0, bad=False):
    """按 rate Hz 连续发帧，返回期间采样到的 (t, T, p) 列表"""
    dt = 1.0 / rate
    t0 = time.time()
    out = []
    nxt_poll = t0
    while time.time() - t0 < secs:
        rig.s.write(pack(p, v, kp, kd, t, bad))
        rig.s.flush()
        now = time.time()
        if now >= nxt_poll:
            nxt_poll = now + 0.05        # 20 Hz 采样 S（不能太密，回显单槽）
            st = rig.cmd("S", 0.04)
            m = RE_MIT.search(st)
            if m:
                out.append((now - t0, float(m.group(1)), float(m.group(2))))
        time.sleep(dt)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--kp", type=float, default=8.0)
    ap.add_argument("--kd", type=float, default=0.05, help="实测稳定上限约 0.2，见 MIT模式_实现与实测.md 第 8 节")
    ap.add_argument("--step", type=float, default=0.2, help="位置阶跃 rad")
    a = ap.parse_args()

    rig = Rig(a.port)
    try:
        time.sleep(2.0)
        print("D:", rig.cmd("D", 0.5))
        print("X:", rig.cmd("X", 0.5))

        # 1) 使能：kp=0 不起力，只用来读初始位置
        print("\n[1] 使能（kp=0, kd=0.1）—— 不应有运动")
        r = send_frames(rig, 0.6, 0.0, 0.0, 0.0, 0.1, 0.0)
        if not r:
            print("    !! 没有拿到 mit 状态，帧可能没被接收")
            return
        p0 = r[-1][2]
        print("    使能成功，p0 = %.4f rad, T = %.3f N·m" % (p0, r[-1][1]))
        cl = cmd_line(rig.cmd("S", 0.6))
        print("    S:", cl[cl.find("mit"):] if "mit" in cl else cl)

        # 2) 保持：给 kp 但目标就是当前位置
        print("\n[2] 位置保持 kp=%.0f kd=%.2f（1.0 s）" % (a.kp, a.kd))
        r = send_frames(rig, 1.0, p0, 0.0, a.kp, a.kd, 0.0)
        ps = [x[2] for x in r]
        Ts = [abs(x[1]) for x in r]
        print("    p 范围 %.4f..%.4f（漂移 %.4f rad）  |T|max=%.3f N·m"
              % (min(ps), max(ps), max(ps) - min(ps), max(Ts)))

        # 3) 阶跃
        tgt = p0 + a.step
        print("\n[3] 位置阶跃 %.3f → %.3f rad（1.2 s）" % (p0, tgt))
        r = send_frames(rig, 1.2, tgt, 0.0, a.kp, a.kd, 0.0)
        ps = [x[2] for x in r]
        Ts = [abs(x[1]) for x in r]
        final = ps[-1]
        print("    终点 %.4f  目标 %.4f  误差 %+.4f rad" % (final, tgt, tgt - final))
        print("    |T| 峰值 %.3f N·m（iq 限幅 5 A × Kt 0.091 = 0.455 N·m）" % max(Ts))
        print("    轨迹:", " ".join("%.3f" % x for x in ps[::max(1, len(ps) // 10)]))

        # 4) 校验坏帧必须被拒
        print("\n[4] 故意发校验错的帧（0.4 s）—— 位置不应变化")
        r = send_frames(rig, 0.4, p0, 0.0, 0.0, 0.0, 0.0, bad=True)
        if r:
            print("    p 变化 %+.4f rad" % (r[-1][2] - final))
        cl = cmd_line(rig.cmd("S", 0.6))
        print("    S:", cl[cl.find("mit"):] if "mit" in cl else cl)

        # 5) 退出
        print("\n[5] 退出（D）")
        print("   ", rig.cmd("D", 0.6))
        print("    S:", cmd_line(rig.cmd("S", 0.6))[-40:])
    finally:
        rig.cmd("D", 0.3)
        rig.close()


if __name__ == "__main__":
    main()
