# -*- coding: utf-8 -*-
"""torque_test.py —— 力矩模式给定/线性度/斜坡验证

流程：逐点 `>iq` 进入力矩模式 → 稳定 → 采 → D。
每点记录 iq 目标 / 斜坡后 / 实测、转速、估算力矩。

注意：力矩模式空载会一路加速到 300 rpm 触发超速保护，所以
  · 每点驻留要短（默认 0.7 s）
  · 转速超过 --maxrpm 立即 D 并标记
  · 小给定（<0.15 A）时静摩擦让它不转，正好用来量死区/线性度

用法:
    python torque_test.py COM27                       # 线性度扫描
    python torque_test.py COM27 --taus 0,5,20,50      # 对比斜坡时间常数
    python torque_test.py COM27 --step 0,0.5          # 阶跃响应
"""
import argparse
import re
import struct
import time

from cog_ff_test import LEN_IN, MSG_IN, Rig, x25

LEN_OUT = 32
MSG_OUT = 30027
CRC_OUT = 58

RE_CUR = re.compile(r"iq_tar=(-?[\d.]+) ramp=(-?[\d.]+) meas=(-?[\d.]+)A T=(-?[\d.]+)Nm")


def collect_dual(rig, secs):
    """同时抓 30025（给定/速度/母线）与 30027（id/iq 实测）

    为什么要两条：30025 的 +4 是 **iq 给定**（s_iq_ref），不是实测；
    实测在 30027 的 +16。之前只看 30025 会把"给定"当成"实测"，
    于是一边读着 0.3 一边纳闷为什么电机不出力。
    """
    fin, fout = [], []
    t0 = time.time()
    nxt = t0 + 0.8
    while time.time() - t0 < secs:
        d = rig.s.read(4096)
        if d:
            rig.raw += d
        if time.time() > nxt:
            rig.s.write(b"S")
            rig.s.flush()
            nxt += 0.8
        i = 0
        n = len(rig.raw)
        while i + 12 <= n:
            if rig.raw[i] != 0xFD:
                i += 1
                continue
            ln = rig.raw[i + 1]
            mid = rig.raw[i + 7] | (rig.raw[i + 8] << 8) | (rig.raw[i + 9] << 16)
            if mid not in (MSG_IN, MSG_OUT):
                i += 1
                continue
            if (mid == MSG_IN and ln != LEN_IN) or (mid == MSG_OUT and ln != LEN_OUT):
                i += 1
                continue
            b = i + 10
            if b + ln + 2 > n:
                break
            crc = rig.raw[b + ln] | (rig.raw[b + ln + 1] << 8)
            extra = 191 if mid == MSG_IN else CRC_OUT
            if x25(rig.raw[i + 1:b + ln], extra) != crc:
                i += 1
                continue
            f = lambda o: struct.unpack_from("<f", rig.raw, b + o)[0]
            if mid == MSG_IN:
                fin.append((time.time(), f(44) / 14.0 / (2 * 3.141592653589793 / 60.0),
                            f(4), f(24)))
            else:
                fout.append((time.time(), f(12), f(16)))
            i = b + ln + 2
        del rig.raw[:i]
    return fin, fout


def send_num(rig, lead, val, dec=3):
    """数值命令：引导符单独发，数字逐字符（历史丢字节坑）"""
    rig.s.reset_input_buffer()
    rig.s.write(b"\r")
    rig.s.flush()
    time.sleep(0.05)
    rig.s.write(lead.encode())
    rig.s.flush()
    time.sleep(0.06)
    for ch in ("%.*f" % (dec, val)):
        rig.s.write(ch.encode())
        rig.s.flush()
        time.sleep(0.012)
    rig.s.write(b"\r")
    rig.s.flush()
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < 0.6:
        d = rig.s.read(512)
        if d:
            buf += d
    txt = "".join(ch if 32 <= ord(ch) < 127 else "\n"
                  for ch in buf.decode("ascii", "replace"))
    lines = [ln.strip() for ln in txt.split("\n") if ln.strip().startswith("[CMD]")]
    return lines[-1] if lines else "(no echo)"


def query_cur(rig):
    rig.s.reset_input_buffer()
    rig.s.write(b">")
    rig.s.flush()
    time.sleep(0.06)
    rig.s.write(b"\r")
    rig.s.flush()
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < 0.6:
        d = rig.s.read(512)
        if d:
            buf += d
    txt = "".join(ch if 32 <= ord(ch) < 127 else "\n"
                  for ch in buf.decode("ascii", "replace"))
    for ln in txt.split("\n"):
        ln = ln.strip()
        if ln.startswith("[CMD] cur"):
            m = RE_CUR.search(ln)
            if m:
                return dict(tar=float(m.group(1)), ramp=float(m.group(2)),
                            meas=float(m.group(3)), torque=float(m.group(4)), raw=ln)
            return dict(tar=None, ramp=None, meas=None, torque=None, raw=ln)
    return None


def run_point(rig, iq, settle, secs, maxrpm):
    echo = send_num(rig, ">", iq)
    if "refused" in echo or "failed" in echo:
        return None, echo
    rig.keep(settle)

    fin, fout = collect_dual(rig, secs)
    st = query_cur(rig)
    rig.cmd("D", 0.3)

    if not fin or not fout:
        return None, "帧不足 in=%d out=%d" % (len(fin), len(fout))

    mean = lambda a: sum(a) / len(a)
    sd = lambda a: (sum((x - mean(a)) ** 2 for x in a) / len(a)) ** 0.5

    tin = fin[-max(5, int(len(fin) * 0.6)):]
    tout = fout[-max(5, int(len(fout) * 0.6)):]
    v = [x[1] for x in tin]
    tar = [x[2] for x in tin]
    iqm = [x[2] for x in tout]
    idm = [x[1] for x in tout]
    return dict(rpm=mean(v), rpm_max=max(v), rpm_min=min(v),
                iq=mean(iqm), iq_sd=sd(iqm), id=mean(idm),
                iq_tar=mean(tar),
                meas_pt=st["meas"] if st else None,
                torque=st["torque"] if st else None,
                over=abs(max(v, key=abs)) > maxrpm), echo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--iqs", default="0.05,0.1,0.15,0.2,0.3,0.5,0.8,1.2")
    ap.add_argument("--taus", default="")          # 非空则做斜坡对比
    ap.add_argument("--step", default="")          # "from,to" 阶跃响应
    ap.add_argument("--settle", type=float, default=0.7)
    ap.add_argument("--secs", type=float, default=0.6)
    ap.add_argument("--maxrpm", type=float, default=200.0)
    a = ap.parse_args()

    rig = Rig(a.port)
    try:
        print("S:", rig.cmd("S", 0.6))
        print("cur:", (query_cur(rig) or {}).get("raw"))

        if a.taus:
            taus = [float(x) for x in a.taus.split(",")]
            print("\n=== 斜坡时间常数对比（iq 阶跃 0 -> 0.5 A）===")
            print("   tau(ms)   0.15s后 ramp   iq实测   rpm")
            for tau in taus:
                print("  ", send_num(rig, '"', tau, dec=1))
                send_num(rig, ">", 0.0)
                time.sleep(0.2)
                send_num(rig, ">", 0.5)
                time.sleep(0.15)
                q = query_cur(rig)
                fin, fout = collect_dual(rig, 0.3)
                rpm = sum(x[1] for x in fin) / len(fin) if fin else 0.0
                iqm = sum(x[2] for x in fout) / len(fout) if fout else 0.0
                rig.cmd("D", 0.3)
                print("   %7.1f    %s       %6.3f  %7.2f"
                      % (tau, ("%6.3f" % q["ramp"]) if q and q["ramp"] is not None else "  n/a ",
                         iqm, rpm))

        if a.step:
            f0, f1 = [float(x) for x in a.step.split(",")]
            print("\n=== 阶跃 %.3f -> %.3f A（tau=0，直通）===" % (f0, f1))
            print("  ", send_num(rig, '"', 0.0, dec=1))
            print("  ", send_num(rig, ">", f0))
            time.sleep(0.4)
            print("  阶跃前:", (query_cur(rig) or {}).get("raw"))
            t0 = time.time()
            send_num(rig, ">", f1)
            fin, fout = collect_dual(rig, 0.5)
            rig.cmd("D", 0.3)
            print("  阶跃后 0.5 s 内 iq 实测序列（30027，每 3 帧取一点）:")
            print("   ", " ".join("%.2f" % x[2] for x in fout[::3]))

        if not a.taus and not a.step:
            iqs = [float(x) for x in a.iqs.split(",")]
            print("\n=== 力矩给定线性度（tau 默认 20 ms）===")
            print("   iq_tar  iq_meas   σ      偏差     rpm    T(N·m)  备注")
            for iq in iqs:
                r, echo = run_point(rig, iq, a.settle, a.secs, a.maxrpm)
                if r is None:
                    print("  %6.2f  失败: %s" % (iq, echo))
                    continue
                note = "超速" if r["over"] else ""
                print("  %6.2f   %6.3f  %5.3f  %+7.3f  %7.2f  %7.4f  %s"
                      % (iq, r["iq"], r["iq_sd"], r["iq"] - iq, r["rpm"],
                         r["torque"] if r["torque"] is not None else 0.0, note))
        return 0
    finally:
        rig.cmd("D", 0.3)
        rig.close()


if __name__ == "__main__":
    raise SystemExit(main())
