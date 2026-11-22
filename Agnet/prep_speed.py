# -*- coding: utf-8 -*-
"""一次性把「清故障 → 电角度对齐 → 速度模式 1000rpm」跑完。

为什么要挤在一个脚本里：当前链路抖动（S 查询约 2/3 成功率），
每多一次往返就多一次失败机会。这里只在必要时发命令，
辨识期间严格被动收（辨识输出会把 s_out 顶掉，中途插 S 会丢结果）。

用法：python prep_speed.py COM27 [--rpm 1000] [--dur 30]
"""
import argparse
import re
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

ENC = 2097152.0


def cmd(rig, c, wait=0.7):
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


def field(s, key):
    m = re.search(key + r"=(0x[0-9a-fA-F]+|\d+)", s or "")
    return m.group(1) if m else None


def status(rig, tries=6):
    for _ in range(tries):
        s = cmd(rig, "S", 0.9)
        if s and "[CMD] st" in s:
            return s
        time.sleep(0.3)
    return ""


def passive(rig, stops, timeout):
    """辨识期间只能被动收：任何一条命令都会把正在发的结果行顶掉。"""
    rig.rx.pause_evt.set()
    time.sleep(0.03)
    try:
        rig.s.reset_input_buffer()
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--rpm", type=float, default=1000.0)
    ap.add_argument("--dur", type=float, default=30.0)
    args = ap.parse_args()

    rig = Rig(args.port)
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.4)

        s = status(rig)
        print("[0] 初始: en=%s cal=%s z=%s f=%s"
              % (field(s, "en"), field(s, "cal"), field(s, "z"), field(s, "f")), flush=True)

        # ---- 1) 清故障 ----
        if field(s, "f") not in ("0x00", None):
            cmd(rig, "X", 0.8)
            time.sleep(0.4)
            s = status(rig, 3)
            print("[1] 清故障后 f=%s" % field(s, "f"), flush=True)

        # ---- 2) 电角度对齐（若已 z=1 则跳过）----
        if field(s, "z") != "1":
            print("[2] Z 电角度对齐（转子转一圈，约 9 s）...", flush=True)
            rig.rx.pause_evt.set()
            time.sleep(0.03)
            rig.s.reset_input_buffer()
            rig.s.write(b"Z")
            rig.s.flush()
            rig.rx.buf = bytearray()
            rig.rx.pause_evt.clear()

            txt, el = passive(rig, ["[CMD] IDENT ok", "[CMD] IDENT FAIL"], 45.0)
            tail = [l.strip() for l in txt.splitlines() if "[CMD] IDENT" in l or "[CMD] Z" in l]
            print("    %.1f s  %s" % (el, tail[-1] if tail else "(无完成行)"), flush=True)

            s = status(rig, 4)
            print("    z=%s f=%s" % (field(s, "z"), field(s, "f")), flush=True)
            if field(s, "z") != "1":
                print("[!] Z 未成功，速度模式联锁不满足（do_speed 要求 z=1），终止")
                return 1
        else:
            print("[2] 电角度已对齐，跳过 Z", flush=True)

        # ---- 3) 使能 + 给定 ----
        print("[3] E 使能", flush=True)
        r = cmd(rig, "E", 0.9)
        if "ON" not in r:
            print("    " + (r.strip() or "(无回显)") + " -> 使能失败")
            return 1
        print("    " + [l for l in r.splitlines() if "[CMD]" in l][-1].strip(), flush=True)

        print("[4] 给定 %.0f rpm，采样 %.0f s" % (args.rpm, args.dur), flush=True)
        rows = []
        t0 = time.time()
        while time.time() - t0 < args.dur:
            with rig.wlock:
                rig.s.write(("=%g\r" % args.rpm).encode())
                rig.s.flush()
            snap, ts, iq_now, enc, enc_t = rig.rx.get_snap()
            if enc is not None and ts:
                rows.append((ts, float(enc), snap.get("temp_mot"),
                             snap.get("iq_ref"), snap.get("vbus")))
            time.sleep(0.3)

        st = status(rig, 3)
        print("[5] 停机", flush=True)
        cmd(rig, "D", 0.7)
    finally:
        try:
            cmd(rig, "D", 0.7)
        except Exception:
            pass
        rig.close()

    if len(rows) < 10:
        print("[!] 采样点不足（%d），无法统计" % len(rows))
        return 1

    cut = max(3, len(rows) // 10)
    use = rows[cut:]
    t_ref = use[0][0]
    n = len(use)
    mt = sum((r[0] - t_ref) for r in use) / n
    me = sum(r[1] for r in use) / n
    num = sum(((r[0] - t_ref) - mt) * (r[1] - me) for r in use)
    den = sum(((r[0] - t_ref) - mt) ** 2 for r in use)
    rpm = (num / den) / ENC * 60.0 if den > 0 else float("nan")

    bins, cur, bt = [], [], use[0][0]
    for r in use:
        if r[0] - bt >= 2.0:
            if len(cur) >= 3:
                m2t = sum(x[0] - cur[0][0] for x in cur) / len(cur)
                m2e = sum(x[1] for x in cur) / len(cur)
                nu = sum(((x[0] - cur[0][0]) - m2t) * (x[1] - m2e) for x in cur)
                de = sum(((x[0] - cur[0][0]) - m2t) ** 2 for x in cur)
                if de > 0:
                    bins.append((nu / de) / ENC * 60.0)
            cur, bt = [], r[0]
        cur.append(r)
    cv = (statistics.pstdev(bins) / statistics.mean(bins) * 100.0) if len(bins) > 1 else float("nan")

    iqs = [r[3] for r in use if r[3] is not None]
    temps = [r[2] for r in use if r[2] is not None]

    print("\n=== 结果 ===")
    print("给定        : %.0f rpm" % args.rpm)
    print("实测(斜率)  : %.2f rpm   误差 %+.3f %%" % (rpm, (rpm - args.rpm) / args.rpm * 100.0))
    print("分箱 CV     : %.2f %%  (%d 个 2 s 箱)" % (cv, len(bins)))
    print("iq 均值     : %.3f A" % (statistics.mean(iqs) if iqs else float("nan")))
    if temps:
        print("温度        : %.1f → %.1f °C" % (temps[0], temps[-1]))
    print("采样点      : %d（丢前 %d）" % (len(use), cut))
    print("S: " + " ".join(x for x in re.findall(r"(?:en|moe|cal|z|f|spd|cur)=[0-9a-fx]+", st) if x))
    return 0


if __name__ == "__main__":
    sys.exit(main())
