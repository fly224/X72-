# -*- coding: utf-8 -*-
"""极低速能力探测 + 死区补偿 A/B 对比。

判据：编码器累计位置差分（唯一在极低速可信的转速来源），
      瞬时 w_elec 在 1 rpm 下量化噪声占主导，不采用。

用法：python low_speed_limit.py COM27
"""
import statistics
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, strip_frames, MSG_PERMISSION  # noqa: E402

ENC = 2097152.0


def cmd(rig, c, wait=0.55):
    rig.rx.pause_evt.set()
    time.sleep(0.03)
    try:
        rig.s.reset_input_buffer()
        rig.s.write(c.encode("latin-1") if isinstance(c, str) else c)
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


def measure(rig, rpm_tgt, dur=25.0):
    """给定 rpm_tgt 跑 dur 秒，返回位置差分统计。"""
    e0 = rig.rx.enc
    if e0 is None:
        return None
    cmd(rig, "=%g\r" % rpm_tgt, 0.35)
    rows, iqs = [], []
    t0 = time.time()
    aborted_iq = False
    while time.time() - t0 < dur:
        rig.s.write(("=%g\r" % rpm_tgt).encode())
        rig.s.flush()
        time.sleep(0.40)
        e = rig.rx.enc
        s = rig.rx.get_snap()[0]
        if e is not None:
            rows.append((time.time() - t0, e))
        if s is not None and s.get("iq_ref") is not None:
            iqs.append(s["iq_ref"])
            if abs(iqs[-1]) > 2.8:
                aborted_iq = True
                break
    e1 = rig.rx.enc
    if len(rows) < 3 or e1 is None:
        return None
    dur_eff = rows[-1][0] - rows[0][0]
    d = e1 - e0
    mean = (d / ENC) / dur_eff * 60.0

    bins = {}
    for t, e in rows:
        bins.setdefault(int(t // 5), []).append((t, e))
    seg = []
    for k in sorted(bins):
        v = bins[k]
        if len(v) < 2:
            continue
        dt = v[-1][0] - v[0][0]
        dc = v[-1][1] - v[0][1]
        if dt > 0.5:
            seg.append((dc / ENC) / dt * 60.0)
    mu = statistics.mean(seg) if seg else 0.0
    cv = (statistics.pstdev(seg) / abs(mu) * 100.0) if (len(seg) >= 2 and abs(mu) > 1e-9) else 0.0
    return dict(mean=mean, d=d, dur=dur_eff, seg=seg, cv=cv,
                iq=(statistics.mean(iqs) if iqs else None),
                iqmax=(max(iqs, key=abs) if iqs else None),
                stuck=(abs(d) < 800), iq_abort=aborted_iq)


def report(tag, m, rpm_tgt):
    if m is None:
        print("  %-22s 无数据（未收到编码器帧）" % tag)
        return
    err = (m["mean"] - rpm_tgt) / rpm_tgt * 100.0 if rpm_tgt else 0.0
    print("  %-22s 平均 %+8.4f rpm (误差 %+7.1f%%)  CV %6.1f%%  iq %.2f/%.2f A  %s"
          % (tag, m["mean"], err, m["cv"], m["iq"] or 0, m["iqmax"] or 0,
             "卡住!" if m["stuck"] else ("iq限幅中止" if m["iq_abort"] else "转动")))
    if m["seg"]:
        print("  %-22s 分段: %s" % ("", " ".join("%+.3f" % v for v in m["seg"])))


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    rig = Rig(port)
    try:
        time.sleep(1.3)
        rig.send(MSG_PERMISSION, bytes([0, 0]))
        time.sleep(0.25)
        cmd(rig, "D", 0.6)
        cmd(rig, "X", 0.7)
        cmd(rig, "X", 0.5)

        # 查当前死区补偿量与开关状态（\ 空输入 = 查询；B = 切换，先读后恢复）
        print("=== 死区补偿现状 ===")
        for l in cmd(rig, "\\\r", 0.6).splitlines():
            if "[CMD]" in l:
                print("  ", l.strip()[:120])
        t = cmd(rig, "B", 0.6)
        for l in t.splitlines():
            if "[CMD]" in l:
                print("  ", l.strip()[:120])
        was_off = "OFF" in t
        if was_off:
            cmd(rig, "B", 0.6)          # 恢复默认 ON
            print("   （已切回 ON）")

        print("\n=== A) 极低速极限（死区补偿 du=0.59% 全量生效）===")
        print("  目标      ", "  ".join(["0.5 rpm", "0.2 rpm", "0.1 rpm"]))
        for tgt in (0.5, 0.2, 0.1):
            m = measure(rig, tgt, 25.0)
            report("%.1f rpm" % tgt, m, tgt)
            cmd(rig, "=0\r", 0.3)
            time.sleep(0.3)

        print("\n=== B) 死区补偿 A/B（1 rpm，每档 25 s）===")
        # 基线：默认 ON + du=59
        cmd(rig, "\\59\r", 0.4)
        m = measure(rig, 1.0, 25.0)
        report("ON du=0.59%", m, 1.0)
        base_cv = m["cv"] if m else None
        cmd(rig, "=0\r", 0.3); time.sleep(0.3)

        # 关闭补偿
        t = cmd(rig, "B", 0.6)
        off_now = "OFF" in t
        m = measure(rig, 1.0, 25.0)
        report("OFF（补偿关闭）", m, 1.0)
        cmd(rig, "=0\r", 0.3); time.sleep(0.3)
        if not off_now:
            cmd(rig, "B", 0.6)      # 若刚才实际是 ON->OFF 已生效，反之补一次
        print("   （补偿已恢复 ON）")

        # du 扫描
        for du in (30, 44, 75, 100):
            cmd(rig, "\\%d\r" % du, 0.4)
            m = measure(rig, 1.0, 25.0)
            report("ON du=%.2f%%" % (du / 100.0), m, 1.0)
            cmd(rig, "=0\r", 0.3); time.sleep(0.3)

        # 恢复默认
        cmd(rig, "\\59\r", 0.4)
        cmd(rig, "=0\r", 0.3)
        cmd(rig, "D", 0.8)
        t = cmd(rig, "S", 0.7)
        for l in t.splitlines():
            if "st en=" in l:
                print("\n收尾: %s" % l.strip()[:140])
                break
        return 0
    finally:
        try:
            rig.s.write(b"=0\r")
            time.sleep(0.15)
            rig.s.write(b"D")
        except Exception:
            pass
        rig.close()


if __name__ == "__main__":
    sys.exit(main())
