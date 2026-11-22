# -*- coding: utf-8 -*-
"""
enc_sweep.py —— 手动转一圈，全行程采集"电角度 − 14×机械角"偏差

用途：判定编码器误差是**恒定偏置**（可用固定补偿修复）还是**随位置变化**
（补偿表损坏，只能重标定/换芯片）。

判据：
  · 偏差 std < 5°、max-min < 15°  → 恒定性成立，可固定补偿
  · 偏差在整圈内大幅摆动          → 补偿表失真，硬件修复

用法：
    python enc_sweep.py COM27 --secs 60 --out enc_sweep.csv
    （启动后手动把电机轴慢慢转一整圈）
"""
import argparse
import csv
import math
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\X72\Agnet")
from thermal_soak import Rig, MSG_PERMISSION, MSG_CTRL_WORD

TWO_PI = 2 * math.pi
ENC = 2097152.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--secs", type=float, default=60.0)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--out", default=r"G:\FOC\X72\Agnet\enc_sweep.csv")
    args = ap.parse_args()

    rig = Rig(args.port, args.baud)
    rows = []
    try:
        time.sleep(1.0)
        rig.send(MSG_PERMISSION, bytes([0, 0])); time.sleep(0.25)
        rig.send(MSG_CTRL_WORD, bytes([0, 0, 1])); time.sleep(0.8)

        print("=" * 70, flush=True)
        print("采集中：请手动把电机轴慢慢转一整圈（%.0f 秒内）" % args.secs, flush=True)
        print("=" * 70, flush=True)

        t0 = time.time()
        rig.rx.pause_evt.set()
        buf = bytearray()
        last_t = 0.0
        while time.time() - t0 < args.secs:
            d = rig.s.read(4096)
            if d:
                buf += d
            # 边走边解析，避免缓冲无限增长
            i, n = 0, len(buf)
            while i + 12 <= n:
                if buf[i] != 0xFD:
                    i += 1
                    continue
                ln = buf[i + 1]
                tot = 12 + ln
                if i + tot > n:
                    break
                mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
                if mid == 30040 and ln >= 12:
                    b = i + 10
                    enc = struct.unpack_from("<q", buf, b)[0]
                    ang = struct.unpack_from("<f", buf, b + 8)[0]
                    now = time.time() - t0
                    if now - last_t >= 0.02:          # 50 Hz 抽稀
                        last_t = now
                        mech = (enc % ENC) / ENC * TWO_PI
                        d_ = (ang - 14.0 * mech) % TWO_PI
                        if d_ > math.pi:
                            d_ -= TWO_PI
                        rows.append({"t": round(now, 3),
                                     "enc": enc,
                                     "mech_deg": round(mech * 180 / math.pi, 2),
                                     "elec_deg": round((ang % TWO_PI) * 180 / math.pi, 2),
                                     "delta_deg": round(d_ * 180 / math.pi, 2)})
                i += tot
            del buf[:i]
            time.sleep(0.002)
    finally:
        rig.rx.pause_evt.clear()
        rig.rx.buf = bytearray()
        rig.close()

    if not rows:
        print("[ERR] 没采到 30040 帧")
        return 1

    with open(args.out, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    ds = [r["delta_deg"] for r in rows]
    mechs = [r["mech_deg"] for r in rows]
    span = max(mechs) - min(mechs)
    mean = sum(ds) / len(ds)
    sd = (sum((x - mean) ** 2 for x in ds) / len(ds)) ** 0.5
    print("\n采样 %d 点，机械角覆盖 %.1f°（%s）"
          % (len(rows), span, "已转整圈" if span > 300 else "未转满一圈"))
    print("偏差 delta = elec − 14×mech：")
    print("  均值 %+.2f°   σ %.2f°   范围 %+.2f° ~ %+.2f°   极差 %.2f°"
          % (mean, sd, min(ds), max(ds), max(ds) - min(ds)))
    if span > 300:
        if sd < 5.0 and (max(ds) - min(ds)) < 15.0:
            print("\n判定：✔ 偏差基本恒定 → 可用固定补偿修复")
            print("      建议补偿值 = %+.2f° = %+.4f rad" % (-mean, -mean * math.pi / 180))
        else:
            print("\n判定：✘ 偏差随位置变化（σ %.1f°, 极差 %.1f°）→ 补偿表失真，需硬件修复"
                  % (sd, max(ds) - min(ds)))
    print("[out] %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
