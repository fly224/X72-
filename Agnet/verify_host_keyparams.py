# -*- coding: utf-8 -*-
"""
独立解码一路抓包（与页面实现无关），用于交叉校验：
  - MAVLink v2 帧边界 + CRC-16/X.25（含 crc_extra）逐帧校验
  - seq 连续性（固件每帧 +1，3 帧/10 ms）
  - 最后一条 30025 / 30027 / 30040 的字段值
  - 页面同款字段：iw、duty、机械角、圈数、累积脉冲
输出 JSON，供 verify_host_keyparams.js 比对。
"""
import json
import struct
import sys

CRC_EXTRA = {30025: 191, 30027: 58, 30040: 160}
ENC = 2097152  # 2^21
POLE_PAIRS = 14


def crc_x25(buf):
    crc = 0xFFFF
    for b in buf:
        tmp = (b ^ crc) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def decode(path):
    raw = open(path, "rb").read()
    i = 0
    frames = 0
    bad_crc = 0
    hits = {"fw_magic_included": 0, "std_mavlink": 0}
    seqs = []
    last = {}
    while i < len(raw):
        if raw[i] != 0xFD:
            i += 1
            continue
        if i + 10 > len(raw):
            break
        ln = raw[i + 1]
        incompat = raw[i + 2]
        seq = raw[i + 4]
        msgid = raw[i + 7] | (raw[i + 8] << 8) | (raw[i + 9] << 16)
        flen = 10 + ln + 2 + (13 if incompat & 0x01 else 0)
        if i + flen > len(raw):
            break
        payload = raw[i + 10:i + 10 + ln]
        crc_rx = raw[i + 10 + ln] | (raw[i + 11 + ln] << 8)
        body = raw[i + 1:i + 10 + ln]
        extra = CRC_EXTRA.get(msgid)
        # 注意：本固件 crc_calc(f, 10+len, extra) 从 f[0]（含 0xFD）开始算，
        # 与标准 MAVLink（从 len 字节开始）差一个字节；两种都试，记录各自命中数。
        ok_std = extra is not None and crc_x25(body + bytes([extra])) == crc_rx
        ok_fw = extra is not None and crc_x25(raw[i:i + 10 + ln] + bytes([extra])) == crc_rx
        ok = ok_std or ok_fw
        if ok_fw:
            hits["fw_magic_included"] += 1
        if ok_std:
            hits["std_mavlink"] += 1
        if ok:
            frames += 1
            seqs.append(seq)
            last[msgid] = payload
            i += flen
        else:
            bad_crc += 1
            i += 1                      # 重新同步
    gaps = 0
    for a, b in zip(seqs, seqs[1:]):
        d = (b - a) & 0xFF
        if d != 1:
            gaps += (1 if d == 0 else d - 1)
    out = {
        "bytes": len(raw),
        "frames": frames,
        "bad_crc": bad_crc,
        "crc_hits": hits,
        "seq_first": seqs[0] if seqs else None,
        "seq_last": seqs[-1] if seqs else None,
        "seq_gaps": gaps,
        "by_id": {},
    }
    for mid in (30025, 30027, 30040):
        p = last.get(mid)
        if not p:
            continue
        if mid == 30025:
            f = struct.unpack_from("<13fB", p, 0)
            # NaN 不能直接进 JSON（json.dumps 会输出非法字面量 NaN），统一转 None
            def nn(v, n=6):
                return None if v != v else round(v, n)
            d25 = {
                "id_tar": round(f[0], 6), "iq_tar": round(f[1], 6),
                "iu": round(f[2], 6), "iv": round(f[3], 6), "iw": round(f[4], 6),
                "elec_theta": round(f[5], 6), "vbus": round(f[6], 6),
                "ua": round(f[7], 6), "ub": round(f[8], 6), "uc": round(f[9], 6),
                "driver_temp": nn(f[10]),
                "mode": f[13],
            }
            # +53 motor_temp 是 LEN 53→57 新增字段；旧抓包没有 → None
            d25["motor_temp"] = nn(struct.unpack_from("<f", p, 53)[0]) if len(p) >= 57 else None
            out["by_id"][mid] = d25
        elif mid == 30027:
            f = struct.unpack_from("<8f", p, 0)
            out["by_id"][mid] = {"id": round(f[3], 6), "iq": round(f[4], 6)}
        else:
            enc = struct.unpack_from("<q", p, 0)[0]
            elec = struct.unpack_from("<f", p, 8)[0]
            frac = enc % ENC
            out["by_id"][mid] = {
                "enc_sum": enc,
                "turns": round(enc / ENC, 9),
                "mech_deg": round(frac / ENC * 360.0, 6),
                "elec_theta": round(elec, 6),
            }
    return out


if __name__ == "__main__":
    p = sys.argv[1] if len(sys.argv) > 1 else "capture.bin"
    print(json.dumps(decode(p), ensure_ascii=False, indent=1))
