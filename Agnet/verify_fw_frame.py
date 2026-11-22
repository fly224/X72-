# -*- coding: utf-8 -*-
"""verify_fw_frame.py —— 固件发送帧的三方闭环校验

链路：
    1) Board/telemetry_tx.c 在主机侧原样编译运行（stub 掉 HAL 发送），产出真实字节；
    2) 本脚本按 MAVLink v2 拆帧、用 crc_extra 复算 CRC；
    3) 用 mavlink_fields（布局来自 91 个 mavlink_msg_*.h）解字段并断言；
    4) 再把载荷交给 servo-web/index.html 的 decodeFrame（Node）解一遍，
       断言页面拿到的 Iq / Id / 母线电压与固件送入值一致。

用法：python Agnet/verify_fw_frame.py
"""
import json
import math
import os
import struct
import subprocess
import sys

HOST_DIR = r"G:/FOC/驱动开发/PG_X72_CAN/host"
HERE = os.path.dirname(os.path.abspath(__file__))
TMP = os.path.join(HERE, "..", ".workbuddy", "tmp")
sys.path.insert(0, HOST_DIR)
sys.path.insert(0, HERE)

import mavlink_fields as MF  # noqa: E402
from mavlink_proto import crc_calculate  # noqa: E402

EXE = os.path.join(TMP, "fw_frame.exe")
NODE = os.path.join(HERE, "verify_host_iq_vbus.js")
NODE_POS = os.path.join(HERE, "verify_host_pos.js")

CRC_EXTRA = {30025: 191, 30027: 58, 30040: 160}
LEN = {30025: 57, 30027: 32, 30040: 12}   # 30025：53→57（+53 motor_temp）

# harness 里 BspServo_GetPosSum() 的 stub 值：2097152 + 524288 = 1.25 圈
ENC_SUM_STUB = 2621440
ENC_COUNTS = 2097152.0


def split_frames(buf):
    out, i = [], 0
    while i < len(buf):
        assert buf[i] == 0xFD, "帧头不是 0xFD @%d" % i
        n = buf[i + 1]
        msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        total = 10 + n + 2
        out.append((msgid, buf[i:i + total], buf[i + 10:i + 10 + n]))
        i += total
    return out


def main():
    if not os.path.isfile(EXE):
        print("缺少 exe，请先按 Agnet/verify_fw_frame.sh 构建")
        return 2
    raw = subprocess.run([EXE], capture_output=True, text=True, encoding="utf-8").stdout
    lines = raw.strip().splitlines()
    hexstr = [l for l in lines if l.startswith("HEX ")][0][4:]
    buf = bytes.fromhex(hexstr)
    print("固件输出: %d B" % len(buf))

    frames = split_frames(buf)
    fails = 0
    by_id = {}
    for msgid, frame, payload in frames:
        extra = CRC_EXTRA[msgid]
        # 标准 MAVLink v2 口径：CRC 覆盖 len..payload（跳过帧首 0xFD）
        want = crc_calculate(frame[1:-2], extra)
        got = frame[-2] | (frame[-1] << 8)
        ok = (want == got) and (len(payload) == LEN[msgid])
        fails += 0 if ok else 1
        by_id.setdefault(msgid, []).append(payload)
        if len(by_id[msgid]) <= 1:
            print("  %-6d LEN=%2d CRC=%04X/%04X  %s" % (msgid, len(payload), got, want, "PASS" if ok else "FAIL"))

    for mid, pl in by_id.items():
        print("  报文 %d × %d 条" % (mid, len(pl)))

    # seq 必须严格 +1（含 255→0 回绕）：发送成功才推进序号，正常路径不允许跳号
    seqs = [f[4] for _, f, _ in frames]
    bad = [(a, b) for a, b in zip(seqs, seqs[1:]) if ((b - a) & 0xFF) != 1]
    print("  seq 连续性: %d 帧首尾 %d→%d，断点 %d 处  %s"
          % (len(seqs), seqs[0] if seqs else -1, seqs[-1] if seqs else -1,
             len(bad), "PASS" if not bad else "FAIL"))
    fails += len(bad)
    # 三条报文必须严格成对：任何一条缺失都会让上位机的去重采样失去对齐
    counts = set(len(v) for v in by_id.values())
    trio_ok = (set(by_id) == set(CRC_EXTRA)) and (len(counts) == 1)
    fails += 0 if trio_ok else 1
    print("  三报文配比 %s  %s" % (
        {k: len(v) for k, v in by_id.items()}, "PASS" if trio_ok else "FAIL"))

    d_in = MF.decode(30025, by_id[30025][0])
    d_out = MF.decode(30027, by_id[30027][0])
    d_pos = MF.decode(30040, by_id[30040][0])
    print("\n30025 解析: iabc=%s vbus=%.3f idq_tar=%s theta=%.4f we=%.3f temp=%r mode=%s"
          % (d_in["iabc_now_A"], d_in["dc_bus_now_V"], d_in["idq_tar_A"],
             d_in["elec_theta_rad"], d_in["elec_angle_speed_rad_s"], d_in["driver_temp"], d_in["mode"]))
    print("30027 解析: idq_now=%s" % (d_out["idq_now_A"],))
    print("30040 解析: enc_sum_p=%s (%.4f 圈) elec_angle=%.4f"
          % (d_pos["enc_sum_p"], d_pos["enc_sum_p"] / ENC_COUNTS, d_pos["elec_angle_now_rad"]))

    # 期望值与 harness stub 一一对应
    theta_sum = 14 * 0.25 + math.pi / 2 + 0.00105 * 3.0
    ialpha, ibeta = 3.0, 0.57735026918962584 * 3.0 + 1.1547005383792515 * (-4.0)
    id_exp = math.cos(theta_sum) * ialpha + math.sin(theta_sum) * ibeta
    iq_exp = math.cos(theta_sum) * ibeta - math.sin(theta_sum) * ialpha

    checks = [
        ("iabc_now_A", d_in["iabc_now_A"], [3.0, -4.0, 1.0], 1e-4),
        ("dc_bus_now_V", d_in["dc_bus_now_V"], 48.3, 1e-4),
        ("idq_tar_A", d_in["idq_tar_A"], [0.0, 2.5], 1e-4),
        ("elec_theta_rad", d_in["elec_theta_rad"], 3.5, 1e-4),
        ("elec_angle_speed", d_in["elec_angle_speed_rad_s"], 42.0, 1e-4),
        ("uabc_tar_V[0]", d_in["uabc_tar_V"][0], (0.55 - 0.5) * 48.3, 1e-4),
        ("uabc_tar_V[1]", d_in["uabc_tar_V"][1], (0.45 - 0.5) * 48.3, 1e-4),
        ("idq_now_A[0]", d_out["idq_now_A"][0], id_exp, 1e-4),
        ("idq_now_A[1]", d_out["idq_now_A"][1], iq_exp, 1e-4),
        ("enc_sum_p", float(d_pos["enc_sum_p"]), float(ENC_SUM_STUB), 0.5),
        ("enc 圈数", d_pos["enc_sum_p"] / ENC_COUNTS, 1.25, 1e-9),
        ("elec_angle_now", d_pos["elec_angle_now_rad"], 3.5, 1e-4),
    ]
    for name, got, exp, tol in checks:
        if isinstance(exp, list):
            ok = all(abs(a - b) <= tol for a, b in zip(got, exp))
        else:
            ok = abs(got - exp) <= tol
        fails += 0 if ok else 1
        print("  %-18s got=%-22s exp=%-22s %s"
              % (name, ("%s" % (got,)), ("%s" % (exp,)), "PASS" if ok else "FAIL"))
    # NTC 双温度：harness stub 送 GAN=41.2 / MOTOR=36.8，两个值刻意不同，
    # 一旦 +40 与 +53 写串位，下面两条断言必定有一条挂掉。
    gan_ok = abs(d_in["driver_temp"] - 41.2) < 1e-3
    fails += 0 if gan_ok else 1
    print("  %-18s got=%-8.3f exp=%-8.3f %s"
          % ("driver_temp(GAN)", d_in["driver_temp"], 41.2, "PASS" if gan_ok else "FAIL"))
    # motor_temp 是 LEN 53→57 新增字段，host 的 mavlink_layouts.json 尚未同步，
    # 这里按固定偏移直接解（协议以固件为准，见 Agnet/CRC口径与seq修正说明.md）
    (motor_c,) = struct.unpack_from("<f", by_id[30025][0], 53)
    motor_ok = abs(motor_c - 36.8) < 1e-3
    fails += 0 if motor_ok else 1
    print("  %-18s got=%-8.3f exp=%-8.3f %s"
          % ("motor_temp(MOTOR)", motor_c, 36.8, "PASS" if motor_ok else "FAIL"))

    # 页面解码器（Node）复核：电流/母线 + 位置分别走各自的提取脚本
    out = subprocess.run(
        ["node", NODE, by_id[30025][0].hex(), by_id[30027][0].hex()],
        capture_output=True, text=True, encoding="utf-8")
    if out.returncode != 0:
        print("\nNode 失败:", out.stderr[:500])
        return 1
    view = json.loads(out.stdout.strip().splitlines()[-1])
    print("\n页面 decodeFrame: Iq=%.3f Id=%.3f Vbus=%.3f 转速=%.1f rpm"
          % (view["iq"], view["id"], view["vbus"], view["vel"]))
    for k, exp in (("iq", iq_exp), ("id", id_exp), ("vbus", 48.3)):
        ok = abs(view[k] - exp) <= 1e-3
        fails += 0 if ok else 1
        print("  %-5s 页面=%.4f 期望=%.4f  %s" % (k, view[k], exp, "PASS" if ok else "FAIL"))

    out = subprocess.run(
        ["node", NODE_POS, by_id[30040][0].hex()],
        capture_output=True, text=True, encoding="utf-8")
    if out.returncode != 0:
        print("\nNode(位置) 失败:", out.stderr[:500])
        return 1
    vp = json.loads(out.stdout.strip().splitlines()[-1])
    print("\n页面 30040: encSum=%d 圈数=%.4f 机械角=%.4f rad (%.4f°) 电角度=%.3f rad"
          % (vp["encSum"], vp["turns"], vp["pos"], vp["degrees"], vp["elecTheta"]))
    pos_checks = [
        ("页圈数", vp["turns"], 1.25, 1e-6),
        ("页机械角", vp["pos"], math.pi / 2, 1e-5),
        ("页角度°", vp["degrees"], 90.0, 1e-3),
        ("页电角度", vp["elecTheta"], 3.5, 1e-4),
        ("页hasPos", 1.0 if vp["hasPos"] else 0.0, 1.0, 0.0),
    ]
    for name, got, exp, tol in pos_checks:
        ok = abs(got - exp) <= tol
        fails += 0 if ok else 1
        print("  %-10s 页面=%-12s 期望=%-12s %s" % (name, got, exp, "PASS" if ok else "FAIL"))

    print("\n合计 FAIL=%d" % fails)
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
