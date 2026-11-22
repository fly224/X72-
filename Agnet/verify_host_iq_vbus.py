# -*- coding: utf-8 -*-
"""验证上位机 30025/30027 解码：用固件头文件导出的布局生成载荷，交给 Node 端 decodeFrame 还原。
用法：python Agnet/verify_host_iq_vbus.py
说明：编码用 host/mavlink_fields.py（字段布局由 91 个 mavlink_msg_*.h 解析而来），
      解码用 servo-web/index.html 内的真实 decodeFrame，因此是跨语言、跨实现的端到端校验。
"""
import json
import math
import os
import subprocess
import sys

HOST_DIR = r"G:/FOC/驱动开发/PG_X72_CAN/host"
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HOST_DIR)

import mavlink_fields as MF  # noqa: E402

NAN = float("nan")

CUR_IN = {
    "idq_tar_A": [1.5, 2.5],
    "iabc_now_A": [3.0, 4.0, 5.0],
    "elec_theta_rad": 0.7,
    "dc_bus_now_V": 48.3,
    "uabc_tar_V": [10.0, -5.0, -5.0],
    "driver_temp": NAN,            # 本板 NTC 未接 ADC，固件送 NaN
    "elec_angle_speed_rad_s": 1400.0,
    "iq_offset_A": 0.1,
    "mode": 2,
}
CUR_OUT = {
    "uabc_tar_comp_V": [1.0, 2.0, 3.0],
    "idq_now_A": [1.25, -3.75],
    "uabc_tar_org_V": [4.0, 5.0, 6.0],
}

POLE = 14.0
expect = {
    "iq": -3.75,
    "id": 1.25,
    "vbus": 48.3,
    "ia": 3.0, "ib": 4.0, "ic": 5.0,
    "vel": 1400.0 / POLE * 60.0 / (2.0 * math.pi),
    "iqTar": 2.5, "idTar": 1.5,
    "mode": 2,
    "tempDash": "--",
}

p_in = MF.encode(30025, CUR_IN)
p_out = MF.encode(30027, CUR_OUT)


def run_node():
    js = os.path.join(HERE, "verify_host_iq_vbus.js")
    out = subprocess.run(
        ["node", js, p_in.hex(), p_out.hex()],
        capture_output=True, text=True, encoding="utf-8")
    if out.returncode != 0:
        print("Node 失败:", out.stderr[:800])
        return None
    return json.loads(out.stdout.strip().splitlines()[-1])


def main():
    print("30025 载荷 %d B / 30027 载荷 %d B" % (len(p_in), len(p_out)))
    got = run_node()
    if got is None:
        return 1

    rows, fails = [], 0
    for k, v in expect.items():
        g = got.get(k)
        if isinstance(v, str):
            ok = (g == v)
            shown = "%r" % g
        else:
            ok = (g is not None) and abs(float(g) - v) <= 1e-4
            shown = "%.6f" % float(g) if g is not None else "None"
        fails += 0 if ok else 1
        rows.append((k, ("%.4f" % v) if not isinstance(v, str) else v, shown, "PASS" if ok else "FAIL"))

    w = max(len(r[0]) for r in rows)
    print("%-*s %14s %14s  %s" % (w, "字段", "期望", "解码", "判定"))
    for r in rows:
        print("%-*s %14s %14s  %s" % (w, r[0], r[1], r[2], r[3]))
    print("\n合计 %d 项，FAIL=%d" % (len(rows), fails))
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
