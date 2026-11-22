# -*- coding: utf-8 -*-
"""验证 30082 AppDebugParam 运行时调参口。

流程（每一步都回读确认，不看回显猜）：
  1) 读 30082 → 打印 16 个槽位的当前值
  2) 写槽位 0（align_id）= 1.6 → 回读应为 1.6
  3) 写越界 10.0 → 应被拒（n_ok=0），回读仍 1.6
  4) 写槽位 1（ol_vlim）= 0.10 → 回读应为 0.10
  5) 写未实现槽位 15 → n_ok 应为 0
  6) 恢复 align_id = 2.0（原值）

用法：python verify_dbg_param.py COM27
"""
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
import mavlink_proto as MV  # noqa: E402
import serial  # noqa: E402

MID = 30082
NAMES = ["align_id", "ol_vlim", "cur_kp", "cur_ki_ts", "spd_kp", "spd_ki_ts",
         "spd_iqmax", "pos_kp", "ff_acc", "ff_fric", "ff_visc", "dead_du",
         "cur_iqmax", "psi", "rsv14", "rsv15"]


def send(s, mid, payload, seq):
    s.write(MV.encode(mid, payload, seq=seq))
    s.flush()


def recv_30082(s, timeout=1.0):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = s.read(4096)
        if d:
            buf += d
        i = 0
        while i < len(buf) - 204:
            if buf[i] == 0xFD:
                ln = buf[i + 1]
                mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
                if mid == MID and ln == 192:
                    return bytes(buf[i + 10:i + 10 + 192])
                i += 12 + ln
            else:
                i += 1
    return None


def read_all(s, seq):
    s.reset_input_buffer()
    send(s, 30000, struct.pack("<I", MID), seq)
    p = recv_30082(s)
    if p is None:
        return None
    return [struct.unpack_from("<f", p, i * 4)[0] for i in range(16)]


def write_one(s, idx, val, seq):
    p = bytearray(192)
    struct.pack_into("<f", p, idx * 4, val)
    struct.pack_into("<I", p, 64, 1 << idx)
    s.reset_input_buffer()
    send(s, MID, bytes(p), seq)
    # 应答是 30082 短帧（2B），不在这里等，靠随后回读判断
    time.sleep(0.15)
    return read_all(s, (seq + 1) & 0xFF)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM27"
    s = serial.Serial(port, 921600, timeout=0.05)
    seq = 1
    ok = 0
    try:
        v = read_all(s, seq); seq = (seq + 1) & 0xFF
        if v is None:
            print("[!] 读 30082 无应答")
            return 1
        print("读回 16 槽位：")
        for i in range(16):
            print("  [%2d] %-10s = %.6g" % (i, NAMES[i], v[i]))

        print("\n写 align_id=1.6 ...")
        v = write_one(s, 0, 1.6, seq); seq = (seq + 2) & 0xFF
        ok += 1 if v and abs(v[0] - 1.6) < 1e-4 else 0
        print("  回读 align_id = %.6g  %s" % (v[0] if v else float('nan'),
                                            "PASS" if v and abs(v[0] - 1.6) < 1e-4 else "FAIL"))

        print("写 align_id=10.0（越界，应拒绝）...")
        v = write_one(s, 0, 10.0, seq); seq = (seq + 2) & 0xFF
        good = v is not None and abs(v[0] - 1.6) < 1e-4
        ok += 1 if good else 0
        print("  回读 align_id = %.6g  %s（应仍为 1.6）"
              % (v[0] if v else float('nan'), "PASS" if good else "FAIL"))

        print("写 ol_vlim=0.10 ...")
        v = write_one(s, 1, 0.10, seq); seq = (seq + 2) & 0xFF
        good = v is not None and abs(v[1] - 0.10) < 1e-5
        ok += 1 if good else 0
        print("  回读 ol_vlim = %.6g  %s" % (v[1] if v else float('nan'), "PASS" if good else "FAIL"))

        print("写未实现槽位 15 ...")
        v = write_one(s, 15, 1.0, seq); seq = (seq + 2) & 0xFF
        good = v is not None and abs(v[15]) < 1e-9
        ok += 1 if good else 0
        print("  回读 [15] = %.6g  %s（应仍为 0）"
              % (v[15] if v else float('nan'), "PASS" if good else "FAIL"))

        print("恢复 align_id=2.0 ...")
        v = write_one(s, 0, 2.0, seq)
        print("  回读 align_id = %.6g" % (v[0] if v else float('nan')))
    finally:
        s.close()

    print("\n结果：%d/4 通过" % ok)
    return 0 if ok == 4 else 1


if __name__ == "__main__":
    sys.exit(main())
