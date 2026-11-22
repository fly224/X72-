# -*- coding: utf-8 -*-
"""验证 30092 AppIdentResult：发一条辨识命令，抓对应 kind 的结果帧。

用法：python v92.py COM27 @ 2 40      # 命令、期望 kind、超时
      python v92.py COM27 + 1 45
"""
import struct
import sys
import time

sys.path.insert(0, r"G:\FOC\驱动开发\PG_X72_CAN\host")
sys.path.insert(0, r"G:\FOC\X72_Agnet")
import mavlink_proto as MV  # noqa: E402
from thermal_soak import Rig, MSG_PERMISSION  # noqa: E402

KIND = {1: "磁链", 2: "RL", 3: "惯量", 4: "摩擦曲线", 5: "齿槽",
        6: "扫频", 7: "编码器", 8: "对齐", 9: "摩擦两点"}

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM27"
CMD = sys.argv[2].encode() if len(sys.argv) > 2 else b"@"
WANT = int(sys.argv[3]) if len(sys.argv) > 3 else 2
TMO = float(sys.argv[4]) if len(sys.argv) > 4 else 45.0

rig = Rig(PORT)
time.sleep(1.2)
rig.send(MSG_PERMISSION, bytes([0, 0]))
time.sleep(0.3)

rig.rx.pause_evt.set()
time.sleep(0.05)
found = None
try:
    rig.s.reset_input_buffer()
    rig.s.write(CMD)
    rig.s.flush()
    b = bytearray()
    t0 = time.time()
    while time.time() - t0 < TMO:
        d = rig.s.read(8192)
        if d:
            b += d
        i = 0
        while i + 12 <= len(b):
            if b[i] == 0xFD:
                ln = b[i + 1]
                if i + 12 + ln + 2 > len(b):
                    break
                mid = b[i + 7] | (b[i + 8] << 8) | (b[i + 9] << 16)
                if mid == 30092 and ln == 40:
                    pay = bytes(b[i + 10:i + 50])
                    crc = b[i + 50] | (b[i + 51] << 8)
                    if MV.crc_calculate(bytes(b[i + 1:i + 10]) + pay, 92) == crc and pay[0] == WANT:
                        found = pay
                        break
                del b[:12 + ln]
            else:
                del b[:1]
        if found:
            break
finally:
    rig.rx.buf = bytearray()
    rig.rx.pause_evt.clear()

if found:
    v = struct.unpack_from("<8f", found, 4)
    print("%s kind=%d ok=%d n=%d resid=%.4g"
          % (KIND.get(found[0], "?"), found[0], found[1],
             found[2] | (found[3] << 8), struct.unpack_from("<f", found, 36)[0]))
    print("  v = " + " ".join("%.4g" % x for x in v))
else:
    print("未捕获 kind=%d（命令回显另见串口）" % WANT)

rig.close()
