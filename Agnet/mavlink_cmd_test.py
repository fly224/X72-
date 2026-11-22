# -*- coding: utf-8 -*-
"""
mavlink_cmd_test.py —— MAVLink 操作指令码联调（2026-11-18）

验证固件首期 5 条操作消息的完整闭环：收帧 → CRC → 门卫 → set/get → 回显应答。
串口是双向混合流（100 Hz 遥测 + 应答帧），用 FrameParser 过滤 0xFD 帧。

用法：
    python mavlink_cmd_test.py COM27
    python mavlink_cmd_test.py COM27 --arm        # 含使能测试（会真实通电，谨慎）
"""
import sys, os, time, struct, argparse

HOST = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "..", "..", "驱动开发", "PG_X72_CAN", "host")
sys.path.insert(0, HOST)
import mavlink_proto as mp

try:
    import serial
except ImportError:
    print("需要 pyserial: pip install pyserial"); sys.exit(1)

READ_PARAM = 30000
CTRL_WORD  = 30054
OP_MODE    = 30056
PERMISSION = 30068
SYS_CMD    = 30069
STATUS_INFO = 30055

# AppParamStatus
ST_OK, ST_RO, ST_RANGE, ST_STATE, ST_NOAUTH = range(5)
ST_NAME = {0:"SUCCESS", 1:"READ_ONLY", 2:"OUT_OF_RANGE", 3:"WRITE_STATE_ERROR", 4:"NO_AUTHORITY"}

HOST_SYS, HOST_COMP = 1, 1


class Link:
    def __init__(self, port, baud=921600):
        self.s = serial.Serial(port, baud, timeout=0.02)
        self.parser = mp.FrameParser()
        self.seq = 0

    def send(self, msgid, payload):
        f = mp.encode(msgid, bytes(payload), seq=self.seq,
                      sysid=HOST_SYS, compid=HOST_COMP)
        self.seq = (self.seq + 1) & 0xFF
        self.s.reset_input_buffer()          # 清掉在途遥测，只读应答
        self.s.write(f); self.s.flush()
        return f

    def wait_ack(self, msgid, timeout=0.6):
        """发后等待指定 msgid 的应答帧（过滤遥测）"""
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < timeout:
            d = self.s.read(4096)
            if d:
                buf += d
                for fr in (self.parser.feed(bytes(buf)) or []):
                    if fr.get("msgid") == msgid:
                        return fr
                buf = self.parser.buf[:]      # 保留未解析完的尾字节
                self.parser.buf = bytearray()
        return None

    def close(self):
        self.s.close()


def show(name, fr, nbytes):
    if fr is None:
        print("  %-26s 无应答 ❌" % name); return False
    pl = fr.get("payload", b"")
    print("  %-26s ACK len=%d payload=%s seq=%s" %
          (name, len(pl), pl[:nbytes].hex(), fr.get("seq")))
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--arm", action="store_true", help="含控制字使能（真实通电）")
    a = ap.parse_args()

    lk = Link(a.port)
    print("=== MAVLink 操作指令码联调 %s @921600 ===" % a.port)
    try:
        # 1) 权限配置回显
        lk.send(PERMISSION, [0, 0])
        fr = lk.wait_ack(PERMISSION)
        ok1 = show("30068 权限(0=上位机接管)", fr, 3)
        if ok1:
            st, ca, ia = fr["payload"][0], fr["payload"][1], fr["payload"][2]
            print("     状态=%s comm_auth=%d internal_auth=%d" % (ST_NAME.get(st, st), ca, ia))

        # 2) 模式设置：速度=3
        lk.send(OP_MODE, [3])
        fr = lk.wait_ack(OP_MODE)
        ok2 = show("30056 模式=3(速度)", fr, 2)
        if ok2:
            print("     状态=%s 回读mode=%d" % (ST_NAME.get(fr["payload"][0], -1), fr["payload"][1]))

        # 3) 非法模式拒写
        lk.send(OP_MODE, [99])
        fr = lk.wait_ack(OP_MODE)
        ok3 = show("30056 模式=99(非法)", fr, 2)
        if ok3:
            print("     状态=%s (期望 OUT_OF_RANGE) 回读mode=%d" %
                  (ST_NAME.get(fr["payload"][0], -1), fr["payload"][1]))

        # 4) READ_PARAM 读状态字
        lk.send(READ_PARAM, list(struct.pack("<I", STATUS_INFO)))
        fr = lk.wait_ack(STATUS_INFO)
        ok4 = show("30000 读30055状态信息", fr, 12)
        if ok4:
            pl = fr["payload"]
            sw, ew, vbus = struct.unpack_from("<IIf", pl, 0)
            print("     statusword=0x%08X error_word=0x%08X vbus=%.2fV" % (sw, ew, vbus))

        # 5) 系统命令：清故障(3)
        lk.send(SYS_CMD, [3])
        fr = lk.wait_ack(SYS_CMD)
        ok5 = show("30069 系统命令=3(清故障)", fr, 2)
        if ok5:
            print("     状态=%s cmd=%d" % (ST_NAME.get(fr["payload"][0], -1), fr["payload"][1]))

        # 6) 控制字：halt=1 封锁（安全，不使能）
        lk.send(CTRL_WORD, [0, 0, 1])
        fr = lk.wait_ack(CTRL_WORD)
        ok6 = show("30054 控制字 halt=1(封锁)", fr, 2)
        if ok6:
            print("     状态=%s cw回读=0x%02X" %
                  (ST_NAME.get(fr["payload"][0], -1), fr["payload"][1]))

        # 7) 可选：控制字使能（真实通电）
        if a.arm:
            lk.send(CTRL_WORD, [1, 0, 0])
            fr = lk.wait_ack(CTRL_WORD)
            ok7 = show("30054 控制字 bit0=1(使能)", fr, 2)
            if ok7:
                print("     状态=%s cw回读=0x%02X" %
                      (ST_NAME.get(fr["payload"][0], -1), fr["payload"][1]))
            time.sleep(0.3)
            lk.send(CTRL_WORD, [0, 0, 1])       # 测完立即封锁
            lk.wait_ack(CTRL_WORD)
            print("     已发 halt 封锁")

        print("\n结果：%s" % ("全部通过 ✅" if all([ok1,ok2,ok3,ok4,ok5,ok6]) else "有失败项 ❌"))
        print("帧统计：", lk.parser.stats)
    finally:
        lk.close()


if __name__ == "__main__":
    main()
