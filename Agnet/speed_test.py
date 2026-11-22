# -*- coding: utf-8 -*-
"""
speed_test.py —— 速度闭环实测：一键进环 + 100 Hz 遥测采集 + 阶跃指标

为什么不用上位机点点点：
  速度环的阶跃只有几十毫秒，人工操作既对不齐时间轴，也没法复现同一组参数。
  本脚本把"清故障→标定→进环→采集→算指标"固化成一条流水线，
  换一档增益（L 键）重跑一遍就能得到干净的对比表。

安全约束（脚本内置，不要绕过）：
  · 每次都先 D（封锁 MOE）再 X（清故障），不带着旧状态进环
  · 只发 ±30 rpm 这类低速指令，deadman 2 s 保活，脚本退出前一定发 D
  · 不做 L 档位自动扫描：档位越高越可能自激，必须人看清楚结果再决定下一档

帧格式（与 telemetry_tx.c 一致）：
  30025 CurrentCtlInput, payload 57 B
    [0..3]  id_tar      [4..7]  iq_tar      [8..19] iabc
    [24..27] vbus       [40..43] temp_gan   [44..47] 14*omega_m
    [53..56] temp_motor
  rpm = payload[44:48] / 14 * 60 / (2*pi)

用法：
  python speed_test.py                       # COM27, +30 rpm, 1x
  python speed_test.py --rpm -30             # 反向
  python speed_test.py --gain 8              # 先按 3 次 L 到 8x 再测
  python speed_test.py --secs 2.0 --port COM5
"""

import argparse
import math
import struct
import sys
import time

MSG_IN = 30025
LEN_IN = 61
CRC_EXTRA_IN = 191
POLE_PAIRS = 14.0
TWO_PI = 2.0 * math.pi


def x25_crc(data, extra):
    """MAVLink 的 CRC-16/X25：先把 extra 当最后一个字节再算一遍。
    必须校验：第一版为了省事只对帧头做三重匹配，结果 ASCII 回显把字节流切碎后，
    错位帧被当成有效数据，母线电压读出 3.9e12 V、转速基线 28.8 rpm —— 全是假的。"""
    crc = 0xFFFF

    def acc(crc, b):
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF

    for b in data:
        crc = acc(crc, b)
    return acc(crc, extra)


class SpeedRig:
    def __init__(self, port, baud=921600):
        import serial  # 延迟导入：--help 时不需要 pyserial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.raw = bytearray()
        self.frames = []      # [(t, rpm, iq_tar, vbus)]

    # ---------------- 原始收发 ----------------
    def send(self, s, settle=0.12):
        """发一条命令并吃掉回显。回显是 ASCII 行，逐字节读到 CRLF 为止。"""
        self.ser.reset_input_buffer()
        self.ser.write(s.encode("ascii"))
        self.ser.flush()
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < 0.6:
            chunk = self.ser.read(256)
            if chunk:
                buf += chunk
                if b"\r\n" in buf:
                    # 取最后一条 [CMD] 行为回显（前面可能混着二进制帧）
                    idx = buf.rfind(b"[CMD]")
                    if idx >= 0:
                        line = bytes(buf[idx:]).split(b"\r\n")[0]
                        return line.decode("ascii", "replace")
                    break
        return ""

    def drain(self, seconds, keep=False):
        """读 seconds 秒的字节流，解析 30025 帧。keep=True 时追加到 self.frames。"""
        t_end = time.time() + seconds
        out = []
        while time.time() < t_end:
            chunk = self.ser.read(512)
            if not chunk:
                continue
            self.raw += chunk
            out.extend(self._parse())

        if keep:
            self.frames.extend(out)
        return out

    def _parse(self):
        """在字节流里找 0xFD + len=57 + msgid=30025 + CRC 校验通过的帧。
        CRC 是必需的：ASCII 回显（[CMD] 行）会与二进制帧交错，
        只靠帧头匹配会把跨帧的垃圾拼成"有效"帧，指标全错。"""
        res = []
        buf = self.raw
        i = 0
        n = len(buf)
        while i + 10 + LEN_IN + 2 <= n:
            if buf[i] != 0xFD:
                i += 1
                continue
            ln = buf[i + 1]
            msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            if ln != LEN_IN or msgid != MSG_IN:
                i += 1
                continue
            body = i + 10
            crc_rx = buf[body + LEN_IN] | (buf[body + LEN_IN + 1] << 8)
            if x25_crc(buf[i + 1:body + LEN_IN], CRC_EXTRA_IN) != crc_rx:
                i += 1                      # 假头，逐字节重找
                continue
            p = buf[body:body + LEN_IN]
            iq_tar = struct.unpack_from("<f", p, 4)[0]
            vbus = struct.unpack_from("<f", p, 24)[0]
            w_elec = struct.unpack_from("<f", p, 44)[0]
            rpm = w_elec / POLE_PAIRS * 60.0 / TWO_PI
            res.append((time.time(), rpm, iq_tar, vbus))
            i = body + LEN_IN + 2
        del buf[:i]
        return res

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def metrics(frames, target):
    """阶跃指标（绝对量，不做 base/amp 归一化）。

    第一版用 (v-base)/(ss-base) 归一化，结果在"进环后立即采样、base 窗口为空"
    时除数趋近 0，超调算出 3459% 这种数字 —— 指标算法本身把结论毁了。
    这里直接用给定值 target 作标尺：tr 看 10%/90%*target，超调看 max/target。
    """
    if len(frames) < 20:
        return None
    t = [f[0] - frames[0][0] for f in frames]
    v = [f[1] for f in frames]
    sgn = 1.0 if target > 0 else -1.0

    vn = [x * sgn for x in v]           # 统一到"正转"方向
    tgt = abs(target)

    i10 = next((i for i, q in enumerate(vn) if q >= 0.10 * tgt), None)
    i90 = next((i for i, q in enumerate(vn) if q >= 0.90 * tgt), None)
    tr = (t[i90] - t[i10]) * 1e3 if (i10 is not None and i90 is not None) else float("nan")

    tail = [x for tt, x in zip(t, vn) if tt > t[-1] - 0.25]
    ss = sum(tail) / len(tail) if tail else vn[-1]
    peak = max(vn)
    os_pct = (peak - tgt) / tgt * 100.0 if tgt > 1e-9 else float("nan")

    band = 0.02 * tgt
    i_set = None
    for i in range(len(vn) - 1, -1, -1):
        if abs(vn[i] - tgt) > band:
            i_set = i + 1
            break
    ts = t[i_set] * 1e3 if (i_set is not None and i_set < len(t)) else float("nan")

    ripple = 0.0
    if len(tail) > 2:
        m = sum(tail) / len(tail)
        ripple = math.sqrt(sum((x - m) ** 2 for x in tail) / len(tail))

    return dict(ss=ss, tr=tr, os=os_pct, ts=ts, ripple=ripple,
                n=len(frames), t_step=frames[0][0], peak=peak)


def show_wave(frames, t_step, width=64, height=13):
    """裁剪到 [-0.05, 0.95] s 的 ASCII 波形，方便直接看有没有振荡/爬升"""
    sel = [(f[0] - t_step, f[1]) for f in frames if -0.05 <= (f[0] - t_step) <= 0.95]
    if len(sel) < 10:
        return
    vs = [v for _, v in sel]
    vmin, vmax = min(vs), max(vs)
    if vmax - vmin < 1e-6:
        vmax = vmin + 1e-6
    rows = []
    for r in range(height):
        hi = vmax - (vmax - vmin) * r / (height - 1)
        line = []
        for c in range(width):
            a = int(c * len(sel) / width)
            b = int((c + 1) * len(sel) / width)
            seg = sel[a:b] if b > a else sel[a:a + 1]
            hit = any(v >= hi for _, v in seg)
            line.append("#" if hit else " ")
        rows.append("".join(line))
    print("  rpm %.2f .. %.2f   (-0.05 .. 0.95 s)" % (vmin, vmax))
    for r, line in enumerate(rows):
        lab = "%7.2f |" % (vmax - (vmax - vmin) * r / (height - 1))
        print("  " + lab + line)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--rpm", type=float, default=30.0,
                    choices=[3.0, 30.0, 100.0, 500.0, 1500.0, 3000.0,
                             -3.0, -30.0, -100.0, -500.0, -1500.0, -3000.0],
                    help="转速档位（固件 O 键表：3/10/30/60/100，负值为反向）")
    ap.add_argument("--gain", type=float, default=1.0,
                    choices=[0.125, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0],
                    help="速度环增益档位（L 键循环 0.125x..8x）")
    ap.add_argument("--secs", type=float, default=1.2, help="阶跃后采集时长 s")
    ap.add_argument("--no-wave", action="store_true")
    args = ap.parse_args()

    try:
        rig = SpeedRig(args.port)
    except Exception as e:
        print("[FAIL] 打开 %s 失败：%s" % (args.port, e))
        print("       常见原因：上位机 Web Serial 还占着串口。先关掉浏览器页面再跑。")
        return 2

    ok = False
    try:
        print("[1] 复位：D -> X")
        print("    " + rig.send("D"))
        print("    " + rig.send("X"))
        time.sleep(0.05)

        st = rig.send("S")
        print("[2] 状态：" + st)
        cal = "cal=1" in st
        zed = "z=1" in st
        if not cal:
            print("[3] 电流零点未标定 -> C（保持电机静止 200 ms）")
            print("    " + rig.send("C", settle=0.4))
            time.sleep(0.4)
            st = rig.send("S")
            print("    " + st)
            cal = "cal=1" in st
        if not zed:
            print("[3] 电角度未对齐 -> Z（VESC 三段式，约 9 s，电机会小幅转动）")
            print("    " + rig.send("Z", settle=0.3))
            for _ in range(14):
                time.sleep(1.0)
                line = rig.send("S")
                if "z=1" in line:
                    print("    " + line)
                    zed = True
                    break
            if not zed:
                print("[FAIL] Z 未完成，放弃")
                return 3

        SCALES = [0.125, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0]
        idx = SCALES.index(args.gain)
        print("    " + rig.send("Y"))     # 复位到 1x（索引 3）
        presses = (idx - 3) % 9           # Y 之后在 1x（索引 3）；往下要走绕一圈
        if presses:
            print("[4] 速度环增益 -> %gx（按 L %d 次）" % (args.gain, presses))
            for _ in range(presses):
                print("    " + rig.send("L"))
        else:
            print("[4] 速度环增益 %gx（模型值）" % args.gain)

        # 转速档位：固件里是 O 键循环的固定表，这里按索引按够次数
        TAB = [3.0, 30.0, 100.0, 500.0, 1500.0, 3000.0]
        tidx = TAB.index(abs(args.rpm))
        for _ in range((tidx - 1) % 6):  # Y 已复位到 30 rpm（索引 1）
            print("    " + rig.send("O"))

        rig.drain(0.15)                     # 丢掉旧帧
        print("[5] 进环 W，给定 %+.1f rpm ..." % args.rpm)
        t_step = time.time()
        echo = rig.send("W" if args.rpm > 0 else "Q", settle=0.05)
        print("    " + echo)
        if "refused" in echo or "failed" in echo:
            print("[FAIL] 进环被拒，停止")
            return 5

        # 边采边保活：每 1.5 s 补发一条同值命令重置 deadman（内部不清积分）
        t_end = time.time() + args.secs
        nxt_keep = time.time() + 1.5
        while time.time() < t_end:
            rig.drain(0.1, keep=True)
            if time.time() >= nxt_keep and time.time() < t_end - 0.05:
                rig.ser.write(b"W" if args.rpm > 0 else b"Q")
                nxt_keep += 1.5

        print("[6] 封锁 D")
        rig.send("D")

        m = metrics(rig.frames, args.rpm)
        if m is None:
            print("[FAIL] 有效帧不足（%d 帧）。可能是遥测被 ASCII 回显挤掉，"
                  "或死区让电机没动" % len(rig.frames))
            return 4

        print("\n" + "=" * 62)
        print("速度阶跃实测  给定 %+.1f rpm  增益 %dx  帧数 %d" % (args.rpm, args.gain, m["n"]))
        print("=" * 62)
        sgn = 1.0 if args.rpm > 0 else -1.0     # metrics 内部把反向统一成正转，显示时还原
        print("  稳态      %8.3f rpm   （误差 %+0.3f rpm）"
              % (m["ss"] * sgn, m["ss"] * sgn - args.rpm))
        print("  峰值      %8.3f rpm" % (m["peak"] * sgn))
        print("  上升时间  %8.1f ms   （10%%→90%%，100 Hz 采样，分辨率 10 ms）" % m["tr"])
        print("  超调      %8.1f %%" % m["os"])
        print("  调节时间  %8.1f ms   （±2%%）" % m["ts"])
        print("  稳态纹波  %8.3f rpm  （末 250 ms 标准差）" % m["ripple"])
        t_last = rig.frames[-1][0]
        tail_iq = [f[2] for f in rig.frames if f[0] > t_last - 0.25]
        iq_mean = sum(tail_iq) / len(tail_iq) if tail_iq else 0.0
        iq_peak = max(abs(f[2]) for f in rig.frames)
        vb = sum(f[3] for f in rig.frames) / max(1, len(rig.frames))
        print("  iq 稳态   %8.3f A    （末 250 ms 均值，含摩擦+损耗）" % iq_mean)
        print("  iq 峰值   %8.3f A    （限幅 3 A，峰值一般出现在起步瞬态）" % iq_peak)
        print("  母线电压  %8.2f V" % vb)

        if not args.no_wave:
            print()
            show_wave(rig.frames, m["t_step"])

        print("\n  判读：")
        if m["os"] > 40:
            print("   · 超调偏大：L 档位降一档（带宽高、噪声放大）或增大 iq 限幅")
        if m["ts"] > 300:
            print("   · 建立太慢：L 档位升一档（当前带宽对这台电机的惯量偏低）")
        if abs(m["ss"] * sgn - args.rpm) > 1.0:
            print("   · 稳态有偏差：检查 dir 符号（速度环成了正反馈的前兆）")
        if iq_peak > 2.9:
            print("   · iq 已到限幅：加速段被限流，tr 由限幅而非增益决定")
        ok = True
        return 0
    finally:
        try:
            rig.ser.write(b"D")
            rig.ser.flush()
        except Exception:
            pass
        rig.close()
        if not ok:
            print("\n[注意] 脚本异常退出，已自动补发 D")


if __name__ == "__main__":
    sys.exit(main())
