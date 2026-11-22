# -*- coding: utf-8 -*-
"""
thermal_soak.py —— 速度模式恒速温漂长测（2026-11-19）

目的：让电机在速度闭环下恒定转速连续运行，测温度上升引起的参数漂移。

为什么要单独写：上位机 HTTP 链路只保留"最新一帧"，无法做 100 Hz 统计；
本脚本独占串口，自定界剥帧（不依赖协议表长度，能正确解析 61 B 的 30025 长帧），
单读线程同时喂遥测统计与 ACK 判定，避免两处 read 抢数据。

采集量
------
30025 CurrentCtlInput（61 B，100 Hz，设备→主机）
    +0  id_ref          d 轴电流给定 A
    +4  iq_ref          q 轴电流给定 A（限幅后）
    +24 vbus            母线电压 V
    +40 driver_temp     NTC_GAN 温度 °C（PA4，驱动/散热器）
    +44 w_elec          电角速度 rad/s（/14 → 机械 rad/s）
    +53 motor_temp      NTC_MOTOR 温度 °C（PC4，绕组探头）
    +57 spd_ref_rpm     斜坡后速度给定 rpm
30027 CurrentCtlOutput（32 B）  +16 iq_now 实测 iq A
30040 MotorPosSensorOutput（12 B） +0 enc_sum_p int64，用于窗口差分测速
30055 AppStatusInfo（8 B）状态字 bit0 使能 / bit1 故障 / bit[5:3] 实际在环模式

漂移指标
--------
1) 转速精度漂移：实测 rpm − 给定 rpm，随温度的变化
2) 电流需求漂移：同转速下 iq 均值变化（摩擦/铁损/铜阻变化的综合反映）
3) 相电阻 R：前后各一次 '@' 注入辨识（2 A），对照铜电阻 α=0.00393 /K 理论
4) 母线电压、驱动/电机双温度曲线

安全设计
--------
- 保活 0.4 s 重发 30019（固件 deadman 2 s；Servo_SetSpeedMode 只在 0→1 上升沿清积分，
  稳态重发只更新给定，不扰动积分）
- 驱动 >85 °C / 电机 >110 °C 记告警（固件降额起点）；>100 / >135 立即中止
- 故障位=1、通信静默 >3 s、连续 5 个采样窗无遥测 → 立即封锁中止
- 任何退出路径（含 Ctrl-C / 异常）都发 halt 封锁

用法
----
    python thermal_soak.py COM27 --rpm 1000 --minutes 60
    python thermal_soak.py COM27 --rpm 1000 --minutes 2      # 预演
    python thermal_soak.py COM27 --rpm 1000 --minutes 60 --no-rl   # 跳过 R 辨识
"""
import argparse
import csv
import json
import math
import os
import re
import struct
import sys
import threading
import time
from datetime import datetime

try:
    import serial
except ImportError:
    print("需要 pyserial: pip install pyserial")
    sys.exit(1)

# ---------------- 协议常量（与固件 mavlink_common.c 严格一致）----------------
STX = 0xFD
MSG_READ_PARAM = 30000
MSG_PS_IN      = 30019
MSG_CUR_IN     = 30025
MSG_CUR_OUT    = 30027
MSG_POS_SENSOR = 30040
MSG_CTRL_WORD  = 30054
MSG_STATUS     = 30055
MSG_OP_MODE    = 30056
MSG_PERMISSION = 30068
MSG_SYS_CMD    = 30069

# crc_extra 取自固件 mavlink_common.c 的 s_meta 表
CRC_EXTRA = {
    MSG_READ_PARAM: 43, MSG_PS_IN: 8, MSG_CUR_IN: 191, MSG_CUR_OUT: 58,
    MSG_POS_SENSOR: 160, MSG_CTRL_WORD: 108, MSG_STATUS: 3,
    MSG_OP_MODE: 26, MSG_PERMISSION: 134, MSG_SYS_CMD: 169,
}
ACK_IDS = (MSG_PS_IN, MSG_CTRL_WORD, MSG_OP_MODE, MSG_PERMISSION, MSG_SYS_CMD)

ENC_COUNTS = 2097152.0            # MT6835 21 bit
POLE_PAIRS = 14.0
TWO_PI = 6.283185307179586
RAD_S_TO_RPM = 60.0 / TWO_PI

ST_NAME = {0: "SUCCESS", 1: "READ_ONLY", 2: "OUT_OF_RANGE",
           3: "WRITE_STATE_ERROR", 4: "NO_AUTHORITY"}


class SetupAbort(Exception):
    """启动阶段某项前置条件不满足，直接中止（不进入长跑循环）。"""

# 固件降额起点 / 本脚本硬中止线
WARN_DRV, WARN_MOT = 85.0, 110.0
STOP_DRV, STOP_MOT = 100.0, 135.0


# ---------------- CRC / 组帧 ----------------
def x25_crc(data, extra):
    """CRC-16/MCRF4XX，覆盖 LEN..payload（不含 0xFD），末追加 crc_extra。
    与固件 MavCrc_Calc() 逐位一致。"""
    crc = 0xFFFF
    for b in data:
        t = (b ^ (crc & 0xFF)) & 0xFF
        t ^= (t << 4) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    t = (extra ^ (crc & 0xFF)) & 0xFF
    t ^= (t << 4) & 0xFF
    crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


def build_frame(msgid, payload, seq, sysid=1, compid=1):
    ln = len(payload)
    head = bytes([ln, 0, 0, seq & 0xFF, sysid, compid]) + (msgid & 0xFFFFFF).to_bytes(3, "little")
    crc = x25_crc(head + payload, CRC_EXTRA[msgid])
    return bytes([STX]) + head + payload + struct.pack("<H", crc)


def strip_frames(raw):
    """剥掉完整二进制帧，只留 ASCII（R 辨识回显里混着 100 Hz 遥测）。"""
    out = bytearray()
    i, n = 0, len(raw)
    while i < n:
        if raw[i] != STX or i + 12 > n:
            out.append(raw[i]); i += 1; continue
        total = 12 + raw[i + 1]
        if i + total <= n:
            i += total
        else:
            out.append(raw[i]); i += 1
    return bytes(out)


def _f(b, off):
    return struct.unpack_from("<f", b, off)[0]


# ---------------- 单读线程：剥帧 + 分派 ----------------
class Rx(threading.Thread):
    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.lock = threading.Lock()
        self.buf = bytearray()
        self.stop_evt = threading.Event()
        self.pause_evt = threading.Event()   # set = 暂停读取，把串口让给 ASCII 交互
        self.frames = 0
        self.crc_err = 0
        self.snap = {}          # 最新 30025 解析结果
        self.snap_t = 0.0
        self.iq_now = float("nan")
        self.enc = None
        self.enc_t = 0.0
        self.sw = None
        self.ew = None
        self.sw_t = 0.0
        self.ack = {}           # msgid -> (status, ts)
        self.last_rx = 0.0

    def run(self):
        err = 0
        while not self.stop_evt.is_set():
            if self.pause_evt.is_set():
                time.sleep(0.01)
                continue
            try:
                d = self.ser.read(8192)
                err = 0
            except Exception:
                # 关键：这里**不能**直接 break。
                # USB 串口偶发 "device reports readiness to read but returned
                # no data"（Windows + CDC/CH340 常见），只要出现一次，接收线程
                # 就永久死亡；上层看到的是"遥测静默"，长测被判 link_silent 中止
                # （2026-11-19 30 组长测在 15 min 处正是这样挂掉的）。
                # 改为计数重试：连续 ~10 s 都失败才认为口真的没了。
                err += 1
                if err > 400:
                    break
                time.sleep(0.025)
                continue
            if not d:
                continue
            self.last_rx = time.time()
            self.buf += d
            self._scan()

    def _scan(self):
        buf = self.buf
        i, n = 0, len(buf)
        while i + 12 <= n:
            if buf[i] != STX:
                i += 1
                continue
            ln = buf[i + 1]
            total = 12 + ln
            if i + total > n:
                break                      # 残帧，留到下一批
            mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            b = i + 10
            ex = CRC_EXTRA.get(mid)
            if ex is not None:
                crc = buf[b + ln] | (buf[b + ln + 1] << 8)
                if x25_crc(bytes(buf[i + 1:b + ln]), ex) != crc:
                    self.crc_err += 1
                    i += 1
                    continue
                self.frames += 1
                self._dispatch(mid, bytes(buf[b:b + ln]))
            i += total
        del buf[:i]

    def _dispatch(self, mid, pl):
        now = time.time()
        with self.lock:
            if mid == MSG_CUR_IN and len(pl) >= 61:
                self.snap = {
                    "id_ref": _f(pl, 0), "iq_ref": _f(pl, 4),
                    "vbus": _f(pl, 24),
                    "temp_drv": _f(pl, 40), "w_elec": _f(pl, 44),
                    "mode": pl[52],
                    "temp_mot": _f(pl, 53), "spd_ref_rpm": _f(pl, 57),
                }
                self.snap_t = now
            elif mid == MSG_CUR_OUT and len(pl) >= 20:
                self.iq_now = _f(pl, 16)
            elif mid == MSG_POS_SENSOR and len(pl) >= 8:
                self.enc = struct.unpack_from("<q", pl, 0)[0]
                self.enc_t = now
            elif mid == MSG_STATUS and len(pl) >= 8:
                self.sw, self.ew = struct.unpack_from("<II", pl, 0)
                self.sw_t = now
            elif mid in ACK_IDS and pl:
                self.ack[mid] = (pl[0], now)
                if mid == MSG_CTRL_WORD and len(pl) >= 2:
                    self.ack[("cw", "back")] = (pl[1], now)

    def get_snap(self):
        with self.lock:
            return dict(self.snap), self.snap_t, self.iq_now, self.enc, self.enc_t

    def get_status(self):
        with self.lock:
            return self.sw, self.ew, self.sw_t

    def get_ack(self, mid):
        with self.lock:
            return self.ack.get(mid)


# ---------------- 设备会话 ----------------
class Rig:
    def __init__(self, port, baud=921600):
        # write_timeout 必须显式给：默认 None 时并发写会让 pyserial 内部状态卡住，
        # 表现为 SerialTimeoutException('Write timeout') 把长测打断（2026-11-19 实测）。
        self.s = serial.Serial(port, baud, timeout=0.05, write_timeout=0.5)
        self.seq = 0
        self.wlock = threading.Lock()      # 保活线程与主循环共用串口，必须串行化
        self.rx = Rx(self.s)
        self.rx.start()

    def send(self, mid, payload):
        payload = bytes(payload)
        with self.wlock:
            f = build_frame(mid, payload, self.seq)
            self.seq = (self.seq + 1) & 0xFF
            self.s.write(f)
            self.s.flush()

    def send_ps(self, rpm, pos_pulse=0, acc_pss=0.0):
        """30019：pos_tar_p@0(i64) / speed_tar_p_s@28(f32) / acc_tar_p_ss@32(f32)。
        必须发满 44 B：固件 dispatch 直接读 pl[32]，短帧会读到缓冲残留。"""
        b = bytearray(44)
        struct.pack_into("<q", b, 0, int(pos_pulse))
        struct.pack_into("<f", b, 28, rpm / 60.0 * ENC_COUNTS)
        struct.pack_into("<f", b, 32, float(acc_pss))
        self.send(MSG_PS_IN, b)

    def wait_ack(self, mid, timeout=0.6):
        t0 = time.time()
        while time.time() - t0 < timeout:
            a = self.rx.get_ack(mid)
            if a and a[1] >= t0 - 0.05:
                return a
            time.sleep(0.01)
        return None

    def read_status(self, timeout=0.6):
        self.send(MSG_READ_PARAM, struct.pack("<I", MSG_STATUS))
        time.sleep(0.05)
        t0 = time.time()
        while time.time() - t0 < timeout:
            sw, ew, t = self.rx.get_status()
            if sw is not None and t >= t0 - 0.05:
                return sw, ew
            time.sleep(0.01)
        return None, None

    def rl_ident(self, timeout=4.0):
        """ASCII '@'：2 A 注入辨识 R/L。要求已 disarm 且转子静止。

        必须暂停读线程独占串口：两个线程同时 read 同一端口会把 '@' 的
        ASCII 回显抢走（回显与 100 Hz 遥测混流），导致辨识明明成功却读不到 R。
        """
        self.rx.pause_evt.set()
        time.sleep(0.08)              # 让读线程从当前 read 返回并让出串口
        try:
            try:
                with self.wlock:
                    try:
                        self.s.reset_input_buffer()
                    except Exception:
                        pass
                    self.s.write(b"@")
                    self.s.flush()
            except Exception as e:
                self.rx.pause_evt.clear()
                return {"r_mohm": None, "l_uh": None, "raw": "write_failed:%r" % e}
            t0 = time.time()
            raw = bytearray()
            while time.time() - t0 < timeout:
                d = self.s.read(4096)
                if d:
                    raw += d
                txt = strip_frames(raw).decode("ascii", "replace")
                if ("RL ok" in txt) or ("RL FAIL" in txt) or ("RL refused" in txt):
                    break
                time.sleep(0.02)
            txt = strip_frames(raw).decode("ascii", "replace")
            m = re.search(r"RL ok R=([\d.]+)mohm L=([\d.]+)uH", txt)
            if m:
                return {"r_mohm": float(m.group(1)), "l_uh": float(m.group(2)), "raw": txt.strip()}
            return {"r_mohm": None, "l_uh": None, "raw": txt.strip()[-200:]}
        finally:
            try:
                self.s.reset_input_buffer()   # 丢掉辨识残留，避免半帧污染扫描器
            except Exception:
                pass
            self.rx.buf = bytearray()
            self.rx.pause_evt.clear()

    def close(self):
        try:
            self.rx.stop_evt.set()
        except Exception:
            pass
        try:
            self.s.close()
        except Exception:
            pass


def sw_parse(sw):
    if sw is None:
        return None, None, None
    return sw & 0x01, (sw >> 1) & 0x01, (sw >> 3) & 0x0F


# ---------------- 主流程 ----------------
def main():
    ap = argparse.ArgumentParser(description="速度模式恒速温漂长测")
    ap.add_argument("port", help="串口，如 COM27")
    ap.add_argument("--rpm", type=float, default=1000.0, help="目标转速，默认 1000")
    ap.add_argument("--minutes", type=float, default=60.0, help="时长分钟，默认 60")
    ap.add_argument("--sample", type=float, default=10.0, help="采样周期 s，默认 10")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--acc", type=float, default=400.0, help="加速度 rad/s²，默认 400（固件默认）")
    ap.add_argument("--no-rl", action="store_true", help="跳过前后相电阻辨识")
    ap.add_argument("--out", default=None, help="CSV 路径，默认 Agnet/thermal_<rpm>_<时间>.csv")
    args = ap.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_csv = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "thermal_%drpm_%s.csv" % (int(round(args.rpm)), stamp))
    rows = []
    aborted = {"why": None}
    warn = set()

    print("=" * 72)
    print("速度模式温漂长测  目标 %.0f rpm（%.2f rad/s）  时长 %.1f min  采样 %.0f s"
          % (args.rpm, args.rpm / RAD_S_TO_RPM, args.minutes, args.sample))
    print("CSV: %s" % out_csv)
    print("=" * 72)

    rig = Rig(args.port, args.baud)
    ka_stop = threading.Event()

    def send_ack(mid, payload, name):
        rig.send(mid, payload)
        a = rig.wait_ack(mid, 1.0)
        if a is None:
            print("  %-22s ACK 超时" % name)
            return None
        print("  %-22s ACK=%s" % (name, ST_NAME.get(a[0], a[0])))
        return a[0]

    try:
        # 0) 等遥测
        t0 = time.time()
        while time.time() - t0 < 3.0:
            snap, st, _, _, _ = rig.rx.get_snap()
            if st > 0:
                break
            time.sleep(0.05)
        snap, st, _, _, _ = rig.rx.get_snap()
        if st == 0:
            print("[ERR] 3 s 内没有收到 30025 遥测，检查端口/波特率/固件")
            rig.close()
            return 1
        print("[ok] 遥测已到：Tdrv=%.1f°C Tmot=%.1f°C vbus=%.1f V"
              % (snap["temp_drv"], snap["temp_mot"], snap["vbus"]))

        sw, ew = rig.read_status()
        en, flt, act = sw_parse(sw)
        print("[ok] 初始状态：使能=%s 故障=%s 在环=%s err=0x%04X"
              % (en, flt, act, ew or 0))

        # 1) 权限（拿不到控制权时后续写入全被拒，必须在这里判死）
        st_perm = send_ack(MSG_PERMISSION, bytes([0, 0]), "30068 权限 HOST")
        if st_perm != 0:
            raise SetupAbort("permission_status=%s" % st_perm)

        # 2) 清故障（有故障才清，避免每次上电都写 Flash 无关状态）
        if flt:
            send_ack(MSG_SYS_CMD, bytes([3]), "30069 清故障")
            time.sleep(0.2)

        # 3) 确保封锁（R 辨识要求 D 状态）
        send_ack(MSG_CTRL_WORD, bytes([0, 0, 1]), "30054 halt 封锁")
        time.sleep(0.8)

        # 4) 冷态 R 辨识
        r_cold, t_cold = None, None
        if not args.no_rl:
            snap, _, _, _, _ = rig.rx.get_snap()
            t_cold = (snap.get("temp_mot"), snap.get("temp_drv"))
            print("[..] 冷态 R 辨识（'@' 2 A 注入）…")
            rl = rig.rl_ident()
            r_cold = rl["r_mohm"]
            print("     冷态: %s" % (rl["raw"] or "(无回显)"))
            if r_cold:
                print("     R=%.1f mΩ  Tdrv=%.1f°C Tmot=%.1f°C"
                      % (r_cold, t_cold[1] or float('nan'), t_cold[0] or float('nan')))
            time.sleep(0.3)

        # 5) 选速度模式
        st_mode = send_ack(MSG_OP_MODE, bytes([3]), "30056 mode=3 速度")
        if st_mode != 0:
            raise SetupAbort("op_mode_status=%s" % st_mode)

        # 6) 加速度旁带：rad/s² → 脉冲/s²（固件 0 表示"不改"，非 0 才写）
        acc_pss = args.acc / TWO_PI * ENC_COUNTS

        # 7) 先在 0 rpm 建立速度环，确认在环后再升速。
        #    关键：do_speed() 的未使能分支会直接开 MOE 并把 s_armed 置 1，
        #    即第一次 30019 就已经通电。保活线程必须在它之前起来，否则
        #    2 s deadman 会在"确认在环"的阶段把环封掉，造成误判。
        ka_rpm = [0.0]

        def keepalive():
            while not ka_stop.is_set():
                try:
                    rig.send_ps(ka_rpm[0], acc_pss=acc_pss)
                except Exception:
                    pass
                ka_stop.wait(0.4)

        threading.Thread(target=keepalive, daemon=True).start()
        print("[ok] 保活已启动 0.4 s / 次（固件 deadman 2 s）")

        rig.send_ps(0.0, acc_pss=acc_pss)      # 建立速度环，给定 0
        time.sleep(0.5)
        send_ack(MSG_CTRL_WORD, bytes([1, 0, 0]), "30054 使能 cw=1")
        time.sleep(0.3)

        sw, ew = rig.read_status()
        en, flt, act = sw_parse(sw)
        print("[ok] 使能后：使能=%s 故障=%s 实际在环=%s" % (en, flt, act))
        if act != 3:
            print("[ERR] 实际在环模式不是 3（速度），中止以免在未知环里长跑")
            aborted["why"] = "active_mode=%s" % act
        else:
            # 8) 升到目标转速
            ka_rpm[0] = args.rpm
            rig.send_ps(args.rpm, acc_pss=acc_pss)
            print("[ok] 升速到 %.0f rpm（加速度 %.0f rad/s²）" % (args.rpm, args.acc))

    except SetupAbort as e:
        print("[ERR] 启动中止: %s" % e)
        aborted["why"] = str(e)
    except Exception as e:
        print("[ERR] 启动阶段异常: %r" % e)
        aborted["why"] = "setup_exception"

    if aborted["why"] is None:
        # 9) 采样循环（保活线程已在升速前启动）
        t_start = time.time()
        t_end = t_start + args.minutes * 60.0
        next_out = t_start
        sample_no = 0
        acc = {}
        enc_win = [None, None, None, None]   # enc0, t0, enc1, t1

        def push(k, v):
            if v is None or v != v:
                return
            a = acc.get(k)
            if a is None:
                acc[k] = [v, v, v, 1]        # sum, min, max, n
            else:
                a[0] += v
                if v < a[1]: a[1] = v
                if v > a[2]: a[2] = v
                a[3] += 1

        print("\n%-8s %-9s %-9s %-9s %-8s %-8s %-7s" %
              ("t(min)", "Tdrv(°C)", "Tmot(°C)", "rpm", "err", "iq(A)", "vbus"))
        print("-" * 72)

        last_status_poll = 0.0
        last_snap_t = 0.0

        try:
            while time.time() < t_end:
                time.sleep(0.2)
                now = time.time()

                # 安全：通信静默。判据 10 s（同 full_soak）：接收线程已改为异常
                # 重试，短暂 USB 抖动不会致死，3 s 太紧会误杀长测。
                if rig.rx.last_rx and (now - rig.rx.last_rx) > 10.0:
                    aborted["why"] = "link_silent_%.1fs" % (now - rig.rx.last_rx)
                    break

                # 安全：状态字轮询 2 s 一次
                if now - last_status_poll > 2.0:
                    last_status_poll = now
                    try:
                        rig.send(MSG_READ_PARAM, struct.pack("<I", MSG_STATUS))
                    except Exception:
                        pass      # 单次写失败不中断长测；真断链由静默检测兜底
                    sw, ew, st = rig.rx.get_status()
                    en, flt, act = sw_parse(sw)
                    if flt == 1:
                        aborted["why"] = "fault_bit_set err=0x%04X" % (ew or 0)
                        break
                    if act is not None and act != 3:
                        aborted["why"] = "left_speed_ring active=%s" % act
                        break

                # 累积遥测
                snap, st, iq_now, enc, enc_t = rig.rx.get_snap()
                if st > last_snap_t:
                    last_snap_t = st
                    push("temp_drv", snap.get("temp_drv"))
                    push("temp_mot", snap.get("temp_mot"))
                    push("spd_ref_rpm", snap.get("spd_ref_rpm"))
                    w = snap.get("w_elec")
                    push("rpm", None if w is None else w / POLE_PAIRS * RAD_S_TO_RPM)
                    push("iq_ref", snap.get("iq_ref"))
                    push("id_ref", snap.get("id_ref"))
                    push("vbus", snap.get("vbus"))
                    if iq_now == iq_now:
                        push("iq_now", iq_now)

                # 温度告警
                a = acc.get("temp_drv")
                if a and a[2] >= WARN_DRV and "drv" not in warn:
                    warn.add("drv")
                    print("  [WARN] 驱动温度已达 %.1f°C（固件 %.0f°C 起降额）" % (a[2], WARN_DRV))
                a = acc.get("temp_mot")
                if a and a[2] >= WARN_MOT and "mot" not in warn:
                    warn.add("mot")
                    print("  [WARN] 电机温度已达 %.1f°C（固件 %.0f°C 起降额）" % (a[2], WARN_MOT))
                if a and a[2] >= STOP_MOT:
                    aborted["why"] = "motor_temp_%.0fC" % a[2]
                    break
                a = acc.get("temp_drv")
                if a and a[2] >= STOP_DRV:
                    aborted["why"] = "driver_temp_%.0fC" % a[2]
                    break

                # 出采样
                if now >= next_out:
                    next_out += args.sample
                    sample_no += 1

                    # 编码器窗口差分转速
                    rpm_enc = None
                    if enc is not None:
                        if enc_win[0] is None:
                            enc_win[0], enc_win[1] = enc, enc_t
                        else:
                            enc_win[2], enc_win[3] = enc, enc_t
                            dt = enc_win[3] - enc_win[1]
                            if dt > 0.5:
                                de = enc_win[2] - enc_win[0]
                                rpm_enc = (de / ENC_COUNTS) / dt * 60.0
                                enc_win[0], enc_win[1] = enc, enc_t

                    def mean(k):
                        a = acc.get(k)
                        return (a[0] / a[3]) if a and a[3] else float("nan")

                    t_rel = now - t_start
                    rpm_mean = mean("rpm")
                    ref_mean = mean("spd_ref_rpm")
                    row = {
                        "t_s": round(t_rel, 1),
                        "iso": datetime.now().strftime("%H:%M:%S"),
                        "temp_drv": round(mean("temp_drv"), 2),
                        "temp_drv_min": round(acc["temp_drv"][1], 2) if "temp_drv" in acc else "",
                        "temp_drv_max": round(acc["temp_drv"][2], 2) if "temp_drv" in acc else "",
                        "temp_mot": round(mean("temp_mot"), 2),
                        "temp_mot_max": round(acc["temp_mot"][2], 2) if "temp_mot" in acc else "",
                        "spd_ref_rpm": round(ref_mean, 1),
                        "rpm_meas": round(rpm_mean, 1),
                        "rpm_enc": "" if rpm_enc is None else round(rpm_enc, 1),
                        "err_rpm": round(rpm_mean - ref_mean, 2),
                        "iq_ref": round(mean("iq_ref"), 4),
                        "iq_now": round(mean("iq_now"), 4),
                        "id_ref": round(mean("id_ref"), 4),
                        "vbus": round(mean("vbus"), 2),
                        "active": act, "en": en, "fault": flt,
                        "frames": rig.rx.frames, "crc_err": rig.rx.crc_err,
                    }
                    rows.append(row)
                    with open(out_csv, "w", newline="", encoding="utf-8-sig") as f:
                        w_ = csv.DictWriter(f, fieldnames=list(row.keys()))
                        w_.writeheader()
                        w_.writerows(rows)

                    print("%-8.1f %-9.2f %-9.2f %-9.1f %-8.2f %-8.4f %-7.1f"
                          % (t_rel / 60.0, row["temp_drv"], row["temp_mot"],
                             row["rpm_meas"], row["err_rpm"], row["iq_ref"], row["vbus"]))
                    acc = {}

        except KeyboardInterrupt:
            aborted["why"] = "ctrl_c"
        except Exception as e:
            aborted["why"] = "loop_exception_%r" % e

        # 9) 收尾：停保活 → 主动减速 → halt
        ka_stop.set()
        time.sleep(0.6)
        try:
            rig.send_ps(0.0)
            t0 = time.time()
            while time.time() - t0 < 6.0:
                snap, st, _, _, _ = rig.rx.get_snap()
                w = snap.get("w_elec")
                rpm_now = 0.0 if w is None else abs(w) / POLE_PAIRS * RAD_S_TO_RPM
                if rpm_now < 5.0:
                    break
                time.sleep(0.1)
            rig.send(MSG_CTRL_WORD, bytes([0, 0, 1]))
            # 封锁后转子自由滑行：R 辨识要求静止，否则转动反电势会污染 R 测量。
            # 1000 rpm 空载滑行可能持续数秒，必须实测等到停稳而不是固定 sleep。
            t1 = time.time()
            while time.time() - t1 < 8.0:
                snap, _, _, _, _ = rig.rx.get_snap()
                w = snap.get("w_elec")
                if w is None or abs(w) / POLE_PAIRS * RAD_S_TO_RPM < 3.0:
                    break
                time.sleep(0.15)
            time.sleep(0.3)
        except Exception:
            pass

        # 10) 热态 R 辨识
        r_hot, t_hot = None, None
        if not args.no_rl:
            snap, _, _, _, _ = rig.rx.get_snap()
            t_hot = (snap.get("temp_mot"), snap.get("temp_drv"))
            print("\n[..] 热态 R 辨识（'@' 2 A 注入）…")
            rl = rig.rl_ident()
            r_hot = rl["r_mohm"]
            print("     热态: %s" % (rl["raw"] or "(无回显)"))

        # 11) 汇总
        summary = {
            "csv": out_csv,
            "rpm_cmd": args.rpm,
            "minutes": args.minutes,
            "samples": len(rows),
            "aborted": aborted["why"],
            "r_cold_mohm": r_cold, "r_hot_mohm": r_hot,
            "t_cold_mot": t_cold[0] if t_cold else None,
            "t_hot_mot": t_hot[0] if t_hot else None,
            "t_cold_drv": t_cold[1] if t_cold else None,
            "t_hot_drv": t_hot[1] if t_hot else None,
            "frames": rig.rx.frames, "crc_err": rig.rx.crc_err,
        }
        if rows:
            summary["rpm_min"] = min(r["rpm_meas"] for r in rows)
            summary["rpm_max"] = max(r["rpm_meas"] for r in rows)
            summary["err_first"] = rows[0]["err_rpm"]
            summary["err_last"] = rows[-1]["err_rpm"]
            summary["temp_drv_first"] = rows[0]["temp_drv"]
            summary["temp_drv_last"] = rows[-1]["temp_drv"]
            summary["temp_mot_first"] = rows[0]["temp_mot"]
            summary["temp_mot_last"] = rows[-1]["temp_mot"]
            summary["iq_first"] = rows[0]["iq_ref"]
            summary["iq_last"] = rows[-1]["iq_ref"]

        print("\n" + "=" * 72)
        print("汇总")
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        print("=" * 72)
        rig.close()
        return 0 if aborted["why"] is None else 2

    else:
        # 启动阶段已判失败：**先停保活再封锁**。
        # 顺序不能反：保活线程持续发 30019，而固件 do_speed() 在 s_armed=0 时
        # 会重新使能并开 MOE，只发 halt 会被下一帧 30019 立刻拉起来。
        ka_stop.set()
        time.sleep(0.6)
        try:
            rig.send(MSG_CTRL_WORD, bytes([0, 0, 1]))
            time.sleep(0.3)
        except Exception:
            pass
        rig.close()
        print("[ABORT] %s" % aborted["why"])
        return 2


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except KeyboardInterrupt:
        code = 130
    sys.exit(code)
