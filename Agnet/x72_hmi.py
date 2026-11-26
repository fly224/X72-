#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
x72_hmi.py —— X72 FOC 网页上位机的 Python 等价实现（协议层）

用途：把「伺服电机控制上位机 · 网页版」（Edge + Web Serial，单文件 HTML）里
      的帧编码/解码 1:1 移植到 Python，以便脚本化地随机下发指令并逐帧校验反馈。

依据：网页源码中的 buildFrame / mavCrc / decodeFrame / cmd* 函数。
      帧格式为 MAVLink v2（0xFD），crc_extra 见 CRC_EXTRA 表。

作者备注：
  * 30019 的 speed 字段单位是 **计数/秒**（不是 rad/s），acc 是 计数/秒²。
    网页里 RPM2PS = ENC_CPR/60。写错单位会导致"下发 100 rpm 实际 0.5 rpm"。
  * 30019 的 pos 字段是 **绝对多圈计数**（i64）。速度模式下必须填 0，
    否则固件会走位置分支把目标设成 0 rad（往零点冲）。
  * 30025 同一条 msgid 双向：下发 53 B，回执 5 B，遥测 61 B，按长度分流。
"""

import struct
import threading
import time
from collections import defaultdict

import serial

ENC_CPR = 2097152                 # MT6835 2^21 计数/圈
TAU = 6.283185307179586
KCNT = ENC_CPR / TAU              # 计数/rad
RAD2CNT = KCNT
CNT2RAD = 1.0 / KCNT
RPM2PS = ENC_CPR / 60.0           # rpm -> 计数/s
RPM2RAD = TAU / 60.0

# ---------------------------------------------------------------- 消息号 / CRC
MSG = {
    'CUR_IN': 30025, 'CUR_OUT': 30027, 'POS_SENSOR': 30040,
    'SPEED_CTL': 30019, 'VEL_PROFILE': 30089, 'SPEED_TRAJ': 30097,
    'CTRL_WORD': 30054, 'OP_MODE': 30056, 'PERMISSION': 30068,
    'SYS_CMD': 30069, 'IDENT_RESULT': 30092, 'DEBUG_PARAM': 30082,
    'LOOP_STATUS': 30091, 'COG_TABLE': 30093, 'TORQUE_CFG': 30095,
    'MIT_CMD': 30065, 'MIT_CFG': 30066, 'CTRL_STATUS': 30090,
    'PARAM_READ': 30000, 'POS_STATUS': 30100, 'RECORD': 30094,
    'ADV_CFG': 30096, 'ENC_COMP': 30098,
}

CRC_EXTRA = {
    30025: 191, 30027: 58, 30040: 160,
    30019: 8, 30054: 108, 30056: 26, 30068: 134, 30089: 89,
    30097: 97, 30069: 169, 30090: 90, 30092: 92, 30082: 218,
    30000: 43, 30091: 91, 30093: 93, 30100: 100, 30095: 95,
    30094: 94,   # 144B 录波，与 mavlink_common.c 的 s_meta 一致
    30096: 96,   # 24B 速度环陷波配置（含阶次跟踪）
    30065: 17, 30066: 147,
}

FAULT_BITS = [(0, 'OVERCURRENT'), (1, 'ENCODER'), (2, 'ADC_SYNC'),
              (3, 'BSP_INIT'), (4, 'SPI'), (5, 'OVERSPEED'),
              (6, 'OVERVOLT'), (7, 'UNDERVOLT'), (8, 'FOLLOW_ERR'),
              (9, 'PHASE_LOSS'), (10, 'CUR_SENSE')]

MODE_NAME = {0: 'NONE', 1: 'POS', 3: 'VEL', 4: 'TORQUE', 12: 'MIT'}

OPMODE_POS, OPMODE_VEL, OPMODE_TORQUE, OPMODE_MIT = 1, 3, 4, 12

# 30069 动作号（Board/servo_cmd.h）
ACT_SAVE_PARAM = 1
ACT_SOFT_RESET = 2
ACT_CLEAR_FAULT = 3
ACT_CAL_CURRENT = 4      # C  电流零点标定，200 ms，需停机
ACT_ALIGN_ZERO = 5       # Z  电角度对齐，约 22 s，需 cal=1
ACT_IDENT_FLUX = 6       # +  磁链 / Kt
ACT_IDENT_RL = 7         # @  R / L
ACT_ENC_CAL = 8          # ^  编码器线性度自校准
ACT_FRIC_2PT = 9         # $  摩擦两点标定
ACT_MECH_SWEEP = 10      # (  机械扫频 20..1000 Hz
ACT_MECH_FRIC = 11       # )  摩擦曲线 8 点
ACT_MECH_INERTIA = 12    # <  折算惯量
ACT_MECH_COG = 13        # }  齿槽转矩
ACT_ABORT = 14           # 中止所有辨识

# 30092 结果帧的 kind —— **与 30069 的动作号不是一套编号**，照 Board/mavlink_common.h
# 的 MAV_ID_* 抄。踩过：按动作号猜 kind，8 项辨识里 7 项永远等不到结果帧。
IDENT_KIND = {1: 'FLUX(psi/Kt)', 2: 'R/L', 3: '惯量', 4: '摩擦曲线',
              5: '齿槽', 6: '机械扫频', 7: '编码器自校准', 8: '电角度对齐',
              9: '摩擦两点', 10: 'CAN自检', 11: 'CAN链路测试'}


def fault_text(w):
    if not w:
        return '正常'
    return '|'.join(n for b, n in FAULT_BITS if w & (1 << b)) or ('0x%04X' % w)


# ---------------------------------------------------------------- CRC / 组帧
def _crc16(data, extra=0):
    """MAVLink CRC-16/MCRF4XX（与网页 mavCrc 完全一致）。"""
    crc = 0xFFFF
    for b in data:
        b ^= crc & 0xFF
        b ^= (b << 4) & 0xFF
        crc = (crc >> 8) ^ (((b << 8) ^ (b << 3) ^ (b >> 4)) & 0xFFFF)
    b = extra ^ (crc & 0xFF)
    b ^= (b << 4) & 0xFF
    crc = (crc >> 8) ^ (((b << 8) ^ (b << 3) ^ (b >> 4)) & 0xFFFF)
    return crc & 0xFFFF


_seq = 0


def build_frame(msgid, payload=b'', extra=None):
    global _seq
    if extra is None:
        extra = CRC_EXTRA.get(msgid, 0)
    body = bytes([len(payload), 0, 0, _seq & 0xFF, 1, 1,
                  msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF]) + payload
    _seq = (_seq + 1) & 0xFF
    crc = _crc16(body, extra)
    return b'\xFD' + body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


# ---------------------------------------------------------------- 载荷构造
def p_ctrl_word(on):
    """30054：使能 cw=0x000F / halt=0；失能 halt=1（固件里 halt 无条件封锁）。"""
    return struct.pack('<HB', 0x000F if on else 0x0000, 0 if on else 1)


def p_op_mode(mode):
    return bytes([mode & 0xFF])


P_PERMISSION = bytes([0, 0])          # comm_auth=0 → 主机持写权限


def p_sys_cmd(cmd):
    return bytes([cmd & 0xFF])


def p_speed_ctl(rpm=None, pos_rad=None, acc_rpm_s=0.0):
    """30019 PosSpeedCtlInput 44 B。
    pos_rad=None 且 rpm 给定 → 纯速度（pos 填 0，固件不置 s_p_valid）。
    pos_rad 给定           → 位置目标（绝对多圈计数）。"""
    p = bytearray(44)
    if pos_rad is not None:
        struct.pack_into('<q', p, 0, int(round(pos_rad * RAD2CNT)))
    if rpm is not None:
        struct.pack_into('<f', p, 28, rpm * RPM2PS)
    if acc_rpm_s:
        struct.pack_into('<f', p, 32, acc_rpm_s * RPM2PS)
    return bytes(p)


def p_vel_profile(is_scurve, jerk_rpm_s3=0.0):
    return struct.pack('<Bxxxf', 1 if is_scurve else 0, jerk_rpm_s3 * RPM2RAD)


def p_speed_traj(t_sec):
    """30097：cmd=1 写入到达时间（s）；cmd=0 只读回。"""
    p = bytearray(32)
    p[0] = 1
    struct.pack_into('<f', p, 4, t_sec)
    return bytes(p)


def p_speed_traj_read():
    return bytes(32)


def p_torque_cfg(id_a, ramp_ms):
    """30095：12 B，mask bit0=id / bit1=ramp。"""
    return struct.pack('<Bxxxff', 0x03, id_a, float(ramp_ms))


def p_cur_in(id_a, iq_a):
    """30025 下发（53 B）：id@0 / iq@4。"""
    p = bytearray(53)
    struct.pack_into('<f', p, 0, id_a)
    struct.pack_into('<f', p, 4, iq_a)
    return bytes(p)


def p_mit_cfg(kp, kd, kt=0.0910):
    return struct.pack('<ffff', kp, kd, 0.0, kt)


def p_mit_cmd(pos_rad, v_rad_s, tau_nm):
    """30065 MitCtlInput 32 B：pos_tar_p(i64 counts)@0 / pos_now_p@8 填0 /
    tq_set_NM@16 / iq_max@20 固件不解析 / speed_tar_p_s(counts/s)@24 / @28 填0。"""
    p = bytearray(32)
    struct.pack_into('<q', p, 0, int(round(pos_rad * RAD2CNT)))
    struct.pack_into('<f', p, 16, tau_nm)
    struct.pack_into('<f', p, 24, v_rad_s * RAD2CNT)
    return bytes(p)


def p_dbg_write_floats(obj, ints=None):
    """30082 写：float 槽 i -> @4i，掩码@64；int 槽 i -> @128+4i，掩码@124。"""
    p = bytearray(192)
    mask = 0
    for i, v in obj.items():
        struct.pack_into('<f', p, i * 4, float(v))
        mask |= (1 << i)
    struct.pack_into('<I', p, 64, mask & 0xFFFFFFFF)
    if ints:
        imask = 0
        for i, v in ints.items():
            struct.pack_into('<i', p, 128 + i * 4, int(v))
            imask |= (1 << i)
        struct.pack_into('<I', p, 124, imask & 0xFFFFFFFF)
    return bytes(p)


def p_dbg_read(struct_id=30082):
    return struct.pack('<I', struct_id)


# ---------------------------------------------------------------- 载荷解码
def _f(p, off):
    v = struct.unpack_from('<f', p, off)[0]
    # 固件里 float 字段未写时偶尔留下 denormal（如 0x80000001 = -1.4e-44），
    # 直接参与比较会得出荒谬结论。统一按 0 处理。
    if -1e-30 < v < 1e-30:
        return 0.0
    return v


def _q(p, off):
    return struct.unpack_from('<q', p, off)[0]


def dec_ctrl_status(p):
    if len(p) < 40:
        return None
    return {
        'cw': struct.unpack_from('<H', p, 0)[0],
        'mode': p[2],
        'flags': p[3],
        'armed': bool(p[3] & 0x04),
        'iqFb': _f(p, 4), 'iqRef': _f(p, 8), 'rpm': _f(p, 12),
        'theta_m': _f(p, 16), 'vbus': _f(p, 20),
        'motorTemp': _f(p, 24), 'temp': _f(p, 28), 'ktc': _f(p, 32),
        'fault': struct.unpack_from('<H', p, 36)[0],
        'txDrop': struct.unpack_from('<H', p, 38)[0],
        't': time.time(),
    }


def dec_loop_status(p):
    if len(p) < 48:
        return None
    return {
        'spdEn': p[0], 'src': p[1], 'sched': p[2], 'encDiag': p[3],
        'wref': _f(p, 4), 'wmeas': _f(p, 8),
        'iqc': _f(p, 12), 'iterm': _f(p, 16),
        'kp': _f(p, 20), 'acc': _f(p, 24), 'iqMax': _f(p, 28),
        'eoff_deg': _f(p, 32), 'psi_mWb': _f(p, 36),
        # +40/+44 在 2026-11-21 起改报过流现场，网页仍标成 kt/ld（已知不一致）。
        # 依据 Board/telemetry_tx.c:407-416。
        'ovcPeakA': _f(p, 40), 'ovcRawU': _f(p, 44),
        't': time.time(),
    }


def dec_pos_status(p):
    if len(p) < 48:
        return None
    return {
        'pNow': _f(p, 0), 'pTgt': _f(p, 4), 'pTgtC': _f(p, 8), 'pRef': _f(p, 12),
        'err': _f(p, 16), 'kp': _f(p, 20), 'vmax': _f(p, 24), 'errmax': _f(p, 28),
        'iTerm': _f(p, 32), 'travel': _f(p, 36),
        'faultCnt': struct.unpack_from('<I', p, 40)[0],
        'st': p[44], 'settled': p[45], 'compEn': p[46], 'posMode': p[47],
        't': time.time(),
    }


POS_ST = ['IDLE', 'MOVE', 'HOLD', 'DONE', 'ERR']


def dec_cur_in(p):
    """30025：5 B=回执，>=53 B=遥测，57/61 为后续扩展。"""
    if len(p) == 5:
        return {'ack': True, 'st': p[0], 'iq': _f(p, 1), 't': time.time()}
    if len(p) < 53:
        return None
    d = {
        'idTar': _f(p, 0), 'iqTar': _f(p, 4),
        'ia': _f(p, 8), 'ib': _f(p, 12), 'ic': _f(p, 16),
        'elecTheta': _f(p, 20), 'vbus': _f(p, 24),
        'uabcTar': (_f(p, 28), _f(p, 32), _f(p, 36)),
        'temp': _f(p, 40), 'elecOmega': _f(p, 44),
        'rpm': _f(p, 44) / 14.0 * 60.0 / TAU,
        'iqOffset': _f(p, 48), 'mode': p[52], 't': time.time(),
    }
    if len(p) >= 57:
        d['motorTemp'] = _f(p, 53)
    if len(p) >= 61:
        d['spdRef'] = _f(p, 57)
    return d


def dec_cur_out(p):
    if len(p) < 32:
        return None
    return {'id': _f(p, 12), 'iq': _f(p, 16), 't': time.time()}


def dec_pos_sensor(p):
    if len(p) < 12:
        return None
    s = _q(p, 0)
    frac = s % ENC_CPR
    return {'encSum': s, 'turns': s / ENC_CPR, 'pos_rad': frac / ENC_CPR * TAU,
            'elecTheta': _f(p, 8), 't': time.time()}


def dec_ident_result(p):
    if len(p) < 20:
        return None
    v = [_f(p, 4 + 4 * i) for i in range(8)]
    return {'kind': p[0], 'ok': p[1], 'n': struct.unpack_from('<H', p, 2)[0],
            'v': v, 'resid': _f(p, 36) if len(p) >= 40 else float('nan'),
            't': time.time()}


def dec_dbg_param(p):
    if len(p) == 2:
        return {'ack': True, 'st': p[0], 'n_ok': p[1], 't': time.time()}
    if len(p) < 192:
        return None
    return {'f': [_f(p, 4 * i) for i in range(16)],
            'iv': [struct.unpack_from('<i', p, 128 + 4 * i)[0] for i in range(16)],
            'nFloat': struct.unpack_from('<I', p, 68)[0],
            'nInt': struct.unpack_from('<I', p, 72)[0], 't': time.time()}


def dec_cog_table(p):
    if len(p) < 16:
        return None
    return {'op': p[0], 'idx': p[1], 'nBlk': p[2], 'flags': p[3],
            'pp': _f(p, 4), 'phase': _f(p, 8), 'zeroDev': _f(p, 12), 't': time.time()}


def dec_sys_cmd(p):
    """30069 应答 2 B：status + cmd。status!=0 表示动作被拒（没入队）。"""
    if len(p) == 2:
        return {'ack': True, 'st': p[0], 'cmd': p[1], 't': time.time()}
    return None


_DECODERS = {
    30090: dec_ctrl_status, 30091: dec_loop_status, 30100: dec_pos_status,
    30025: dec_cur_in, 30027: dec_cur_out, 30040: dec_pos_sensor,
    30092: dec_ident_result, 30082: dec_dbg_param, 30093: dec_cog_table,
    30069: dec_sys_cmd,
}


# ---------------------------------------------------------------- 链路
class Link:
    """串口 + 后台解析线程。last[msgid] 保存最近一次解码结果。"""

    def __init__(self, port='COM8', baud=921600):
        self.port = port
        self.baud = baud
        self.ser = None
        self.last = {}
        self.seq = defaultdict(list)      # msgid -> [(t, payload)]
        self.acks = defaultdict(list)     # 回执帧（30025 5B / 30082 2B / 30065 9B）
        self.n_rx = 0
        self.n_crc_bad = 0
        self.n_unknown = 0
        self.n_tx = 0
        self.n_write_err = 0              # 写失败次数（诊断 ST-Link VCP 健康度）
        self.n_reopen = 0                 # 自动重开串口次数
        self._dead_streak = 0             # 连续"重开也没救回来"的次数
        self._cooldown_until = 0.0        # 写方向熔断：冷却期内不再重开
        self._paused = False              # 烧录/复位期间暂停 reader
        self._buf = bytearray()
        self._alive = True
        self._lock = threading.Lock()
        self._wlock = threading.Lock()      # 保活线程与主线程都会写串口
        self._rlock = threading.Lock()      # ★ 重开串口必须互斥：两个线程同时
                                            #   close/open 同一个句柄会让 pyserial
                                            #   访问已释放的 HANDLE → 进程直接
                                            #   SIGSEGV（2026-11-23 首夜实测 exit 139）
        self._last_reopen = 0.0
        self.ser = self._open()
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()

    def write_dead(self):
        """写方向是否已判定失效（重开也救不回来）。上层据此暂停测试等他恢复。"""
        return self._dead_streak >= 20

    def pause(self):
        """烧录/复位 MCU 前调用：停 reader、关串口。

        必须做 —— `-rst` 会让 USB VCP 断电重枚举，reader 线程还拿着旧 HANDLE
        继续 read，正是段错误的另一个来源。"""
        self._paused = True
        s = self.ser
        self.ser = None
        try:
            if s is not None:
                s.close()
        except Exception:
            pass

    def resume(self):
        """烧录完成后调用：等 VCP 重新枚举再打开。"""
        self._paused = False
        for _ in range(8):
            time.sleep(0.4)
            if self._reopen(min_gap=0.0):
                return True
        return False

    def _open(self):
        """打开串口。**必须给 write_timeout**：不给的话 ser.write 在
        ST-Link VCP 卡死时会无限阻塞（实测卡 37 s 不返回），整个长测就僵住。"""
        s = serial.Serial(self.port, self.baud, timeout=0.02, write_timeout=0.35)
        s.dsrdtr = False
        s.rtscts = False
        try:
            s.reset_input_buffer()
        except Exception:
            pass
        return s

    def _reopen(self, min_gap=0.25):
        """ST-Link VCP 偶发 'Write timeout' / '设备不识别此命令(22)'。

        2026-11-23 实测：**close + open 即可恢复，不需要物理插拔**。
        这就是通宵长测能活下去的关键 —— 一次卡死不该让 12 小时的测试归零。

        两道保护：
          · `_rlock` 互斥：两个线程同时 close/open 会让 pyserial 操作已释放的
            HANDLE，进程直接 SIGSEGV（首夜 exit 139 的元凶之一）；
          · 冷却期：写方向真的死了时（如 ST-Link 固件挂死），重开一万次也没用，
            无脑重试只会刷屏并污染统计，所以连续失败 20 次后每分钟才试一次。
        """
        if time.time() < self._cooldown_until:
            return False
        if not self._rlock.acquire(blocking=False):
            return False                  # 别的线程正在重开，交给它
        try:
            if time.time() - self._last_reopen < min_gap:
                return False
            self._last_reopen = time.time()
            self.n_reopen += 1
            old = self.ser
            self.ser = None               # 先摘引用：reader 下一轮就会跳过
            try:
                if old is not None:
                    old.close()
            except Exception:
                pass
            for _ in range(3):
                try:
                    self.ser = self._open()
                    return True
                except Exception:
                    time.sleep(0.15)
            return False
        finally:
            self._rlock.release()

    def _note_write(self, ok):
        """记录一次写结果，维护"写方向是否死了"的熔断状态。"""
        if ok:
            self._dead_streak = 0
            return
        self._dead_streak += 1
        if self._dead_streak >= 20 and self._cooldown_until < time.time():
            self._cooldown_until = time.time() + 60.0
            self.n_dead_enter = getattr(self, 'n_dead_enter', 0) + 1

    def close(self):
        self._alive = False
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass

    def _reader(self):
        err_n = 0
        while self._alive:
            if self._paused:
                time.sleep(0.05)
                continue
            s = self.ser
            if s is None:
                time.sleep(0.02)
                continue
            try:
                chunk = s.read(4096)
                err_n = 0
            except Exception:
                # 软复位 / 重烧 / VCP 卡死 都会让读中断。不能一出错就退出线程，
                # 否则复位后链路看起来"活着但收不到帧"。连续失败就重开。
                if not self._alive:
                    break
                err_n += 1
                if err_n >= 5:
                    err_n = 0
                    self._reopen()
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            self._buf += chunk
            self._parse()

    def _parse(self):
        buf = self._buf
        while True:
            i = buf.find(b'\xFD')
            if i < 0:
                del buf[:]
                return
            if i:
                del buf[:i]
            if len(buf) < 12:
                return
            ln = buf[1]
            total = 12 + ln
            if len(buf) < total:
                return
            msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
            extra = CRC_EXTRA.get(msgid)
            if extra is None:
                # 未登记的消息没有 crc_extra，无法校验，直接跳过一字节重新同步
                self.n_unknown += 1
                del buf[:1]
                continue
            calc = _crc16(bytes(buf[1:10 + ln]), extra)
            got = buf[10 + ln] | (buf[11 + ln] << 8)
            if calc != got:
                self.n_crc_bad += 1
                del buf[:1]
                continue
            payload = bytes(buf[10:10 + ln])
            del buf[:total]
            self.n_rx += 1
            dec = _DECODERS.get(msgid)
            val = dec(payload) if dec else {'raw': payload}
            if val is None:
                continue
            # 回执帧（30025 的 5 B、30082 的 2 B）不能覆盖 last 里的周期遥测，
            # 否则读 iq 会读到一条 5 字节回执，字段全错。只进历史队列。
            if val.get('ack'):
                self.acks[msgid].append(val)
                if len(self.acks[msgid]) > 64:
                    del self.acks[msgid][:32]
                continue
            self.last[msgid] = val
            # 30090/30091 也要留历史：力矩动作要"窗口结束后回头扫帧"，
            # 不能靠阻塞式 get() 采样（会等新帧，350 ms 只拿到一两帧）。
            if msgid in (30092, 30082, 30090, 30091):
                self.seq[msgid].append(val)
                if len(self.seq[msgid]) > 400:
                    del self.seq[msgid][:200]

    # -- 发送 ------------------------------------------------------
    def send(self, msgid, payload=b''):
        """发一帧。**不抛异常**：写失败就自动重开串口重试一次，仍失败返回 False。

        为什么不能抛：通宵长测里一次 VCP 卡死就会让整个 soak 崩掉（实测 20:03
        那次跑到第 8 步就 Traceback 退出）。发送失败是一次链路抖动，不是逻辑错误，
        上层应该继续跑并靠判据发现"指令没生效"。
        """
        frame = build_frame(msgid, payload)
        with self._wlock:
            for attempt in (0, 1):
                s = self.ser
                if s is None:
                    if attempt == 0 and self._reopen():
                        continue
                    self._note_write(False)
                    return False
                try:
                    s.write(frame)
                    self.n_tx += 1
                    self._note_write(True)
                    return True
                except Exception:
                    self.n_write_err += 1
                    if attempt == 0 and self._reopen():
                        continue
                    self._note_write(False)
                    return False
        return False

    def get(self, msgid, max_age=0.5):
        d = self.last.get(msgid)
        if d and (time.time() - d.get('t', 0)) < max_age:
            return d
        return None

    # -- 语义封装（与网页按钮一一对应）-----------------------------
    def permission(self):
        self.send(MSG['PERMISSION'], P_PERMISSION)

    def op_mode(self, mode):
        self.send(MSG['OP_MODE'], p_op_mode(mode))

    def ctrl_word(self, on):
        self.send(MSG['CTRL_WORD'], p_ctrl_word(on))

    def sys_cmd(self, cmd):
        self.send(MSG['SYS_CMD'], p_sys_cmd(cmd))

    def speed_ctl(self, rpm=None, pos_rad=None, acc_rpm_s=0.0):
        self.send(MSG['SPEED_CTL'], p_speed_ctl(rpm, pos_rad, acc_rpm_s))

    def cur_in(self, id_a, iq_a):
        self.send(MSG['CUR_IN'], p_cur_in(id_a, iq_a))

    def torque_cfg(self, id_a, ramp_ms):
        self.send(MSG['TORQUE_CFG'], p_torque_cfg(id_a, ramp_ms))

    def mit_cfg(self, kp, kd):
        self.send(MSG['MIT_CFG'], p_mit_cfg(kp, kd))

    def mit_cmd(self, pos_rad, v_rad_s, tau_nm):
        self.send(MSG['MIT_CMD'], p_mit_cmd(pos_rad, v_rad_s, tau_nm))

    def traj(self, t_sec):
        self.send(MSG['SPEED_TRAJ'], p_speed_traj(t_sec))

    def traj_read(self):
        self.send(MSG['SPEED_TRAJ'], p_speed_traj_read())

    def vel_profile(self, is_scurve, jerk_rpm_s3=0.0):
        self.send(MSG['VEL_PROFILE'], p_vel_profile(is_scurve, jerk_rpm_s3))

    def dbg_read(self):
        self.send(MSG['PARAM_READ'], p_dbg_read())

    def dbg_write(self, floats=None, ints=None):
        self.send(MSG['DEBUG_PARAM'], p_dbg_write_floats(floats or {}, ints))


if __name__ == '__main__':
    import sys
    lk = Link(sys.argv[1] if len(sys.argv) > 1 else 'COM8')
    time.sleep(1.2)
    print('rx=%d tx=%d crc_bad=%d unknown=%d' % (lk.n_rx, lk.n_tx, lk.n_crc_bad, lk.n_unknown))
    for mid in (30090, 30091, 30100, 30025, 30040):
        d = lk.get(mid, 5.0)
        print('%-6d %s' % (mid, '无' if not d else {k: v for k, v in d.items() if k != 't'}))
    c = lk.get(30090, 5.0)
    if c:
        print('故障字 0x%04X = %s' % (c['fault'], fault_text(c['fault'])))
        print('模式 %d(%s) armed=%s' % (c['mode'], MODE_NAME.get(c['mode'], '?'), c['armed']))
    lk.close()
