# -*- coding: utf-8 -*-
"""
mavlink_mode_test.py —— 4 控制模式 MAVLink 指令集实测（2026-11-18）

验证 4 个模式纯靠 MAVLink 指令完成：选模式(30056) → 给定(30025/30019/30065/30066)
→ 固件自动装配环 → 遥测确认真实在环。每步读 ACK 回读 + S 状态行 + 遥测转速。

模式映射：
    力矩  mode=4  → 30025 idq_tar_A[1] (iq A)
    速度  mode=3  → 30019 speed_tar_p_s (p/s)，实测 rpm
    位置  mode=1  → 30019 pos_tar_p (脉冲)，实测位置
    MIT   mode=12 → 30066 kp/kd + 30065 pos/speed/tq

安全：脚本退出一律发 D 封锁（经 MAVLink halt + ASCII D 双保险）。

用法：
    python mavlink_mode_test.py COM27              # 全部 4 模式（真实通电，谨慎）
    python mavlink_mode_test.py COM27 --mode 4     # 只测力矩
    python mavlink_mode_test.py COM27 --dry-run    # 只测门卫/拒写，不通电
"""
import sys, os, time, struct, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.join(HERE, "..", "..", "驱动开发", "PG_X72_CAN", "host")
sys.path.insert(0, HOST)
sys.path.insert(0, HERE)
import mavlink_proto as mp
from cog_ff_test import Rig
from mit_sweep import scan

try:
    import serial
except ImportError:
    print("需要 pyserial: pip install pyserial"); sys.exit(1)

READ_PARAM = 30000
CTRL_WORD  = 30054
OP_MODE    = 30056
PERMISSION = 30068
SYS_CMD    = 30069
STATUS_INFO= 30055
CUR_IN     = 30025
PSSPD_IN   = 30019
MIT_IN     = 30065
MIT_CFG    = 30066

ENC_COUNTS = 2097152.0
TWO_PI     = 6.283185307179586
RAD2RPM    = 60.0 / TWO_PI

ST_NAME = {0:"SUCCESS", 1:"READ_ONLY", 2:"OUT_OF_RANGE",
           3:"WRITE_STATE_ERROR", 4:"NO_AUTHORITY"}
HOST_SYS, HOST_COMP = 1, 1


def strip_mavframes(raw):
    """从字节流里剥掉完整 MAVLink v2 帧（0xFD + LEN + ... + CRC），
    保留其余 ASCII。帧内 LEN 来自 buf[1]，整帧长 = 10 + LEN + 2。"""
    out = bytearray()
    i = 0
    n = len(raw)
    while i < n:
        if raw[i] != 0xFD or i + 12 > n:
            out.append(raw[i]); i += 1; continue
        plen = raw[i + 1]
        total = 10 + plen + 2
        if i + total <= n:
            i += total                      # 整帧跳过
        else:
            out.append(raw[i]); i += 1      # 残帧：当噪声留一个字节
    return bytes(out)


class Link:
    def __init__(self, port, baud=921600):
        self.s = serial.Serial(port, baud, timeout=0.02)
        self.parser = mp.FrameParser()
        self.seq = 0

    def send(self, msgid, payload):
        f = mp.encode(msgid, bytes(payload), seq=self.seq,
                      sysid=HOST_SYS, compid=HOST_COMP)
        self.seq = (self.seq + 1) & 0xFF
        # 不 reset_input_buffer：固件持续发 100 Hz 遥测，reset 会截断在途帧导致失步/CRC 错。
        # FrameParser 会自行跳过非 0xFD 噪声与坏帧。
        self.s.write(f); self.s.flush()
        return f

    def wait_ack(self, msgid, timeout=0.35):
        t0 = time.time()
        while time.time() - t0 < timeout:
            d = self.s.read(4096)
            if d:
                for fr in (self.parser.feed(d) or []):   # FrameParser 自维护 buf，直接喂新字节
                    if fr.get("msgid") == msgid:
                        return fr
        return None

    def ascii_cmd(self, cmd, wait=0.25):
        """发 ASCII 命令并读回显。回显里混着 100 Hz 二进制遥测帧，
        按帧长字段把 0xFD 帧整段剥掉，只留 ASCII —— 否则 'mit T=.. p=..'
        会被遥测帧从中间插入打断。"""
        self.s.reset_input_buffer()
        self.s.write(cmd.encode() if isinstance(cmd, str) else cmd)
        self.s.flush()
        time.sleep(wait)
        raw = bytearray(); t0 = time.time()
        while time.time() - t0 < 0.55:
            d = self.s.read(4096)
            if d: raw += d
        return strip_mavframes(raw).decode('ascii', 'replace')

    def close(self):
        self.s.close()


def mode_payload(mode):
    lk.send(OP_MODE, [mode])
    fr = lk.wait_ack(OP_MODE)
    st = fr["payload"][0] if fr else -1
    back = fr["payload"][1] if fr and len(fr["payload"]) > 1 else -1
    print("  30056 mode=%-2d → %-18s 回读=%s" % (mode, ST_NAME.get(st, st), back))
    return st == 0


def ctrl_halt():
    lk.send(CTRL_WORD, [0, 0, 1])
    fr = lk.wait_ack(CTRL_WORD)
    return fr is not None


def read_mode():
    """用 30000 读 30055 状态字，bit[5:3]=实际在环模式、bit1=故障、bit0=使能。
    纯 MAVLink，不依赖 ASCII S 行的时序。返回 (active_mode, enabled, fault)。"""
    lk.send(READ_PARAM, list(struct.pack('<I', STATUS_INFO)))
    fr = lk.wait_ack(STATUS_INFO)
    if not fr or len(fr["payload"]) < 8:
        return None, None, None
    sw, ew = struct.unpack_from('<II', fr["payload"], 0)
    active = (sw >> 3) & 0x0F
    enabled = sw & 0x01
    fault = (sw >> 1) & 0x01
    return active, enabled, fault


def read_status():
    """读 S 行：cur/spd 模式标志、idq、moe、f（复用 Link 的同一串口，仅作辅助显示）"""
    s = lk.ascii_cmd('S', 0.35)
    for ln in s.split('\n'):
        if 'st en=' not in ln:
            continue
        import re
        def g(k, pat=r'(\d+)'):
            m = re.search(r'\b' + k + r'=' + pat, ln); return m.group(1) if m else '?'
        return dict(moe=g('moe'), f=g('f', r'0x([0-9a-fA-F]+)'),
                    cur=g('cur'), spd=g('spd'), idq=g('idq', r'([-\d.]+)'),
                    led=g('led'))
    return {}


def read_rpm(dur=0.5):
    """从 30040 enc_sum_p 的时间差分读真实机械转速 (rpm)。
    30040 只有 enc_sum(int64)+电角度，无直接转速字段；用首末差分即可。"""
    lk.s.reset_input_buffer()
    buf = bytearray(); samples = []   # (t, enc)
    t0 = time.time()
    while time.time() - t0 < dur:
        d = lk.s.read(4096)
        if d: buf += d
        got = []; k = scan(buf, got); del buf[:k]
        for kind, v in got:
            if kind == 'pos': samples.append((time.time(), v))
        time.sleep(0.002)
    if len(samples) < 2:
        return 0.0, 0
    t_first, e_first = samples[0]
    t_last,  e_last  = samples[-1]
    dt = t_last - t_first
    if dt <= 0: return 0.0, len(samples)
    d_enc = e_last - e_first                       # 脉冲
    rpm = (d_enc / ENC_COUNTS) / dt * 60.0
    return rpm, len(samples)


def test_torque(iq_a=0.3, secs=0.5):
    print("\n[力矩模式] mode=4 iq=%.2f A" % iq_a)
    if not mode_payload(4): return False
    pl = bytearray(53)
    struct.pack_into('<f', pl, 4, iq_a)          # idq_tar_A[1] = iq
    lk.send(CUR_IN, pl)
    fr = lk.wait_ack(CUR_IN)
    st = fr["payload"][0] if fr else -1
    back = struct.unpack_from('<f', fr["payload"], 1)[0] if fr and len(fr["payload"]) >= 5 else float('nan')
    print("  30025 iq=%.2f → %-18s 回读iq=%.3f A" % (iq_a, ST_NAME.get(st, st), back))
    time.sleep(secs)
    active, en, fault = read_mode()
    print("  状态字: active=%s enabled=%s fault=%s" % (active, en, fault))
    ok = (st == 0 and active == 4 and en == 1 and fault == 0)
    ctrl_halt()
    return ok


def test_speed(rpm=60.0, secs=0.7):
    print("\n[速度模式] mode=3 target=%.0f rpm" % rpm)
    if not mode_payload(3): return False
    w = rpm / RAD2RPM                            # rpm → rad/s
    p_s = w / TWO_PI * ENC_COUNTS                # rad/s → p/s
    pl = bytearray(44)
    struct.pack_into('<f', pl, 28, p_s)          # speed_tar_p_s
    lk.send(PSSPD_IN, pl)
    fr = lk.wait_ack(PSSPD_IN)
    st = fr["payload"][0] if fr else -1
    back = struct.unpack_from('<f', fr["payload"], 1)[0] if fr and len(fr["payload"]) >= 5 else float('nan')
    print("  30019 w=%.0frpm(p_s=%.0f) → %-18s 回读w=%.1f rpm" %
          (rpm, p_s, ST_NAME.get(st, st), back * RAD2RPM))
    time.sleep(secs)
    active, en, fault = read_mode()
    rpm_now, n = read_rpm(0.5)
    print("  状态字: active=%s enabled=%s fault=%s | 遥测转速=%.1f rpm (n=%d)" %
          (active, en, fault, rpm_now, n))
    ok = (st == 0 and active == 3 and en == 1 and fault == 0 and abs(rpm_now) > 5)
    ctrl_halt()
    return ok


def test_pos(delta_pulse=None, secs=0.7):
    print("\n[位置模式] mode=1 target=+0.5 rad")
    if not mode_payload(1): return False
    # 先读当前累积脉冲作为基准（经 30000 读 30061 太重，直接用 S 行不做绝对基准，测相对移动）
    target_p = int(round(0.5 / TWO_PI * ENC_COUNTS))
    pl = bytearray(44)
    struct.pack_into('<q', pl, 0, target_p)      # pos_tar_p（相对上电零点的绝对脉冲）
    lk.send(PSSPD_IN, pl)
    fr = lk.wait_ack(PSSPD_IN)
    st = fr["payload"][0] if fr else -1
    back = struct.unpack_from('<f', fr["payload"], 1)[0] if fr and len(fr["payload"]) >= 5 else float('nan')
    print("  30019 pos=%d p(%.3f rad) → %-18s 回读=%.3f rad" %
          (target_p, target_p/ENC_COUNTS*TWO_PI, ST_NAME.get(st, st), back))
    time.sleep(secs)
    active, en, fault = read_mode()
    print("  状态字: active=%s enabled=%s fault=%s" % (active, en, fault))
    ok = (st == 0 and active == 1 and en == 1 and fault == 0)
    ctrl_halt()
    return ok


def read_mit_pnow():
    """读 MIT 内部当前位置 p_now。MAVLink pos_tar_p 是绝对位置脉冲，
    必须换算自 MIT 内部的 p_now 才同口径（它比 enc_sum 多一个上电零点 base）。
    做法复用项目已验证的 mit_test：发 kp=0 的 0x02 原生帧使能 MIT（零冲击），
    再用 cmd_line 从混杂流取 '[CMD] ... mit T=.. p=<p_now>'。"""
    # 纯 MAVLink：先缓存 kp=0/kd=0（未使能时 CFG 只写缓存不重发），
    # 再发 MIT_IN 使能（零冲击），固件在 30065 应答 payload[5..8] 回读实际 p_now。
    cfg = bytearray(16)
    struct.pack_into('<f', cfg, 0, 0.0)          # kp=0
    struct.pack_into('<f', cfg, 4, 0.0)          # kd=0
    lk.send(MIT_CFG, cfg); lk.wait_ack(MIT_CFG)
    inp = bytearray(32)                          # p=0 任意，kp=0 不会有力矩
    lk.send(MIT_IN, inp)
    fr = lk.wait_ack(MIT_IN)
    if not fr or len(fr["payload"]) < 9:
        return None
    p_now = struct.unpack_from('<f', fr["payload"], 5)[0]
    return p_now


def read_enc_pulse(dur=0.25):
    """从 30040 enc_sum_p 读当前多圈位置（脉冲），作为 MIT 目标基准。"""
    lk.s.reset_input_buffer()
    buf = bytearray(); vals = []
    t0 = time.time()
    while time.time() - t0 < dur:
        d = lk.s.read(4096)
        if d: buf += d
        got = []; k = scan(buf, got); del buf[:k]
        for kind, v in got:
            if kind == 'pos': vals.append(v)
    return vals[-1] if vals else 0


def wait_still(max_s=2.0):
    """等电机完全静止（连续 3 次 enc_sum 相同）；MIT kp=0 起步前必须静止。"""
    prev = None; stable = 0; t0 = time.time()
    while time.time() - t0 < max_s:
        v = read_enc_pulse(0.15)
        if v == prev:
            stable += 1
            if stable >= 3: return v
        else:
            stable = 0
        prev = v
    return prev


def test_mit(secs=0.7):
    print("\n[MIT 模式] mode=12 kp=30 kd=0.05（优化默认值）")
    wait_still(2.0)   # 前一模式残留转速会让 kp=0 的 MIT 自由滑行，先等静止
    if not mode_payload(12): return False
    # 第一步：kp=0 零刚度使能，读 MIT 内部 p_now 作为目标基准（复刻 mit_run.py 起步）
    pnow = read_mit_pnow()
    if pnow is None:
        print("  无法读 MIT p_now ❌"); ctrl_halt(); return False
    print("  kp=0 起步，读 p_now=%.4f rad" % pnow)
    active0, en0, fault0 = read_mode()
    print("  起步状态字: active=%s enabled=%s fault=%s" % (active0, en0, fault0))
    if fault0:
        print("  起步即故障 ❌"); ctrl_halt(); return False
    # 第二步：先把正确目标灌进缓存（kp 仍 0，零冲击），再升 kp。
    # 顺序很重要：SetMitConfig 在 MIT 在环时会用缓存 p + 新 kp 立即刷新一次，
    # 若缓存 p 还是起步的 0，kp 一升就会用错误目标冲击。
    target_p = int(round(pnow / TWO_PI * ENC_COUNTS))
    inp = bytearray(32)
    struct.pack_into('<q', inp, 0, target_p)     # pos_tar_p = p_now（绝对脉冲）
    struct.pack_into('<f', inp, 16, 0.0)
    struct.pack_into('<f', inp, 24, 0.0)
    lk.send(MIT_IN, inp)
    fr = lk.wait_ack(MIT_IN)
    st = fr["payload"][0] if fr else -1
    print("  30065 p=%.4frad(%d) → %-18s" % (pnow, target_p, ST_NAME.get(st, st)))
    # 第三步：kp=30/kd=0.05 升刚度（缓存 p 已正确，SetMitConfig 重发安全）
    cfg = bytearray(16)
    struct.pack_into('<f', cfg, 0, 30.0)
    struct.pack_into('<f', cfg, 4, 0.05)
    struct.pack_into('<f', cfg, 12, 0.091)
    lk.send(MIT_CFG, cfg)
    fr = lk.wait_ack(MIT_CFG)
    st_cfg = fr["payload"][0] if fr else -1
    print("  30066 kp=30 kd=0.05 → %-18s" % ST_NAME.get(st_cfg, st_cfg))
    time.sleep(secs)
    active, en, fault = read_mode()
    print("  状态字: active=%s enabled=%s fault=%s" % (active, en, fault))
    ok = (st == 0 and st_cfg == 0 and active == 12 and en == 1 and fault == 0)
    ctrl_halt()
    return ok


def test_guard():
    """门卫测试（不通电）：非法模式、非法给定、权限"""
    print("\n[门卫拒写] 非法值应被拒、不误报成功")
    lk.send(OP_MODE, [99]); fr = lk.wait_ack(OP_MODE)
    st = fr["payload"][0] if fr else -1
    print("  30056 mode=99 → %-18s (期望 OUT_OF_RANGE)" % ST_NAME.get(st, st))
    lk.send(PERMISSION, [1, 0]); fr = lk.wait_ack(PERMISSION)  # 收回权限
    lk.send(OP_MODE, [4]); fr = lk.wait_ack(OP_MODE)
    st = fr["payload"][0] if fr else -1
    print("  无权限 mode=4 → %-18s (期望 NO_AUTHORITY)" % ST_NAME.get(st, st))
    lk.send(PERMISSION, [0, 0]); lk.wait_ack(PERMISSION)      # 还权限
    return True


if __name__ == "__main__":
    global lk
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--mode", type=int, choices=[1,3,4,12], help="只测一个模式")
    ap.add_argument("--dry-run", action="store_true", help="只测门卫，不通电")
    a = ap.parse_args()

    lk = Link(a.port)
    print("=== 4 控制模式 MAVLink 指令集实测 %s @921600 ===" % a.port)
    results = {}
    try:
        # 初始：封锁 + 清故障 + 权限
        ctrl_halt()
        lk.ascii_cmd('D', 0.3); lk.ascii_cmd('X', 0.3)
        lk.send(SYS_CMD, [3]); lk.wait_ack(SYS_CMD)
        lk.send(PERMISSION, [0, 0]); lk.wait_ack(PERMISSION)
        stt = read_status()
        print("初始状态: moe=%s f=0x%s cal/z 见 S 行" % (stt.get('moe'), stt.get('f')))

        results['guard'] = test_guard()
        if not a.dry_run:
            if a.mode is None or a.mode == 4: results['torque'] = test_torque(0.3)
            if a.mode is None or a.mode == 3: results['speed']  = test_speed(60.0)
            if a.mode is None or a.mode == 1: results['pos']    = test_pos()
            if a.mode is None or a.mode == 12: results['mit']   = test_mit()

        # 收尾封锁
        ctrl_halt(); lk.ascii_cmd('D', 0.3)
        stt = read_status()
        print("\n收尾状态: moe=%s f=0x%s (应 moe=0)" % (stt.get('moe'), stt.get('f')))
        print("\n结果：", ", ".join("%s=%s" % (k, "PASS" if v else "FAIL")
                                     for k, v in results.items()))
        print("帧统计：", lk.parser.stats)
    finally:
        ctrl_halt(); lk.ascii_cmd('D', 0.3)
        lk.close()
