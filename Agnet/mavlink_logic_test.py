# -*- coding: utf-8 -*-
"""
mavlink_logic_test.py —— 指令集控制逻辑优化后的回归脚本（2026-11-18）

针对本轮 5 项修复：
  1. 收帧单缓冲错分发 → 突发 6 帧必须收到 6 个 ACK；
  2. 应答未发完被覆盖 → 突发帧 ACK 一一对应；
  3. MIT 首次装配缺 MOE 回读 → 正常 MIT 仍 active=12/moe=1，不误报；
  4. 控制字使能与期望模式脱节 → "先旁带给定、再切模式、再控制字使能"应进目标环；
  5. SAVE_PARAM 幂等空实现 → 封锁态发 30069 cmd=1 应真落 Flash。

复用 mavlink_mode_test.py 的 Link / 状态字读取 / MIT 实测流程，避免两套协议口径。

用法：
    python mavlink_logic_test.py COM27              # 全部（真实通电，谨慎）
    python mavlink_logic_test.py COM27 --no-power   # 只做不通电项（突发 ACK + 封锁态存参）
"""
import sys, os, time, struct, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# 先导入 base：它负责把 host/ 与 Agnet/ 加进 sys.path，并提供同一份协议口径。
# 直接先 import mavlink_proto 会因 host/ 未入 path 而 ModuleNotFoundError。
import mavlink_mode_test as base

READ_PARAM  = 30000
CTRL_WORD   = 30054
OP_MODE     = 30056
PERMISSION  = 30068
SYS_CMD     = 30069
PSSPD_IN    = 30019

ST_NAME = base.ST_NAME
TWO_PI  = base.TWO_PI
ENC_COUNTS = base.ENC_COUNTS
RAD2RPM = base.RAD2RPM


def burst_ack_test(lk, n=6):
    """连续发 n 条无副作用的权限帧，不逐条等 ACK；应收到 n 条同 msgid ACK。
    旧实现在 pending/ack_pending 时会覆盖缓冲，突发场景会丢 ACK 或错分发。"""
    print("\n[突发 ACK] 连发 %d 条 30068(0,0)，不等 ACK" % n)
    base.ctrl_halt()
    for _ in range(n):
        lk.send(PERMISSION, [0, 0])

    got = []
    t0 = time.time()
    while time.time() - t0 < 2.0 and len(got) < n:
        d = lk.s.read(4096)
        if not d:
            time.sleep(0.002)
            continue
        # FrameParser 自维护残帧缓冲，直接喂新字节（不要手动 reset/re-feed）
        for fr in (lk.parser.feed(d) or []):
            if fr.get("msgid") == PERMISSION:
                got.append(fr["payload"][0])

    ok = len(got) >= n and all(s == 0 for s in got[:n])
    print("  收到 ACK=%d/%d，状态=%s → %s" %
          (len(got), n, got[:n], "PASS" if ok else "FAIL"))
    return ok


def save_param_test(lk, armed=False):
    """封锁态存参应成功（真写 Flash）；使能态必须被拒。"""
    print("\n[存参命令] 30069 cmd=1（armed=%s）" % armed)
    lk.send(SYS_CMD, [1])
    fr = lk.wait_ack(SYS_CMD, timeout=2.0)
    st = fr["payload"][0] if fr else -1
    expect = 3 if armed else 0            # 3=WRITE_STATE_ERROR
    ok = (st == expect)
    print("  → %-18s（期望 %s）%s" %
          (ST_NAME.get(st, st), ST_NAME.get(expect),
           "PASS" if ok else "FAIL"))
    return ok


def ctrl_word_mode_test(lk, rpm=60.0):
    """先以 mode=4 旁带写入速度给定（不装配），再切 mode=3，最后控制字使能。
    优化前：控制字使能走 do_enable() 位置保持，active=1；
    优化后：arm_with_mode() 按已就绪速度给定装配，active=3。"""
    print("\n[控制字→模式] 旁带速度 %.0f rpm → mode=3 → CW.SwitchOn" % rpm)
    base.ctrl_halt()
    lk.ascii_cmd('D', 0.3)

    base.mode_payload(4)                  # 力矩模式：SetPosSpeed 只缓存、不装配
    pl = bytearray(44)
    p_s = (rpm / RAD2RPM) / TWO_PI * ENC_COUNTS
    struct.pack_into('<q', pl, 0, int(round(0.5 / TWO_PI * ENC_COUNTS)))
    struct.pack_into('<f', pl, 28, p_s)
    lk.send(PSSPD_IN, pl)
    fr = lk.wait_ack(PSSPD_IN)
    st = fr["payload"][0] if fr else -1
    back = struct.unpack_from('<f', fr["payload"], 1)[0] if fr and len(fr["payload"]) >= 5 else 0.0
    print("  30019 旁带缓存 → %-18s 回读w=%.1f rpm" % (ST_NAME.get(st, st), back * RAD2RPM))

    base.mode_payload(3)                  # 只登记模式，不动环
    active0, en0, _ = base.read_mode()
    print("  切模式后（使能前）: active=%s enabled=%s（应仍为 0）" % (active0, en0))

    lk.send(CTRL_WORD, [1, 0, 0])         # cw bit0=1, halt=0
    fr = lk.wait_ack(CTRL_WORD)
    st_cw = fr["payload"][0] if fr else -1
    time.sleep(0.6)
    active, en, fault = base.read_mode()
    rpm_now, n = base.read_rpm(0.5)
    print("  控制字 → %-18s active=%s enabled=%s fault=%s 实测=%.1f rpm" %
          (ST_NAME.get(st_cw, st_cw), active, en, fault, rpm_now))

    ok = (st == 0 and st_cw == 0 and active == 3 and en == 1 and
          fault == 0 and abs(rpm_now) > 5)
    base.ctrl_halt()
    lk.ascii_cmd('D', 0.3)
    return ok


def mit_path_test(lk):
    """复用已验证的 MIT 实测流程，确认 MOE 回读修复不破坏正常路径。"""
    print("\n[MIT 路径] 复用 mavlink_mode_test.test_mit()")
    return base.test_mit()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM27")
    ap.add_argument("--no-power", action="store_true", help="跳过通电源/存参项")
    a = ap.parse_args()

    base.lk = base.Link(a.port)
    lk = base.lk
    print("=== 指令集控制逻辑回归 %s @921600 ===" % a.port)
    results = {}
    try:
        base.ctrl_halt()
        lk.ascii_cmd('D', 0.3)
        lk.ascii_cmd('X', 0.3)
        lk.send(PERMISSION, [0, 0]); lk.wait_ack(PERMISSION)
        stt = base.read_status()
        print("初始: moe=%s f=0x%s" % (stt.get('moe'), stt.get('f')))

        results['burst_ack'] = burst_ack_test(lk)
        if not a.no_power:
            results['ctrl_word_mode'] = ctrl_word_mode_test(lk)
            results['mit_path'] = mit_path_test(lk)
        results['save_param'] = save_param_test(lk, armed=False)

        base.ctrl_halt()
        lk.ascii_cmd('D', 0.3)
        stt = base.read_status()
        print("\n收尾: moe=%s f=0x%s（应 moe=0）" % (stt.get('moe'), stt.get('f')))
        print("\n结果：", ", ".join("%s=%s" % (k, "PASS" if v else "FAIL")
                                     for k, v in results.items()))
        print("帧统计：", lk.parser.stats)
        return 0 if all(results.values()) else 1
    finally:
        base.ctrl_halt()
        lk.ascii_cmd('D', 0.3)
        lk.close()


if __name__ == "__main__":
    raise SystemExit(main())
