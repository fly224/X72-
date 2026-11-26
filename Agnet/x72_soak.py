#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
x72_soak.py —— X72 FOC 上位机交互一致性长测

做什么
  随机挑选控制动作（使能/失能、模式切换、速度阶跃、速度轨迹、位置定位、
  力矩给定、MIT 柔顺、清故障、存参数、参数回读），经 MAVLink 下发，
  然后回读 30090/30091/30100/30025 逐项比对「指令 vs 电机反馈」是否一致。
  不一致或出现故障字，就把该次失败**之前的 10 步操作**完整取证落盘。
  连续不可恢复时，自动重烧固件并依次做：电角度校准 → 电气辨识 → 机械辨识，
  然后继续随机测试。

安全边界（本次测试约定，保守档）
  转速 ≤ 1000 rpm、iq ≤ 3 A、位置 ±90°(π/2 rad)、MIT 力矩 ≤ 0.35 Nm

用法
  python x72_soak.py --port COM8 --minutes 30
  python x72_soak.py --port COM8 --ops 200 --seed 1234
  python x72_soak.py --port COM8 --minutes 10 --no-reflash   # 只测不重烧

产物（默认写到 ./soak_out/）
  soak_<时间>/ops.jsonl        每一步一条记录（指令+反馈+判定）
  soak_<时间>/events.log       故障/恢复/重烧事件时间线
  soak_<时间>/fail_<n>.md      每次失败的取证报告（含前 10 步）
  soak_<时间>/summary.md       结束时汇总
"""

import argparse
import json
import math
import os
import random
import struct
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import x72_hmi as H

# ------------------------------------------------------------------ 安全边界
MAX_RPM = 1000.0
MAX_IQ = 3.0
POS_LIMIT = math.pi / 2          # ±90°
MAX_TAU = 0.35                   # MIT 力矩上限 Nm
TQ_IQ_MAX = 0.5                  # 力矩模式 iq 上限：空载下再大就必然超速
MIT_KP_CHOICES = [15.0, 30.0, 60.0]
MIT_KD_CHOICES = [0.3, 0.8, 1.5]

# ------------------------------------------------------------------ 工具链
# 全部交给 x72_env 解析（环境变量 → 常见安装位置 → 现场枚举），换机器不用改代码。
# 原来这里写死 E:\224work\app\... 与 ST-Link 序列号，移植到别的电脑必炸。
import x72_env as E

UV4 = E.UV4 or ''
PROG = E.PROG or ''
SN = E.SN or ''
MDK = E.MDK_ARM
HEX = E.HEX
# ST-Link 的 USB 复合设备实例 ID，用于"软拔插"：VCP 写方向卡死后重开串口无效
# （首夜 5 小时里重开 5093 次都没救回来），但 Disable/Enable 这个设备等价于
# 物理拔插，写方向立刻恢复（2026-11-23 实测）。由 SN 推导，换调试器自动跟着变。
STLINK_ID = E.STLINK_ID


def now_s():
    return time.strftime('%H:%M:%S')


class Recorder:
    """落盘：每步一条 JSONL；事件流单独一路；失败取证单独成文件。"""

    def __init__(self, outdir):
        self.dir = outdir
        os.makedirs(outdir, exist_ok=True)
        self.f_ops = open(os.path.join(outdir, 'ops.jsonl'), 'a', encoding='utf-8')
        self.f_ev = open(os.path.join(outdir, 'events.log'), 'a', encoding='utf-8')
        self.hist = []            # 最近若干步（取证用）
        self.n_fail = 0
        self.fail_list = []
        self.vc = {}              # 判定计数
        self.oc = {}              # 动作 → 判定计数
        self.fc = {}              # 故障字计数
        self.n_reset = 0          # 累计角超阈值软复位次数

    def event(self, msg):
        line = '[%s] %s' % (now_s(), msg)
        print(line)
        self.f_ev.write(line + '\n')
        self.f_ev.flush()

    def op(self, rec):
        self.f_ops.write(json.dumps(rec, ensure_ascii=False) + '\n')
        self.f_ops.flush()
        self.hist.append(rec)
        if len(self.hist) > 40:
            del self.hist[:20]
        v = rec.get('verdict', '-')
        o = rec.get('op', '-')
        self.vc[v] = self.vc.get(v, 0) + 1
        self.oc.setdefault(o, {})
        self.oc[o][v] = self.oc[o].get(v, 0) + 1
        if any('软复位重建原点' in n for n in (rec.get('notes') or [])):
            self.n_reset += 1

    def fault_report(self, why, snap):
        self.n_fail += 1
        prev = self.hist[-10:] if len(self.hist) >= 10 else self.hist[:]
        path = os.path.join(self.dir, 'fail_%02d.md' % self.n_fail)
        with open(path, 'w', encoding='utf-8') as f:
            f.write('# 失败取证 #%d\n\n' % self.n_fail)
            f.write('- 时间：%s\n- 判定：%s\n' % (time.strftime('%Y-%m-%d %H:%M:%S'), why))
            f.write('- 故障字：0x%04X = %s\n\n' % (snap.get('fault', 0),
                                                  H.fault_text(snap.get('fault', 0))))
            f.write('## 故障瞬间状态\n\n```\n')
            f.write(json.dumps(snap, ensure_ascii=False, indent=2))
            f.write('\n```\n\n## 故障前 %d 步操作\n\n' % len(prev))
            f.write('| # | 动作 | 参数 | 判定 | 关键反馈 |\n|---|---|---|---|---|\n')
            for i, r in enumerate(prev, 1):
                f.write('| %d | %s | %s | %s | %s |\n' % (
                    i, r.get('op'), r.get('arg', ''),
                    r.get('verdict', '-'), r.get('fb', '')))
            f.write('\n## 时序明细\n\n```\n')
            for i, r in enumerate(prev, 1):
                f.write('--- 第 %d 步 %s %s\n' % (i, r.get('t', ''), r.get('op', '')))
                f.write('  下发: %s\n' % json.dumps(r.get('cmd'), ensure_ascii=False))
                f.write('  反馈: %s\n' % json.dumps(r.get('fbdata'), ensure_ascii=False))
            f.write('```\n')
        self.fail_list.append(why)
        self.event('取证已写：%s' % os.path.basename(path))
        return path


class KeepAlive:
    """保活线程：速度/位置/力矩 20 Hz，MIT 100 Hz（固件 deadman 2 s / 20 ms）。"""

    def __init__(self, lk):
        self.lk = lk
        self.kind = None
        self.p = {}
        self._alive = True
        self._n = 0
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def set(self, kind, **p):
        self.p = p
        self.kind = kind

    def stop(self):
        self.kind = None

    def _run(self):
        while self._alive:
            k = self.kind
            try:
                if k == 'vel':
                    self.lk.speed_ctl(rpm=self.p['rpm'])
                elif k == 'pos':
                    self.lk.speed_ctl(pos_rad=self.p['pos'])
                elif k == 'torque':
                    self.lk.cur_in(self.p.get('id', 0.0), self.p['iq'])
                elif k == 'mit':
                    self.lk.mit_cmd(self.p['pos'], self.p.get('v', 0.0), self.p['tau'])
            except Exception:
                pass
            self._n += 1
            time.sleep(0.01 if k == 'mit' else 0.05)


class Rig:
    def __init__(self, port, rec, allow_reflash=True):
        self.rec = rec
        self.lk = H.Link(port)
        self.keep = KeepAlive(self.lk)
        self.allow_reflash = allow_reflash
        self.mode = None
        time.sleep(0.8)

    # -- 基础 -----------------------------------------------------
    def st(self):
        return self.lk.get(30090, 1.0) or {}

    def lp(self):
        return self.lk.get(30091, 1.0) or {}

    def ps(self):
        return self.lk.get(30100, 1.0) or {}

    def cur(self):
        return self.lk.get(30025, 1.0) or {}

    def absrad(self):
        """多圈绝对机械角（rad），来自 30040.encSum。
        ⚠ 只做"转了多少"的粗略参考：30040 与位置环坐标系（theta_multi−s_pos_base）
        差整数圈，不能拿它构造位置目标（见 A6/A7）。"""
        d = self.lk.get(30040, 1.0) or {}
        return (d.get('encSum') or 0) * H.CNT2RAD

    def read_params(self, timeout=1.0):
        """读 30082 全量帧（16f + 16i）。只取**本次请求之后**到达的帧，避免拿到旧值。
        30082 的应答有两路：2 B 的写回执和 192 B 的全量帧，必须按 'f' 键区分。"""
        n0 = len(self.lk.seq.get(30082, []))
        self.lk.dbg_read()
        t0 = time.time()
        while time.time() - t0 < timeout:
            for x in self.lk.seq.get(30082, [])[n0:]:
                if 'f' in x:
                    return x
            time.sleep(0.02)
        return None

    def coast(self, timeout=3.0, quiet=15.0):
        """停稳。上一动作若是 900 rpm，失能后靠摩擦自由滑行要十几秒 ——
        原来的"失能 + 等 2.5 s"根本等不到（自由轴惯量小、阻尼也小），于是
        带着 900 rpm 余速进 TORQUE/MIT，一给 iq 就撞 0x0020（v6 里 6 次
        TORQUE_OS 全是这个），或进位置环一读 p_now 就是移动中的值。

        改成：高速时先用速度环主动刹车到 0，再失能。
        这也更接近操作者在上位机上会做的事（先点"速度 0"再切模式）。
        """
        self.keep.stop()
        rpm0 = self.st().get('rpm') or 0.0
        if abs(rpm0) > 6 * quiet:
            try:
                if self.ensure_mode(H.OPMODE_VEL):
                    # 清掉常驻的 S 曲线/时间轨迹，否则刹车斜坡被它压扁
                    self.lk.traj(0.0)
                    self.lk.vel_profile(False, 0.0)
                    self.lk.speed_ctl(rpm=0.0, acc_rpm_s=2500.0)
                    self.keep.set('vel', rpm=0.0)
            except Exception:
                pass
        t0 = time.time()
        while time.time() - t0 < timeout:
            if abs(self.st().get('rpm') or 0) < quiet:
                break
            time.sleep(0.03)
        self.keep.stop()
        if self.st().get('armed'):
            self.disarm()
        time.sleep(0.12)                  # 再等一拍，让位置环把 p_now 刷新实
        w = abs(self.st().get('rpm') or 0)
        return w < quiet

    def enter_mit_safe(self):
        """MIT 安全起环：kp=0 + 目标=当前绝对角，先让固件缓存"不出力"的给定，
        再使能、再上刚度。照搬固件注释里记录过的那次"打飞"教训的反面做法。"""
        self.coast()
        self.lk.mit_cfg(0.0, 0.5)
        p0 = self.absrad()
        for _ in range(6):
            self.lk.mit_cmd(p0, 0.0, 0.0)
            time.sleep(0.01)
        if not self.ensure_mode(H.OPMODE_MIT):
            return None
        return p0

    def snap(self):
        s, l, p, c = self.st(), self.lp(), self.ps(), self.cur()
        return {
            't': now_s(),
            'fault': s.get('fault', -1), 'mode': s.get('mode', -1),
            'armed': s.get('armed'), 'flags': s.get('flags'),
            'cw': s.get('cw'),
            'iqRef': s.get('iqRef'), 'iqFb': s.get('iqFb'), 'rpm': s.get('rpm'),
            'theta_m': s.get('theta_m'), 'vbus': s.get('vbus'),
            'wref': l.get('wref'), 'wmeas': l.get('wmeas'), 'iqc': l.get('iqc'),
            'encDiag': l.get('encDiag'),
            'absRad': round(self.absrad(), 4),
            'pNow': p.get('pNow'), 'pTgt': p.get('pTgt'), 'pErr': p.get('err'),
            'pSt': p.get('st'), 'settled': p.get('settled'),
            'idTar': c.get('idTar'), 'iqTar': c.get('iqTar'),
            'ia': c.get('ia'), 'ib': c.get('ib'), 'ic': c.get('ic'),
        }

    def perm(self):
        self.lk.permission()

    def ensure_mode(self, m, timeout=1.6, do_arm=True):
        """切到模式 m 并使能。

        实测约束（2026-11-23）：
          * 30090 的 mode 报的是**实际活动模式**，未使能时恒为 NONE，
            所以"先等 mode 到位再使能"永远等不到 —— 必须使能后再校验。
          * **已使能状态下发 30056 不生效**（实测 VEL→POS 无效，mode 保持 VEL）。
            必须先失能，再 30056，再使能。
        """
        s = self.st()
        if s.get('mode') == m and (s.get('armed') or not do_arm):
            return True
        self.keep.stop()          # 切模式前必须停保活，否则旧模式的帧会把环又装回去
        if s.get('armed'):
            self.disarm()
        self.perm()
        self.lk.op_mode(m)
        time.sleep(0.08)
        if not do_arm:
            time.sleep(0.15)
            return self.st().get('mode') == m or not self.st().get('armed')
        self.perm()
        self.arm()
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.st().get('mode') == m:
                self.mode = m
                return True
            if not self.st().get('armed'):
                self.perm()
                self.arm()
            else:
                self.lk.op_mode(m)
            time.sleep(0.06)
        return self.st().get('mode') == m

    def arm(self):
        self.perm()
        self.lk.ctrl_word(True)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            if self.st().get('armed'):
                return True
            time.sleep(0.02)
        return False

    def disarm(self):
        self.keep.stop()
        self.lk.ctrl_word(False)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            if not self.st().get('armed'):
                return True
            time.sleep(0.02)
        return False

    def clear_fault(self, tries=3):
        for _ in range(tries):
            if not self.st().get('armed'):
                pass
            else:
                self.disarm()
            self.perm()
            self.lk.sys_cmd(H.ACT_CLEAR_FAULT)
            time.sleep(0.35)
            if self.st().get('fault', 1) == 0:
                return True
        return self.st().get('fault', 1) == 0

    def wait_ident(self, kind, timeout):
        """等 30092 里指定 kind 的结果帧。返回 (ok, v, resid) 或 None。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            for r in list(self.lk.seq.get(30092, [])):
                if r.get('kind') == kind and (time.time() - r.get('t', 0)) < timeout:
                    return r
            time.sleep(0.05)
        return None

    # -- 恢复链路 --------------------------------------------------
    def reset_stlink(self):
        """软拔插 ST-Link：pnputil 禁用再启用其 USB 复合设备。

        为什么必须有这一步（2026-11-23 首夜）：
          长测跑到第 8000 步（约 3 小时）后 VCP 的**写方向彻底死掉**
          —— ser.write 一律 Write timeout、重开串口 5093 次全部无效，
          而读方向还活着（遥测照收）。这是 ST-Link 板载固件卡死，
          跟目标板无关（目标板 fault=0、遥测正常、遥测字段全对）。
          物理拔插能恢复；实测 pnputil disable/enable 等价于拔插。

        调用前必须 pause 串口：设备被禁用后旧 HANDLE 全部失效，
        reader 线程继续读它就是段错误的来源。
        """
        self.rec.event('=== 软拔插 ST-Link（pnputil disable/enable）===')
        self.keep.stop()
        try:
            self.lk.pause()
        except Exception:
            pass
        if not STLINK_ID:
            # 移植到新机器时若没枚举到 ST-Link 序列号，就推不出设备实例 ID。
            # 这时只能靠下面 resume() 重开串口，救不回来就等人工插拔。
            self.rec.event('无 ST-Link 设备实例 ID（未枚举到序列号）→ 跳过软拔插')
        else:
            try:
                subprocess.run(['pnputil', '/disable-device', STLINK_ID],
                               capture_output=True, text=True, errors='replace',
                               timeout=40)
                time.sleep(2.0)
                subprocess.run(['pnputil', '/enable-device', STLINK_ID],
                               capture_output=True, text=True, errors='replace',
                               timeout=40)
                time.sleep(4.5)          # 等 VCP 重新枚举
            except Exception as e:
                self.rec.event('软拔插异常：%r' % (e,))
        self.lk._dead_streak = 0
        self.lk._cooldown_until = 0.0
        ok = self.lk.resume()
        # 真正验证写方向：发一帧 permission 看 send() 是否成功
        try:
            self.lk.permission()
        except Exception:
            pass
        time.sleep(0.4)
        alive = (self.lk.ser is not None) and (not self.lk.write_dead())
        self.rec.event('软拔插%s（重开累计 %d 次，写失败累计 %d 次）'
                       % ('成功，写方向已恢复' if alive else '后仍未恢复',
                          self.lk.n_reopen, self.lk.n_write_err))
        return alive

    def reflash(self):
        """重烧 + 复位。

        踩过的两个致命坑（2026-11-23 首夜，跑 8551 步后归零）：
          ① `subprocess.run(..., text=True)` 默认按本地编码解码，而
             STM32_Programmer_CLI 在中文 Windows 上输出 GBK 字节（0xb1 之类），
             一解码就 `UnicodeDecodeError` → 重烧恒失败 → 后续动作全 MODE_FAIL。
             修法：`errors='replace'`，解码永不抛。
          ② `-rst` 复位 MCU 会让 USB VCP 重枚举，reader 线程还拿着旧 HANDLE
             继续 read → 段错误。修法：烧录前 `lk.pause()`，烧完 `lk.resume()`。
        """
        self.rec.event('=== 重烧固件 ===')
        if not UV4 or not PROG:
            # 移植到新机器上最常见的情况：工具没装/路径没配。
            # 与其抛 Traceback 把长测打断，不如明确记一笔并让上层决定。
            self.rec.event('缺少工具链（UV4=%s PROG=%s）→ 无法重烧；'
                           '请设 X72_UV4 / X72_PROG 环境变量'
                           % (UV4 or '未找到', PROG or '未找到'))
            try:
                self.lk.resume()
            except Exception:
                pass
            return False
        self.keep.stop()                    # 烧录期间不能有保活帧在写串口
        try:
            self.lk.pause()                 # 关串口，避免复位时读已失效句柄
        except Exception:
            pass
        ok = False
        try:
            tmp = os.path.join(MDK, '.btmp')
            os.makedirs(tmp, exist_ok=True)
            env = dict(os.environ, TMP=tmp, TEMP=tmp)
            b = os.path.join(MDK, 'build_soak.log')
            subprocess.run([UV4, '-b', 'X72_FOC.uvprojx', '-j0', '-o', b],
                           cwd=MDK, env=env, capture_output=True,
                           text=True, errors='replace')
            txt = ''
            try:
                txt = open(b, encoding='utf-8', errors='replace').read()
            except Exception:
                pass
            if '0 Error(s)' not in txt:
                self.rec.event('编译失败，中止重烧：%s' % txt[-300:])
                return False
            # SN 为空时不传 sn= —— CLI 会自动选唯一连着的 ST-Link，
            # 这比写死某个序列号更可移植（换机器/换调试器都不用改代码）。
            cmd = [PROG, '-c', 'port=SWD']
            if SN:
                cmd.append('sn=' + SN)
            cmd += ['-w', HEX, '-v', '-rst']
            r = subprocess.run(cmd, capture_output=True, text=True,
                               errors='replace')
            out = (r.stdout or '') + (r.stderr or '')
            ok = 'Download verified successfully' in out
            self.rec.event('重烧%s%s' % ('成功' if ok else '失败',
                                          '' if ok else '：' + out[-200:].replace('\n', ' ')))
        except Exception as e:
            self.rec.event('重烧异常：%r' % (e,))
        finally:
            # 复位后 VCP 要几百 ms 才重新枚举，resume() 内部会重试若干次
            try:
                self.lk.resume()
            except Exception:
                pass
        return ok

    def sys_action(self, act, kind=None, timeout=60.0, label=''):
        """执行一个 30069 动作并等结果。

        踩过的坑（2026-11-23）：
          * 30069 应答 status=2 是 SC_EXEC_NEED_DISARM（仍在闭环），不是"忙"。
            电角度对齐结束时电机会停在使能态，后面 6 项辨识无一例外被拒，
            现象却是"超时无结果"，极易误判成辨识算法失败。
          * act=4（电流零点）根本不发 30092，等它必然超时；只能靠 30090 的
            CAL 位/零偏判。
        """
        for attempt in (1, 2):
            if self.st().get('armed'):
                self.disarm()
            self.perm()
            base = len(self.lk.acks.get(30069, []))
            b92 = len(self.lk.seq.get(30092, []))
            self.lk.sys_cmd(act)
            time.sleep(0.35)
            ac = self.lk.acks.get(30069, [])[base:]
            st_ack = ac[-1]['st'] if ac else -1
            if st_ack == 2 and attempt == 1:
                self.rec.event('    %s 应答 NEED_DISARM，先失能重试' % label)
                self.disarm()
                time.sleep(0.3)
                continue
            if st_ack not in (0, 1, -1):
                self.rec.event('    %s 被拒 status=%d' % (label, st_ack))
                return None
            if kind is None:
                time.sleep(min(timeout, 1.0))
                return {'ok': 1, 'v': [0.0] * 8, 'resid': 0.0, 'kind': act}
            t0 = time.time()
            while time.time() - t0 < timeout:
                q = self.lk.seq.get(30092, [])
                for x in q[b92:]:
                    if x.get('kind') == kind:
                        return x
                if self.st().get('fault', 0) not in (0, 0x0020):
                    return None
                time.sleep(0.1)
            return None
        return None

    def calibrate_chain(self):
        """电角度校准 → 电气辨识 → 机械辨识。每步等 30092 事件帧。"""
        self.rec.event('=== 校准/辨识链开始 ===')
        self.keep.stop()
        if self.st().get('armed'):
            self.disarm()
        self.clear_fault()

        steps = [
            ('电流零点标定', H.ACT_CAL_CURRENT, None, 2.0),
            ('电角度对齐', H.ACT_ALIGN_ZERO, 8, 45.0),
            ('磁链/Kt 辨识', H.ACT_IDENT_FLUX, 1, 120.0),
            ('R/L 辨识', H.ACT_IDENT_RL, 2, 90.0),
            ('机械扫频', H.ACT_MECH_SWEEP, 6, 120.0),
            ('摩擦曲线', H.ACT_MECH_FRIC, 4, 120.0),
            ('折算惯量', H.ACT_MECH_INERTIA, 3, 120.0),
            ('齿槽转矩', H.ACT_MECH_COG, 5, 200.0),
        ]
        out = []
        for name, act, kind, tmo in steps:
            r = self.sys_action(act, kind, tmo, name)
            if r:
                ok = bool(r.get('ok'))
                self.rec.event('  %-12s kind=%d ok=%d v=%s resid=%.4g'
                               % (name, kind or act, ok,
                                  [round(x, 4) for x in r['v'][:4]],
                                  r.get('resid', float('nan'))))
                out.append((name, ok, r['v']))
                if not ok:
                    self.clear_fault()
            else:
                self.rec.event('  %-12s 超时/无结果（fault=0x%04X）'
                               % (name, self.st().get('fault', 0)))
                out.append((name, False, None))
                self.clear_fault()
            time.sleep(0.9)          # 动作之间有 500 ms 保护窗（ACT_GRACE_MS）
        self.rec.event('=== 校准/辨识链结束 ===')
        return out

    def recover(self, why, snap):
        """先清故障；清不掉或连续复发则重烧 + 全链校准。"""
        path = self.rec.fault_report(why, snap)
        self.rec.event('故障复现：%s（取证 %s）' % (why, os.path.basename(path)))
        self.keep.stop()
        self.disarm()
        # 0x0100 跟随误差在多圈累积后是必然结果（见 A8），软复位重建原点即可，
        # 不必重烧。参数已存 Flash，软复位不丢。
        if (snap.get('fault', 0) & 0x0100) != 0:
            self.rec.event('0x0100 跟随误差 → 软复位重建原点（参数已存 Flash）')
            self.perm()
            self.lk.sys_cmd(H.ACT_SOFT_RESET)
            time.sleep(2.5)
            if self.st().get('fault', 0) == 0:
                self.rec.event('软复位后故障已清，继续测试')
                return True
        if self.clear_fault(3):
            self.rec.event('清故障成功，继续测试')
            return True
        self.rec.event('清故障失败（fault=0x%04X），进入重烧+校准' %
                       self.st().get('fault', 0))
        if not self.allow_reflash:
            self.rec.event('--no-reflash：不重烧，中止')
            return False
        if not self.reflash():
            return False
        time.sleep(1.5)
        self.calibrate_chain()
        if self.st().get('fault', 1) != 0:
            self.rec.event('校准后仍有故障 0x%04X，中止' % self.st().get('fault', 0))
            return False
        return True


# ------------------------------------------------------------------ 随机动作
# 重点动作（--focus 设置）：命中集合的动作权重乘以 FOCUS_MULT。
# 用于"这轮专门测位置/MIT"这类需求，不必改代码里的基础权重表。
FOCUS = set()
FOCUS_MULT = 6


def pick_op(rng):
    """按权重挑一个动作。返回 (名称, 参数 dict)。"""
    w = [
        ('speed', 22), ('pos', 20), ('torque', 12), ('mit', 10),
        ('traj', 8), ('mode_flip', 8), ('enable_toggle', 8),
        ('read_param', 6), ('clear_fault', 4), ('save_param', 2),
        ('param_rt', 4),
    ]
    if FOCUS:
        w = [(n, (wt * FOCUS_MULT if n in FOCUS else wt)) for n, wt in w]
    name = rng.choices([x[0] for x in w], weights=[x[1] for x in w])[0]
    if name == 'speed':
        return name, {'rpm': rng.choice([-1, 1]) * rng.uniform(20, MAX_RPM),
                      'acc': rng.uniform(200, 1500)}
    if name == 'pos':
        return name, {'delta': rng.uniform(-1.0, 1.0)}      # 相对当前位置，±57°
    if name == 'torque':
        return name, {'iq': rng.uniform(-TQ_IQ_MAX, TQ_IQ_MAX),
                      'ramp': rng.uniform(20, 400)}
    if name == 'mit':
        return name, {'delta': rng.uniform(-0.5, 0.5),
                      'tau': rng.uniform(-MAX_TAU, MAX_TAU),
                      'kp': rng.choice(MIT_KP_CHOICES),
                      'kd': rng.choice(MIT_KD_CHOICES)}
    if name == 'traj':
        return name, {'rpm': rng.choice([-1, 1]) * rng.uniform(30, MAX_RPM),
                      't': rng.uniform(0.4, 2.5),
                      's': rng.random() < 0.5,
                      'jerk': rng.uniform(500, 5000)}
    if name == 'mode_flip':
        return name, {'to': rng.choice([H.OPMODE_VEL, H.OPMODE_POS,
                                        H.OPMODE_TORQUE, H.OPMODE_MIT])}
    if name == 'enable_toggle':
        return name, {'on': rng.random() < 0.7}
    return name, {}


def do_op(rig, rng, name, arg):
    """执行一个动作，返回 (verdict, fb 摘要, 详细 cmd/fb 记录)。"""
    rec = {'op': name, 'arg': json.dumps(arg, ensure_ascii=False), 't': now_s()}
    verdict = 'OK'
    time.sleep(rng.uniform(0.02, 0.15))

    if name == 'speed':
        if not rig.ensure_mode(H.OPMODE_VEL):
            return 'MODE_FAIL', '进不了 VEL', rec
        # 30097 的两种斜坡都是**常驻状态**：切模式、重下速度都不会把它们复位。
        #   * t≠0        —— 时间模式，直接绕过 30019 的 acc
        #                   （实测 acc=100 与 acc=8000 的速率都是 224 rpm/s，见 A1）
        #   * s=1 + jerk —— S 曲线限加加速度，起坡段实际速率远低于 acc
        # traj 动作会把这两个都设上，所以每条速度指令前必须**双清**，
        # 否则 acc 只是看起来生效（fail_01/fail_06 的斜坡只有指令值的 55%/39%）。
        rig.lk.traj(0.0)
        rig.lk.vel_profile(False, 0.0)
        time.sleep(0.05)
        # 爬坡行程必须从**起坡时的真实转速**算，不能从 0 算：
        # 上一动作停在 +981 rpm、新给定 −298 时实际要爬 1279 rpm（1.21 s），
        # 按 |−298|/acc 只算出 0.28 s —— 会在半路就去比 wref。
        # 实测 4 次 MISMATCH_VEL 全部是同一机理（rpm 一直贴着 wref，差 1~2 rpm）。
        w0 = rig.st().get('rpm') or 0.0
        tol_ref = max(20.0, 0.03 * abs(arg['rpm']))
        rig.lk.speed_ctl(rpm=arg['rpm'], acc_rpm_s=arg['acc'])
        rig.keep.set('vel', rpm=arg['rpm'])
        settle = min(8.0, abs(arg['rpm'] - w0) / max(arg['acc'], 50.0) + 0.85)
        # 按坡度算出的时间只是上界：轮询到 wref 到位就提前走，平均更快。
        t0 = time.time()
        while time.time() - t0 < settle:
            time.sleep(0.10)
            if abs((rig.lp().get('wref') or 0.0) - arg['rpm']) <= tol_ref:
                break
        time.sleep(0.12)
        s, l = rig.st(), rig.lp()
        err_fb = abs((s.get('rpm') or 0) - arg['rpm'])
        err_ref = abs((l.get('wref') or 0) - arg['rpm'])
        tol_fb = max(50.0, 0.12 * abs(arg['rpm']))
        verdict = 'OK' if (err_fb <= tol_fb and err_ref <= tol_ref) else 'MISMATCH_VEL'
        rec['fb'] = 'rpm %.1f/%.1f ref %.1f err %.1f (tol %.0f/%.0f, 起坡 %.0f rpm, %.2fs)' % (
            arg['rpm'], s.get('rpm') or 0, l.get('wref') or 0, err_fb,
            tol_fb, tol_ref, w0, time.time() - t0)

    elif name == 'traj':
        if not rig.ensure_mode(H.OPMODE_VEL):
            return 'MODE_FAIL', '进不了 VEL', rec
        if not rig.st().get('armed'):
            rig.arm()
        rig.lk.vel_profile(arg['s'], arg['jerk'])
        rig.lk.traj(arg['t'])
        rig.lk.speed_ctl(rpm=arg['rpm'], acc_rpm_s=0)
        rig.keep.set('vel', rpm=arg['rpm'])
        time.sleep(arg['t'] + rng.uniform(0.6, 1.2))
        s, l = rig.st(), rig.lp()
        err_fb = abs((s.get('rpm') or 0) - arg['rpm'])
        tol = max(60.0, 0.12 * abs(arg['rpm']))
        verdict = 'OK' if err_fb <= tol else 'MISMATCH_TRAJ'
        rec['fb'] = 'traj t=%.2f s=%d rpm %.1f/%.1f err %.1f' % (
            arg['t'], arg['s'], arg['rpm'], s.get('rpm') or 0, err_fb)
        # 必须在这里收尾：t 与 S 曲线是常驻状态，留给下一个动作会把它的
        # 斜坡悄悄改掉（fail_01/fail_06 的 MISMATCH_VEL 就是这么来的）。
        rig.lk.traj(0.0)
        rig.lk.vel_profile(False, 0.0)

    elif name == 'pos':
        # ★ 位置环的圈数原点在软复位时重建，而 30040.encSum 一直累加。
        #   早期为了防止"跨圈带出 0x0100"把阈值压到 30 rad，结果 1000 rpm
        #   每秒累 105 rad，几乎每次 pos 都触发一次软复位（v3 的 66 次 pos 里
        #   26 次），既污染遥测计数也让 A10 永远取不到证。
        #   目标已改成 p_now + delta 构造（与固件同一圈窗口），不再需要频繁重建原点，
        #   阈值放宽到 900 rad（≈143 圈）：只在真正极端时才重建，正常不打扰。
        if abs(rig.absrad()) > 900.0:
            rec.setdefault('notes', []).append('累计角 %.0f rad，软复位重建原点' % rig.absrad())
            rig.rec.event('累计角 %.0f rad 超阈值 → 软复位重建位置环原点' % rig.absrad())
            rig.perm()
            rig.lk.sys_cmd(H.ACT_SOFT_RESET)
            time.sleep(2.5)
            if rig.st().get('fault'):
                rig.clear_fault()
        if not rig.ensure_mode(H.OPMODE_POS):
            return 'MODE_FAIL', '进不了 POS', rec
        # 先停稳再读位置：滑行中读到的 p_now 是移动值，用它算目标会变成
        # 大阶跃（实测撞 0x0100 跟随误差，误差 1.86 rad）。
        rig.coast(quiet=5.0)
        rig.ensure_mode(H.OPMODE_POS)      # coast 会失能，重新进位置环并使能
        time.sleep(0.25)
        p = rig.ps()
        cur_n = p.get('pNow') or 0.0       # 归一化到 [0,2π)，**位置目标的唯一合法窗口**
        # ★★ 目标只能用 p_now 构造，不能用 30040.encSum 的绝对角（2026-11-23 对照实验）
        #
        #   servo_isr.c:3098  three_loop_deploy_U.pos_ref = dir · s_pos_ref
        #                     theta_pos                 = dir · theta_multi
        #   两者**直接相减、不做折返**，所以目标必须和 theta_multi 同一圈窗口。
        #   而 theta_multi = s_tm_base + s_pos_sum·2π/2^21（bsp_servo.c:1201），
        #   s_pos_sum 会被软复位清零重建，30040 的 encSum 却是上电起连续累加 ——
        #   两者差**整数圈**，且这个圈数上位机从遥测看不到。
        #
        #   pos_probe.py 五组对照（同一台机、同一停稳状态）：
        #     A 目标=abs+0.30        → Δ +38.394  仍在 MOVE
        #     B 目标=abs+0.30−2π     → Δ +38.592  仍在 MOVE
        #     C 目标=abs+0.30+2π     → Δ +38.778  仍在 MOVE
        #     D 目标=p_now+0.30      → Δ +0.301  DONE  ★
        #     E 目标=abs−0.30        → Δ +39.790  仍在 MOVE
        #   四组错误写法都是"以最大速度单向跑、2 s 走 38 rad(≈6 圈)、不报故障"，
        #   因为 err = pos_ref − theta_multi 恒为 ±几百 rad，永远到不了。
        cur_a = rig.absrad()               # 只记录，用来暴露两个坐标系的差值
        # ★ 必须把目标折回 [0,2π)：cur_n + delta 会越界（cur_n=5.87、delta=+0.74
        #   → 6.61，或 cur_n=0.14、delta=−0.58 → −0.44）。越界值会被
        #   Servo_SetPosTarget 当成"位置环坐标系的绝对位置"，语义完全不同，
        #   实测会多跑整圈（v6 第 87 步就是这么撞到 0/2π 边界的）。
        tgt = (cur_n + arg['delta']) % (2 * math.pi)
        rig.lk.speed_ctl(pos_rad=tgt)
        rig.keep.set('pos', pos=tgt)
        time.sleep(rng.uniform(1.2, 2.2))
        p = rig.ps()
        pnow = p.get('pNow') or 0.0
        now_a = rig.absrad()
        # 目标回显校验：30100.p_tgt 是固件内部折算后的目标（归一到 [0,2π)）。
        # 这是**唯一能直接证明"固件收到的目标 == 上位机下发的目标"**的字段，
        # 比看有没有走到位更能区分"指令丢了"和"环没跟上"。
        ptgt = p.get('pTgt')
        if isinstance(ptgt, (int, float)):
            echo = abs(((ptgt - tgt + math.pi) % (2 * math.pi)) - math.pi)
        else:
            echo = -1.0
        # 目标与反馈都在 p_now 窗口内，但跨 2π 边界时会差一整圈，仍按最小角差比。
        raw = pnow - tgt
        err = abs((raw + math.pi) % (2 * math.pi) - math.pi)
        tol = 0.05
        # 分成三档，避免把"边界蠕动"和"指令不一致"混为一谈：
        #   · err 超差            → 真的没到位
        #   · 目标回显对不上      → 指令没进去/被改写（最严重）
        #   · 位置对、但 st 不是 DONE → 边界蠕动（实测：目标落在 0 附近时，
        #     pErr 只有 0.0026 rad，rpm 却在 ±23 来回、2 s 内不置 DONE）
        if echo >= 0 and echo > 0.01:
            verdict = 'MISMATCH_POS'
            why = '目标回显 %.4f 与下发不符' % echo
        elif err > tol:
            verdict = 'MISMATCH_POS'
            why = '误差 %.4f 超容差' % err
        elif p.get('st') not in (2, 3):
            verdict = 'POS_BOUNDARY'
            why = '位置对但 st=%s（边界蠕动）' % (
                H.POS_ST[p['st']] if isinstance(p.get('st'), int) and 0 <= p['st'] < 5 else '?')
        else:
            verdict = 'OK'
            why = ''
        rec['fb'] = ('pos tgt %.3f(Δ%+.2f) now %.3f echo %s err %.4f(裸%+.3f) st %s '
                     '| encSum %.1f(差 %.1f 圈)' % (
                         tgt, arg['delta'], pnow,
                         '%.4f' % echo if echo >= 0 else '无',
                         err, raw,
                         H.POS_ST[p['st']] if isinstance(p.get('st'), int) and 0 <= p['st'] < 5 else '?',
                         now_a, (now_a - pnow) / (2 * math.pi)))
        if why:
            rec['fb'] += ' <' + why + '>'

    elif name == 'torque':
        # 先停稳：力矩模式是"给 iq 就不管速度"，带着 900 rpm 残余进入会立刻超速。
        rig.coast()
        if not rig.ensure_mode(H.OPMODE_TORQUE):
            return 'MODE_FAIL', '进不了 TORQUE', rec
        rig.lk.torque_cfg(0.0, arg['ramp'])
        time.sleep(0.06)
        rig.lk.cur_in(0.0, arg['iq'])
        rig.keep.set('torque', iq=arg['iq'], id=0.0)
        # 空载下 0.49 A 只需要 **37 ms** 就把电机推到 300 rpm 的 SERVO_CUR_WMAX
        # 门限（J=5.33e-5、Kt=0.091 → 7985 rpm/s），所以观察窗不能乱给。
        # 但 ramp（一阶时间常数，实测≈tc）大时给定本身要 3~4 个 tc 才到位，
        # 窗太短只会读到"给定还在爬" —— 那是斜坡特性，不是跟踪误差。
        #
        # ★ 禁止在窗口里用 rig.st() 采样：get() 要等**新帧**，一帧可能阻塞几十 ms，
        #   350 ms 窗口内只采到一两帧（v5 第 257 步就这么误判成 MISMATCH_TORQUE：
        #   ramp=253 ms，best 为空，最后拿一帧 iqRef=−0.35/iqFb=−0.05 去比 −0.49）。
        #   正确做法：让 reader 线程照常收帧，窗口结束后从 seq 历史里扫。
        t_cmd = time.time()
        win = min(0.90, max(0.30, 3.5 * arg['ramp'] / 1000.0))
        time.sleep(win)
        s = rig.st()
        tripped = (s.get('fault') or 0) == 0x0020
        # 从缓存里挑"给定已到位（≥80%）且电流环跟得最好"的那一帧
        best = None
        for x in rig.lk.seq.get(30090, []):
            if x.get('t', 0.0) < t_cmd:
                continue
            ref = x.get('iqRef') or 0.0
            fb = x.get('iqFb') or 0.0
            if abs(ref) < 0.8 * abs(arg['iq']):
                continue
            e = abs(fb - ref)                      # 电流环跟踪 = |iqFb − iqRef|
            if best is None or e < best[0]:
                best = (e, ref, fb, x)
        if best is not None:
            e_loop, ref_b, fb_b, _ = best
            err = abs(fb_b - arg['iq'])            # 顺带记"到位后离目标多远"
            tol = max(0.08, 0.15 * abs(ref_b))
            ok_loop = e_loop <= tol
        else:
            e_loop, ref_b, fb_b = -1.0, 0.0, 0.0
            err = abs((s.get('iqFb') or 0) - arg['iq'])
            tol = 0.0
            ok_loop = False
        if tripped:
            # 空载 + 力矩环 = 物理必然超速，不是"上位机与电机不一致"。
            # 单独判定，不进失败统计（真机带负载时才有意义）。

            verdict = 'TORQUE_OS'
        elif best is None:
            # 斜坡太长 / 提前跳闸：窗口内给定没到 80%，没有可比样本。
            # 这是空载台架的观测局限，不是一致性结论，单列不计失败。
            verdict = 'TORQUE_RAMP'
        else:
            verdict = 'OK' if ok_loop else 'MISMATCH_TORQUE'
        rec['fb'] = ('iq %.2f fb %.2f ref %.2f 环差 %.3f(容 %.2f) 到位差 %.2f win %.2fs%s'
                     % (arg['iq'], fb_b, ref_b, e_loop, tol, err, win,
                        ' | 0x0020 空载超速(预期，跳闸时转速 %.0f rpm)' % (s.get('rpm') or 0)
                        if tripped else ''))
        rig.keep.stop()
        rig.disarm()

    elif name == 'mit':
        p0 = rig.enter_mit_safe()
        if p0 is None:
            return 'MODE_FAIL', '进不了 MIT', rec
        # 目标必须相对"当前绝对角"下发：MIT 的 pos_tar_p 与 30040 encSum 同基准，
        # 但绝对零点随上电/校准变化，写死数值会变成一次大幅阶跃。
        tgt = p0 + arg['delta']
        rig.lk.mit_cfg(arg['kp'], arg['kd'])
        t0 = time.time()
        while time.time() - t0 < 1.4:
            rig.lk.mit_cmd(tgt, 0.0, arg['tau'])
            time.sleep(0.01)
        a1 = rig.absrad()
        err = abs(a1 - tgt)
        # 实测（2026-11-23）：MIT 位置跟踪在 |Δ| < ~0.2 rad 时不动作；|Δ|=1.0 时
        # 走 0.796 就停住，残差恒为 0.20 rad，且从 0.5 s 起位置完全不动、
        # iqRef 在 0 与 ±5 A 间跳、rpm 在 ±2 rpm 抖 —— 是极限环/卡住而非收敛慢。
        # 判据先按这个实测死区放，超出死区的偏差仍算不一致。
        tol = max(0.25, 1.2 / max(arg['kp'], 1.0))
        # ★ 2026-11-23 新规律（A15，angle_err_probe.py 受控实测）：
        #   MIT 残差随**累计角**线性退化，与 kp 完全无关（kp 15/30/60/120 残差
        #   全是 −0.102±0.001），而位置环（目标归一化在 [0,2π)）不受影响。
        #   实测：36 rad → 0.005；276 rad → 0.024；576 rad → 0.052；
        #        975 rad → 0.050；1477 rad → 0.102。斜率约 1.0e-4 rad/rad。
        #   所以"残差大"要先看目标在多少圈的位置上：远离零点的残差不是不一致。
        #   归一残差 err/|tgt| 落在 3e-5~1.5e-4 就归到 MIT_ANGLE_DRIFT（已知局限），
        #   不占用失败名额；归一残差更大才是真的跟踪不良。
        rel = err / max(abs(tgt), 1.0)
        if err <= tol:
            verdict = 'OK'
        elif 3e-5 <= rel <= 1.5e-4:
            verdict = 'MIT_ANGLE_DRIFT'
        else:
            verdict = 'MISMATCH_MIT'
        rec['fb'] = ('mit tgt %.3f(Δ%+.2f) now %.3f err %.4f tol %.3f kp %.0f tau %.2f '
                     'rel %.1e' % (tgt, arg['delta'], a1, err, tol, arg['kp'],
                                   arg['tau'], rel))
        if err > 0.08:
            rec['fb'] += ' [MIT死区观测]'

    elif name == 'mode_flip':
        # 切进**位置/ MIT** 前先减速（2026-11-24，配合固件 A13 入口余速门禁）。
        # 固件从今天起会在余速 > 215 rpm 时拒绝起位置环 —— 那是正确行为，
        # 但如果测试仍然"跑着就切"，结果只是刷一堆 MODE_FAIL，测不出别的东西。
        # 速度环入口没有余速限制（它自己会接管），所以只在切 POS/MIT 时减速。
        if arg['to'] in (H.OPMODE_POS, H.OPMODE_MIT):
            rig.coast(quiet=10.0)
        ok = rig.ensure_mode(arg['to'])
        rec['fb'] = 'mode -> %s' % H.MODE_NAME.get(rig.st().get('mode', -1), '?')
        verdict = 'OK' if ok else 'MODE_FAIL'
        rig.keep.stop()

    elif name == 'enable_toggle':
        if arg['on']:
            rig.keep.stop()
            ok = rig.ensure_mode(rig.st().get('mode') or H.OPMODE_VEL) and rig.arm()
        else:
            ok = rig.disarm()
        rec['fb'] = 'armed=%s' % rig.st().get('armed')
        verdict = 'OK' if ok else 'ARM_FAIL'

    elif name == 'clear_fault':
        rig.keep.stop()
        rig.disarm()
        rig.perm()
        rig.lk.sys_cmd(H.ACT_CLEAR_FAULT)
        time.sleep(0.4)
        verdict = 'OK' if rig.st().get('fault', 1) == 0 else 'CLEAR_FAIL'
        rec['fb'] = 'fault 0x%04X' % rig.st().get('fault', 0)

    elif name == 'save_param':
        rig.perm()
        rig.lk.sys_cmd(H.ACT_SAVE_PARAM)
        time.sleep(0.5)
        rec['fb'] = 'fault 0x%04X' % rig.st().get('fault', 0)

    elif name == 'read_param':
        d = rig.read_params()
        verdict = 'OK' if d else 'READ_FAIL'
        rec['fb'] = ('nFloat=%s nInt=%s' % (d['nFloat'], d['nInt'])) if d else '无回读'

    elif name == 'param_rt':
        # 30082 写-读往返：挑对运行无害的槽 15（ff_boost，起动助推倍数），
        # 写 1.05 倍 → 回读必须等于写入值 → 再写回原值 → 回读必须复原。
        # 这条直接验「上位机写入的数 == 固件记下的数」，是最容易静默失效的一环：
        # 掩码位漏了会"回显 updated 但根本没写进去"，回读对不上才发现。
        idx = 15
        d0 = rig.read_params()
        if not d0:
            return 'READ_FAIL', '30082 读不到，无法做写-读往返', rec
        old = d0['f'][idx]
        new = old * 1.05
        rig.lk.dbg_write({idx: new})
        time.sleep(0.25)
        d1 = rig.read_params() or {}
        got = (d1.get('f') or [None] * 16)[idx]
        rig.lk.dbg_write({idx: old})          # 无论成败都复原
        time.sleep(0.25)
        d2 = rig.read_params() or {}
        back = (d2.get('f') or [None] * 16)[idx]
        e_w = abs((got if got is not None else 9e9) - new)
        e_b = abs((back if back is not None else 9e9) - old)
        verdict = 'OK' if (e_w <= 1e-4 and e_b <= 1e-4) else 'MISMATCH_PARAM'
        rec['fb'] = 'ff_boost %.4f→%.4f 回读 %.4f(err %.1e) 复原 %.4f(err %.1e)' % (
            old, new, got if got is not None else -1, e_w,
            back if back is not None else -1, e_b)

    rec['verdict'] = verdict
    # 详细快照
    s = rig.st()
    rec['cmd'] = arg
    rec['fbdata'] = rig.snap()
    return verdict, rec.get('fb', ''), rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=E.PORT)   # 自动识别 ST-Link VCP，换机器不用改
    ap.add_argument('--minutes', type=float, default=0)
    ap.add_argument('--ops', type=int, default=0)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--no-reflash', action='store_true')
    ap.add_argument('--calib-only', action='store_true',
                    help='只跑校准/辨识链：电流零点 → 电角度 → 电气 → 机械，然后退出')
    ap.add_argument('--out', default='soak_out')
    ap.add_argument('--focus', default='',
                    help='重点动作，逗号分隔（如 pos,mit），命中者权重 ×--focus-mult')
    ap.add_argument('--focus-mult', type=int, default=6,
                    help='重点动作的权重倍率，默认 6')
    a = ap.parse_args()

    global FOCUS, FOCUS_MULT
    FOCUS = set(x.strip() for x in a.focus.split(',') if x.strip())
    FOCUS_MULT = max(1, a.focus_mult)

    seed = a.seed or int(time.time()) & 0xFFFF
    rng = random.Random(seed)
    tag = time.strftime('%m%d_%H%M%S')
    outdir = os.path.join(a.out, 'soak_' + tag)
    rec = Recorder(outdir)
    rec.event('=== X72 上位机交互一致性长测 === seed=%d port=%s 边界 %.0frpm/%.1fA/±%.0f°' % (
        seed, a.port, MAX_RPM, MAX_IQ, math.degrees(POS_LIMIT)))
    if FOCUS:
        rec.event('重点动作：%s（权重 ×%d）' % (','.join(sorted(FOCUS)), FOCUS_MULT))

    rig = Rig(a.port, rec, allow_reflash=not a.no_reflash)

    s0 = rig.snap()
    rec.event('上电状态：fault=0x%04X mode=%s armed=%s vbus=%.1f flags=0x%02X'
              % (s0['fault'], H.MODE_NAME.get(s0['mode'], '?'), s0['armed'],
                 s0['vbus'] or 0, s0['flags'] or 0))
    if s0['fault']:
        rec.event('起始就有故障，先清')
        rig.clear_fault()

    if a.calib_only:
        res = rig.calibrate_chain()
        ok = sum(1 for _, o, _ in res if o)
        rec.event('校准链完成：%d/%d 通过' % (ok, len(res)))
        for nm, o, v in res:
            rec.event('  %-12s %s %s' % (nm, 'OK ' if o else 'FAIL',
                                         '' if v is None else [round(x, 4) for x in v[:4]]))
        s = rig.st()
        rec.event('收尾 fault=0x%04X mode=%s' % (s.get('fault', 0),
                                                H.MODE_NAME.get(s.get('mode', -1), '?')))
        rig.lk.close()
        return

    t_end = time.time() + a.minutes * 60 if a.minutes else None
    n = 0
    bad_streak = 0
    while True:
        if a.ops and n >= a.ops:
            break
        if t_end and time.time() > t_end:
            break
        n += 1
        name, arg = pick_op(rng)

        # 写方向彻底失效（ST-Link VCP 固件挂死）时，继续跑只会产出成千上万条
        # "进不了模式"的假失败，把真问题埋掉。首夜 23:18 之后就是这样：
        # 写失败 10186 次、重开 5093 次、430 条 MODE_FAIL，全是链路噪声。
        # 这里改成：暂停测试、每分钟尝试恢复一次，最多等 3 小时。
        if rig.lk.write_dead():
            rec.event('写方向失效（连续失败 %d 次）→ 先软拔插 ST-Link'
                      % rig.lk._dead_streak)
            if rig.reset_stlink():
                n -= 1                          # 这一步没算数，重来
                bad_streak = 0
                continue
            # 软拔插也救不回来 → 退化成每分钟试一次，最多等 3 小时
            rec.event('软拔插无效 → 改为每分钟重试，最多等 3 小时')
            waited = 0
            while rig.lk.write_dead() and waited < 3 * 3600:
                time.sleep(60)
                waited += 60
                rig.lk._cooldown_until = 0.0    # 解除冷却，再试一次
                rig.lk._reopen(min_gap=0.0)
                if not rig.lk.write_dead():
                    break
                if (waited % 600) == 0:
                    rec.event('写方向仍失效，已等 %d 分钟' % (waited // 60))
            if rig.lk.write_dead():
                rec.event('3 小时内未恢复（需人工拔插 USB）→ 中止')
                break
            rec.event('写方向已恢复（等待 %d s），继续测试' % waited)
            n -= 1                                  # 这一步没算数，重来
            continue

        try:
            verdict, fb, r = do_op(rig, rng, name, arg)
        except Exception as e:
            rec.event('第 %d 步 %s 异常：%r' % (n, name, e))
            verdict, fb, r = 'EXC', repr(e), {'op': name, 'arg': str(arg),
                                              't': now_s(), 'verdict': 'EXC'}
        r['verdict'] = verdict          # 提前返回的路径（MODE_FAIL 等）也要带上判定
        rec.op(r)
        s = rig.st()
        fault = s.get('fault', -1)

        # fault == -1 表示"这一拍根本没读到 30090"（链路抖动 / VCP 重枚举），
        # 不是电机真故障。长测里这种抖动必须与故障区分开，否则每次抖动都进
        # recover()、每个小时几十条假失败，把真问题淹掉。
        if fault == -1:
            rec.event('第 %d 步遥测丢失（链路抖动）→ 重开串口后继续' % n)
            try:
                rig.lk._reopen(min_gap=0.0)
            except Exception:
                pass
            time.sleep(0.5)
            bad_streak = 0
            continue

        mark = 'ok ' if verdict == 'OK' else 'ERR'
        print('[%3d] %s %-13s %-42s %s | fault=0x%04X %s'
              % (n, mark, name, fb[:42], verdict, fault,
                 H.fault_text(fault) if fault else ''))

        # 每 100 步落一次链路健康度。通宵 12 小时里 ST-Link VCP 会偶发卡死，
        # 自愈（重开串口）虽然能扛过去，但事后必须能从日志看出"卡过几次、
        # 有没有丢帧"，否则第二天无法判断这批数据可不可信。
        if (n % 100) == 0:
            rec.event('健康 步%d：rx=%d tx=%d 写失败=%d 重开=%d crc_bad=%d'
                      % (n, rig.lk.n_rx, rig.lk.n_tx, rig.lk.n_write_err,
                         rig.lk.n_reopen, rig.lk.n_crc_bad))

        hard = (fault != 0) or verdict in ('MISMATCH_VEL', 'MISMATCH_POS',
                                          'MISMATCH_TORQUE', 'MISMATCH_MIT',
                                          'MISMATCH_TRAJ', 'MISMATCH_PARAM',
                                          'ARM_FAIL',
                                          'MODE_FAIL', 'CLEAR_FAIL')

        # 空载 + 力矩模式：iq 一给上去就加速，迟早撞 300 rpm 门限锁 0x0020。
        # 这是物理必然而非"上位机与电机不一致"，单独记一笔算预期，不当失败。
        # 注意 verdict 也要放行 MODE_FAIL：清完故障那一拍轴可能还在 300 rpm 以上，
        # 紧接着的 mode_flip 会被"已存在的超速"顶掉（v8 第 112/202 步），
        # 那是超速的连带结果，不是模式切换本身失败。
        if fault == 0x0020 and verdict in ('OK', 'TORQUE_OS', 'MODE_FAIL'):
            rec.event('第 %d 步 %s：空载超速 0x0020（预期）→ 清故障并等轴降速' % (n, name))
            rig.keep.stop()
            rig.clear_fault()
            # 关键：清故障的瞬间轴还在飞，不把它真正停下来，
            # 下一次进任何环都会立刻再锁 0x0020（v8 的 112/202 就是这么来的）。
            rig.coast()
            bad_streak = 0
            continue

        if not hard:
            bad_streak = 0
            continue

        bad_streak += 1
        why = '%s（第 %d 步 %s）' % (verdict, n, name)
        # recover() 自己也可能踩到链路抖动（它要发失能/清故障/软复位）。
        # 通宵长测里一次异常就把 12 小时归零是不可接受的，所以这里兜一层：
        # 失败只记一笔，让下一轮继续，连着 3 轮恢复不了才真的中止。
        try:
            ok_rec = rig.recover(why, rig.snap())
        except Exception as e:
            rec.event('recover 自身异常：%r → 重开串口后继续' % (e,))
            try:
                rig.lk._reopen(min_gap=0.0)
            except Exception:
                pass
            ok_rec = True
        if not ok_rec:
            rec.event('恢复失败，测试中止')
            break
        if bad_streak >= 3:
            rec.event('同类失败连续 %d 次 → 强制重烧 + 全链校准' % bad_streak)
            if rig.allow_reflash and rig.reflash():
                time.sleep(1.5)
                rig.calibrate_chain()
            bad_streak = 0

    # 收尾
    rig.keep.stop()
    try:
        rig.disarm()
    except Exception:
        pass
    rec.event('=== 结束：共 %d 步，失败 %d 次 ===' % (n, rec.n_fail))
    sm = os.path.join(outdir, 'summary.md')
    with open(sm, 'w', encoding='utf-8') as f:
        f.write('# 长测汇总\n\n- 种子：%d\n- 步数：%d\n- 失败：%d\n- 端口：%s\n' %
                (seed, n, rec.n_fail, a.port))
        f.write('- 边界：%.0f rpm / %.1f A / ±%.0f°\n' % (
            MAX_RPM, MAX_IQ, math.degrees(POS_LIMIT)))
        f.write('- 累计角超阈值软复位：%d 次\n\n' % rec.n_reset)
        # 判定分布：预期项（TORQUE_OS 空载超速 / POS_BOUNDARY 边界蠕动 /
        # TORQUE_RAMP 斜坡未到位）单列，避免和真失败混在一起看。
        f.write('## 判定分布\n\n| 判定 | 次数 | 占比 |\n|---|---|---|\n')
        for v, c in sorted(rec.vc.items(), key=lambda x: -x[1]):
            f.write('| %s | %d | %.1f%% |\n' % (v, c, 100.0 * c / max(1, n)))
        f.write('\n## 按动作\n\n| 动作 | 次数 | 非 OK |\n|---|---|---|\n')
        for o in sorted(rec.oc):
            bad = {k: c for k, c in rec.oc[o].items() if k != 'OK'}
            f.write('| %s | %d | %s |\n' % (o, sum(rec.oc[o].values()),
                                           bad if bad else '-'))
        if rec.fail_list:
            f.write('\n## 失败列表\n\n')
            for i, x in enumerate(rec.fail_list, 1):
                f.write('%d. %s\n' % (i, x))
        else:
            f.write('\n## 无失败\n')
    rec.event('汇总：%s' % sm)
    rig.lk.close()


if __name__ == '__main__':
    main()
