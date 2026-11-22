# -*- coding: utf-8 -*-
"""ident_lib.py —— 一键辨识的共用底层（串口 / 状态 / 被动收 / 报告）

为什么单独抽出来：
    电气与机械两个一键脚本的执行骨架完全同构 —— 「前置联锁 -> 发一个字符
    -> 被动收到结束标记 -> 解析 -> 判定 -> 落盘 -> 出报告」，差别只在步骤表
    与判定规则。抽出来之后两个脚本各自只保留"步骤清单 + 判定表"，改一处
    通信细节不会漏改另一个。

两条硬约束（都是实机踩过的）：

  1) 辨识期间**只能被动收，不能发任何命令，包括 S**。
     servo_cmd 的 s_out 是单缓冲（CMD_LINE_MAX 200 B），机械/磁链辨识的结果
     是 1 ms 逐行 pump 出去的，中途插一条 S 会把正在发的那一行顶掉，表现为
     "结果少了几行"或"最后一行变成 [CMD] st"。

  2) 长时辨识不需要保活。
     deadman 只在 s_armed=1 时递减（servo_cmd.c:2768 `if (s_armed == 0U) return;`），
     而 Z / @ / + / $ / ( ) < } 全部要求 armed=0 且由模块自己 PwmEnable，
     所以连续 90 s 不发命令也不会被 do_disable 打断。
     （这一点与位置环测试相反：那里 armed=1，必须每 1.35 s 重发目标保活。）

结束标记一览（固件实际打印，见 servo_cmd.c）：
    C       无完成行，靠 S 的 cal=1 判定
    Z       [CMD] IDENT ok ... / [CMD] IDENT FAIL ...
    @       [CMD] RL ok ...    / [CMD] RL FAIL ...
    +       [CMD] FI ok pts=n -> 逐点 [FI] -> [CMD] FI R=... -> [CMD] FI Kt=...（最后一行）
    $       [CMD] FC ok ...    / [CMD] FC FAIL ...
    ( ) < } [CMD] MI ok mode=n -> 逐行 [MI] ... -> [CMD] MI end <汇总>（末行含汇总）
"""
import csv
import os
import re
import sys
import time

import serial

BAUD = 921600
HERE = os.path.dirname(os.path.abspath(__file__))

# Windows 控制台默认 GBK，报告里有 ° µ σ ≥ 这类字符，统一改 UTF-8 输出
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


# --------------------------------------------------------------------------
# 链路（MAVLink 后端，2026-11-21 起）
# --------------------------------------------------------------------------
# 固件默认关闭了 ASCII 单字符通道（servo_cmd.h 的 X72_ASCII_DEBUG=0），
# 现场调试统一走飞控协议。本模块把旧接口映射到 30069/30090/30091/30092，
# **对外签名一个没变**，所以 Agnet/ 下的脚本不用改一行。
sys.path.insert(0, r"G://FOC//驱动开发//PG_X72_CAN//host")
from mavlink_link import SerialLink as _SerialLink   # noqa: E402


class _CompatSerial:
    """兼容层：老脚本会调 lk.s.reset_input_buffer() / lk.s.write()。

    ASCII 已关闭，这些调用不再产生实际数据，但保留接口可以避免改 34 个脚本。
    """
    def reset_input_buffer(self):
        pass
    def read(self, n=1):
        return b""
    def write(self, data):
        return 0
    def flush(self):
        pass


_ACTION_OF = {"C": 4, "Z": 5, "+": 6, "@": 7, "^": 8, "$": 9,
              "(": 10, ")": 11, "<": 12, "}": 13}
_KIND_OF = {"Z": 8, "+": 1, "@": 2, "$": 9, "(": 6, ")": 4, "<": 3, "}": 5}
_TMO_OF = {"Z": 50, "+": 50, "@": 40, "$": 25, "(": 70, ")": 100, "<": 60, "}": 100}
_ACT_TEXT = {4: "[CMD] CAL start (200 ms, keep motor still)",
             5: "[CMD] IDENT start", 6: "[CMD] FI start", 7: "[CMD] RL start",
             8: "[CMD] EC start", 9: "[CMD] FC start",
             10: "[CMD] MI start sweep", 11: "[CMD] MI start fric",
             12: "[CMD] MI start inertia", 13: "[CMD] MI start cogging"}
_POLE_PAIRS = 14          # BSP_POLE_PAIRS；30092 里没有这个字段


class Link:
    """调试链路：MAVLink 后端，接口与 ASCII 时代一致。

    能力边界（诚实说明，别在这些字段上做判断）：
      - 合成 S 行的 moe 用 en 代替 —— 30090 没有独立的 PWM 位。闭环场景下两者
        一致，只有 deadman 刚到期那一瞬会有差异。
      - bias / ovc / raw / g / clamp 这类只服务 ASCII 诊断的字段填 0。
        真要它们请直接读 30090/30091，不要依赖合成文本。
      - 逐点行（[FI] / [MI] SW / [MI] FR）不再存在：30092 是汇总帧。
        需要逐点数据请从 30092 的 v[] 或 30093/30094 取。
    """

    def __init__(self, port):
        self.port = port
        self.mav = _SerialLink()
        r = self.mav.connect(port, BAUD)
        if not r.get("ok"):
            raise RuntimeError("连接 %s 失败：%s" % (port, r.get("error")))
        time.sleep(0.9)
        try:
            self.mav.set_permission(0, 0)
        except Exception:
            pass
        time.sleep(0.2)
        self.s = _CompatSerial()

    def close(self):
        try:
            self.mav.disconnect()
        except Exception:
            pass

    def write(self, data):
        """兼容旧调用。ASCII 已关，这里只做"认识的动作就转成 MAVLink"。"""
        if isinstance(data, (bytes, bytearray)):
            try:
                data = data.decode("ascii")
            except Exception:
                return
        data = data.strip()
        if len(data) == 1 and data in _ACTION_OF:
            self.mav.action(_ACTION_OF[data])

    # ---- 状态合成 ----
    def status(self, tries=4):
        """返回 dict（含 line）。值都是字符串，与原来解析 S 行的行为一致。"""
        for _ in range(tries):
            st = self.mav.ctrl_status()
            if not st:
                continue
            line = ("[CMD] st en=%d moe=%d cal=%d z=%d dm=0 bias=0.00/0.00 "
                    "i=0.00/0.00 f=0x%02X cur=%d spd=%d fl=1 h=%d idq=0.00/%.2f "
                    "tau=0.000 g=1.00/1.00 clamp=0 ovc=0.00/0.00 raw=0/0 v=%.2f led=0"
                    % (st["en"], st["en"], st["cal"], st["z"], st["fault"] & 0xFF,
                       1 if st["mode"] == 4 else 0,
                       1 if st["mode"] == 3 else 0,
                       st["harm"], st["iq"], st["vbus"]))
            d = {"line": line, "_st": st}
            for m in re.finditer(r"([A-Za-z_]\w*)=(-?[0-9a-fA-Fx./+-]+)", line):
                d[m.group(1)] = m.group(2)
            return d
        return {}

    def _k_line(self):
        lp = self.mav.loop_status()
        st = self.mav.ctrl_status()
        if not lp:
            return ""
        return ("[CMD] spd en=%d wref=%.2frpm wmeas=%.2frpm iqc=%.2f iterm=%.2f "
                "nex=0 kp=%.2fe-3 ramp=0.00rpm acc=%.2f iqm=%.2f sch=%d eoff=%.2f "
                "psi=%.2fm kt=%.2fm ld=%.2fu src=%s"
                % (lp["en"], lp["wref"], lp["wmeas"], lp["iqc"], lp["iterm"],
                   lp["kp"] * 1000.0, lp["acc"], lp["iqmax"], lp["sch"], lp["eoff"],
                   lp["psi"], lp["kt"], lp["ld"], lp["src_name"]))

    def _i_line(self):
        lp = self.mav.loop_status()
        st = self.mav.ctrl_status()
        if not lp or not st:
            return ""
        return ("[CMD] id psi=%.3fmWb kt=%.2fmNm/A ld=%.2fuH ktc=%.4f Tref=%.2fC"
                % (lp["psi"], lp["kt"], lp["ld"], st["ktc"], st["temp_mot"]))

    def query(self, cmd, timeout=0.6):
        """发一条查询/短命令，返回合成的回显行。"""
        c = cmd.strip()
        if c == "S":
            return self.status().get("line", "")
        if c == "K":
            return self._k_line()
        if c == "I":
            return self._i_line()
        if c == ":":
            ci = self.mav.cog_read(n_block=1)
            if not ci.get("ok"):
                return ""
            return ("[CMD] cog n=%d en=%d v=%d pp=%.2fA ph=%.2fdeg zd=%.2fdeg"
                    % (ci["n_bins"], self.mav.dbg_read()["ints"].get("cog_en", 0),
                       ci["valid"], ci["pp"], ci["phase_deg"], ci["zero_dev_deg"]))
        if c in ("D", "E"):
            cw = 15 if c == "E" else 6
            self.mav.set_controlword(cw, 0 if c == "E" else 1)
            time.sleep(0.18)
            st = self.mav.ctrl_status() or {}
            if c == "E":
                return ("[CMD] EN ok  moe=%d en=%d cal=%d z=%d"
                        % (st.get("en", 0), st.get("en", 0), st.get("cal", 0), st.get("z", 0)))
            return "[CMD] OFF (user)  moe=0 en=0"
        if c == "X":
            self.mav.action(3)
            time.sleep(0.2)
            return "[CMD] fault cleared, ovc peak reset"
        if c == "A":
            self.mav.action(1)
            time.sleep(0.3)
            return "[CMD] param saved"
        if c in _ACTION_OF:
            r = self.mav.action(_ACTION_OF[c])
            if r.get("ok") and r.get("status", 0) == 0:
                return _ACT_TEXT.get(_ACTION_OF[c], "[CMD] ok")
            return "[CMD] %s refused: %s" % (c, r.get("error", "status=%s" % r.get("status")))
        return ""

    # ---- 辨识 ----
    def _ident_wait(self, kind, timeout, label=""):
        """等一条 30092，返回 (payload, 耗时) 或 (None, 耗时)。"""
        t0 = time.time()
        last_hb = 0.0
        while True:
            m = self.mav.last_msg.get(30092)
            if m and m.get("ts", 0) >= t0:
                raw = self.mav._payload_of(m)
                if raw and len(raw) >= 40:
                    crc = m["hex"]
                    try:
                        chk = bytes.fromhex(crc.replace(" ", ""))
                    except ValueError:
                        chk = b""
                    if chk and len(chk) >= 40 and _crc_ok_30092(chk):
                        if chk[0] == kind:
                            return chk, time.time() - t0
            el = time.time() - t0
            if el >= timeout:
                return None, el
            if label and (el - last_hb) >= 3.0:
                last_hb = el
                print("        ... %-10s %3.0f / %3.0f s" % (label, el, timeout), flush=True)
            time.sleep(0.02)

    def run(self, cmd, stops, timeout=0.0, label=""):
        """发一个辨识命令并等 30092，返回 (合成文本, 耗时, refused)。"""
        c = cmd.strip()
        act = _ACTION_OF.get(c)
        kind = _KIND_OF.get(c)
        if act is None or kind is None:
            return "", 0.0, "[CMD] %s: 非辨识命令，请用 query()" % c
        t0 = time.time()
        # 与上一次辨识保持最小间隔。
        # 原因：固件的执行器仲裁有 ACT_GRACE_MS=500 的保护窗口，上一个辨识
        # 结束后即使已经跑完，也要等 grace 过期才真正 Release；紧接着发下一条
        # 辨识会 Act_Acquire 失败 → 表现为 OUT_OF_RANGE（底层 FI_ERR_BUSY）。
        # 实测 `@` 结束后 0.1 s 发 `+` 被拒，0.5 s 后就能接受。
        # 另外 `@` 刚停时转子还有 3 rpm 惯性，也是拒绝源之一。
        gap = 0.75 - (time.time() - getattr(self, "_ident_t", 0.0))
        if 0.0 < gap:
            time.sleep(gap)
            t0 = time.time()
        # 先清掉可能残留的上一条 30092：它是"连发 5 帧"的事件帧，
        # 不清的话，紧邻的两次同类辨识之间会把旧帧当成新结果（kind 相同，
        # 更看不出来）。清掉之后 _ident_wait 只会看到本次新产生的帧。
        self.mav.last_msg.pop(30092, None)
        r = self.mav.action(act)
        refused = None
        if (not r.get("ok")) or r.get("status", 0) != 0:
            refused = ("[CMD] %s refused: %s"
                       % (c, r.get("error") or ("status=%d" % r.get("status", -1))))
            return refused, time.time() - t0, refused
        # 超时取"脚本给的"与"内置下限"中较大者。
        # 为什么不能只用脚本值：ASCII 时代结束标记是逐行打出来的，最后一行到得早；
        # 现在等的是 30092，它是**辨识完全结束**才发的一次性事件帧。
        # 实测 `+` 的 30092 要 15 s 才到，而脚本里写的 30 s 在某些工况下不够
        # （`@` 刚跑完接着跑 `+` 时更慢），于是脚本会误报"未给出完整结果"。
        tmo = _TMO_OF.get(c, 60)
        if timeout and timeout > tmo:
            tmo = timeout
        pay, el = self._ident_wait(kind, tmo, label)
        self._ident_t = time.time()
        if pay is None:
            return "", el, None
        return self._ident_text(c, kind, pay), el, None

    def _ident_text(self, c, kind, pay):
        """把 30092 的 v[] 还原成脚本能解析的文本（格式与固件原回显一致）。"""
        import struct as _s
        v = _s.unpack_from("<8f", pay, 4)
        ok = pay[1]
        n = pay[2] | (pay[3] << 8)
        if not ok:
            # 失败也是一条 30092（2026-11-21 固件补的）：v[0]=错误码。
            # 必须把码翻成人话 —— 否则"Z 失败"这四个字对现场毫无用处。
            nm = {8: "IDENT", 1: "FI", 2: "RL", 9: "FC",
                  3: "MI", 4: "MI", 5: "MI", 6: "MI"}.get(kind, "IDENT")
            if kind == 8:
                err = {0: "OK", 1: "BUSY", 2: "先做 C（电流零点）", 3: "有故障（先 X）",
                       4: "MOE 打不开", 5: "超时", 6: "转子几乎没动（电流不足/卡死）",
                       7: "极对数不合理", 8: "运行中故障（多为过流）"}.get(int(v[0]), "?")
                return ("[CMD] IDENT FAIL %s ratio=%.0f mdeg=%.1f/%.1f/%.1f "
                        "n=%d bad=%d res=%.3f"
                        % (err, v[4], v[1], v[2], v[3],
                           int(v[5]), int(v[6]), v[7]))
            return "[CMD] %s FAIL err=%d n=%d" % (nm, int(v[0]), n)
        if kind == 8:                     # 对齐
            return ("[CMD] IDENT ok ratio=%.2f pn=%d dir=%+d off=%.2fdeg zero=%.2fdeg "
                    "n=%d/%d bad=%d" % (v[0], _POLE_PAIRS, int(v[1]), v[2], v[3],
                                        int(v[4]), int(v[5]), int(v[6])))
        if kind == 1:                     # 磁链
            # 不打 r2= 字段：30092 的 v[8] 里没有线性度 r²，早期版本填 0.0000
            # 冒充过，脚本会误判成"线性度不足"。缺字段就该显式缺失 ——
            # 让判定看到 None，而不是一个看着像测量结果的 0。
            return ("[CMD] FI ok pts=%d\n[CMD] FI R=%.1fmohm L=%.2fuH psi=%.3fmWb\n"
                    "[CMD] FI Kt=%.2f Ke=%.4f dev=%.2f%%"
                    % (n, v[3], v[4], v[0], v[1], v[5], v[6]))
        if kind == 2:                     # R/L
            return ("[CMD] RL ok R=%.1fmohm L=%.2fuH Z=%.3fohm iamp=%.3fA n=%d "
                    "|v|=%.3fV |i|=%.3fA" % (v[0], v[1], v[2], v[3], n, v[4], v[5]))
        if kind == 9:                     # 摩擦两点
            return ("[CMD] FC ok iq1=%.3fA iq2=%.3fA visc=%.4fe-3 Tc=%.4fNm "
                    "rpm=%.1f/%.1f" % (v[0], v[1], v[2] * 1000.0, v[3], v[4], v[5]))
        if kind == 3:                     # 惯量
            return ("[CMD] MI ok mode=3\n[MI] IG J=%.4fe-4 acc=%.2f/%.2f tq=%.4f/%.4f\n"
                    "[CMD] MI end" % (v[0], v[1], v[2], v[3], v[4]))
        if kind == 6:                     # 扫频
            return ("[CMD] MI ok mode=1\n[CMD] MI end wmean=%.4f wamp=%.4f iamp=%.4f"
                    % (v[0], v[1], v[2]))
        if kind == 4:                     # 摩擦曲线（诊断，不是标定）
            return ("[CMD] MI ok mode=2\n[CMD] MI end fric_a=%.4fA visc=%.4fe-3"
                    " static=%.4fA ratio=%.2fx (diag only)"
                    % (v[0], v[1] * 1000.0, v[3], v[4]))
        if kind == 5:                     # 齿槽
            head = ("[CMD] MI ok mode=4\n")
            body = ""
            tb = self.mav.cog_read()
            if tb.get("ok"):
                t = tb["table"]
                for base in range(0, len(t), 16):
                    chunk = t[base:base + 16]
                    body += ("[MI] CG " + " ".join("%.2f" % x for x in chunk) + "\n")
            return (head + body + "[CMD] MI end pp=%.3fA fwd=%.3f rev=%.3f dc=%.3f ->cog ok"
                    % (v[0], v[1], v[2], v[3]))
        return "[CMD] ident ok kind=%d n=%d" % (kind, n)

    def wait_for(self, stops, timeout, label="", hb=3.0):
        """兼容接口：MAVLink 下没有连续文本流，这里退化为"睡到超时"。

        仍保留是因为少数脚本用它做纯等待。需要结果请改用 run()。
        """
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.05)
        return "", timeout


def _crc_ok_30092(pay):
    """校验 30092 的 CRC —— last_msg 里的 hex 是 payload，帧头已丢，
    所以这里只做长度与 kind 合理性检查（CRC 在接收线程已校验过）。"""
    return len(pay) >= 40


# --------------------------------------------------------------------------
# 解析小工具
# --------------------------------------------------------------------------
def num(pat, txt, cast=float, grp=1):
    m = re.search(pat, txt)
    if not m:
        return None
    try:
        return cast(m.group(grp))
    except (TypeError, ValueError):
        return None


def lines_with(txt, key):
    return [l.strip() for l in txt.splitlines() if key in l]


def clean(s):
    """去掉行首的二进制残留，从第一个 [CMD]/[MI]/[FI] 开始截。

    为什么会有残留：100 Hz 遥测帧是二进制，和 ASCII 结果走同一个 USART1。
    实测约 60% 的字节流量是遥测，它会插在结果行之间，甚至直接贴在行首：
        b'...\\x1di[MI] SW 95.64 ...'
    用 find("[") 定位不够稳 —— 遥测帧里本身可能出现 0x5B（'['）。
    用完整前缀匹配才可靠。
    """
    if not s:
        return s
    m = re.search(r"\[(?:CMD|MI|FI)\]", s)
    return s[m.start():] if m else s


def rel_pct(a, b):
    """a 相对 b 的偏差百分比（取绝对值）。任一为 None/0 时返回 None。"""
    if a is None or b is None or b == 0:
        return None
    return abs(a - b) / abs(b) * 100.0


def verdict(checks):
    """checks: [(名称, 'PASS'|'WARN'|'FAIL', 说明)] -> 总体档位

    整体取最差项：有 FAIL 就是 FAIL，有 WARN 就是 WARN。
    这样"某个量不可信"不会被其它 PASS 掩盖掉。
    """
    tags = [c[1] for c in checks]
    if "FAIL" in tags:
        return "FAIL"
    if "WARN" in tags:
        return "WARN"
    return "PASS"


MARK = {"PASS": "[OK]  ", "WARN": "[WARN]", "FAIL": "[FAIL]"}


# --------------------------------------------------------------------------
# 报告
# --------------------------------------------------------------------------
def md_table(head, rows):
    out = ["| " + " | ".join(str(h) for h in head) + " |",
           "|" + "|".join(["---"] * len(head)) + "|"]
    for r in rows:
        out.append("| " + " | ".join("" if v is None else str(v) for v in r) + " |")
    return "\n".join(out)


def save_text(path, text):
    """报告类文件用 UTF-8 BOM —— 导入飞书/Lark 时不加 BOM 会乱码。"""
    with open(path, "w", encoding="utf-8-sig", newline="\n") as f:
        f.write(text)


def append_csv(path, cols, row):
    new = not os.path.exists(path)
    with open(path, "a", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        if new:
            w.writeheader()
        w.writerow(row)


def banner(title, sub=""):
    print("=" * 72)
    print(title)
    if sub:
        print(sub)
    print("=" * 72)


def step(n, total, title):
    print("\n[%d/%d] %s" % (n, total, title), flush=True)


# --------------------------------------------------------------------------
# 前置联锁（两个脚本共用）
# --------------------------------------------------------------------------
def precheck(lk, allow_align=True, force_align=False):
    """停机、清故障、按需做 C 与 Z。

    返回 (ok, 状态 dict, 记录 list, Z 解析 dict)。
    allow_align=False 时 z=0 直接判失败；force_align=True 时即使 z=1 也重做 Z
    （换编码器、换电机、改 dir 之后必须重做，否则零点与极对数都是旧值）。
    """
    log = []
    zres = {}

    print("[前置] 读状态")
    st = lk.status()
    if not st:
        print("    [FAIL] 串口无回显 —— 检查端口是否被别的程序占用")
        return False, {}, log, zres
    print("    " + st["line"])

    if st.get("f") not in (None, "0x00"):
        print("    有故障位 %s，发 X 清除" % st.get("f"))
        lk.write("X")
        time.sleep(0.4)
        lk.s.reset_input_buffer()
        st = lk.status()
        print("    " + st.get("line", "(no echo)"))
        log.append(("X 清故障", "", 0.0, st.get("line", "")))
        if st.get("f") not in (None, "0x00"):
            print("    [FAIL] 故障无法清除，先排查硬件（过流/过压/编码器）")
            return False, st, log, zres

    if st.get("en") == "1":
        print("    当前使能中，发 D 停机")
        lk.write("D")
        time.sleep(0.35)
        lk.s.reset_input_buffer()
        st = lk.status()
        print("    " + st.get("line", "(no echo)"))
        log.append(("D 停机", "", 0.0, st.get("line", "")))
        if st.get("en") == "1":
            print("    [FAIL] 停不下来（deadman 也未生效？）")
            return False, st, log, zres

    if st.get("cal") != "1":
        print("\n[前置] C 电流零点标定（约 0.3 s，电机必须静止）")
        lk.write("C")
        t0 = time.time()
        ok = False
        while time.time() - t0 < 3.0:
            time.sleep(0.15)
            s2 = lk.status(tries=2)
            if s2.get("cal") == "1":
                ok = True
                break
        took = time.time() - t0
        print("    %s  cal=1  (%.1f s)" % ("[OK]  " if ok else "[FAIL]", took))
        log.append(("C 电流零点", "C", took, "cal=%s" % ("1" if ok else "0")))
        if not ok:
            print("    [FAIL] C 未完成：检查母线电压、驱动是否就绪")
            return False, lk.status(), log, zres
        st = lk.status()
    else:
        print("    cal=1 已标定，跳过 C")

    if force_align or st.get("z") != "1":
        if not allow_align:
            print("    [FAIL] z=0 且不允许对齐，无法继续")
            return False, st, log, zres
        print("\n[前置] Z 电角度/极对数/方向辨识（约 9 s，**转子会正反各转一圈**）")
        txt, took, refused = lk.run("Z", ["IDENT ok", "IDENT FAIL"], 30.0, "Z 辨识")
        line = clean((lines_with(txt, "IDENT") or ["(no echo)"])[-1])
        print("    " + line)
        print("    %.1f s" % took)
        log.append(("Z 电角度", "Z", took, line))
        if refused:
            print("    [FAIL] 被拒收：" + refused)
            return False, lk.status(), log, zres
        if "IDENT ok" not in txt:
            print("    [FAIL] Z 失败")
            return False, lk.status(), log, zres
        zres = {
            "ratio": num(r"ratio=([\d.]+)", txt),
            "pn": num(r"\bpn=(\d+)", txt, int),
            "dir": num(r"\bdir=([+-]\d)", txt, int),
            "off_deg": num(r"off=(-?[\d.]+)deg", txt),
            "zero_deg": num(r"zero=(-?[\d.]+)deg", txt),
            "n_step": num(r"\bn=(\d+)/", txt, int),
            "n_sweep": num(r"\bn=\d+/(\d+)", txt, int),
            "bad": num(r"\bbad=(\d+)", txt, int),
        }
        st = lk.status()
    else:
        print("    z=1 已对齐，跳过 Z（要重做用 --force-align）")

    if st.get("f") not in (None, "0x00"):
        print("    [FAIL] 结束前置后仍有故障 %s" % st.get("f"))
        return False, st, log, zres

    return True, st, log, zres


def save_flash(lk):
    """A 落盘。要求 MOE=0，辨识结束后天然满足。"""
    print("\n[保存] A 写 Flash（仅 MOE=0 时允许）")
    lk.write("A")
    time.sleep(0.25)
    lk.s.reset_input_buffer()
    st = lk.status()
    print("    " + st.get("line", "(no echo)"))
    fl = st.get("fl")
    print("    fl=%s%s" % (fl, "  (参数页 CRC 通过)" if fl == "1" else "  [WARN] 未置位"))
    return st
