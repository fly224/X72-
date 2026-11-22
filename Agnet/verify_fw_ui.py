# -*- coding: utf-8 -*-
"""上位机「固件调试」页 —— 静态回归验证（不接硬件、不开浏览器）

做三件事：
  1) 从 index.html 抽出整段脚本做 node --check 语法检查
  2) 抽出固件调试模块，拼装解析测试（用真实 [CMD] 回显样例）并运行
  3) 用极简 DOM 模拟器跑 mountFw()，确认界面构建链不抛异常

用法：
    python verify_fw_ui.py
    python verify_fw_ui.py --html "C:/Users/92714/WorkBuddy/2026-11-14-12-33-56/servo-web/index.html"

退出码 0 = 全通过。
"""
from __future__ import print_function

import argparse
import io
import os
import subprocess
import sys
import tempfile

DEFAULT_HTML = r"C:/Users/92714/WorkBuddy/2026-11-14-12-33-56/servo-web/index.html"
ANCHOR_START = "const FW_RPM_TAB"
ANCHOR_END = "/* =========================================================================\n * 主循环 & 导航"

# ---------------------------------------------------------------- 测试模板

PARSE_HEAD = r'''
/* 最小 shim：只测解析逻辑，不碰 DOM */
let activeAxis = 0;
let activeModule = "dashboard";
const telemetry = [{ enabled: false, vel: 29.4, iq: 0.35, vbus: 30.7 }];
function toast() {}
const POLE_PAIRS = 14;
function $(s) { return null; }
function el() { return { style: {}, appendChild() {}, setAttribute() {}, querySelector() { return null; } }; }
function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }
// __RING__
'''

PARSE_TAIL = r'''
fwPush = function () {};

let all = true;
function chk(name, cond) { console.log((cond ? "PASS  " : "FAIL  ") + name); all = all && cond; }

/* ---------- S 状态行 ---------- */
fwOnCmdLine("st en=1 moe=1 cal=1 z=1 dm=1998 bias=-0.01/0.02 i=0.00/0.00 f=0x00 cur=1 spd=1 fl=1 h=1 idq=0.31/0.35 g=1.00/1.00 clamp=0");
chk("st: en/moe/cal/z/f/fl 全字段", fw.st.en === "1" && fw.st.moe === "1" && fw.st.cal === "1" &&
    fw.st.z === "1" && fw.st.f === "0x00" && fw.st.fl === "1" && fw.st.dm === "1998");
chk("st: f 与 fl 不串键", fw.st.f === "0x00" && fw.st.fl === "1");
chk("st: spdOn / harmOn 同步", fw.spdOn === true && fw.harmOn === true);
chk("st: 同步 ax.enabled", telemetry[0].enabled === true);

/* ---------- K 速度环详情 ---------- */
fwOnCmdLine("spd en=1 wref=30.00rpm wmeas=29.48rpm iqc=0.352 iterm=0.121 nex=2016 kp=583.38e-3 kiTs=40.76e-3");
chk("K: wref/wmeas", fw.k.wref === 30 && fw.k.wmeas === 29.48);
chk("K: kp=0.58338（不重复乘 1e-3）", Math.abs(fw.k.kp - 0.58338) < 1e-9);
chk("K: kiTs=0.04076", Math.abs(fw.k.kits - 0.04076) < 1e-9);
chk("K: nex=2016", fw.k.nex === 2016);

/* 后加字段单独匹配：混在主正则里会让老固件的行整条解析失败 */
fwOnCmdLine("spd en=1 wref=30.00rpm wmeas=29.48rpm iqc=0.352 iterm=0.121 nex=2016 kp=583.38e-3 kiTs=40.76e-3 ramp=30.00rpm acc=400.00 iqm=3.00");
chk("K: ramp/acc/iqm", Math.abs(fw.k.ramp - 30) < 1e-9 &&
    Math.abs(fw.k.acc - 400) < 1e-9 && Math.abs(fw.k.iqm - 3) < 1e-9);
chk("K: 带 ramp 时 kp 仍正确", Math.abs(fw.k.kp - 0.58338) < 1e-9);

/* ---------- O / L / Y 档位 ---------- */
fwOnCmdLine("spd rpm -> 3.00 (W/Q uses it)");
chk("O: 3 rpm -> 索引 0", fw.rpmIdx === 0);
fwOnCmdLine("spd rpm -> 3000.00 (W/Q uses it)");
chk("O: 3000 rpm -> 索引 5", fw.rpmIdx === 5);
fwOnCmdLine("spd gain -> 32.00x kp=18668.22e-3 kiTs=1304.41e-3");
chk("L: 32x -> 索引 8", fw.gainIdx === 8);
fwOnCmdLine("spd gain reset 1.00x kp=583.38e-3 kiTs=40.76e-3 rpm=30.00");
chk("Y: 双档位复位到 1 / 3", fw.gainIdx === 3 && fw.rpmIdx === 1);

/* ---------- W / D ---------- */
fwOnCmdLine("SPD on: w_ref=30.00rpm kp=583.38e-3 kiTs=40.76e-3 iqmax=3A moe=1");
chk("W: spdOn 置位", fw.spdOn === true);
fwOnCmdLine("OFF (D)  moe=0 en=0");
chk("D: spdOn 清零且 ax.enabled=false", fw.spdOn === false && telemetry[0].enabled === false);

/* ---------- @ R/L ---------- */
fwOnCmdLine("RL ok R=405.09mohm L=80.12uH Z=0.65ohm iamp=1.55A n=60 |v|=0.81V |i|=2.00A moe=0");
chk("RL: R/L/Z/iamp", fw.rl && Math.abs(fw.rl.r - 405.09) < 1e-9 && Math.abs(fw.rl.l - 80.12) < 1e-9 &&
    Math.abs(fw.rl.z - 0.65) < 1e-9 && Math.abs(fw.rl.iamp - 1.55) < 1e-9);

/* ---------- # 注入幅值 ---------- */
fwOnCmdLine("inj amp -> 0.6V (next @ uses it)");
chk("#: 0.6V -> 索引 0", fw.injIdx === 0);
fwOnCmdLine("inj amp -> 2.5V (next @ uses it)");
chk("#: 2.5V -> 索引 3", fw.injIdx === 3);

/* ---------- ^ 编码器自校准 ---------- */
fwOnCmdLine("EC ok turns=51.60 avg=599.90rpm [595.26..595.90] chip=3 -> !!! POWER CYCLE REQUIRED (>6s) !!!");
chk("^: turns/avg/区间/chip", fw.ec && fw.ec.chip === 3 && Math.abs(fw.ec.turns - 51.6) < 1e-9 &&
    fw.ec.lo === 595.26 && fw.ec.hi === 595.9);

/* ---------- % 谐波 ---------- */
fwOnCmdLine("harm ON h5=0.002/0.001 h7=0.003/0.002");
chk("%: ON + h5/h7 残余", fw.harmOn === true && fw.harm && fw.harm.d5 === 0.002 && fw.harm.q7 === 0.002);
fwOnCmdLine("harm OFF h5=0.010/0.009 h7=0.012/0.011");
chk("%: OFF 后状态翻转", fw.harmOn === false);

/* ---------- A / ! ---------- */
fwOnCmdLine("PS saved (bias+zero+ratio+spd gains)");
fwOnCmdLine("PS FAIL db=1 bk=2 pg=126 sz=2048 er=0 pr=0 rb=0 al=0");
chk("A: 成功 / 失败两行都不崩", true);

/* ---------- & 波形 ---------- */
fwOnCmdLine("REC start 1024pt@20kHz=51ms, lines follow");
fw.rec.rpm = 500;
fwOnRecLine("00A4FF9C0123");
chk("REC: hex->int16/131（含负数）", fw.rec.data.length === 3 &&
    Math.abs(fw.rec.data[0] - 164 / 131) < 1e-12 && Math.abs(fw.rec.data[1] + 100 / 131) < 1e-12);
fwOnCmdLine("REC end n=1024");
chk("REC: end 置完成标志", fw.rec.done === true);

/* ---------- 循环键换算全组合 ---------- */
let ok6 = true, ok9 = true;
for (let c = 0; c < 6; c++) for (let t = 0; t < 6; t++) if (((c + ((t - c + 6) % 6)) % 6) !== t) ok6 = false;
for (let c = 0; c < 9; c++) for (let t = 0; t < 9; t++) if (((c + ((t - c + 9) % 9)) % 9) !== t) ok9 = false;
chk("O 循环换算 6x6 全组合", ok6);
chk("L 循环换算 9x9 全组合", ok9);

/* ---------- FFT 幅频精度（合成已知信号） ---------- */
(function () {
  const N = 1024, A = 0.25, f0 = 700;
  fw.rec = { data: [], done: false, rpm: 500 };
  for (let i = 0; i < N; i++) fw.rec.data.push(A * Math.sin(2 * Math.PI * f0 * i / FW_REC_FS));
  fwBuildSpec();
  const s = fw.spec, df = FW_REC_FS / N;
  chk("FFT: 峰值频率 " + s.pkF.toFixed(1) + " Hz（真值 700，分辨率 " + df.toFixed(1) + "）",
      Math.abs(s.pkF - f0) <= df);
  const ampErr = Math.abs(s.pk - A) / A;
  chk("FFT: 峰值幅度 " + s.pk.toFixed(4) + " A（真值 0.25，误差 " + (ampErr * 100).toFixed(1) + "%）", ampErr < 0.02);
  chk("FFT: 6 次频点 " + s.f6.toFixed(0) + " Hz（rpm=500 -> 700）", Math.abs(s.f6 - 700) < 1e-6);
})();

console.log("\n" + (all ? "===== PARSE ALL PASS =====" : "===== PARSE 有 FAIL ====="));
process.exit(all ? 0 : 1);
'''

MOUNT_HEAD = r'''
/* 极简 DOM 模拟：只为验证 mountFw() 的构建链不抛异常 */
class FakeEl {
  constructor(tag) { this.tag = tag; this.children = []; this.style = {}; this.attrs = {}; this.className = ""; this._tc = ""; }
  appendChild(c) { this.children.push(c); return c; }
  setAttribute(k, v) { this.attrs[k] = v; }
  addEventListener() {}
  querySelector() { return null; }
  getContext() { return new Proxy({}, { get: () => () => {} }); }
  get clientWidth() { return 600; }
  get clientHeight() { return 150; }
  set innerHTML(v) { this.children = []; this._html = v; }
  get innerHTML() { return this._html || ""; }
  set textContent(v) { this._tc = v; this.children = []; }
  get textContent() { return this._tc; }
}
const _root = new FakeEl("section");
const document = {
  createElement: t => new FakeEl(t),
  createTextNode: t => ({ text: t }),
  querySelector: s => (s === "#m-fw" ? _root : null),
};
const window = { devicePixelRatio: 1 };
const performance = { now: () => Date.now() };

const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v)) ? "--" : Number(v).toFixed(d);
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const $ = (s) => document.querySelector(s);
const el = (tag, props = {}, kids = []) => {
  const e = document.createElement(tag);
  for (const k in props) {
    const v = props[k];
    if (v === null || v === undefined) continue;
    if (k === "class") e.className = v;
    else if (k === "html") e.innerHTML = v;
    else if (k.startsWith("on") && typeof v === "function") e.addEventListener(k.slice(2), v);
    else e.setAttribute(k, v);
  }
  (Array.isArray(kids) ? kids : [kids]).forEach(c => c && e.appendChild(typeof c === "string" ? document.createTextNode(c) : c));
  return e;
};

let activeAxis = 0, activeModule = "fw", source = "none";
const telemetry = [{ name: "X72", id: 0, enabled: false, vel: 29.4, iq: 0.35, vbus: 30.7, rxT: Date.now() }];
function isStale(ax) { return !ax || !ax.rxT || (performance.now() - ax.rxT) > 1500; }
let _linkOverride = null;
function curLink() { return _linkOverride; }
function curLinkOverride() { _linkOverride = globalThis.serial; }
function toast() {}
const POLE_PAIRS = 14;
// __RING__
'''

MOUNT_TAIL = r'''
let ok = true;

try { mountFw(); console.log("PASS  mountFw() 构建完成，顶层块 " + _root.children.length + " 个"); }
catch (e) { ok = false; console.log("FAIL  mountFw(): " + e.message); }

(function () {
  let nCard = 0, nBtn = 0, nCanvas = 0, nChip = 0;
  (function walk(n) {
    if (!n || !n.children) return;
    for (const c of n.children) {
      if (!c || !c.tag) continue;
      if (c.className === "card") nCard++;
      if (c.className === "fw-chip") nChip++;
      if (c.tag === "button") nBtn++;
      if (c.tag === "canvas") nCanvas++;
      walk(c);
    }
  })(_root);
  console.log("      卡片 " + nCard + " / 按钮 " + nBtn + " / canvas " + nCanvas + " / chip " + nChip);
  if (nCard !== 5) { ok = false; console.log("FAIL  卡片数应为 5"); }
  if (nCanvas !== 3) { ok = false; console.log("FAIL  canvas 应为 3（跟踪 + 时域 + 频谱）"); }
  if (nBtn < 18) { ok = false; console.log("FAIL  按钮应 >=18（含速度/前馈/标定各组）"); }
  if (nChip !== 22) { ok = false; console.log("FAIL  chip 应为 22（7 命令 + 6 转速 + 9 增益）"); }
})();

try { tickFw(); tickFw(); console.log("PASS  tickFw() 反复调用无异常"); }
catch (e) { ok = false; console.log("FAIL  tickFw(): " + e.message); }

try { fwSetRpm(3000); fwSetGain(8); fwSetGain(3); console.log("PASS  档位函数可用"); }
catch (e) { ok = false; console.log("FAIL  档位函数: " + e.message); }

if (fwSend("W") !== false) { ok = false; console.log("FAIL  无串口时应拒绝"); }
else console.log("PASS  无串口时 fwSend 返回 false");

for (let i = 0; i < 30; i++) fwQ.items.push("X");
if (fwSend("W") !== false) { ok = false; console.log("FAIL  队列满时应拒绝"); }
else console.log("PASS  队列 >24 时拒绝新命令");
fwQ.items.length = 0;

/* 数值命令必须拼 CR：固件解析器只在收到 \r 时才 apply，否则 300 ms 后超时丢弃。
 * 注意这里每次手动清 fwQ.busy —— 队列有 60 ms 间隔（靠 setTimeout 复位），
 * 同步连续发三条时后两条只会入队，不会真的写串口。 */
(function () {
  let lastSent = "";
  source = "serial";
  globalThis.serial = { sendRawAscii: (t) => { lastSent = t; return Promise.resolve(); } };
  curLinkOverride();

  const one = (prefix, v, expect) => {
    fwQ.busy = false;
    fwQ.items.length = 0;
    lastSent = "";
    fwSendNum(prefix, v);
    return lastSent === expect;
  };

  const a = one("=", 30, "=30\r");
  const b = one("~", 400.5, "~400.5\r");
  const c = one("|", 3, "|3\r");
  const d = one("=", -30, "=-30\r");
  const e = one("=", 1234.5678, "=1234.568\r");   // 三位小数取整
  const f = one("[", 0.32, "[0.32\r");
  const g = one("]", 17.2, "]17.2\r");
  const h = one("{", 1.0, "{1\r");

  console.log((a && b && c && d && e && f && g && h ? "PASS" : "FAIL") +
              "  fwSendNum 拼接 CR：=30 / ~400.5 / |3 / =-30 / 三位小数 -> " +
              JSON.stringify([a, b, c, d, e, f, g, h]));

  if (!(a && b && c && d && e && f && g && h)) ok = false;
  fwQ.busy = false; fwQ.items.length = 0;
  source = "none";
  globalThis.serial = null;
})();

console.log("\n" + (ok ? "===== MOUNT ALL PASS =====" : "===== MOUNT 有 FAIL ====="));
process.exit(ok ? 0 : 1);
'''


# ---------------------------------------------------------------- 工具

def find_script(src):
    """返回 <script> 与 </script> 之间的 JS 源码。"""
    a = src.index("<script>") + len("<script>")
    b = src.index("</script>", a)
    return src[a:b]


def extract_fw_block(src):
    a = src.index(ANCHOR_START)
    b = src.index(ANCHOR_END)
    return src[a:b]


def extract_braced(src, marker):
    """按花括号配对截取一段（class / 多行函数体）。"""
    i = src.index(marker)
    depth = 0
    j = i
    while j < len(src):
        if src[j] == "{":
            depth += 1
        elif src[j] == "}":
            depth -= 1
            if depth == 0:
                return src[i:j + 1]
        j += 1
    raise ValueError(marker)


def extract_fft(src):
    i = src.index("function fftRadix2(re, im) {")
    j = src.index("\n}\n", i) + 3
    return src[i:j]


def run_node(path, title):
    print("\n---- %s ----" % title)
    r = subprocess.run(["node", path], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out = r.stdout.decode("utf-8", "replace")
    print(out.rstrip())
    return r.returncode == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--html", default=DEFAULT_HTML)
    ap.add_argument("--keep", action="store_true", help="保留生成的临时 JS")
    args = ap.parse_args()

    if not os.path.isfile(args.html):
        print("找不到页面文件：%s" % args.html)
        return 2

    src = io.open(args.html, encoding="utf-8").read()
    tmpdir = os.path.join(os.path.dirname(os.path.abspath(args.html)), ".fw_ui_test")
    if not os.path.isdir(tmpdir):
        os.makedirs(tmpdir)

    ok = True

    # 1) 语法
    js_all = find_script(src)
    p_syn = os.path.join(tmpdir, "all.js")
    io.open(p_syn, "w", encoding="utf-8").write(js_all)
    print("---- 语法检查（%d 行） ----" % js_all.count("\n"))
    r = subprocess.run(["node", "--check", p_syn], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if r.returncode == 0:
        print("PASS  全脚本语法 OK")
    else:
        ok = False
        print("FAIL  语法：" + r.stdout.decode("utf-8", "replace")[:800])

    fw_block = extract_fw_block(src)
    fft_src = extract_fft(src)
    ring_src = extract_braced(src, "class Ring {")
    print("      fw 模块 %d 行 / %d 字符" % (fw_block.count("\n"), len(fw_block)))

    # 2) 解析测试
    p_parse = os.path.join(tmpdir, "parse_test.js")
    io.open(p_parse, "w", encoding="utf-8").write(
        PARSE_HEAD.replace("// __RING__", ring_src) + fft_src + fw_block + PARSE_TAIL)
    ok = run_node(p_parse, "回显解析 + FFT") and ok

    # 3) 挂载测试
    p_mount = os.path.join(tmpdir, "mount_test.js")
    io.open(p_mount, "w", encoding="utf-8").write(
        MOUNT_HEAD.replace("// __RING__", ring_src) + fft_src + fw_block + MOUNT_TAIL)
    ok = run_node(p_mount, "界面构建（DOM 模拟）") and ok

    if not args.keep:
        for f in (p_syn, p_parse, p_mount):
            try:
                os.remove(f)
            except OSError:
                pass

    print("\n" + ("=" * 46))
    print("总体：" + ("ALL PASS" if ok else "有 FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
