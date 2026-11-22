// 用真实抓包回放「页面自己的」解析链路，并与 Python 独立解码结果交叉比对：
//   class SerialLink.feed/parse（含 ASCII 行分支）→ decodeFrame → handleAsciiLine → updateKeyParams
// 断言落在「关键参数」面板每个格子的最终文本上。
// 用法：node verify_host_keyparams.js <capture.bin> <expected.json> [chunkBytes]
const fs = require("fs");

const HTML = "C:/Users/92714/WorkBuddy/2026-11-14-12-33-56/servo-web/index.html";
const BIN = process.argv[2];
const EXPECT = process.argv[3];
const CHUNK = Number(process.argv[4] || 4096);
if (!BIN || !EXPECT) { console.error("用法: node verify_host_keyparams.js <capture.bin> <expected.json> [chunk]"); process.exit(2); }

const exp = JSON.parse(fs.readFileSync(EXPECT, "utf8"));
const html = fs.readFileSync(HTML, "utf8");
const scripts = [...html.matchAll(/<script[^>]*>([\s\S]*?)<\/script>/g)].map(m => m[1]);
const js = scripts.find(s => s.includes("function decodeFrame"));
if (!js) throw new Error("未找到含 decodeFrame 的内联脚本");

function extractFn(src, name) {
  const i = src.indexOf("function " + name);
  if (i < 0) throw new Error("未找到 function " + name);
  let d = 0, k = src.indexOf("{", i);
  for (; k < src.length; k++) {
    if (src[k] === "{") d++;
    else if (src[k] === "}") { d--; if (d === 0) { k++; break; } }
  }
  return src.slice(i, k);
}
function extractClass(src, name) {
  const i = src.indexOf("class " + name + " ");
  if (i < 0) throw new Error("未找到 class " + name);
  let d = 0, started = false, k = i;
  for (; k < src.length; k++) {
    if (src[k] === "{") { d++; started = true; }
    else if (src[k] === "}") { d--; if (started && d === 0) { k++; break; } }
  }
  return src.slice(i, k);
}

// ---- 可控时钟：按块推进，让 fps / 新鲜度判定可复现 ----
let T = 1e6;
globalThis.performance = { now: () => T };

globalThis.POLE_PAIRS = 14;
globalThis.fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v)) ? "--" : Number(v).toFixed(d);
const STALE_MS = 1500;
globalThis.STALE_MS = STALE_MS;
globalThis.isStale = (ax) => !ax || !ax.rxT || (performance.now() - ax.rxT) > STALE_MS;
globalThis.pushSample = () => {};
globalThis.paramValues = {};
globalThis.FAULT_MSGS = {};
globalThis.logFault = () => {};

globalThis.CFG = { axes: [{ id: 0, name: "J1" }] };
globalThis.telemetry = [
  { id: 0, name: "J1", mode: 2, enabled: false, status: 0, fault: 0, vel: 0, iq: 0, id: 0, torque: 0,
    pos: 0, temp: 25, vbus: 48, ia: 0, ib: 0, ic: 0, elecTheta: 0, iqTar: 0, idTar: 0,
    encSum: 0, turns: 0, hasPos: false, elecOmega: 0, uabcTar: [0, 0, 0], uabcTarOrg: [0, 0, 0],
    iqOffset: 0, diag: null, diagT: 0, rxT: 0, lastRingT: 0, online: false },
];
globalThis.activeAxis = 0;
globalThis.MSG_ID = { TELEMETRY: 1, STATUS: 2, FAULT: 3, PARAM: 16, ACK: 17, CUR_IN: 30025, CUR_OUT: 30027, POS_SENSOR: 30040 };

// 页面的 CRC_EXTRA 表与 mavCrc 实现：用于验证接收侧「软 CRC 校验」与 Python 独立实现字节级一致
globalThis.CRC_EXTRA = eval("(" + js.match(/const CRC_EXTRA = (\{[^}]*\});/)[1] + ")");

// 页面里 const linkStat 的形状在此复刻（页面自身实现，仅变量作用域需提升为 global）
const lsSrc = js.match(/const linkStat = (\{[^}]*\});/)[1];
globalThis.linkStat = eval("(" + lsSrc + ")");

const kpStart = js.indexOf("const KP_DEFS");
const kpEnd = js.indexOf("];", kpStart) + 2;
globalThis.KP_DEFS = eval("(" + js.slice(kpStart, kpEnd).replace(/^const KP_DEFS = /, "").replace(/;$/, "") + ")");

let elCount = 0;
globalThis.el = (tag, props = {}, kids = []) => {
  elCount++;
  const n = { tag, textContent: "", children: [], style: {}, appendChild(c) { this.children.push(c); } };
  for (const k of (Array.isArray(kids) ? kids : [kids])) if (typeof k === "string") n.textContent += k;
  return n;
};

eval(extractFn(js, "mavCrc"));
eval(extractFn(js, "decodeFrame"));
eval(extractFn(js, "handleAsciiLine"));
eval(extractFn(js, "asciiLineEnd"));
eval(extractFn(js, "noteFrameSeq"));
eval(extractFn(js, "buildKeyParams"));
eval(extractFn(js, "updateKeyParams"));
const SerialLink = eval("(" + extractClass(js, "SerialLink") + ")");

// ---- 回放（分块 + 推进时钟；顺带验证 ASCII 行跨块的边界处理） ----
const raw = fs.readFileSync(BIN);
const lk = new SerialLink();
for (let off = 0; off < raw.length; off += CHUNK) {
  lk.feed(raw.slice(off, off + CHUNK));
  T += 300;                      // 每块 60 ms（仅用于 fps/新鲜度判定，不参与字段解码）
}
T += 5;                         // 最后一帧之后 5 ms

const root = { children: [], appendChild(c) { this.children.push(c); } };
const nodes = buildKeyParams(root);
const ax = telemetry[0];
updateKeyParams(nodes, ax);
const got = {};
for (const k of Object.keys(nodes)) got[k] = nodes[k].textContent;

const checks = [];
const ck = (name, cond, detail) => checks.push({ name, ok: !!cond, detail: String(detail) });
const near = (a, b, tol) => Math.abs(Number(a) - Number(b)) <= tol;
const id = exp.by_id || {};
const i25 = id["30025"] || {}, i27 = id["30027"] || {}, i40 = id["30040"] || {};

ck("帧数 页面 == Python", linkStat.frames === exp.frames, linkStat.frames + " vs " + exp.frames);
ck("seq 缺失帧数 == Python", linkStat.gapMissing === exp.seq_gaps, linkStat.gapMissing + " vs " + exp.seq_gaps);
// 页面软 CRC（标准口径）与 Python 双口径统计必须完全对齐：
// 旧固件抓包 → crcErr == fw_magic_included；新固件 → crcOk == std_mavlink
if (exp.crc_hits) {
  ck("CRC 通过数 == Python std 口径", linkStat.crcOk === exp.crc_hits.std_mavlink,
     linkStat.crcOk + " vs " + exp.crc_hits.std_mavlink);
  ck("CRC 失败数 == Python 旧口径命中数", linkStat.crcErr === exp.crc_hits.fw_magic_included,
     linkStat.crcErr + " vs " + exp.crc_hits.fw_magic_included);
}
ck("30040 已收到", ax.hasPos === true, ax.hasPos);

ck("母线 页面≈Python", near(ax.vbus, i25.vbus, 1e-3), ax.vbus + " vs " + i25.vbus);
ck("iu 页面≈Python", near(ax.ia, i25.iu, 1e-3), ax.ia + " vs " + i25.iu);
ck("iv 页面≈Python", near(ax.ib, i25.iv, 1e-3), ax.ib + " vs " + i25.iv);
ck("iw 页面≈Python", near(ax.ic, i25.iw, 1e-3), ax.ic + " vs " + i25.iw);
ck("电角度 页面≈Python", near(ax.elecTheta, i25.elec_theta, 1e-3), ax.elecTheta + " vs " + i25.elec_theta);
ck("Id 页面≈Python", near(ax.id, i27.id, 1e-3), ax.id + " vs " + i27.id);
ck("Iq 页面≈Python", near(ax.iq, i27.iq, 1e-3), ax.iq + " vs " + i27.iq);
ck("累积脉冲 页面≈Python", ax.encSum === i40.enc_sum, ax.encSum + " vs " + i40.enc_sum);
ck("圈数 页面≈Python", near(ax.turns, i40.turns, 1e-6), ax.turns + " vs " + i40.turns);
ck("机械角 页面≈Python", near(ax.pos * 180 / Math.PI, i40.mech_deg, 1e-3),
   (ax.pos * 180 / Math.PI).toFixed(4) + " vs " + i40.mech_deg);

// [DIAG] 只在真机开机 3 s 打一次；用纯帧流（如固件 harness 输出）回放时不存在，此时跳过相关断言
if (ax.diag) {
  ck("[DIAG] rate=10.0 kHz", near(ax.diag.rateKHz, 10.0, 1e-6), ax.diag.rateKHz);
  ck("[DIAG] step max=8862", ax.diag.stepMax === 8862, ax.diag.stepMax);
  ck("[DIAG] cpu=52.1%", near(ax.diag.cpuPct, 52.1, 0.05), ax.diag.cpuPct);
  ck("[DIAG] model=7426", ax.diag.hot && ax.diag.hot.model === 7426, ax.diag.hot && ax.diag.hot.model);
  ck("[DIAG] adc1/adc2=60018/30009", ax.diag.adc1 === 60018 && ax.diag.adc2 === 30009,
     ax.diag.adc1 + "/" + ax.diag.adc2);
  // busy=0xNN 是 2026-11-15 起新增字段：旧固件无 → "--"；新固件正常 → "无 (0x00)"；异常 → "0xNN"
  ck("面板-启动失败现场文本合法",
     got.kpBusy === "--" || got.kpBusy === "无 (0x00)" || /^0x[0-9a-f]{2}$/.test(got.kpBusy), got.kpBusy);
}

// 面板文本 = 页面 fmt 规则（把 ax 值按同规则格式化后比对，避免依赖具体数值）
ck("面板-母线文本", got.kpVbus === fmt(ax.vbus, 2), got.kpVbus);
ck("面板-iu 文本", got.kpIa === fmt(ax.ia, 3), got.kpIa);
ck("面板-累积脉冲文本", got.kpEncP === String(ax.encSum), got.kpEncP);
if (ax.diag) {
  ck("面板-控制节拍文本", got.kpRate === fmt(ax.diag.rateKHz, 1), got.kpRate);
  ck("面板-单拍峰值文本", got.kpStep === String(ax.diag.stepMax), got.kpStep);
  ck("面板-CPU 文本", got.kpCpu === fmt(ax.diag.cpuPct, 1), got.kpCpu);
  ck("面板-model 文本", got.kpModel === String(ax.diag.hot.model), got.kpModel);
  ck("面板-ADC 文本", got.kpAdc === ax.diag.adc1 + " : " + ax.diag.adc2, got.kpAdc);
} else {
  ck("面板-无 DIAG 时显示未收到", got.kpRate === "未收到" && got.kpBusy === "未收到", got.kpRate + " / " + got.kpBusy);
}
ck("面板-占空比 = (u/vbus+0.5)*100", got.kpDa === fmt((ax.uabcTar[0] / ax.vbus + 0.5) * 100, 2), got.kpDa);
// 双温度：Python 侧解出的值与页面面板文本必须一致（None = NaN/无该字段 → "--"）
ck("面板-驱动器温度", got.kpTempGan === (i25.driver_temp == null ? "--" : fmt(i25.driver_temp, 1)),
   got.kpTempGan + " vs " + i25.driver_temp);
ck("面板-电机温度", got.kpTempMotor === (i25.motor_temp == null ? "--" : fmt(i25.motor_temp, 1)),
   got.kpTempMotor + " vs " + i25.motor_temp);
ck("面板-收帧率有值", /^\d+\.\d$/.test(got.kpFps), got.kpFps);
ck("面板-跳变文本含次数", /\d+ 次 \/ 缺 \d+/.test(got.kpSeqGap), got.kpSeqGap);
ck("面板-CRC 文本含通过数", got.kpCrc && got.kpCrc.indexOf(String(linkStat.crcOk)) === 0, got.kpCrc);

let fail = 0;
for (const c of checks) { if (!c.ok) fail++; console.log((c.ok ? "[PASS] " : "[FAIL] ") + c.name + "  → " + c.detail); }
console.log("---- 面板快照 ----");
console.log(JSON.stringify(got, null, 1));
console.log("RESULT " + (checks.length - fail) + "/" + checks.length + (fail ? "  FAIL" : "  ALL PASS"));
process.exit(fail ? 1 : 0);
