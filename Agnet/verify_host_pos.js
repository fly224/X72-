// 从 servo-web/index.html 抽出真实 decodeFrame，回放 30040 载荷，验证位置换算。
// 用法：node verify_host_pos.js <30040 payload hex> [htmlPath]
// 说明：载荷来自固件 telemetry_tx.c 的真实组帧输出（verify_fw_frame_harness.c），
//       解码走页面里的 decodeFrame 原文，因此是"固件 → 协议布局 → 上位机"的端到端校验。
const fs = require("fs");

const HTML = process.argv[3] || "C:/Users/92714/WorkBuddy/2026-11-14-12-33-56/servo-web/index.html";
const html = fs.readFileSync(HTML, "utf8");
const m = html.match(/<script[^>]*>([\s\S]*?)<\/script>/);
if (!m) throw new Error("未找到内联脚本");
const js = m[1];

function extractFn(src, name) {
  const i = src.indexOf("function " + name);
  if (i < 0) throw new Error("未找到函数 " + name);
  let depth = 0, k = src.indexOf("{", i);
  for (; k < src.length; k++) {
    if (src[k] === "{") depth++;
    else if (src[k] === "}") { depth--; if (depth === 0) { k++; break; } }
  }
  return src.slice(i, k);
}

// 页面的 MSG_ID 原文（含 POS_SENSOR），直接取出来用，避免两处硬编码漂移
const midMatch = js.match(/const MSG_ID\s*=\s*\{[\s\S]*?\};/);
if (!midMatch) throw new Error("未找到 MSG_ID");
const MSG_ID_SRC = midMatch[0];
const MSG_ID = new Function(MSG_ID_SRC + "; return MSG_ID;")();   // const 是块级作用域，不能靠 eval 外泄
globalThis.MSG_ID = MSG_ID;
if (MSG_ID.POS_SENSOR !== 30040) throw new Error("MSG_ID.POS_SENSOR 不是 30040： " + MSG_ID.POS_SENSOR);

// ---- 最小运行环境（只补齐 decodeFrame 依赖） ----
const CFG = { channels: ["vel", "iq", "id", "torque", "pos", "temp", "vbus"],
              axes: [{ id: 0, name: "J1" }] };
const telemetry = CFG.axes.map(a => ({
  ...a, iq: 0, id: 0, vbus: 0, ia: 0, ib: 0, ic: 0, elecTheta: 0,
  idTar: 0, iqTar: 0, rxT: 0, temp: 25, vel: 0, mode: 0, torque: 0, pos: 0,
  encSum: 0, turns: 0, hasPos: false, real: false,
}));
globalThis.CFG = CFG;
globalThis.telemetry = telemetry;
globalThis.activeAxis = 0;
globalThis.performance = { now: () => Date.now() };
globalThis.pushSample = () => {};
globalThis.paramValues = {};
globalThis.FAULT_MSGS = {};
globalThis.logFault = () => {};
globalThis.fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v)) ? "--" : Number(v).toFixed(d);

eval(extractFn(js, "decodeFrame"));
globalThis.decodeFrame = decodeFrame;

function hex2u8(h) {
  const u = new Uint8Array(h.length / 2);
  for (let i = 0; i < u.length; i++) u[i] = parseInt(h.substr(i * 2, 2), 16);
  return u;
}

const payload = hex2u8(process.argv[2]);
if (payload.length < 12) throw new Error("载荷不足 12 B：" + payload.length);

decodeFrame(MSG_ID.POS_SENSOR, payload);

const ax = telemetry[0];
const ENC = 2097152;
console.log(JSON.stringify({
  encSum: ax.encSum,
  turns: Number(ax.turns.toFixed(6)),
  pos: Number(ax.pos.toFixed(6)),
  degrees: Number((ax.pos * 180 / Math.PI).toFixed(4)),
  elecTheta: Number(ax.elecTheta.toFixed(6)),
  hasPos: ax.hasPos,
  real: ax.real,
  expectTurns: Number((ax.encSum / ENC).toFixed(6)),
  turnsMatches: Math.abs(ax.turns - ax.encSum / ENC) < 1e-9,
}));
