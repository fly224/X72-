// 从 servo-web/index.html 抽出真实 decodeFrame，用 Python 生成的载荷回放，输出解码结果 JSON。
// 用法：node verify_host_iq_vbus.js <30025 hex> <30027 hex>
const fs = require("fs");

const HTML = process.argv[4] || "C:/Users/92714/WorkBuddy/2026-11-14-12-33-56/servo-web/index.html";
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

const pm = js.match(/const POLE_PAIRS\s*=\s*(\d+)/);
if (!pm) throw new Error("未找到 POLE_PAIRS");
globalThis.POLE_PAIRS = Number(pm[1]);

// ---- 最小运行环境（只补齐 decodeFrame 依赖，不执行页面其它逻辑） ----
const CFG = { channels: ["vel", "iq", "id", "torque", "pos", "temp", "vbus"],
              axes: [{ id: 0, name: "J1" }, { id: 1, name: "J2" }, { id: 2, name: "J3" }] };
const telemetry = CFG.axes.map(a => ({
  ...a, iq: 0, id: 0, vbus: 0, ia: 0, ib: 0, ic: 0, elecTheta: 0,
  idTar: 0, iqTar: 0, rxT: 0, temp: 25, vel: 0, mode: 0, torque: 0, pos: 0,
}));
globalThis.CFG = CFG;
globalThis.telemetry = telemetry;
globalThis.activeAxis = 0;
globalThis.MSG_ID = { TELEMETRY: 1, STATUS: 2, FAULT: 3, PARAM: 16, ACK: 17, CUR_IN: 30025, CUR_OUT: 30027 };
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

decodeFrame(30025, hex2u8(process.argv[2]));
decodeFrame(30027, hex2u8(process.argv[3]));

const ax = telemetry[0];
console.log(JSON.stringify({
  iq: ax.iq, id: ax.id, vbus: ax.vbus,
  ia: ax.ia, ib: ax.ib, ic: ax.ic,
  vel: ax.vel, iqTar: ax.iqTar, idTar: ax.idTar,
  mode: ax.mode, tempDash: globalThis.fmt(ax.temp, 1),
}));
