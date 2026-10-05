// present.html -> MP4 at 1x, from the first motion to the end, real time (no cuts): headless Chrome over CDP draws
// each frame through the page's own seek() / frame() at FPS and screenshots it; replay/encode_mp4.py encodes H.264.
//   node replay/render_mp4.mjs sessions/<id> [--fps 25] [--layout portrait] [--out file.mp4]
// Needs node >= 22 (WebSocket), Chrome, and ~/lerobot-mps-venv (PyAV + Pillow). Reads the session; writes the MP4 only.
import { spawn } from "node:child_process";
import { existsSync, mkdtempSync } from "node:fs";
import { tmpdir, homedir } from "node:os";
import { join, resolve, dirname } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const args = process.argv.slice(2), opt = k => { const i = args.indexOf(k); return i >= 0 ? args[i + 1] : null; };
const dir = resolve(args[0] || ""), fps = +(opt("--fps") || 25), portrait = opt("--layout") === "portrait";
const page = join(dir, "present.html");
if (!existsSync(page)) { console.error(`no present.html in ${dir} (run replay/build_replay.py on it first)`); process.exit(2); }
const out = resolve(opt("--out") || join(dir, `present_1x${portrait ? "_portrait" : ""}.mp4`));
const [W, H] = portrait ? [720, 1080] : [1920, 1080];
const CH = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome", PORT = 9300 + Math.floor(Math.random() * 500);
const chrome = spawn(CH, ["--headless=new", "--disable-gpu", "--hide-scrollbars", `--remote-debugging-port=${PORT}`,
  `--window-size=${W},${H}`, `--user-data-dir=${mkdtempSync(join(tmpdir(), "kh-render-"))}`, "about:blank"], { stdio: "ignore" });
const sleep = ms => new Promise(r => setTimeout(r, ms));
let tabs; for (let i = 0; i < 80 && !tabs; i++) { try { tabs = await (await fetch(`http://127.0.0.1:${PORT}/json`)).json(); } catch { await sleep(150); } }
const ws = new WebSocket(tabs.find(t => t.type === "page").webSocketDebuggerUrl); await new Promise(r => ws.onopen = r);
let id = 0; const pend = new Map();
ws.onmessage = m => { const d = JSON.parse(m.data); if (pend.has(d.id)) { pend.get(d.id)(d); pend.delete(d.id); } };
const send = (method, params = {}) => new Promise(r => { const i = ++id; pend.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
const ev = async expr => { const r = await send("Runtime.evaluate", { expression: expr, returnByValue: true });
  if (r.result?.exceptionDetails) throw new Error(JSON.stringify(r.result.exceptionDetails).slice(0, 400)); return r.result?.result?.value; };
await send("Emulation.setDeviceMetricsOverride", { width: W, height: H, deviceScaleFactor: 1, mobile: false });
await send("Page.enable");
const url = pathToFileURL(page).href + `?real=1&record=1&scale=1${portrait ? "&layout=portrait" : ""}`;
await send("Page.navigate", { url }); await sleep(2500);
const end = await ev("TAU_END");
if (typeof end !== "number") throw new Error("the page did not load its script (TAU_END missing)");
const n = Math.ceil(end * fps) + fps;                                   // + 1 s on the last frame
const here = dirname(fileURLToPath(import.meta.url));
const enc = spawn(join(homedir(), "lerobot-mps-venv/bin/python"), [join(here, "encode_mp4.py"), out, String(fps)], { stdio: ["pipe", "inherit", "inherit"] });
const t0 = Date.now();
for (let i = 0; i < n; i++) {
  await ev(`seek(${Math.min(i / fps, end)}); frame(performance.now()); 0`);
  const shot = await send("Page.captureScreenshot", { format: "jpeg", quality: 92 });
  const jpg = Buffer.from(shot.result.data, "base64"), head = Buffer.alloc(4); head.writeUInt32BE(jpg.length);
  if (!enc.stdin.write(Buffer.concat([head, jpg]))) await new Promise(r => enc.stdin.once("drain", r));
  if (i % (fps * 30) === 0) console.log(`  ${(i / fps).toFixed(0)} s of ${end.toFixed(0)} s (${((Date.now() - t0) / 1000).toFixed(0)} s so far)`);
}
enc.stdin.end(); await new Promise(r => enc.on("close", r));
ws.close(); chrome.kill();
console.log(`${out}: ${n} frames, ${(n / fps).toFixed(1)} s at ${fps} fps, 1x from the first motion, real time (no cuts)`);
