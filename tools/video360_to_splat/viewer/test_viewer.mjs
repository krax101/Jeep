// Headless end-to-end test of the splat viewer (rendering + controls).
// Usage: node test_viewer.mjs [--keep-open]
// Needs: playwright-core + a chromium (CHROMIUM_PATH env or playwright default).

import { chromium } from "playwright";
import { createServer } from "node:http";
import { readFile } from "node:fs/promises";
import { fileURLToPath } from "node:url";
import { dirname, join, extname } from "node:path";

const root = dirname(fileURLToPath(import.meta.url));
const mime = { ".html": "text/html", ".splat": "application/octet-stream" };

const server = createServer(async (req, res) => {
  try {
    const path = join(root, req.url.split("?")[0].replace(/^\//, "") || "index.html");
    const body = await readFile(path);
    res.writeHead(200, { "content-type": mime[extname(path)] || "application/octet-stream" });
    res.end(body);
  } catch {
    res.writeHead(404).end("not found");
  }
});
await new Promise((r) => server.listen(0, "127.0.0.1", r));
const port = server.address().port;

const browser = await chromium.launch({
  executablePath: process.env.CHROMIUM_PATH || undefined,
  args: ["--no-sandbox", "--use-angle=swiftshader"],
});
const page = await browser.newPage({ viewport: { width: 800, height: 600 } });
page.on("pageerror", (e) => { console.error("PAGE ERROR:", e.message); process.exitCode = 1; });

let failures = 0;
function check(name, ok, detail = "") {
  console.log(`  ${ok ? "OK  " : "FAIL"} ${name}${detail ? ` (${detail})` : ""}`);
  if (!ok) failures++;
}
const settle = (ms = 400) => page.waitForTimeout(ms);
const sample = (x, y) => page.evaluate(([x, y]) => window.__viewer.samplePixel(x, y), [x, y]);
const setCam = (...a) => page.evaluate((a) => window.__viewer.setCamera(...a), a);
const dominant = (px) => ["r", "g", "b"][px.slice(0, 3).indexOf(Math.max(...px.slice(0, 3)))];

await page.goto(`http://127.0.0.1:${port}/index.html?url=test.splat`);
await page.waitForFunction(() => window.__viewer && window.__viewer.drawCount === 5, null, { timeout: 10000 });
console.log("scene loaded, 5 splats drawn");

// Rendering geometry: camera at origin looking +z (world y is down).
await setCam(0, 0, 0, 0, 0);
await settle();
const W = 800, H = 600;
const centre = await sample(W / 2, H / 2);
check("occlusion: red in front of green wins centre", dominant(centre) === "r" && centre[0] > 100, `centre=${centre}`);

const fy = H / (2 * Math.tan((65 * Math.PI) / 180 / 2));
const off = Math.round((fy * 2) / 5); // screen offset of the ±2-unit markers at z=5
const right = await sample(W / 2 + off, H / 2);
check("+x renders screen-right (blue)", dominant(right) === "b", `px=${right}`);
const up = await sample(W / 2, H / 2 - off);
check("-y renders screen-up (yellow)", up[0] > 100 && up[1] > 100 && up[2] < 80, `px=${up}`);
const down = await sample(W / 2, H / 2 + off);
check("+y renders screen-down (white)", down[0] > 100 && down[1] > 100 && down[2] > 100, `px=${down}`);

// Mouse-look math: aim yaw/pitch at the side markers, they should hit centre.
await setCam(0, 0, 0, Math.atan2(2, 5), 0);
await settle();
check("yaw right centres blue", dominant(await sample(W / 2, H / 2)) === "b");
await setCam(0, 0, 0, 0, Math.atan2(2, 5)); // positive pitch = look up
await settle();
const pitched = await sample(W / 2, H / 2);
check("pitch up centres yellow", pitched[0] > 100 && pitched[1] > 100 && pitched[2] < 80, `px=${pitched}`);

// Keyboard: WASD / Space / Ctrl move the camera the right way.
await setCam(0, 0, 0, 0, 0);
const cam0 = await page.evaluate(() => ({ ...window.__viewer.cam }));
for (const [code, axis, sign, name] of [
  ["KeyW", "z", +1, "W moves forward (+z)"],
  ["KeyS", "z", -1, "S moves back"],
  ["KeyD", "x", +1, "D strafes right (+x)"],
  ["KeyA", "x", -1, "A strafes left"],
  ["Space", "y", -1, "Space moves up (-y)"],
  ["ControlLeft", "y", +1, "Ctrl moves down (+y)"],
]) {
  const before = await page.evaluate((a) => window.__viewer.cam[a], axis);
  await page.keyboard.down(code);
  await settle(250);
  await page.keyboard.up(code);
  const after = await page.evaluate((a) => window.__viewer.cam[a], axis);
  check(name, Math.sign(after - before) === sign, `${before.toFixed(2)} -> ${after.toFixed(2)}`);
}

// Reset key.
await page.keyboard.press("KeyR");
const reset = await page.evaluate(() => window.__viewer.cam.z);
check("R resets camera", reset === -5, `z=${reset}`);

// Visual confirmation that movement changes the rendered image.
await setCam(-1, 0, 0, 0, 0);
await settle();
const shifted = await sample(W / 2 + Math.round(fy / 5), H / 2);
check("strafe shifts scene on screen", dominant(shifted) === "r", `px=${shifted}`);

// .ply loading path: same scene converted from a 3DGS-format ply.
await page.goto(`http://127.0.0.1:${port}/index.html?url=test.ply`);
await page.waitForFunction(() => window.__viewer && window.__viewer.drawCount === 5, null, { timeout: 10000 });
await setCam(0, 0, 0, 0, 0);
await settle();
const plyCentre = await sample(W / 2, H / 2);
check(".ply loads and renders (red at centre)", dominant(plyCentre) === "r" && plyCentre[0] > 100, `px=${plyCentre}`);

await setCam(-1.5, 1.2, -1, 0.25, 0.15);
await settle();
if (process.env.SHOT_PATH) await page.screenshot({ path: process.env.SHOT_PATH });
await browser.close();
server.close();
console.log(failures ? `FAIL (${failures})` : "PASS");
process.exit(failures ? 1 : 0);
