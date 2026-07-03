const STATE = { case: null, frame: 0, playing: false, speed: 1 };

const GROUP_NAMES = {
  A: "Straight", B: "Smooth turn", C: "Near-90 corner", D: "S-shape",
  E: "X-crossing", F: "Figure-eight", G: "Two-crossing",
};

async function loadIndex() {
  const res = await fetch("../out/index.json");
  const data = await res.json();
  const ul = document.getElementById("case-list");
  ul.innerHTML = "";
  let lastGroup = null;
  for (const c of data.cases) {
    if (c.group !== lastGroup) {
      const h = document.createElement("li");
      h.className = "group-header";
      h.textContent = GROUP_NAMES[c.group] || c.group;
      ul.appendChild(h);
      lastGroup = c.group;
    }
    const li = document.createElement("li");
    const pass = c.verdict.passed;
    li.innerHTML = `${c.name}<span class="badge ${pass ? "pass" : "fail"}">${pass ? "PASS" : "FAIL"}</span>`;
    li.onclick = () => selectCase(c.case_id, li);
    ul.appendChild(li);
  }
}

async function loadCase(caseId) {
  const res = await fetch(`../out/${caseId}.json`);
  STATE.case = await res.json();
  STATE.frame = 0;
  const sc = document.getElementById("scrubber");
  sc.max = STATE.case.frames.length - 1;
  sc.value = 0;
}

function routeXY(c) {
  return { e: c.route.points_e, n: c.route.points_n };
}

function fitTransform(canvas, minE, maxE, minN, maxN, pad = 20) {
  const w = canvas.width, h = canvas.height;
  const sx = (w - 2 * pad) / Math.max(maxE - minE, 1e-6);
  const sy = (h - 2 * pad) / Math.max(maxN - minN, 1e-6);
  const s = Math.min(sx, sy);
  const toX = (e) => pad + (e - minE) * s;
  const toY = (n) => h - pad - (n - minN) * s;  // north up
  return { toX, toY, s };
}

function routeBounds(c) {
  const { e, n } = routeXY(c);
  return {
    minE: Math.min(...e), maxE: Math.max(...e),
    minN: Math.min(...n), maxN: Math.max(...n),
  };
}

function drawPanorama() {
  const c = STATE.case, cv = document.getElementById("panorama");
  const ctx = cv.getContext("2d");
  ctx.clearRect(0, 0, cv.width, cv.height);
  if (!c) return;
  const b = routeBounds(c);
  const T = fitTransform(cv, b.minE, b.maxE, b.minN, b.maxN);
  const { e, n } = routeXY(c);
  // route polyline
  ctx.strokeStyle = "#4477cc"; ctx.lineWidth = 2; ctx.beginPath();
  ctx.moveTo(T.toX(e[0]), T.toY(n[0]));
  for (let i = 1; i < e.length; i++) ctx.lineTo(T.toX(e[i]), T.toY(n[i]));
  ctx.stroke();
  // numbered waypoints
  const wi = c.route.waypoint_indices, wl = c.route.waypoint_labels;
  ctx.fillStyle = "#cc3a3a"; ctx.font = "bold 13px sans-serif";
  for (let k = 0; k < wi.length; k++) {
    const x = T.toX(e[wi[k]]), y = T.toY(n[wi[k]]);
    ctx.beginPath(); ctx.arc(x, y, 4, 0, 2 * Math.PI); ctx.fill();
    ctx.fillText(String(wl[k]), x + 6, y - 6);
  }
  // direction arrows between waypoints
  ctx.strokeStyle = "#cc3a3a"; ctx.fillStyle = "#cc3a3a";
  for (let k = 0; k + 1 < wi.length; k++) {
    const mi = Math.floor((wi[k] + wi[k + 1]) / 2);
    drawArrow(ctx, T.toX(e[mi]), T.toY(n[mi]),
              Math.atan2(-(n[mi + 1] - n[mi]), e[mi + 1] - e[mi]));
  }
}

function drawArrow(ctx, x, y, ang) {
  const L = 8;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x - L * Math.cos(ang - 0.4), y - L * Math.sin(ang - 0.4));
  ctx.moveTo(x, y);
  ctx.lineTo(x - L * Math.cos(ang + 0.4), y - L * Math.sin(ang + 0.4));
  ctx.stroke();
}

async function selectCase(caseId, li) {
  document.querySelectorAll("#case-list li").forEach((x) => x.classList.remove("active"));
  if (li) li.classList.add("active");
  STATE.playing = false;
  await loadCase(caseId);
  BEV_STATIC = null;
  drawPanorama();
  renderFrame();   // defined in Task 10
}

window.addEventListener("DOMContentLoaded", loadIndex);

// ---- offscreen static layers (anti-flicker) ----
let BEV_STATIC = null, BEV_T = null;

function buildBevStatic() {
  const c = STATE.case, cv = document.getElementById("bev");
  const b = routeBounds(c);
  BEV_T = fitTransform(cv, b.minE - 3, b.maxE + 3, b.minN - 3, b.maxN + 3);
  BEV_STATIC = document.createElement("canvas");
  BEV_STATIC.width = cv.width; BEV_STATIC.height = cv.height;
  const ctx = BEV_STATIC.getContext("2d");
  const { e, n } = routeXY(c);
  ctx.strokeStyle = "#9db4d8"; ctx.lineWidth = 2; ctx.beginPath();
  ctx.moveTo(BEV_T.toX(e[0]), BEV_T.toY(n[0]));
  for (let i = 1; i < e.length; i++) ctx.lineTo(BEV_T.toX(e[i]), BEV_T.toY(n[i]));
  ctx.stroke();
}

function worldToBody(pe, pn, ex, ny, yaw) {
  const de = ex - pe, dn = ny - pn;
  const cs = Math.cos(yaw), sn = Math.sin(yaw);
  return { x: de * cs + dn * sn, y: -de * sn + dn * cs };  // x fwd, y left
}

function drawBev() {
  const c = STATE.case, cv = document.getElementById("bev");
  const ctx = cv.getContext("2d");
  if (!BEV_STATIC) buildBevStatic();
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.drawImage(BEV_STATIC, 0, 0);
  // driven trajectory up to current frame
  ctx.strokeStyle = "#222"; ctx.lineWidth = 2; ctx.beginPath();
  for (let i = 0; i <= STATE.frame; i++) {
    const f = c.frames[i];
    const x = BEV_T.toX(f.true_pose.e), y = BEV_T.toY(f.true_pose.n);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();
  // car marker
  const f = c.frames[STATE.frame];
  drawCar(ctx, BEV_T, f.true_pose.e, f.true_pose.n, f.true_pose.h);
}

function drawCar(ctx, T, e, n, yaw) {
  const L = 1.8, W = 0.9;  // meters (half-extents)
  const corners = [[L, W], [L, -W], [-L, -W], [-L, W]];
  ctx.fillStyle = "rgba(204,58,58,0.8)"; ctx.beginPath();
  corners.forEach(([fx, fy], i) => {
    const ex = e + fx * Math.cos(yaw) - fy * Math.sin(yaw);
    const ny = n + fx * Math.sin(yaw) + fy * Math.cos(yaw);
    const x = T.toX(ex), y = T.toY(ny);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.closePath(); ctx.fill();
}

function drawDriver() {
  const c = STATE.case, cv = document.getElementById("driver");
  const ctx = cv.getContext("2d");
  ctx.clearRect(0, 0, cv.width, cv.height);
  const f = c.frames[STATE.frame];
  if (f.cursor_s == null) return;
  const ahead = c.config.ahead, behind = c.config.behind;
  // body-frame fixed transform: x forward -> up, y left -> left
  const w = cv.width, h = cv.height, ppm = (h - 20) / (ahead - behind);
  const toX = (by) => w / 2 - by * ppm;   // +y left -> screen left
  const toY = (bx) => h - 10 - (bx - behind) * ppm; // +x forward -> up
  // slice of route in [cursor_s+behind, cursor_s+ahead]
  const s = c.route.s, e = c.route.points_e, n = c.route.points_n;
  const loS = f.cursor_s + behind, hiS = f.cursor_s + ahead;
  ctx.strokeStyle = "#2e9e5b"; ctx.lineWidth = 3; ctx.beginPath();
  let started = false;
  for (let i = 0; i < s.length; i++) {
    if (s[i] < loS || s[i] > hiS) continue;
    const b = worldToBody(f.meas_pose.e, f.meas_pose.n, e[i], n[i], f.meas_pose.h);
    const x = toX(b.y), y = toY(b.x);
    if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
  }
  ctx.stroke();
  // car at origin (width=1.8m, height=3.6m, centered)
  ctx.fillStyle = "#cc3a3a";
  ctx.fillRect(toX(0.9), toY(1.8), 1.8 * ppm, 3.6 * ppm);
  if (f.end_flag) {
    ctx.fillStyle = "#cc3a3a"; ctx.font = "12px sans-serif";
    ctx.fillText("route ends", 8, 16);
  }
  if (document.getElementById("persp-toggle").checked) drawPerspective(ctx, f);
}

function drawPerspective(ctx, f) {
  // nice-to-have default pinhole overlay: cam height 1.4m, pitch -2deg, hfov 60
  const cv = ctx.canvas, cx = cv.width * 0.75, cyv = cv.height * 0.35;
  const fpx = (cv.width * 0.25) / Math.tan((60 * Math.PI / 180) / 2);
  const H = 1.4, pitch = -2 * Math.PI / 180;
  ctx.strokeStyle = "#a0522d"; ctx.lineWidth = 2; ctx.beginPath();
  const s = STATE.case.route.s, e = STATE.case.route.points_e, n = STATE.case.route.points_n;
  let started = false;
  for (let i = 0; i < s.length; i++) {
    if (s[i] < f.cursor_s || s[i] > f.cursor_s + STATE.case.config.ahead) continue;
    const b = worldToBody(f.meas_pose.e, f.meas_pose.n, e[i], n[i], f.meas_pose.h);
    if (b.x <= 0.5) continue;
    const px = cx - (b.y / b.x) * fpx * 0.25;
    const py = cyv + (H / b.x + pitch) * fpx * 0.25;
    if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
  }
  ctx.stroke();
}

function renderFrame() {
  const c = STATE.case; if (!c) return;
  drawBev(); drawDriver(); updateTelemetry(); drawPanoramaDot();
  document.getElementById("scrubber").value = STATE.frame;
  document.getElementById("frame-label").textContent =
    `${STATE.frame} / ${c.frames.length - 1}`;
}

function drawPanoramaDot() {
  drawPanorama();  // static redraw (cheap) then dot
  const c = STATE.case, cv = document.getElementById("panorama");
  const ctx = cv.getContext("2d");
  const b = routeBounds(c);
  const T = fitTransform(cv, b.minE, b.maxE, b.minN, b.maxN);
  const f = c.frames[STATE.frame];
  ctx.fillStyle = "#111";
  ctx.beginPath(); ctx.arc(T.toX(f.true_pose.e), T.toY(f.true_pose.n), 3, 0, 2 * Math.PI); ctx.fill();
}

function updateTelemetry() {
  const c = STATE.case, f = c.frames[STATE.frame];
  const deg = (r) => (90 - r * 180 / Math.PI).toFixed(2);  // compass
  const set = (id, v) => (document.getElementById(id).textContent = v);
  set("tm-heading", `${deg(f.meas_pose.h)}° (N-CW)`);
  set("tm-speed", `${(f.speed * 3.6).toFixed(1)} km/h`);
  set("tm-pos", `(${f.meas_pose.e.toFixed(2)}, ${f.meas_pose.n.toFixed(2)})`);
  set("tm-estdev", f.est_lat_dev == null ? "–" : `${f.est_lat_dev.toFixed(3)} m`);
  set("tm-truedev", f.true_lat_dev == null ? "–" : `${f.true_lat_dev.toFixed(3)} m`);
  const lastS = c.route.s[c.route.s.length - 1];
  set("tm-progress", f.cursor_s == null ? "–" : `${(f.cursor_s / lastS * 100).toFixed(1)}%`);
  set("tm-seg", f.matched_seg == null ? "–" : String(f.matched_seg));
  set("tm-frame", `${STATE.frame} / ${c.frames.length - 1}`);
  const v = c.verdict;
  set("tm-verdict", `${v.passed ? "PASS" : "FAIL"} (mis ${v.mismatches})`);
}

let lastTick = 0;
function tick(ts) {
  if (STATE.playing && STATE.case) {
    const dtMs = 1000 / (10 * STATE.speed);   // frames stored at 10 Hz
    if (ts - lastTick >= dtMs) {
      lastTick = ts;
      if (STATE.frame < STATE.case.frames.length - 1) { STATE.frame++; renderFrame(); }
      else STATE.playing = false;
    }
  }
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);

window.addEventListener("DOMContentLoaded", () => {
  document.getElementById("btn-play").onclick = () => { STATE.playing = !STATE.playing; };
  document.getElementById("btn-step-fwd").onclick = () => {
    if (STATE.case && STATE.frame < STATE.case.frames.length - 1) { STATE.frame++; renderFrame(); }
  };
  document.getElementById("btn-step-back").onclick = () => {
    if (STATE.case && STATE.frame > 0) { STATE.frame--; renderFrame(); }
  };
  document.getElementById("scrubber").oninput = (ev) => {
    if (STATE.case) { STATE.frame = parseInt(ev.target.value, 10); renderFrame(); }
  };
  document.getElementById("speed").onchange = (ev) => { STATE.speed = parseFloat(ev.target.value); };
  document.getElementById("persp-toggle").onchange = () => renderFrame();
});
