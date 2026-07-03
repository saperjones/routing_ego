const STATE = { case: null, frame: 0, playing: false, speed: 1, mode: "real" };

const GROUP_NAMES = {
  A: "Straight", B: "Smooth turn", C: "Near-90 corner", D: "S-shape",
  E: "X-crossing", F: "Figure-eight", G: "Two-crossing",
};

async function loadIndex() {
  const url = STATE.mode === "real" ? "../out/real/index.json" : "../out/index.json";
  const ul = document.getElementById("case-list");
  ul.innerHTML = "";
  let data;
  try { data = await (await fetch(url)).json(); }
  catch (e) { ul.innerHTML = "<li>(no cases — run ./run.sh " +
    (STATE.mode === "real" ? "gen-real" : "gen") + ")</li>"; return; }
  let lastGroup = null;
  for (const c of data.cases) {
    if (STATE.mode === "sim" && c.group !== lastGroup) {
      const h = document.createElement("li");
      h.className = "group-header";
      h.textContent = GROUP_NAMES[c.group] || c.group;
      ul.appendChild(h); lastGroup = c.group;
    }
    const li = document.createElement("li");
    const badge = c.verdict
      ? `<span class="badge ${c.verdict.passed ? "pass" : "fail"}">${c.verdict.passed ? "PASS" : "FAIL"}</span>`
      : "";
    li.innerHTML = `${c.name}${badge}`;
    li.onclick = () => selectCase(c.case_id, li);
    ul.appendChild(li);
  }
}

async function loadCase(caseId) {
  const dir = STATE.mode === "real" ? "../out/real/" : "../out/";
  STATE.case = await (await fetch(`${dir}${caseId}.json`)).json();
  STATE.frame = 0;
  STATE.basemapImgs = null;
  if (STATE.case.mode === "real" && STATE.case.basemap) {
    STATE.basemapImgs = {};
    await Promise.all(STATE.case.basemap.tiles.map(t => new Promise(res => {
      const img = new Image();
      img.onload = () => { STATE.basemapImgs[t.file] = img; res(); };
      img.onerror = () => res();
      img.src = `../out/real/${t.file}`;
    })));
  }
  const sc = document.getElementById("scrubber");
  sc.max = STATE.case.frames.length - 1; sc.value = 0;
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
  BEVREAL_STATIC = null;
  drawPanorama();
  renderFrame();   // defined in Task 10
}

window.addEventListener("DOMContentLoaded", loadIndex);

function selectTab(mode) {
  STATE.mode = mode;
  document.getElementById("tab-real").classList.toggle("active", mode === "real");
  document.getElementById("tab-sim").classList.toggle("active", mode === "sim");
  STATE.case = null; STATE.playing = false;
  loadIndex();
}
window.addEventListener("DOMContentLoaded", () => {
  document.getElementById("tab-real").onclick = () => selectTab("real");
  document.getElementById("tab-sim").onclick = () => selectTab("sim");
});

// ---- offscreen static layers (anti-flicker) ----
let BEV_STATIC = null, BEV_T = null;
let BEVREAL_STATIC = null, BEVREAL_T = null;

function mercatorGlobalPx(lon, lat, z) {         // matches osm.lonlat_to_global_px
  const n = 2 ** z, T = 256;
  const x = (lon + 180) / 360 * n * T;
  const y = (1 - Math.asinh(Math.tan(lat * Math.PI / 180)) / Math.PI) / 2 * n * T;
  return { x, y };
}

// Build the fixed lon/lat -> canvas transform for a real case, fitting the
// route+track bounds (and basemap if present) into the BEV canvas.
function buildRealTransform(c, cv) {
  const z = c.basemap ? c.basemap.z : 18;
  const pts = c.route_llh.concat(c.ego_track_llh);
  let minx = 1e18, miny = 1e18, maxx = -1e18, maxy = -1e18;
  const bounds = c.basemap
    ? [[c.basemap.y0 * 256, c.basemap.x0 * 256],
       [(c.basemap.y0 + c.basemap.ny) * 256, (c.basemap.x0 + c.basemap.nx) * 256]]
    : null;
  if (bounds) { miny = bounds[0][0]; minx = bounds[0][1]; maxy = bounds[1][0]; maxx = bounds[1][1]; }
  else for (const [la, lo] of pts) {
    const p = mercatorGlobalPx(lo, la, z);
    minx = Math.min(minx, p.x); maxx = Math.max(maxx, p.x);
    miny = Math.min(miny, p.y); maxy = Math.max(maxy, p.y);
  }
  const pad = 8;
  const s = Math.min((cv.width - 2 * pad) / (maxx - minx), (cv.height - 2 * pad) / (maxy - miny));
  const ox = pad + (cv.width - 2 * pad - (maxx - minx) * s) / 2;
  const oy = pad + (cv.height - 2 * pad - (maxy - miny) * s) / 2;
  return { z, toX: gx => ox + (gx - minx) * s, toY: gy => oy + (gy - miny) * s };
}

function llToCanvas(T, lon, lat) {
  const p = mercatorGlobalPx(lon, lat, T.z);
  return { x: T.toX(p.x), y: T.toY(p.y) };
}

function drawTrackReal(ctx, T, llh, color, width) {
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
  for (let i = 0; i < llh.length; i++) {
    const p = llToCanvas(T, llh[i][1], llh[i][0]);
    if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
  }
  ctx.stroke();
}

function drawArrowsReal(ctx, T, llh, idxs, color) {
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
  for (const i of idxs) {
    if (i + 1 >= llh.length) continue;
    const a = llToCanvas(T, llh[i][1], llh[i][0]);
    const b = llToCanvas(T, llh[i + 1][1], llh[i + 1][0]);
    const ang = Math.atan2(b.y - a.y, b.x - a.x);
    const L = 7;
    ctx.beginPath();
    ctx.moveTo(b.x, b.y);
    ctx.lineTo(b.x - L * Math.cos(ang - 0.4), b.y - L * Math.sin(ang - 0.4));
    ctx.moveTo(b.x, b.y);
    ctx.lineTo(b.x - L * Math.cos(ang + 0.4), b.y - L * Math.sin(ang + 0.4));
    ctx.stroke();
  }
}

function buildBevRealStatic(cv) {
  const c = STATE.case;
  BEVREAL_T = buildRealTransform(c, cv);
  BEVREAL_STATIC = document.createElement("canvas");
  BEVREAL_STATIC.width = cv.width; BEVREAL_STATIC.height = cv.height;
  const ctx = BEVREAL_STATIC.getContext("2d");
  const T = BEVREAL_T;
  if (c.basemap && STATE.basemapImgs) {
    for (const t of c.basemap.tiles) {
      const img = STATE.basemapImgs[t.file];
      if (!img) continue;
      const gx = t.x * 256, gy = t.y * 256;   // tile's top-left global px
      const x0 = T.toX(gx), y0 = T.toY(gy);
      const x1 = T.toX(gx + 256), y1 = T.toY(gy + 256);
      ctx.drawImage(img, x0, y0, x1 - x0, y1 - y0);
    }
  } else {
    ctx.fillStyle = "#eef0f2"; ctx.fillRect(0, 0, cv.width, cv.height);
    ctx.strokeStyle = "#dfe3e8"; ctx.lineWidth = 1;
    for (let x = 0; x < cv.width; x += 40) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, cv.height); ctx.stroke(); }
    for (let y = 0; y < cv.height; y += 40) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(cv.width, y); ctx.stroke(); }
  }
  drawTrackReal(ctx, T, c.route_llh, "#4477cc", 3);       // planned route
  drawTrackReal(ctx, T, c.ego_track_llh, "#222", 2);      // ego driven track
  drawArrowsReal(ctx, T, c.route_llh, c.route_arrow_idx, "#2b5fb0");
  drawArrowsReal(ctx, T, c.ego_track_llh, c.ego_arrow_idx, "#cc3a3a");
}

function drawBevReal() {
  const c = STATE.case, cv = document.getElementById("bev");
  const ctx = cv.getContext("2d");
  if (!BEVREAL_STATIC) buildBevRealStatic(cv);
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.drawImage(BEVREAL_STATIC, 0, 0);
  const f = c.frames[STATE.frame];
  const p = llToCanvas(BEVREAL_T, f.meas_ll.lon, f.meas_ll.lat);
  ctx.fillStyle = "#cc3a3a";
  ctx.beginPath(); ctx.arc(p.x, p.y, 5, 0, 2 * Math.PI); ctx.fill();   // car marker
}

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
  if (document.getElementById("persp-toggle").checked) { drawWindshield(ctx, f); return; }
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
}

// PERSP: pinhole camera constants (driver's eye). Exposed for tweaking.
// pitch_deg is a downward tilt: enough to bring the near road (a few m ahead)
// into frame so the path emanates from the driver, not just near the horizon.
const PERSP = { H: 1.4, pitch_deg: 10, hfov_deg: 70, half_width: 0.7 };

// Windshield ("stereoscopic") view: project the route onto the road plane
// ahead through a forward-looking pinhole camera, with a horizon, a ground
// grid for depth, and the trajectory as a ribbon that narrows into the
// distance and converges toward the vanishing point.
function drawWindshield(ctx, f) {
  const cv = ctx.canvas, w = cv.width, h = cv.height;
  const H = PERSP.H;
  const pitch = PERSP.pitch_deg * Math.PI / 180;         // downward camera pitch
  const fpx = (w / 2) / Math.tan((PERSP.hfov_deg * Math.PI / 180) / 2);
  const cx = w / 2, cy = h / 2;
  const cosT = Math.cos(pitch), sinT = Math.sin(pitch);
  const horizon = cy - fpx * Math.tan(pitch);            // image row X -> infinity

  // Ground point (X forward, Y left, on the road H below the camera) -> pixel.
  function project(X, Y) {
    const depth = X * cosT + H * sinT;                   // along optical axis
    if (depth <= 0.05) return null;
    const right = -Y;                                    // camera image-right = -left
    const down = H * cosT - X * sinT;                    // camera image-down
    return { u: cx + fpx * right / depth, v: cy + fpx * down / depth };
  }

  // sky, ground, horizon
  const hy = Math.max(0, Math.min(h, horizon));
  ctx.fillStyle = "#dbe9f6"; ctx.fillRect(0, 0, w, hy);
  ctx.fillStyle = "#e9ebee"; ctx.fillRect(0, hy, w, h - hy);
  ctx.strokeStyle = "#9fb0c3"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, hy); ctx.lineTo(w, hy); ctx.stroke();

  // ground grid for depth cue
  const XMAX = STATE.case.config.ahead;
  ctx.strokeStyle = "#cbd1d9"; ctx.lineWidth = 1;
  for (const Y of [-4, -2, 0, 2, 4]) {                   // longitudinal lines
    let started = false; ctx.beginPath();
    for (let X = 0.5; X <= XMAX; X += 0.5) {
      const p = project(X, Y); if (!p) continue;
      if (!started) { ctx.moveTo(p.u, p.v); started = true; } else ctx.lineTo(p.u, p.v);
    }
    ctx.stroke();
  }
  for (const X of [5, 10, 15, 20, 25, 30]) {             // lateral distance lines
    if (X > XMAX) break;
    const a = project(X, -5), b = project(X, 5);
    if (a && b) { ctx.beginPath(); ctx.moveTo(a.u, a.v); ctx.lineTo(b.u, b.v); ctx.stroke(); }
  }

  // forward center line (straight-ahead reference from the driver)
  ctx.strokeStyle = "rgba(120,130,145,0.5)"; ctx.lineWidth = 1; ctx.setLineDash([4, 6]);
  ctx.beginPath(); ctx.moveTo(cx, hy); ctx.lineTo(cx, h); ctx.stroke(); ctx.setLineDash([]);

  // route ribbon: edges offset +/- half_width in the body frame
  const HW = PERSP.half_width;
  const s = STATE.case.route.s, e = STATE.case.route.points_e, n = STATE.case.route.points_n;
  const loS = f.cursor_s, hiS = f.cursor_s + XMAX;
  const left = [], right = [], mid = [];
  for (let i = 0; i < s.length; i++) {
    if (s[i] < loS || s[i] > hiS) continue;
    const b = worldToBody(f.meas_pose.e, f.meas_pose.n, e[i], n[i], f.meas_pose.h);
    if (b.x <= 0.05) continue;
    const pl = project(b.x, b.y + HW), pr = project(b.x, b.y - HW), pm = project(b.x, b.y);
    if (pl) left.push(pl);
    if (pr) right.push(pr);
    if (pm) mid.push(pm);
  }
  if (left.length > 1 && right.length > 1) {
    ctx.beginPath();
    ctx.moveTo(left[0].u, left[0].v);
    for (const p of left) ctx.lineTo(p.u, p.v);
    for (let i = right.length - 1; i >= 0; i--) ctx.lineTo(right[i].u, right[i].v);
    ctx.closePath();
    ctx.fillStyle = "rgba(46,158,91,0.45)"; ctx.fill();
    ctx.strokeStyle = "#2e9e5b"; ctx.lineWidth = 2; ctx.stroke();
  }
  if (mid.length > 1) {
    ctx.strokeStyle = "#1c6b3f"; ctx.lineWidth = 2; ctx.setLineDash([7, 7]);
    ctx.beginPath(); ctx.moveTo(mid[0].u, mid[0].v);
    for (const p of mid) ctx.lineTo(p.u, p.v);
    ctx.stroke(); ctx.setLineDash([]);
  }

  // driver anchor: a "hood" band across the bottom + an ego arrow marking the
  // car's position (bottom-center, pointing forward) so the view is clearly
  // centered on the driver.
  ctx.fillStyle = "#3a3f47";
  ctx.beginPath();
  ctx.moveTo(w * 0.30, h); ctx.lineTo(w * 0.70, h);
  ctx.lineTo(w * 0.60, h - 16); ctx.lineTo(w * 0.40, h - 16);
  ctx.closePath(); ctx.fill();
  ctx.fillStyle = "#cc3a3a";
  ctx.beginPath();
  ctx.moveTo(cx, h - 20); ctx.lineTo(cx - 7, h - 8); ctx.lineTo(cx + 7, h - 8);
  ctx.closePath(); ctx.fill();

  ctx.fillStyle = "#556"; ctx.font = "11px sans-serif";
  ctx.fillText("driver view (perspective)", 8, 16);
  if (f.end_flag) { ctx.fillStyle = "#cc3a3a"; ctx.fillText("route ends", 8, 30); }
}

function renderFrame() {
  const c = STATE.case; if (!c) return;
  if (STATE.case.mode === "real") drawBevReal(); else drawBev();
  drawDriver(); updateTelemetry(); drawPanoramaDot();
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
  set("tm-verdict", v ? `${v.passed ? "PASS" : "FAIL"} (mis ${v.mismatches})` : "— (real data)");
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
