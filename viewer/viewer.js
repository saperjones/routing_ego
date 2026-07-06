const STATE = { case: null, frame: 0, playing: false, speed: 1, mode: "real",
                offline: null };   // offline: {frames:[...], strategy} when loaded

const GROUP_NAMES = {
  A: "Straight", B: "Smooth turn", C: "Near-90 corner", D: "S-shape",
  E: "X-crossing", F: "Figure-eight", G: "Two-crossing",
};

async function loadIndex() {
  const url = STATE.mode === "real" ? "../out/real/index.json" : "../out/index.json";
  const ul = document.getElementById("case-list");
  ul.innerHTML = "";
  let data;
  try { data = await (await fetch(url, { cache: "no-store" })).json(); }
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
  STATE.case = await (await fetch(`${dir}${caseId}.json`, { cache: "no-store" })).json();
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
  if (c.mode === "pre") { drawPanoramaPre(false); return; }
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
  STATE.offline = null;                    // stale for the new case
  await loadCase(caseId);
  BEV_STATIC = null;
  BEVREAL_STATIC = null;
  ROUTE_JS_CASE = null;
  drawPanorama();
  clearOffline();
  updateOfflineButton();
  renderFrame();   // defined in Task 10
}

window.addEventListener("DOMContentLoaded", loadIndex);

function clearCanvas(id) {
  const cv = document.getElementById(id);
  if (cv) cv.getContext("2d").clearRect(0, 0, cv.width, cv.height);
}

// Enter/leave pre-processed mode: the projection path comes from the file, so
// the live algorithm/window controls are disabled and a read-only caption shows
// the config the offline run actually used.
function applyPreMode(on) {
  for (const id of ["algo-select", "corner-style", "p-radius", "p-behind",
                    "p-ahead", "p-corner", "p-transition", "compare-toggle"]) {
    const el = document.getElementById(id);
    if (el) el.disabled = on;
  }
  const cap = document.getElementById("pre-config");
  if (!cap) return;
  if (on && STATE.case && STATE.case.config) {
    const cf = STATE.case.config;
    cap.style.display = "";
    cap.textContent = `pre-processed output — strategy=${cf.strategy}, `
      + `ahead=${cf.ahead_m} m, behind=${cf.behind_m} m, ds=${cf.sample_ds_m} m`;
  } else {
    cap.style.display = "none"; cap.textContent = "";
  }
}

function selectTab(mode) {
  STATE.mode = mode;
  document.getElementById("tab-real").classList.toggle("active", mode === "real");
  document.getElementById("tab-sim").classList.toggle("active", mode === "sim");
  document.getElementById("tab-pre").classList.toggle("active", mode === "pre");
  STATE.case = null; STATE.playing = false;
  STATE.offline = null;
  clearOffline();
  // reset compare view back to the single driver view when switching tabs
  const cmp = document.getElementById("compare-toggle");
  if (cmp && cmp.checked) {
    cmp.checked = false;
    document.getElementById("driver-fig").style.display = "";
    document.getElementById("compare-fig").style.display = "none";
  }
  const pre = mode === "pre";
  document.getElementById("pre-picker").style.display = pre ? "" : "none";
  document.getElementById("cases-head").style.display = pre ? "none" : "";
  document.getElementById("case-list").style.display = pre ? "none" : "";
  BEV_STATIC = null; BEVREAL_STATIC = null; ROUTE_JS_CASE = null;
  applyPreMode(pre);
  updateOfflineButton();
  if (pre) {
    document.getElementById("case-list").innerHTML = "";
    document.getElementById("pre-status").textContent =
      "Choose a folder containing routing_projection.json + planned_route.json.";
    clearCanvas("bev"); clearCanvas("driver"); clearCanvas("panorama");
  } else {
    loadIndex();
  }
}
window.addEventListener("DOMContentLoaded", () => {
  document.getElementById("tab-real").onclick = () => selectTab("real");
  document.getElementById("tab-sim").onclick = () => selectTab("sim");
  document.getElementById("tab-pre").onclick = () => selectTab("pre");
});

// --- Pre-processed mode -------------------------------------------------
// Visualize a colleague's offline output folder (ego_route_llh.json +
// planned_route.json + routing_projection.json) with no algorithm re-run: the
// per-frame body-frame path is read straight from routing_projection.json.

// Cumulative length (m) of a WGS-84 [lat,lon] polyline (equirectangular; the
// route spans tens of metres so the flat-earth error is negligible). Used only
// for the telemetry progress %.
function llhPathLenM(llh) {
  const R = 6378137, d2r = Math.PI / 180;
  let s = 0;
  for (let i = 1; i < llh.length; i++) {
    const [la1, lo1] = llh[i - 1], [la2, lo2] = llh[i];
    const mlat = (la1 + la2) / 2 * d2r;
    const dx = (lo2 - lo1) * d2r * Math.cos(mlat) * R;
    const dy = (la2 - la1) * d2r * R;
    s += Math.hypot(dx, dy);
  }
  return s;
}

// Pure builder (on window for unit testing): turn the parsed projection +
// planned-route JSON into a synthetic "case" the existing real-data renderers
// accept unchanged. `pose.lat/lon` is already WGS-84, so no GCJ-02 conversion.
window.buildPreCase = function (projection, plannedRoute, folderName) {
  const frames = (projection.frames || []).map((fr) => ({
    meas_pose: { e: fr.pose.e, n: fr.pose.n, h: fr.pose.yaw },
    meas_ll: { lat: fr.pose.lat, lon: fr.pose.lon },
    speed: fr.speed,
    est_lat_dev: fr.lat_dev,
    true_lat_dev: null,        // no ground truth for pre-processed data
    cursor_s: fr.cursor_s,
    matched_seg: fr.matched_seg,
    end_flag: !!fr.end_flag,
    path: fr.path || [],
  }));
  const route_llh = (plannedRoute && plannedRoute.planned_route) || [];
  const route_waypoints = (plannedRoute && plannedRoute.waypoints) || [];
  const ego_track_llh = frames.map((f) => [f.meas_ll.lat, f.meas_ll.lon]);
  const spaced = (n, k) => {
    const out = []; if (n <= 1) return out;
    const step = Math.max(1, Math.floor(n / k));
    for (let i = 0; i < n - 1; i += step) out.push(i);
    return out;
  };
  return {
    mode: "pre",
    case_id: folderName || "pre-processed",
    name: folderName || "pre-processed",
    route_llh, route_waypoints, ego_track_llh,
    route_arrow_idx: spaced(route_llh.length, 8),
    ego_arrow_idx: spaced(ego_track_llh.length, 8),
    basemap: null,                                   // -> gray graticule BEV
    route_total_len_m: route_llh.length > 1 ? llhPathLenM(route_llh) : null,
    config: (projection.meta && projection.meta.config) || {},
    status_message: (projection.status && projection.status.message) || "",
    frames,
    verdict: null,
  };
};

// Config for pre-mode rendering: the look-ahead window comes from the file's
// meta.config (NOT the disabled sliders).
function preConfig() {
  const cf = (STATE.case && STATE.case.config) || {};
  return Object.assign({}, ProjectRoute.DEFAULT_CONFIG, {
    strategy: cf.strategy || "human_centered",
    behind_m: cf.behind_m != null ? cf.behind_m : 5,
    ahead_m: cf.ahead_m != null ? cf.ahead_m : 40,
  });
}

// Fit a WGS-84 [lat,lon] set into a canvas (Web-Mercator, north-up).
function buildLLTransform(cv, llhList, pad = 14) {
  const z = 18;
  let minx = 1e18, miny = 1e18, maxx = -1e18, maxy = -1e18;
  for (const [la, lo] of llhList) {
    const p = mercatorGlobalPx(lo, la, z);
    minx = Math.min(minx, p.x); maxx = Math.max(maxx, p.x);
    miny = Math.min(miny, p.y); maxy = Math.max(maxy, p.y);
  }
  const s = Math.min((cv.width - 2 * pad) / Math.max(maxx - minx, 1e-6),
                     (cv.height - 2 * pad) / Math.max(maxy - miny, 1e-6));
  const ox = pad + (cv.width - 2 * pad - (maxx - minx) * s) / 2;
  const oy = pad + (cv.height - 2 * pad - (maxy - miny) * s) / 2;
  return { z, toX: (gx) => ox + (gx - minx) * s, toY: (gy) => oy + (gy - miny) * s };
}

// Panorama for pre mode: planned route (blue) + numbered waypoints (red) in
// Web-Mercator, plus the current-frame position dot when withDot.
function drawPanoramaPre(withDot) {
  const c = STATE.case, cv = document.getElementById("panorama");
  const ctx = cv.getContext("2d");
  ctx.clearRect(0, 0, cv.width, cv.height);
  const all = (c.route_llh || []).concat(c.ego_track_llh || []);
  if (!all.length) return;
  const T = buildLLTransform(cv, all);
  if (c.route_llh && c.route_llh.length) {
    ctx.strokeStyle = "#4477cc"; ctx.lineWidth = 2; ctx.beginPath();
    c.route_llh.forEach(([la, lo], i) => {
      const p = llToCanvas(T, lo, la);
      if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
    });
    ctx.stroke();
  }
  if (c.route_waypoints && c.route_waypoints.length) {
    ctx.fillStyle = "#cc3a3a"; ctx.font = "bold 12px sans-serif";
    c.route_waypoints.forEach(([la, lo], k) => {
      const p = llToCanvas(T, lo, la);
      ctx.beginPath(); ctx.arc(p.x, p.y, 3, 0, 2 * Math.PI); ctx.fill();
      ctx.fillText(String(k), p.x + 5, p.y - 5);
    });
  }
  if (withDot) {
    const f = c.frames[STATE.frame];
    const p = llToCanvas(T, f.meas_ll.lon, f.meas_ll.lat);
    ctx.fillStyle = "#111";
    ctx.beginPath(); ctx.arc(p.x, p.y, 3, 0, 2 * Math.PI); ctx.fill();
  }
}

// Driver view for pre mode (top-down): the file's path (green) + the real
// driven trajectory (orange dashed) in the current body frame + car at origin.
function drawDriverPre(cv, f) {
  const c = STATE.case;
  const ctx = cv.getContext("2d");
  ctx.clearRect(0, 0, cv.width, cv.height);
  const cfg = preConfig();
  const behindLive = cfg.behind_m, aheadLive = cfg.ahead_m;
  const w = cv.width, h = cv.height;
  const ppm = (h - 20) / (aheadLive + behindLive);
  const toX = (by) => w / 2 - by * ppm;
  const toY = (bx) => h - 10 - (bx + behindLive) * ppm;
  const cur = f.meas_pose, yawUse = f.meas_pose.h;   // overlay in the vehicle frame
  // real driven trajectory (orange dashed)
  ctx.strokeStyle = "rgba(230,140,0,0.9)"; ctx.lineWidth = 2; ctx.setLineDash([5, 4]);
  ctx.beginPath();
  let prevb = null;
  for (let i = 0; i < c.frames.length; i++) {
    const g = c.frames[i].meas_pose; if (!g) continue;
    const b = worldToBody(cur.e, cur.n, g.e, g.n, yawUse);
    if (b.x < -behindLive || b.x > aheadLive) { prevb = null; continue; }
    const x = toX(b.y), y = toY(b.x);
    if (!prevb || Math.hypot(b.x - prevb.x, b.y - prevb.y) > 3.0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
    prevb = b;
  }
  ctx.stroke(); ctx.setLineDash([]);
  // generated path straight from the file (green)
  ctx.strokeStyle = "#2e9e5b"; ctx.lineWidth = 3; ctx.beginPath();
  f.path.forEach(([bx, by], i) => {
    const x = toX(by), y = toY(bx);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
  // car at origin
  ctx.fillStyle = "#cc3a3a"; ctx.fillRect(toX(0.9), toY(1.8), 1.8 * ppm, 3.6 * ppm);
  ctx.font = "11px sans-serif";
  ctx.fillStyle = "#2e9e5b"; ctx.fillText("— pre-processed path", 8, 16);
  ctx.fillStyle = "rgba(230,140,0,0.95)"; ctx.fillText("- - real trajectory", 8, 30);
  if (f.end_flag) {
    ctx.fillStyle = "#cc3a3a"; ctx.font = "12px sans-serif";
    ctx.fillText("route ends", 8, 44);
  }
}

// Install a pre-processed case from already-parsed JSON, then render. Split out
// of handlePreFolder so it can be driven without a File object (tests).
function loadPreCase(projection, plannedRoute, folderName) {
  STATE.case = buildPreCase(projection, plannedRoute, folderName);
  STATE.frame = 0; STATE.playing = false; STATE.offline = null; STATE.basemapImgs = null;
  BEV_STATIC = null; BEVREAL_STATIC = null; BEVREAL_T = null;
  const sc = document.getElementById("scrubber");
  sc.max = STATE.case.frames.length - 1; sc.value = 0;
  applyPreMode(true);
  updateOfflineButton();
  renderFrame();
  return STATE.case;
}
window.loadPreCase = loadPreCase;

// Read the picked folder's files (client-side) and render.
async function handlePreFolder(files) {
  const el = document.getElementById("pre-status");
  const byName = {};
  for (const f of files) {
    const base = (f.name || "").split("/").pop();
    if (!(base in byName)) byName[base] = f;
  }
  const projFile = byName["routing_projection.json"];
  if (!projFile) {
    el.textContent = "routing_projection.json not found in the selected folder.";
    return;
  }
  let projection;
  try { projection = JSON.parse(await projFile.text()); }
  catch (e) { el.textContent = "Failed to parse routing_projection.json: " + e; return; }
  let plannedRoute = null;
  const routeFile = byName["planned_route.json"];
  if (routeFile) { try { plannedRoute = JSON.parse(await routeFile.text()); } catch (e) { plannedRoute = null; } }
  if (!projection.frames || !projection.frames.length) {
    const m = projection.status && projection.status.message;
    el.textContent = "No frames in routing_projection.json." + (m ? " " + m : "");
    return;
  }
  const folderName = (files[0] && files[0].webkitRelativePath)
    ? files[0].webkitRelativePath.split("/")[0] : "pre-processed";
  loadPreCase(projection, plannedRoute, folderName);
  let msg = `Loaded "${folderName}" — ${STATE.case.frames.length} frames, `
    + `strategy "${STATE.case.config.strategy || "?"}".`;
  if (!plannedRoute) msg += " (planned_route.json missing — route & progress omitted.)";
  else if (projection.status && projection.status.generated === false)
    msg += " Note: status.generated=false — " + (projection.status.message || "");
  el.textContent = msg;
  renderFrame();
}

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
  const s = Math.min((cv.width - 2 * pad) / Math.max(maxx - minx, 1e-6),
                     (cv.height - 2 * pad) / Math.max(maxy - miny, 1e-6));
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
  // yellow oriented arrow (distinct from blue route / red arrows); points along
  // heading. ENU yaw -> screen angle is -h (mercator is north-up, screen y down).
  drawCarMarker(ctx, p.x, p.y, -f.meas_pose.h);
}

function drawCarMarker(ctx, x, y, ang) {
  const L = 9, W = 6;                       // pixels: nose length / half-width
  const pts = [[L, 0], [-L * 0.7, W], [-L * 0.7, -W]];   // arrow: nose + two tails
  ctx.beginPath();
  pts.forEach(([fx, fy], i) => {
    const px = x + fx * Math.cos(ang) - fy * Math.sin(ang);
    const py = y + fx * Math.sin(ang) + fy * Math.cos(ang);
    if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
  });
  ctx.closePath();
  ctx.fillStyle = "#ffd21e"; ctx.fill();                 // yellow
  ctx.strokeStyle = "#7a5c00"; ctx.lineWidth = 1.5; ctx.stroke();
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

// --- live projection (JS twin of the Python algorithm) ---
let ROUTE_JS = null, ROUTE_JS_CASE = null, CURSOR_MEMO = null, MEMO_UPTO = -1;

function ensureRouteJs(c) {
  if (ROUTE_JS_CASE === c) return;
  ROUTE_JS = ProjectRoute.buildRoute(c.route.points_e, c.route.points_n, c.route.s);
  ROUTE_JS.seg_of_index = null;               // not needed for display
  ROUTE_JS_CASE = c;
  CURSOR_MEMO = new Array(c.frames.length).fill(null);
  MEMO_UPTO = -1;
}

function currentConfig() {
  return Object.assign({}, ProjectRoute.DEFAULT_CONFIG, {
    strategy: document.getElementById("algo-select").value,
    min_turn_radius_m: parseFloat(document.getElementById("p-radius").value),
    behind_m: parseFloat(document.getElementById("p-behind").value),
    ahead_m: parseFloat(document.getElementById("p-ahead").value),
    corner_angle_deg: parseFloat(document.getElementById("p-corner").value),
    corner_style: document.getElementById("corner-style").value,
    clothoid_transition_m: parseFloat(document.getElementById("p-transition").value),
  });
}

// Advance the monotonic cursor memo up to frameIdx (matching params are fixed,
// so the cursor does not depend on the live sliders — only the output does).
function cursorAt(c, frameIdx) {
  ensureRouteJs(c);
  const cfg = ProjectRoute.DEFAULT_CONFIG;    // fixed matching params
  let state = MEMO_UPTO >= 0 ? { cursor_s: CURSOR_MEMO[MEMO_UPTO], initialized: true } : null;
  for (let i = MEMO_UPTO + 1; i <= frameIdx; i++) {
    const p = c.frames[i].meas_pose;
    const m = ProjectRoute.match(ROUTE_JS, p.e, p.n, p.h, cfg, state);
    CURSOR_MEMO[i] = m.cursor_s;
    state = { cursor_s: m.cursor_s, initialized: true };
  }
  if (frameIdx > MEMO_UPTO) MEMO_UPTO = frameIdx;
  return CURSOR_MEMO[frameIdx];
}

// Body-frame path for this frame under the live config. Reuses the memoized
// cursor so slider/selector changes recompute only the output (instant).
function computeBodyPath(c, frameIdx, cfgOverride) {
  ensureRouteJs(c);
  const f = c.frames[frameIdx];
  const cursor = cursorAt(c, frameIdx);
  const cfg = cfgOverride || currentConfig();
  const state = { cursor_s: cursor, initialized: true };
  // re-run project_route from the known cursor: pass a state whose cursor equals
  // this frame's cursor and a pose on that spot so match() returns it unchanged.
  const out = ProjectRoute.projectRoute(ROUTE_JS, f.meas_pose, cfg, state);
  return { pts: out.path.map(([x, y]) => ({ x, y })), yaw: out.yaw_used };
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
  if (!f.meas_pose) return;
  const persp = document.getElementById("persp-toggle").checked;
  if (c.mode === "pre") {
    if (persp) drawWindshield(ctx, f, preConfig(), null);
    else drawDriverPre(cv, f);
    return;
  }
  if (persp) { drawWindshield(ctx, f, currentConfig(), null); return; }
  drawDriverTopDown(cv, f, currentConfig(), null);
}

// Draw one top-down driver-view panel: the generated path (green) + the real
// driven trajectory overlay (orange dashed) + the car, all in the body frame of
// `cfg`. Reused by the single driver view (label=null → full legend) and by each
// compare-mode panel (label=strategy name → compact heading). `cfg` carries the
// strategy, so every panel runs the SAME renderer with a different config.
function drawDriverTopDown(cv, f, cfg, label) {
  const c = STATE.case;
  const ctx = cv.getContext("2d");
  ctx.clearRect(0, 0, cv.width, cv.height);
  const behindLive = cfg.behind_m;
  const aheadLive = cfg.ahead_m;
  const w = cv.width, h = cv.height;
  const ppm = (h - 20) / (aheadLive + behindLive);
  const toX = (by) => w / 2 - by * ppm;
  const toY = (bx) => h - 10 - (bx + behindLive) * ppm;
  // compute the generated path first — it also reports the frame yaw it used
  // (for human_centered that is the curve tangent, not the vehicle heading), so
  // the overlay is drawn in the SAME frame and stays comparable.
  const cp = computeBodyPath(c, STATE.frame, cfg);
  const pts = cp.pts, yawUse = cp.yaw;
  // overlay: the real driven trajectory (ego track) in that frame (orange).
  const cur = f.meas_pose;
  ctx.strokeStyle = "rgba(230,140,0,0.9)"; ctx.lineWidth = 2; ctx.setLineDash([5, 4]);
  ctx.beginPath();
  let prevb = null;
  for (let i = 0; i < c.frames.length; i++) {
    const g = c.frames[i].meas_pose; if (!g) continue;
    const b = worldToBody(cur.e, cur.n, g.e, g.n, yawUse);
    if (b.x < -behindLive || b.x > aheadLive) { prevb = null; continue; }
    const x = toX(b.y), y = toY(b.x);
    if (!prevb || Math.hypot(b.x - prevb.x, b.y - prevb.y) > 3.0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
    prevb = b;
  }
  ctx.stroke(); ctx.setLineDash([]);

  // offline overlay (single view only): Python offline path (solid green) +
  // live JS-twin path (dashed blue) for parity checking.
  const off = (!label) ? offlineFramePath(STATE.frame) : null;
  const drawPoly = (poly) => {
    ctx.beginPath();
    poly.forEach((b, i) => {
      const x = toX(b.y), y = toY(b.x);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  };
  if (off) {
    ctx.strokeStyle = "#2e9e5b"; ctx.lineWidth = 3; drawPoly(off);          // offline (Python)
    ctx.strokeStyle = "#2b6fd6"; ctx.lineWidth = 2; ctx.setLineDash([4, 4]); // live (JS)
    drawPoly(pts); ctx.setLineDash([]);
  } else {
    ctx.strokeStyle = "#2e9e5b"; ctx.lineWidth = 3; drawPoly(pts);
  }
  // car at origin (width=1.8m, height=3.6m, centered)
  ctx.fillStyle = "#cc3a3a";
  ctx.fillRect(toX(0.9), toY(1.8), 1.8 * ppm, 3.6 * ppm);
  if (label) {
    // compact heading for a compare panel
    ctx.font = "11px sans-serif"; ctx.fillStyle = "#334";
    ctx.fillText(label, 6, 14);
  } else if (off) {
    ctx.font = "11px sans-serif";
    ctx.fillStyle = "#2e9e5b"; ctx.fillText("— offline (Python)", 8, 16);
    ctx.fillStyle = "#2b6fd6"; ctx.fillText("- - live (JS)", 8, 30);
    ctx.fillStyle = "rgba(230,140,0,0.95)"; ctx.fillText("- - real trajectory", 8, 44);
  } else {
    // full legend for the single driver view
    ctx.font = "11px sans-serif";
    ctx.fillStyle = "#2e9e5b"; ctx.fillText("— generated", 8, 16);
    ctx.fillStyle = "rgba(230,140,0,0.95)"; ctx.fillText("- - real trajectory", 8, 30);
  }
  if (f.end_flag) {
    ctx.fillStyle = "#cc3a3a"; ctx.font = "12px sans-serif";
    ctx.fillText("route ends", 8, label ? 28 : 44);
  }
}

// Compare mode: tile all five strategies side-by-side, each in its own panel,
// sharing the current frame and the live corner/window sliders.
const COMPARE_STRATEGIES = [
  ["raw", "Raw"], ["centered", "Centered"], ["smoothed", "Smoothed"],
  ["human", "Human"], ["human_centered", "Human centered"],
];
function drawCompare() {
  const c = STATE.case, f = c.frames[STATE.frame];
  if (!f.meas_pose) return;
  const base = currentConfig();
  const persp = document.getElementById("persp-toggle").checked;
  document.getElementById("compare-title").textContent =
    "Compare all algorithms (" + (persp ? "perspective" : "top-down") + ")";
  for (const [strategy, label] of COMPARE_STRATEGIES) {
    const cv = document.getElementById("cmp-" + strategy);
    if (!cv) continue;
    const cfg = Object.assign({}, base, { strategy });
    if (persp) drawWindshield(cv.getContext("2d"), f, cfg, label);
    else drawDriverTopDown(cv, f, cfg, label);
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
function drawWindshield(ctx, f, cfg, label) {
  cfg = cfg || currentConfig();
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
  const XMAX = cfg.ahead_m;
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
  const livePts = (STATE.case.mode === "pre")
    ? f.path.map(([x, y]) => ({ x, y })).filter(p => p.x >= 0)   // path straight from file
    : computeBodyPath(STATE.case, STATE.frame, cfg).pts.filter(p => p.x >= 0);
  const off = (!label) ? offlineFramePath(STATE.frame) : null;
  const offFwd = off ? off.filter(p => p.x >= 0) : null;
  const pts = offFwd || livePts;   // ribbon = offline path when loaded, else live
  const left = [], right = [], mid = [];
  for (const b of pts) {
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
  // offline parity overlay: draw the live JS path as a dashed blue line so it can
  // be compared against the offline ribbon (offline path is the ribbon above).
  if (offFwd) {
    const lp = [];
    for (const b of livePts) { const p = project(b.x, b.y); if (p) lp.push(p); }
    if (lp.length > 1) {
      ctx.strokeStyle = "#2b6fd6"; ctx.lineWidth = 2; ctx.setLineDash([5, 5]);
      ctx.beginPath(); ctx.moveTo(lp[0].u, lp[0].v);
      for (const p of lp) ctx.lineTo(p.u, p.v);
      ctx.stroke(); ctx.setLineDash([]);
    }
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
  ctx.fillText(label || (offFwd ? "offline (Python, green) vs live (JS, blue)"
                                : "driver view (perspective)"), 8, 16);
  if (f.end_flag) { ctx.fillStyle = "#cc3a3a"; ctx.fillText("route ends", 8, 30); }
}

function renderFrame() {
  const c = STATE.case; if (!c) return;
  if (c.mode === "real" || c.mode === "pre") drawBevReal(); else drawBev();
  if (c.mode !== "pre" && document.getElementById("compare-toggle").checked) drawCompare();
  else drawDriver();
  updateTelemetry(); drawPanoramaDot();
  document.getElementById("scrubber").value = STATE.frame;
  document.getElementById("frame-label").textContent =
    `${STATE.frame} / ${c.frames.length - 1}`;
}

function drawPanoramaDot() {
  if (STATE.case && STATE.case.mode === "pre") { drawPanoramaPre(true); return; }
  drawPanorama();  // static redraw (cheap) then dot
  const c = STATE.case, cv = document.getElementById("panorama");
  const ctx = cv.getContext("2d");
  const b = routeBounds(c);
  const T = fitTransform(cv, b.minE, b.maxE, b.minN, b.maxN);
  const f = c.frames[STATE.frame];
  const pose = f.true_pose || f.meas_pose;   // real cases carry meas_pose, not true_pose
  ctx.fillStyle = "#111";
  ctx.beginPath(); ctx.arc(T.toX(pose.e), T.toY(pose.n), 3, 0, 2 * Math.PI); ctx.fill();
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
  if (c.mode === "pre") {
    const L = c.route_total_len_m;
    set("tm-progress", (f.cursor_s == null || !L) ? "–" : `${(f.cursor_s / L * 100).toFixed(1)}%`);
  } else {
    const lastS = c.route.s[c.route.s.length - 1];
    set("tm-progress", f.cursor_s == null ? "–" : `${(f.cursor_s / lastS * 100).toFixed(1)}%`);
  }
  set("tm-seg", f.matched_seg == null ? "–" : String(f.matched_seg));
  set("tm-frame", `${STATE.frame} / ${c.frames.length - 1}`);
  const v = c.verdict;
  set("tm-verdict", c.mode === "pre" ? "— (pre-processed)"
    : v ? `${v.passed ? "PASS" : "FAIL"} (mis ${v.mismatches})` : "— (real data)");
}

// --- Offline (Python) processing ----------------------------------------
// The offline path for a given frame (or null when nothing is loaded / frame
// out of range). Shape mirrors ProjectOutput.path: [[x,y],...] -> [{x,y},...].
function offlineFramePath(frameIdx) {
  const o = STATE.offline;
  if (!o || !o.frames) return null;
  const fr = o.frames[frameIdx];
  if (!fr || !fr.path) return null;
  return fr.path.map(([x, y]) => ({ x, y }));
}

// Clear any loaded offline result and revert to the live view. Called whenever
// the algorithm/sliders/case/tab change, so a shown offline path always matches
// the settings it was computed with.
function clearOffline(msg) {
  const had = !!STATE.offline;
  STATE.offline = null;
  const el = document.getElementById("offline-status");
  if (el) el.textContent = msg || "";
  if (had && STATE.case) renderFrame();
}

function updateOfflineButton() {
  const btn = document.getElementById("btn-offline");
  if (!btn) return;
  const compare = document.getElementById("compare-toggle").checked;
  btn.disabled = STATE.mode !== "real" || compare || !STATE.case;
  btn.title = STATE.mode !== "real"
    ? "Offline test runs on real datasets only"
    : compare ? "Disabled while 'compare all' is on (offline shows one algorithm)"
              : "Run the Python project_route offline on this dataset and overlay the result";
}

async function runOffline() {
  if (STATE.mode !== "real" || !STATE.case) return;
  if (document.getElementById("compare-toggle").checked) return;
  const btn = document.getElementById("btn-offline");
  const el = document.getElementById("offline-status");
  const cfg = currentConfig();
  btn.disabled = true;
  el.textContent = "Start processing…";
  try {
    const resp = await fetch("/api/offline", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ dataset_id: STATE.case.case_id, config: cfg }),
    });
    const data = await resp.json();
    if (!resp.ok || !data.status || !data.status.generated) {
      const m = (data.status && data.status.message) || `HTTP ${resp.status}`;
      el.textContent = "Failed: " + m;
      STATE.offline = null;
    } else {
      STATE.offline = { frames: data.frames, strategy: cfg.strategy };
      el.textContent = `Done — you can check the results (${data.status.n_frames} frames, `
        + `strategy "${cfg.strategy}"). Green = offline (Python), dashed blue = live (JS).`;
      renderFrame();
    }
  } catch (e) {
    el.textContent = "Failed: " + e;
    STATE.offline = null;
  } finally {
    updateOfflineButton();
  }
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
  document.getElementById("compare-toggle").onchange = (ev) => {
    const on = ev.target.checked;
    document.getElementById("driver-fig").style.display = on ? "none" : "";
    document.getElementById("compare-fig").style.display = on ? "" : "none";
    clearOffline();                 // offline shows one algorithm; incompatible with compare
    updateOfflineButton();
    renderFrame();
  };
  // changing the algorithm or any slider invalidates a loaded offline result
  document.getElementById("algo-select").onchange = () => { clearOffline(); renderFrame(); };
  document.getElementById("corner-style").onchange = () => { clearOffline(); renderFrame(); };
  for (const id of ["p-radius", "p-behind", "p-ahead", "p-corner", "p-transition"]) {
    document.getElementById(id).oninput = (ev) => {
      const v = ev.target.value;
      document.getElementById(id + "-v").textContent = v;
      clearOffline();
      renderFrame();
    };
  }
  document.getElementById("btn-offline").onclick = runOffline;
  document.getElementById("btn-pre-folder").onclick = () =>
    document.getElementById("pre-folder").click();
  document.getElementById("pre-folder").onchange = (ev) => handlePreFolder(ev.target.files);
  updateOfflineButton();
});
