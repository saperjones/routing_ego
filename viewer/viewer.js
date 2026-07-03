const STATE = { case: null, frame: 0, playing: false, speed: 1 };

async function loadIndex() {
  const res = await fetch("../out/index.json");
  const data = await res.json();
  const ul = document.getElementById("case-list");
  ul.innerHTML = "";
  for (const c of data.cases) {
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
  drawPanorama();
  renderFrame();   // defined in Task 10
}

window.addEventListener("DOMContentLoaded", loadIndex);
