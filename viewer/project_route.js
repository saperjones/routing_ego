// DOM-free twin of src/parking_proj/project_route.py + smoothing.py.
// Parity-tested against the Python reference (tests/e2e/test_parity_py_js.py).
(function (root) {
  const DEFAULT_CONFIG = {
    strategy: "smoothed", behind_m: 5.0, ahead_m: 70.0, sample_ds_m: 0.5,
    search_ahead_m: 15.0, search_back_m: 0.3, heading_gate_deg: 60.0,
    min_turn_radius_m: 5.0, corner_angle_deg: 10.0, simplify_eps_m: 0.20,
  };

  function toBody(de, dn, yaw) {
    const c = Math.cos(yaw), s = Math.sin(yaw);
    return [de * c + dn * s, -de * s + dn * c];   // +x fwd, +y left
  }
  function unit(dx, dy) { const n = Math.hypot(dx, dy); return n < 1e-9 ? [0, 0] : [dx / n, dy / n]; }

  function rdp(pts, eps) {
    if (pts.length < 3) return pts.slice();
    const [x0, y0] = pts[0], [x1, y1] = pts[pts.length - 1];
    const dx = x1 - x0, dy = y1 - y0, seg2 = dx * dx + dy * dy;
    let dmax = -1, idx = 0;
    for (let i = 1; i < pts.length - 1; i++) {
      const [px, py] = pts[i]; let d;
      if (seg2 === 0) d = Math.hypot(px - x0, py - y0);
      else { let t = ((px - x0) * dx + (py - y0) * dy) / seg2; t = t < 0 ? 0 : t > 1 ? 1 : t;
             d = Math.hypot(px - (x0 + t * dx), py - (y0 + t * dy)); }
      if (d > dmax) { dmax = d; idx = i; }
    }
    if (dmax > eps) return rdp(pts.slice(0, idx + 1), eps).slice(0, -1).concat(rdp(pts.slice(idx), eps));
    return [pts[0], pts[pts.length - 1]];
  }

  function resample(pts, ds) {
    if (pts.length < 2) return pts.slice();
    const out = [pts[0]]; let [px, py] = pts[0], acc = 0;
    for (let i = 1; i < pts.length; i++) {
      let [qx, qy] = pts[i]; let seg = Math.hypot(qx - px, qy - py);
      while (seg > 0 && acc + seg >= ds) {
        const t = (ds - acc) / seg; px += t * (qx - px); py += t * (qy - py);
        out.push([px, py]); seg = Math.hypot(qx - px, qy - py); acc = 0;
      }
      acc += seg; px = qx; py = qy;
    }
    const last = pts[pts.length - 1];
    if (out[out.length - 1][0] !== last[0] || out[out.length - 1][1] !== last[1]) out.push(last);
    return out;
  }

  function smoothCorners(pts, R, angleDeg, ds, eps) {
    if (pts.length < 3) return resample(pts, ds);
    const verts = rdp(pts, eps);
    if (verts.length < 3) return resample(verts, ds);
    const thresh = angleDeg * Math.PI / 180;
    const out = [verts[0]];
    for (let i = 1; i < verts.length - 1; i++) {
      const [ax, ay] = verts[i - 1], [vx, vy] = verts[i], [bx, by] = verts[i + 1];
      const [d1x, d1y] = unit(vx - ax, vy - ay), [d2x, d2y] = unit(bx - vx, by - vy);
      let dot = d1x * d2x + d1y * d2y; dot = dot < -1 ? -1 : dot > 1 ? 1 : dot;
      const delta = Math.acos(dot);
      if (delta < thresh) { out.push([vx, vy]); continue; }
      const cross = d1x * d2y - d1y * d2x, tanHalf = Math.tan(delta / 2);
      if (tanHalf < 1e-9) { out.push([vx, vy]); continue; }
      const T = Math.min(R * tanHalf, 0.5 * Math.hypot(vx - ax, vy - ay), 0.5 * Math.hypot(bx - vx, by - vy));
      if (T < 1e-6) { out.push([vx, vy]); continue; }
      const rEff = T / tanHalf;
      const p1x = vx - T * d1x, p1y = vy - T * d1y;
      const [nx, ny] = cross >= 0 ? [-d1y, d1x] : [d1y, -d1x];
      const cx = p1x + rEff * nx, cy = p1y + rEff * ny;
      const a1 = Math.atan2(p1y - cy, p1x - cx), sign = cross >= 0 ? 1 : -1;
      const steps = Math.max(1, Math.ceil(rEff * delta / ds));
      out.push([p1x, p1y]);
      for (let k = 1; k <= steps; k++) {
        const a = a1 + sign * delta * (k / steps);
        out.push([cx + rEff * Math.cos(a), cy + rEff * Math.sin(a)]);
      }
    }
    out.push(verts[verts.length - 1]);
    return resample(out, ds);
  }

  function indexAtS(route, s) {
    const arr = route.s, L = route.length;
    if (s >= L) return arr.length - 1;
    s = s < 0 ? 0 : s;
    let lo = 0, hi = arr.length;                 // first index with arr[i] > s, minus 1
    while (lo < hi) { const m = (lo + hi) >> 1; if (arr[m] <= s) lo = m + 1; else hi = m; }
    return Math.max(0, lo - 1);
  }
  function pointAtS(route, s) { return route.points[indexAtS(route, s)]; }

  function bestInRange(route, pe, pn, yaw, loS, hiS, gate) {
    const lo = indexAtS(route, Math.max(loS, 0)), hi = Math.max(indexAtS(route, Math.min(hiS, route.length)), lo);
    let best = lo, bestD = Infinity, anyGated = false;
    for (let i = lo; i <= hi; i++) {
      const [ex, ny] = route.points[i], [tx, ty] = route.tangents[i];
      let dy = Math.atan2(ty, tx) - yaw; dy = Math.abs(((dy + Math.PI) % (2 * Math.PI)) - Math.PI);
      if (dy <= gate) anyGated = true;
    }
    for (let i = lo; i <= hi; i++) {
      const [ex, ny] = route.points[i], [tx, ty] = route.tangents[i];
      let dy = Math.atan2(ty, tx) - yaw; dy = Math.abs(((dy + Math.PI) % (2 * Math.PI)) - Math.PI);
      if (anyGated && dy > gate) continue;
      const d = (ex - pe) ** 2 + (ny - pn) ** 2;
      if (d < bestD) { bestD = d; best = i; }
    }
    return best;
  }

  function match(route, pe, pn, yaw, cfg, state) {
    const gate = cfg.heading_gate_deg * Math.PI / 180;
    let cursor;
    if (!state || !state.initialized) {
      cursor = route.s[bestInRange(route, pe, pn, yaw, 0, route.length, gate)];
    } else {
      const mi = bestInRange(route, pe, pn, yaw, state.cursor_s - cfg.search_back_m,
                             state.cursor_s + cfg.search_ahead_m, gate);
      cursor = Math.max(state.cursor_s, route.s[mi]);
    }
    const ci = indexAtS(route, cursor), [mx, my] = route.points[ci], [tx, ty] = route.tangents[ci];
    const latDev = (pe - mx) * (-ty) + (pn - my) * (tx);
    return { cursor_s: cursor, matched_seg: route.seg_of_index ? route.seg_of_index[ci] : ci,
             lat_dev: latDev, end_flag: (cursor + cfg.ahead_m) >= route.length - 1e-9 };
  }

  function projectRoute(route, pose, cfg, state) {
    const m = match(route, pose.e, pose.n, pose.h, cfg, state);
    const [ax, ay] = pointAtS(route, m.cursor_s);
    const latShift = toBody(ax - pose.e, ay - pose.n, pose.h)[1];
    const lo = Math.max(m.cursor_s - cfg.behind_m, 0), hi = Math.min(m.cursor_s + cfg.ahead_m, route.length);
    const n = Math.floor((hi - lo) / cfg.sample_ds_m) + 1;
    const behind = [], fwd = [];
    for (let k = 0; k < n; k++) {
      const s = lo + k * cfg.sample_ds_m, [qx, qy] = pointAtS(route, s);
      let [bx, by] = toBody(qx - pose.e, qy - pose.n, pose.h);
      if (cfg.strategy !== "raw") by -= latShift;
      (s < m.cursor_s ? behind : fwd).push([bx, by]);
    }
    let f = fwd;
    if (cfg.strategy === "smoothed" && fwd.length >= 3)
      f = smoothCorners(fwd, cfg.min_turn_radius_m, cfg.corner_angle_deg, cfg.sample_ds_m, cfg.simplify_eps_m);
    return { path: behind.concat(f), cursor_s: m.cursor_s, lat_dev: m.lat_dev,
             matched_seg: m.matched_seg, end_flag: m.end_flag,
             state: { cursor_s: m.cursor_s, initialized: true } };
  }

  // Build route {points,s,tangents,length,seg_of_index} from baked points_e/points_n(/s).
  function buildRoute(points_e, points_n, s_opt) {
    const points = points_e.map((e, i) => [e, points_n[i]]);
    let s = s_opt;
    if (!s) { s = [0]; for (let i = 1; i < points.length; i++)
      s.push(s[i - 1] + Math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])); }
    const tangents = points.map((_, i) => {
      const a = points[Math.max(0, i - 1)], b = points[Math.min(points.length - 1, i + 1)];
      return unit(b[0] - a[0], b[1] - a[1]);
    });
    return { points, s, tangents, length: s[s.length - 1] };
  }

  root.ProjectRoute = { DEFAULT_CONFIG, projectRoute, match, rdp, smoothCorners, resample,
                        toBody, indexAtS, pointAtS, bestInRange, buildRoute };
})(typeof window !== "undefined" ? window : globalThis);
