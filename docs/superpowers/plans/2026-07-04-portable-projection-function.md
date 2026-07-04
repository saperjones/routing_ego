# Portable Route-Projection Function + Live Demo — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract the route→body-frame projection into one portable, config-driven function with three strategies (raw / centered / smoothed-with-arc-fillet), port it to a DOM-free JS twin, and make the demo compute every path live with a left-panel algorithm selector + parameter sliders.

**Architecture:** A pure Python function `project_route(route, pose, config, state) -> ProjectOutput` (matching + strategy) plus a `smoothing.py` (RDP + arc fillet). `projection.py`'s `Projector` becomes a thin wrapper over the shared matcher; baked path fields are removed from the JSON. A DOM-free `viewer/project_route.js` mirrors the Python and runs live in the viewer; a Python↔JS parity test binds them.

**Tech Stack:** Python 3.12 + numpy; static HTML/Canvas JS; pytest + Playwright (headless Chromium).

## Global Constraints

- Body frame is `+x` forward, `+y` left; heading `yaw` = CCW from +E, radians.
- Output window is `[−behind_m, +ahead_m]` sampled at `sample_ds_m`; defaults `behind_m=5.0`, `ahead_m=70.0`, `sample_ds_m=0.5`.
- Three strategies, selected by `config.strategy` ∈ {`"raw"`, `"centered"`, `"smoothed"`}; `centered`/`smoothed` subtract `lat_shift` (car-frame lateral of the anchor at `cursor_s`); `smoothed` additionally arc-fillets the forward path to curvature ≤ `1/min_turn_radius_m`.
- Matching defaults: `search_ahead_m=15.0`, `search_back_m=0.3`, `heading_gate_deg=60.0`; cursor is monotonic (`cursor_s = max(cursor_s, matched_s)`).
- Smoothing defaults: `min_turn_radius_m=5.0`, `corner_angle_deg=10.0`, `simplify_eps_m=0.20`.
- Python is authoritative; a Python↔JS parity test must pass (tolerance 1e-3 m).
- Grading and the 14 acceptance verdicts must stay identical (matching output is strategy-independent).
- All randomness stays seeded / deterministic (no RNG added).
- Update `spec_design.md` and `CLAUDE.md` (architecture invariant changes: viewer runs the shared algorithm live).

---

### Task 1: `smoothing.py` — RDP + arc-fillet corner smoothing

**Files:**
- Create: `src/parking_proj/smoothing.py`
- Test: `tests/test_smoothing.py`

**Interfaces:**
- Produces: `rdp(pts, eps) -> list[(x,y)]`; `smooth_corners(pts, min_radius, corner_angle_deg, ds, eps) -> list[(x,y)]`; `resample(pts, ds) -> list[(x,y)]`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_smoothing.py`:

```python
import math
import numpy as np
from parking_proj.smoothing import rdp, smooth_corners


def _l_shape(step=0.1):
    up = [(0.0, round(y, 3)) for y in np.arange(0.0, 10.0 + step / 2, step)]
    right = [(round(x, 3), 10.0) for x in np.arange(step, 10.0 + step / 2, step)]
    return up + right


def _max_heading_step(pts):
    m = 0.0
    for i in range(2, len(pts)):
        a = math.atan2(pts[i - 1][1] - pts[i - 2][1], pts[i - 1][0] - pts[i - 2][0])
        b = math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0])
        m = max(m, abs((b - a + math.pi) % (2 * math.pi) - math.pi))
    return m


def test_rdp_collapses_l_shape_to_three_vertices():
    verts = rdp(_l_shape(), eps=0.2)
    assert len(verts) == 3                       # start, corner, end
    assert verts[0] == (0.0, 0.0)


def test_raw_corner_is_sharp():
    assert _max_heading_step(_l_shape()) > 1.5   # ~pi/2 jump at the corner


def test_smooth_corners_bounds_curvature():
    R, ds = 5.0, 0.5
    out = smooth_corners(_l_shape(), min_radius=R, corner_angle_deg=10.0, ds=ds, eps=0.2)
    # arc sampled at ds on radius R turns by ~ds/R per step; allow tolerance
    assert _max_heading_step(out) <= ds / R + 0.05
    # endpoints preserved (roughly): starts near origin, ends near (10,10)
    assert abs(out[0][0]) < 1e-6 and abs(out[0][1]) < 1e-6


def test_smooth_corners_keeps_straight_line_straight():
    line = [(0.0, y) for y in np.arange(0.0, 10.01, 0.1)]
    out = smooth_corners(line, min_radius=5.0, corner_angle_deg=10.0, ds=0.5, eps=0.2)
    assert _max_heading_step(out) < 1e-6
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=src .venv/bin/python -m pytest tests/test_smoothing.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'parking_proj.smoothing'`.

- [ ] **Step 3: Implement `smoothing.py`**

Create `src/parking_proj/smoothing.py`:

```python
"""Fast, embedded-friendly polyline smoothing: RDP corner detection + circular-arc fillet.

All operations are closed-form (no iteration-to-converge); curvature on filleted
corners is bounded by 1/min_radius, so the resulting path is drivable at a
minimum turning radius of min_radius.
"""
import math


def rdp(pts, eps):
    """Ramer-Douglas-Peucker polyline simplification. pts: list of (x, y)."""
    if len(pts) < 3:
        return list(pts)
    x0, y0 = pts[0]
    x1, y1 = pts[-1]
    dx, dy = x1 - x0, y1 - y0
    seg2 = dx * dx + dy * dy
    dmax, idx = -1.0, 0
    for i in range(1, len(pts) - 1):
        px, py = pts[i]
        if seg2 == 0.0:
            d = math.hypot(px - x0, py - y0)
        else:
            t = ((px - x0) * dx + (py - y0) * dy) / seg2
            t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
            d = math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
        if d > dmax:
            dmax, idx = d, i
    if dmax > eps:
        left = rdp(pts[:idx + 1], eps)
        right = rdp(pts[idx:], eps)
        return left[:-1] + right
    return [pts[0], pts[-1]]


def resample(pts, ds):
    """Uniformly resample a polyline at spacing ds (arc length)."""
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    px, py = pts[0]
    acc = 0.0
    for i in range(1, len(pts)):
        qx, qy = pts[i]
        seg = math.hypot(qx - px, qy - py)
        while seg > 0.0 and acc + seg >= ds:
            t = (ds - acc) / seg
            px, py = px + t * (qx - px), py + t * (qy - py)
            out.append((px, py))
            seg = math.hypot(qx - px, qy - py)
            acc = 0.0
        acc += seg
        px, py = qx, qy
    if out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


def _unit(dx, dy):
    n = math.hypot(dx, dy)
    return (0.0, 0.0) if n < 1e-9 else (dx / n, dy / n)


def smooth_corners(pts, min_radius, corner_angle_deg, ds, eps):
    """Replace sharp corners of a polyline with circular-arc fillets (radius >= min_radius).

    Returns a new polyline resampled at ds with curvature <= 1/min_radius on
    arcs and 0 on straights. corner_angle_deg: only fillet turns sharper than
    this. eps: RDP tolerance for corner-vertex detection.
    """
    if len(pts) < 3:
        return resample(pts, ds)
    verts = rdp(pts, eps)
    if len(verts) < 3:
        return resample(verts, ds)
    thresh = math.radians(corner_angle_deg)
    out = [verts[0]]
    for i in range(1, len(verts) - 1):
        ax, ay = verts[i - 1]
        vx, vy = verts[i]
        bx, by = verts[i + 1]
        d1x, d1y = _unit(vx - ax, vy - ay)
        d2x, d2y = _unit(bx - vx, by - vy)
        dot = max(-1.0, min(1.0, d1x * d2x + d1y * d2y))
        delta = math.acos(dot)                       # unsigned turn angle
        if delta < thresh:
            out.append((vx, vy))
            continue
        cross = d1x * d2y - d1y * d2x                 # > 0 => left turn
        half = delta / 2.0
        tan_half = math.tan(half)
        if tan_half < 1e-9:
            out.append((vx, vy))
            continue
        T = min(min_radius * tan_half,
                0.5 * math.hypot(vx - ax, vy - ay),
                0.5 * math.hypot(bx - vx, by - vy))
        if T < 1e-6:
            out.append((vx, vy))
            continue
        r_eff = T / tan_half
        p1x, p1y = vx - T * d1x, vy - T * d1y
        nx, ny = (-d1y, d1x) if cross >= 0 else (d1y, -d1x)   # toward turn center
        cx, cy = p1x + r_eff * nx, p1y + r_eff * ny
        a1 = math.atan2(p1y - cy, p1x - cx)
        sign = 1.0 if cross >= 0 else -1.0
        steps = max(1, int(math.ceil(r_eff * delta / ds)))
        out.append((p1x, p1y))
        for k in range(1, steps + 1):
            a = a1 + sign * delta * (k / steps)
            out.append((cx + r_eff * math.cos(a), cy + r_eff * math.sin(a)))
    out.append(verts[-1])
    return resample(out, ds)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=src .venv/bin/python -m pytest tests/test_smoothing.py -q`
Expected: PASS (5 passed).

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/smoothing.py tests/test_smoothing.py
git commit -m "feat: smoothing.py — RDP + circular-arc fillet (curvature <= 1/R_min)"
```

---

### Task 2: `project_route.py` — the portable function (matching + 3 strategies)

**Files:**
- Create: `src/parking_proj/project_route.py`
- Test: `tests/test_project_route.py`

**Interfaces:**
- Consumes: `smoothing.smooth_corners`; `transform.to_body_frame`; `Route` (`.points`, `.s`, `.tangents`, `.length`, `.point_at_s`, `.index_at_s`, `.seg_of_index`).
- Produces: `ProjectConfig`, `ProjectState`, `ProjectOutput` dataclasses; `SEARCH_AHEAD = 15.0`; `project_route(route, pose_e, pose_n, yaw, config, state=None, speed=None) -> ProjectOutput`; `_match(route, pose_e, pose_n, yaw, cfg, state) -> (cursor_s, matched_seg, lat_dev, end_flag)`; `_best_in_range(route, pos_e, pos_n, yaw, lo_s, hi_s, gate_rad) -> int`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_project_route.py`:

```python
import math
import numpy as np
import pytest
from parking_proj.geometry import route_from_waypoints
from parking_proj.project_route import project_route, ProjectConfig, ProjectState


def straight():
    return route_from_waypoints([[0.0, 0.0], [100.0, 0.0]], ["1", "2"])


def l_route():
    # 30 m east, then 30 m north — a 90 deg left corner
    return route_from_waypoints([[0.0, 0.0], [30.0, 0.0], [30.0, 30.0]], ["1", "2", "3"])


def _max_heading_step(path):
    m = 0.0
    for i in range(2, len(path)):
        a = math.atan2(path[i - 1][1] - path[i - 2][1], path[i - 1][0] - path[i - 2][0])
        b = math.atan2(path[i][1] - path[i - 1][1], path[i][0] - path[i - 1][0])
        m = max(m, abs((b - a + math.pi) % (2 * math.pi) - math.pi))
    return m


def test_raw_keeps_lateral_offset():
    r = straight()
    cfg = ProjectConfig(strategy="raw")
    out = project_route(r, 10.0, 2.0, 0.0, cfg)      # 2 m left of an east route
    fwd = [p for p in out.path if p[0] >= 0]
    assert min(p[1] for p in fwd) < -1.9             # route sits ~2 m to the right (kept)
    assert out.lat_dev == pytest.approx(2.0, abs=1e-6)


def test_centered_removes_offset():
    r = straight()
    out = project_route(r, 10.0, 2.0, 0.0, ProjectConfig(strategy="centered"))
    assert max(abs(p[1]) for p in out.path) < 1e-6   # offset nulled
    assert out.lat_dev == pytest.approx(2.0, abs=1e-6)


def test_window_bounds_and_spacing():
    r = straight()
    out = project_route(r, 20.0, 0.0, 0.0, ProjectConfig(strategy="centered",
                                                         behind_m=5.0, ahead_m=70.0))
    xs = [p[0] for p in out.path]
    assert min(xs) == pytest.approx(-5.0, abs=0.6)
    assert max(xs) == pytest.approx(70.0, abs=0.6)


def test_monotonic_cursor_catches_alongtrack_jump():
    r = straight()
    st = ProjectState()
    o1 = project_route(r, 0.0, 0.0, 0.0, ProjectConfig(), st)
    o2 = project_route(r, 10.0, 0.0, 0.0, ProjectConfig(), o1.state)
    assert o2.cursor_s == pytest.approx(10.0, abs=0.2)   # 15 m window catches the jump


def test_smoothed_rounds_the_corner():
    r = l_route()
    # sit at the start heading east so the corner is in the look-ahead
    sharp = project_route(r, 0.0, 0.0, 0.0, ProjectConfig(strategy="centered")).path
    smooth = project_route(r, 0.0, 0.0, 0.0,
                           ProjectConfig(strategy="smoothed", min_turn_radius_m=5.0,
                                         sample_ds_m=0.5)).path
    assert _max_heading_step(sharp) > 1.0            # centered has the sharp 90 deg
    assert _max_heading_step(smooth) <= 0.5 / 5.0 + 0.05   # smoothed is bounded (<= ds/R)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=src .venv/bin/python -m pytest tests/test_project_route.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'parking_proj.project_route'`.

- [ ] **Step 3: Implement `project_route.py`**

Create `src/parking_proj/project_route.py`:

```python
"""Portable route -> body-frame projection: monotonic matching + 3 strategies.

The single entry point is project_route(). It is pure: the caller holds a small
ProjectState (the monotonic progress cursor) and passes it back each frame, so
the function is deterministic and portable (no hidden state). Body frame is
+x forward, +y left; yaw is CCW from +E in radians.
"""
import math
from dataclasses import dataclass
import numpy as np
from .transform import to_body_frame
from .smoothing import smooth_corners

SEARCH_AHEAD = 15.0


@dataclass
class ProjectConfig:
    strategy: str = "smoothed"          # "raw" | "centered" | "smoothed"
    behind_m: float = 5.0
    ahead_m: float = 70.0
    sample_ds_m: float = 0.5
    search_ahead_m: float = SEARCH_AHEAD
    search_back_m: float = 0.3
    heading_gate_deg: float = 60.0
    min_turn_radius_m: float = 5.0
    corner_angle_deg: float = 10.0
    simplify_eps_m: float = 0.20


@dataclass
class ProjectState:
    cursor_s: float = None
    initialized: bool = False


@dataclass
class ProjectOutput:
    path: list          # list of [x, y] in the body frame, -behind_m .. +ahead_m
    cursor_s: float
    lat_dev: float
    matched_seg: int
    end_flag: bool
    state: ProjectState


def _best_in_range(route, pos_e, pos_n, yaw, lo_s, hi_s, gate_rad):
    lo = route.index_at_s(max(lo_s, 0.0))
    hi = route.index_at_s(min(hi_s, route.length))
    hi = max(hi, lo)
    idxs = np.arange(lo, hi + 1)
    pts = route.points[idxs]
    d2 = (pts[:, 0] - pos_e) ** 2 + (pts[:, 1] - pos_n) ** 2
    yaws = np.arctan2(route.tangents[idxs][:, 1], route.tangents[idxs][:, 0])
    dyaw = np.abs((yaws - yaw + math.pi) % (2 * math.pi) - math.pi)
    gated = dyaw <= gate_rad
    if not np.any(gated):
        gated = np.ones_like(d2, dtype=bool)
    masked = np.where(gated, d2, np.inf)
    return int(idxs[int(np.argmin(masked))])


def _match(route, pose_e, pose_n, yaw, cfg, state):
    gate = math.radians(cfg.heading_gate_deg)
    if state is None or not state.initialized:
        mi = _best_in_range(route, pose_e, pose_n, yaw, 0.0, route.length, gate)
        cursor_s = float(route.s[mi])
    else:
        mi = _best_in_range(route, pose_e, pose_n, yaw,
                            state.cursor_s - cfg.search_back_m,
                            state.cursor_s + cfg.search_ahead_m, gate)
        cursor_s = max(state.cursor_s, float(route.s[mi]))
    ci = route.index_at_s(cursor_s)
    mp = route.points[ci]
    tang = route.tangents[ci]
    normal_left = np.array([-tang[1], tang[0]])
    lat_dev = float(np.dot(np.array([pose_e, pose_n]) - mp, normal_left))
    matched_seg = int(route.seg_of_index[ci])
    end_flag = (cursor_s + cfg.ahead_m) >= route.length - 1e-9
    return cursor_s, matched_seg, lat_dev, end_flag


def project_route(route, pose_e, pose_n, yaw, config, state=None, speed=None):
    cfg = config
    cursor_s, matched_seg, lat_dev, end_flag = _match(route, pose_e, pose_n, yaw, cfg, state)
    ax, ay = route.point_at_s(cursor_s)
    _, lat_shift = to_body_frame(ax - pose_e, ay - pose_n, yaw)   # car-frame lateral of anchor
    lo = max(cursor_s - cfg.behind_m, 0.0)
    hi = min(cursor_s + cfg.ahead_m, route.length)
    n = int((hi - lo) / cfg.sample_ds_m) + 1
    behind_pts, fwd_pts = [], []
    for k in range(n):
        s = lo + k * cfg.sample_ds_m
        qx, qy = route.point_at_s(s)
        bx, by = to_body_frame(qx - pose_e, qy - pose_n, yaw)
        if cfg.strategy != "raw":
            by -= lat_shift
        (behind_pts if s < cursor_s else fwd_pts).append((bx, by))
    if cfg.strategy == "smoothed" and len(fwd_pts) >= 3:
        fwd_pts = smooth_corners(fwd_pts, cfg.min_turn_radius_m, cfg.corner_angle_deg,
                                 cfg.sample_ds_m, cfg.simplify_eps_m)
    path = [[x, y] for x, y in (behind_pts + fwd_pts)]
    return ProjectOutput(path=path, cursor_s=cursor_s, lat_dev=lat_dev,
                         matched_seg=matched_seg, end_flag=end_flag,
                         state=ProjectState(cursor_s=cursor_s, initialized=True))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=src .venv/bin/python -m pytest tests/test_project_route.py -q`
Expected: PASS (6 passed).

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/project_route.py tests/test_project_route.py
git commit -m "feat: project_route() — portable matching + raw/centered/smoothed strategies"
```

---

### Task 3: Fold `Projector` onto `_match`; drop baked path fields; regenerate

**Files:**
- Modify: `src/parking_proj/projection.py` (rewrite `Projector` as a wrapper; remove `follow_path`, `FOLLOW_AHEAD`, `FOLLOW_DS`)
- Modify: `src/parking_proj/generate.py` (remove baked path emission)
- Modify: `src/parking_proj/generate_real.py` (remove baked path emission)
- Delete: `tests/test_follow_path.py`, `tests/test_generate_follow_path.py`
- Test: existing `tests/test_search_window.py` + acceptance suite must stay green

**Interfaces:**
- Consumes: `project_route._match`, `_best_in_range`, `ProjectConfig`, `ProjectState`, `SEARCH_AHEAD`.
- Produces: `Projector` (same public surface: `.step()`, `.reset()`, `.w_search`, `.ahead`, `.behind`; returns `ProjectionResult`), `SEARCH_AHEAD` re-exported from `projection`.

- [ ] **Step 1: Rewrite `projection.py` as a thin wrapper**

Replace the entire contents of `src/parking_proj/projection.py` with:

```python
"""Back-compat matcher wrapper. The algorithm now lives in project_route.py;
Projector delegates to its shared matcher so there is a single implementation."""
import math
from dataclasses import dataclass
from .project_route import _match, ProjectConfig, ProjectState, SEARCH_AHEAD  # noqa: F401


@dataclass
class ProjectionResult:
    cursor_s: float
    matched_index: int
    matched_seg: int
    est_lat_dev: float
    end_flag: bool
    gate_widened: bool


class Projector:
    def __init__(self, route, ahead=20.0, behind=-5.0,
                 w_search=SEARCH_AHEAD, eps_back=0.3, gate_deg=60.0):
        self.route = route
        self.ahead = ahead
        self.behind = behind
        self.w_search = w_search
        self._cfg = ProjectConfig(ahead_m=ahead, behind_m=abs(behind),
                                  search_ahead_m=w_search, search_back_m=eps_back,
                                  heading_gate_deg=gate_deg)
        self.reset()

    def reset(self):
        self._state = ProjectState()

    def step(self, pose_e, pose_n, yaw):
        cursor_s, matched_seg, lat_dev, end_flag = _match(
            self.route, pose_e, pose_n, yaw, self._cfg, self._state)
        self._state = ProjectState(cursor_s=cursor_s, initialized=True)
        ci = self.route.index_at_s(cursor_s)
        return ProjectionResult(cursor_s=cursor_s, matched_index=ci,
                                matched_seg=matched_seg, est_lat_dev=lat_dev,
                                end_flag=end_flag, gate_widened=False)
```

- [ ] **Step 2: Delete the superseded follow_path tests**

```bash
git rm tests/test_follow_path.py tests/test_generate_follow_path.py
```

- [ ] **Step 3: Remove baked path emission from `generate.py`**

In `src/parking_proj/generate.py`: change the import line 5 to `from .projection import Projector`. In the frame loop, delete the `fp, lat_shift = ...` lines and the `"follow_path"` / `"lat_shift"` dict entries. Change the `config` dict to `{"ahead": proj.ahead, "behind": proj.behind}` (drop `follow_ahead`/`follow_ds`). The frame dict becomes:

```python
    for f, r in zip(frames, results):
        frame_dicts.append({
            "t": round(f.t, 3),
            "speed": round(f.speed, 3),
            "true_pose": {"e": round(f.true_e, 3), "n": round(f.true_n, 3),
                          "h": round(f.true_yaw, 5)},
            "meas_pose": {"e": round(f.meas_e, 3), "n": round(f.meas_n, 3),
                          "h": round(f.meas_yaw, 5),
                          "pitch": round(f.pitch, 6), "roll": round(f.roll, 6)},
            "cursor_s": None if r is None else round(r.cursor_s, 3),
            "matched_seg": None if r is None else r.matched_seg,
            "est_lat_dev": None if r is None else round(r.est_lat_dev, 4),
            "true_lat_dev": round(grading.true_lat_dev(route, f), 4),
            "end_flag": None if r is None else bool(r.end_flag),
            "gt_seg": f.gt_seg,
            "gt_s": round(f.gt_s, 3),
        })
```

and

```python
        "config": {"ahead": proj.ahead, "behind": proj.behind},
```

- [ ] **Step 4: Remove baked path emission from `generate_real.py`**

In `src/parking_proj/generate_real.py`: change import line 8 to `from .projection import Projector`. Delete the `fp, lat_shift = follow_path(...)` call and the `"follow_path"`/`"lat_shift"` dict entries; set `config` to `{"ahead": proj.ahead, "behind": proj.behind}`. The frame loop becomes:

```python
    for i in range(len(ds.meas_e)):
        r = proj.step(float(ds.meas_e[i]), float(ds.meas_n[i]), float(ds.meas_yaw[i]))
        frames.append({
            "t": _r(ds.t_us[i] / 1e6),
            "speed": _r(ds.speed[i]),
            "meas_pose": {"e": _r(ds.meas_e[i]), "n": _r(ds.meas_n[i]),
                          "h": _r(ds.meas_yaw[i], 5)},
            "meas_ll": {"lat": _r(ds.ego_llh[i, 0], 7), "lon": _r(ds.ego_llh[i, 1], 7)},
            "cursor_s": _r(r.cursor_s),
            "matched_seg": r.matched_seg,
            "est_lat_dev": _r(r.est_lat_dev, 4),
            "end_flag": bool(r.end_flag),
        })
```

and

```python
        "config": {"ahead": proj.ahead, "behind": proj.behind},
```

- [ ] **Step 5: Run the full suite + regenerate; verify acceptance unchanged**

Run: `PYTHONPATH=src .venv/bin/python -m pytest -q`
Expected: PASS (acceptance 14 cases unchanged; `test_search_window.py`, `test_smoothing.py`, `test_project_route.py` pass; no import errors).

Run: `PYTHONPATH=src .venv/bin/python -m parking_proj.generate`
Run: `PYTHONPATH=src .venv/bin/python -m parking_proj.generate_real`
Expected: both write without error; `out/*.json` frames no longer contain `follow_path`/`lat_shift`.

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "refactor: Projector wraps shared matcher; drop baked path fields from JSON"
```

---

### Task 4: `viewer/project_route.js` — DOM-free JS twin + parity test

**Files:**
- Create: `viewer/project_route.js`
- Create: `tests/e2e/parity_harness.html`
- Create: `tests/e2e/test_parity_py_js.py`

**Interfaces:**
- Consumes (parity fixture): Python `project_route` outputs for the same inputs.
- Produces: `window.ProjectRoute = { DEFAULT_CONFIG, projectRoute, match, rdp, smoothCorners, resample, toBody, indexAtS, buildRoute }`. `projectRoute(route, pose, cfg, state) -> {path, cursor_s, lat_dev, matched_seg, end_flag, state}`; `route = {points:[[e,n]…], s:[…], tangents:[[tx,ty]…], length}`; `pose = {e, n, h}`.

- [ ] **Step 1: Write the parity harness page**

Create `tests/e2e/parity_harness.html`:

```html
<!doctype html><html><head><meta charset="utf-8">
<script src="/viewer/project_route.js"></script></head>
<body><script>
window.runCase = (route, pose, cfg) =>
  window.ProjectRoute.projectRoute(route, pose, cfg, null);
</script></body></html>
```

- [ ] **Step 2: Write the failing parity test**

Create `tests/e2e/test_parity_py_js.py`:

```python
import http.client, os, socket, subprocess, sys, time
from pathlib import Path
import pytest

pytest.importorskip("playwright.sync_api")
from playwright.sync_api import sync_playwright  # noqa: E402

pytestmark = pytest.mark.e2e
REPO = Path(__file__).resolve().parents[2]
SRC = REPO / "src"


def _free_port():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close(); return p


def _wait(port, path, timeout=15):
    end = time.time() + timeout
    while time.time() < end:
        try:
            c = http.client.HTTPConnection("127.0.0.1", port, timeout=1)
            c.request("GET", path)
            if c.getresponse().status == 200:
                return True
        except OSError:
            time.sleep(0.1)
    return False


def _fixtures():
    """(route_dict, pose, cfg_dict, expected_path) tuples from the Python impl."""
    sys.path.insert(0, str(SRC))
    from parking_proj.geometry import route_from_waypoints
    from parking_proj.project_route import project_route, ProjectConfig
    from dataclasses import asdict
    cases = []
    routes = {
        "straight": route_from_waypoints([[0.0, 0.0], [100.0, 0.0]], ["1", "2"]),
        "corner": route_from_waypoints([[0.0, 0.0], [30.0, 0.0], [30.0, 30.0]], ["1", "2", "3"]),
    }
    poses = [(10.0, 2.0, 0.0), (25.0, 1.0, 0.1), (0.0, 0.0, 0.0)]
    for rk, r in routes.items():
        rd = {"points": r.points.tolist(), "s": r.s.tolist(),
              "tangents": r.tangents.tolist(), "length": r.length}
        for (pe, pn, yaw) in poses:
            for strat in ("raw", "centered", "smoothed"):
                cfg = ProjectConfig(strategy=strat)
                out = project_route(r, pe, pn, yaw, cfg)
                cases.append((rd, {"e": pe, "n": pn, "h": yaw}, asdict(cfg), out.path))
    return cases


def test_js_matches_python():
    port = _free_port()
    proc = subprocess.Popen([sys.executable, "-m", "http.server", str(port)],
                            cwd=REPO, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        assert _wait(port, "/tests/e2e/parity_harness.html")
        with sync_playwright() as p:
            try:
                br = p.chromium.launch()
            except Exception as exc:
                pytest.skip(f"chromium unavailable: {exc}")
            pg = br.new_page()
            pg.goto(f"http://127.0.0.1:{port}/tests/e2e/parity_harness.html")
            pg.wait_for_function("() => !!window.runCase")
            for rd, pose, cfg, expected in _fixtures():
                got = pg.evaluate("([r,po,c]) => window.runCase(r,po,c).path",
                                  [rd, pose, cfg])
                assert len(got) == len(expected), (cfg["strategy"], len(got), len(expected))
                for (gx, gy), (ex, ey) in zip(got, expected):
                    assert abs(gx - ex) < 1e-3 and abs(gy - ey) < 1e-3, \
                        (cfg["strategy"], gx, gy, ex, ey)
            br.close()
    finally:
        proc.terminate(); proc.wait()
```

- [ ] **Step 3: Run parity test to verify it fails**

Run: `PYTHONPATH=src .venv/bin/python -m pytest tests/e2e/test_parity_py_js.py -m e2e -q`
Expected: FAIL (project_route.js missing → `window.runCase` never defined / 404).

- [ ] **Step 4: Implement `viewer/project_route.js`**

Create `viewer/project_route.js` (mirror the Python exactly):

```javascript
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
```

- [ ] **Step 5: Run parity test to verify it passes**

Run: `PYTHONPATH=src .venv/bin/python -m pytest tests/e2e/test_parity_py_js.py -m e2e -q`
Expected: PASS. If any strategy mismatches, fix the JS to match the Python line-for-line (the parity fixture bakes `tangents`, so tangent computation is not a source of drift here).

- [ ] **Step 6: Commit**

```bash
git add viewer/project_route.js tests/e2e/parity_harness.html tests/e2e/test_parity_py_js.py
git commit -m "feat: project_route.js DOM-free twin + Python<->JS parity test"
```

---

### Task 5: Viewer — live compute + left-panel selector & sliders

**Files:**
- Modify: `viewer/index.html` (load `project_route.js`; replace the recenter checkbox with a selector + sliders)
- Modify: `viewer/viewer.js` (live path compute with a per-frame cursor memo; drive both driver views)

**Interfaces:**
- Consumes: `window.ProjectRoute` (Task 4); baked `case.route.points_e/points_n/s` and `case.frames[i].meas_pose`.
- Produces: DOM `#algo-select`, `#p-radius`, `#p-behind`, `#p-ahead`, `#p-corner`; JS `currentConfig()`, `computeBodyPath(caseObj, frameIdx)`.

- [ ] **Step 1: Add controls to `index.html`**

Replace the driver-view `<h3>` block (lines 22–24, the one with `recenter-toggle`) with:

```html
    <div class="fig"><h3>Driver view (+x up, +y left)
      <label>algo
        <select id="algo-select">
          <option value="raw">Raw (keep offset)</option>
          <option value="centered">Centered (no offset)</option>
          <option value="smoothed" selected>Smoothed (drivable corners)</option>
        </select></label>
      <label><input type="checkbox" id="persp-toggle"> perspective</label>
      <div id="algo-params">
        <label>R_min <input type="range" id="p-radius" min="3" max="12" step="0.5" value="5"><span id="p-radius-v">5.0</span> m</label>
        <label>behind <input type="range" id="p-behind" min="0" max="10" step="1" value="5"><span id="p-behind-v">5</span> m</label>
        <label>ahead <input type="range" id="p-ahead" min="20" max="100" step="5" value="70"><span id="p-ahead-v">70</span> m</label>
        <label>corner° <input type="range" id="p-corner" min="5" max="45" step="5" value="10"><span id="p-corner-v">10</span>°</label>
      </div></h3>
      <canvas id="driver" width="560" height="300"></canvas></div>
```

Add the module script tag in `<head>` (or before `viewer.js`): `<script src="project_route.js"></script>` — ensure it loads **before** `viewer.js`.

- [ ] **Step 2: Add live-compute helpers to `viewer.js`**

After `worldToBody` (line ~278), replace the old `bodyRoutePoints` function (lines ~284–298) with:

```javascript
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
function computeBodyPath(c, frameIdx) {
  ensureRouteJs(c);
  const f = c.frames[frameIdx];
  const cursor = cursorAt(c, frameIdx);
  const cfg = currentConfig();
  const state = { cursor_s: cursor, initialized: true };
  // re-run project_route from the known cursor: pass a state whose cursor equals
  // this frame's cursor and a pose on that spot so match() returns it unchanged.
  const out = ProjectRoute.projectRoute(ROUTE_JS, f.meas_pose, cfg, state);
  return out.path.map(([x, y]) => ({ x, y }));
}
```

- [ ] **Step 3: Point both driver views at the live path**

In `drawDriver` (top-down), replace `const pts = bodyRoutePoints(c, f, behind, ahead);` with:

```javascript
  const behindLive = parseFloat(document.getElementById("p-behind").value);
  const aheadLive = parseFloat(document.getElementById("p-ahead").value);
  const ppm = (h - 20) / (aheadLive + behindLive);
  const toX = (by) => w / 2 - by * ppm;
  const toY = (bx) => h - 10 - (bx + behindLive) * ppm;
  const pts = computeBodyPath(c, STATE.frame);
```

(Delete the earlier `const ahead = c.config.ahead, behind = c.config.behind;` and the `ppm/toX/toY` lines that used them — the live window governs the scale now.) In `drawWindshield`, replace `const pts = bodyRoutePoints(STATE.case, f, 0, XMAX);` with `const pts = computeBodyPath(STATE.case, STATE.frame).filter(p => p.x >= 0);` and set `const XMAX = parseFloat(document.getElementById("p-ahead").value);`.

- [ ] **Step 4: Wire the controls to re-render**

In the `DOMContentLoaded` handler (near line 525), remove the `recenter-toggle` line and add:

```javascript
  document.getElementById("algo-select").onchange = () => renderFrame();
  for (const id of ["p-radius", "p-behind", "p-ahead", "p-corner"]) {
    document.getElementById(id).oninput = (ev) => {
      const v = ev.target.value;
      document.getElementById(id + "-v").textContent = v;
      renderFrame();
    };
  }
```

Also, where a case is selected/loaded (the click handler that sets `STATE.case`), reset the memo: `ROUTE_JS_CASE = null;` so `ensureRouteJs` rebuilds for the new case.

- [ ] **Step 5: Smoke-check in the browser (headless)**

Run:
```bash
PYTHONPATH=src .venv/bin/python -m http.server 8011 &
sleep 1
PYTHONPATH=src .venv/bin/python - <<'PY'
from playwright.sync_api import sync_playwright
with sync_playwright() as p:
    b=p.chromium.launch(); pg=b.new_page(); errs=[]
    pg.on("pageerror", lambda e: errs.append(str(e)))
    pg.goto("http://127.0.0.1:8011/viewer/index.html")
    pg.wait_for_selector("#case-list li"); pg.click("#tab-sim")
    pg.locator("#case-list li:not(.group-header)").first.click()
    pg.wait_for_function("(s)=>{const c=document.querySelector(s);if(!c)return false;const d=c.getContext('2d').getImageData(0,0,c.width,c.height).data;for(let i=3;i<d.length;i+=4)if(d[i]>0)return true;return false;}", arg="#driver", timeout=8000)
    print("driver rendered, errors:", errs)
    b.close()
PY
kill %1
```
Expected: `driver rendered, errors: []`.

- [ ] **Step 6: Commit**

```bash
git add viewer/index.html viewer/viewer.js
git commit -m "feat(viewer): live projection with algorithm selector + parameter sliders"
```

---

### Task 6: Web-visible e2e tests

**Files:**
- Modify: `tests/e2e/test_viewer_e2e.py` (replace the recenter/follow_path tests with selector/slider/corner tests)

**Interfaces:**
- Consumes: the live viewer (Task 5), `window.ProjectRoute`.

- [ ] **Step 1: Replace the obsolete recenter tests**

In `tests/e2e/test_viewer_e2e.py`, delete `test_recenter_toggle_default_and_changes_view` and `test_recenter_centers_route_ahead` (they referenced `#recenter-toggle` / baked `follow_path`). Update `_select` if it referenced the checkbox (it does not). Add:

```python
def test_algo_selector_switches_driver_view(viewer):
    page, _ = viewer
    _select(page, "X-crossing (high)")
    page.select_option("#algo-select", "centered")
    page.wait_for_timeout(80)
    sig_c = page.evaluate(_SIGNATURE, "#driver")
    page.select_option("#algo-select", "raw")
    page.wait_for_timeout(80)
    sig_r = page.evaluate(_SIGNATURE, "#driver")
    assert sig_c != sig_r, "raw vs centered should differ when offset is present"
    page.select_option("#algo-select", "smoothed")


def test_smoothed_rounds_sharp_corner(viewer):
    page, _ = viewer
    _select(page, "Corner (low)")            # near-90 deg scenario (group C)
    # a frame with the corner in the look-ahead
    page.eval_on_selector("#scrubber",
        "el => { el.value = Math.floor(el.max*0.15); el.dispatchEvent(new Event('input')); }")
    def max_heading_step(strategy):
        page.select_option("#algo-select", strategy)
        page.wait_for_timeout(80)
        return page.evaluate("""() => {
          const c = STATE.case, f = STATE.frame;
          const p = computeBodyPath(c, f).filter(q => q.x >= 0);
          let m = 0;
          for (let i = 2; i < p.length; i++) {
            const a = Math.atan2(p[i-1].y-p[i-2].y, p[i-1].x-p[i-2].x);
            const b = Math.atan2(p[i].y-p[i-1].y, p[i].x-p[i-1].x);
            m = Math.max(m, Math.abs(((b-a+Math.PI)%(2*Math.PI))-Math.PI));
          }
          return m;
        }""")
    sharp = max_heading_step("centered")
    smooth = max_heading_step("smoothed")
    assert sharp > 0.5, sharp                # centered keeps the sharp corner
    assert smooth < sharp * 0.6, (sharp, smooth)   # smoothed clearly rounds it


def test_radius_slider_widens_arc(viewer):
    page, _ = viewer
    _select(page, "Corner (low)")
    page.eval_on_selector("#scrubber",
        "el => { el.value = Math.floor(el.max*0.15); el.dispatchEvent(new Event('input')); }")
    page.select_option("#algo-select", "smoothed")
    page.eval_on_selector("#p-radius", "el => { el.value = 4; el.dispatchEvent(new Event('input')); }")
    page.wait_for_timeout(80)
    sig4 = page.evaluate(_SIGNATURE, "#driver")
    page.eval_on_selector("#p-radius", "el => { el.value = 10; el.dispatchEvent(new Event('input')); }")
    page.wait_for_timeout(80)
    sig10 = page.evaluate(_SIGNATURE, "#driver")
    assert sig4 != sig10, "changing min turning radius must change the smoothed path"
```

- [ ] **Step 2: Run the full e2e suite (3× for order-independence)**

Run: `for i in 1 2 3; do PYTHONPATH=src .venv/bin/python -m pytest -m e2e -q 2>&1 | tail -2; done`
Expected: all pass each run (includes the parity test from Task 4 and the sim/real coverage tests). Fix any residual reference to removed controls.

- [ ] **Step 3: Commit**

```bash
git add tests/e2e/test_viewer_e2e.py
git commit -m "test(e2e): algorithm selector, corner-smoothing, and R_min slider (web-visible)"
```

---

### Task 7: Documentation

**Files:**
- Create: `docs/project_route_function.md`
- Modify: `spec_design.md`, `algorithm_description.md`, `README.md`, `CLAUDE.md`

**Interfaces:**
- Consumes: the shipped function, strategies, and viewer from Tasks 1–6.

- [ ] **Step 1: Write `docs/project_route_function.md`**

Create the function-description deliverable covering: the I/O contract (`project_route` signature, `ProjectConfig`/`ProjectState`/`ProjectOutput`, units and body-frame conventions, `route` representation, `state` as compact progress); the three strategies and how they compose; the arc-fillet math (RDP corner detection, `T = R·tan(δ/2)`, T-clamp → `R_eff ≥ R_min`, arc construction) and the curvature ≤ 1/R_min feasibility guarantee; the "revert to straight after the turn" per-frame behavior; the performance characteristics (windowed search, closed-form arcs, no convergence loops) for the embedded target; and the clothoid-ready `corner_curve` extension point. Every symbol must match the code in `project_route.py`/`smoothing.py`.

- [ ] **Step 2: Update `spec_design.md` and `algorithm_description.md`**

In both, replace the follow-path/offset-removal sections with the new model: one function, three strategies, the fillet math, and the new invariant that the viewer runs the shared algorithm live (Python authoritative + parity test). Remove references to baked `follow_path`/`lat_shift`.

- [ ] **Step 3: Update `README.md`**

Update the "How it works" and "Real data" sections and the test counts: describe `project_route` + the three strategies + the smoothed corner behavior; document the viewer's algorithm selector and parameter sliders and that all paths are computed live in the browser (JSON carries raw inputs only).

- [ ] **Step 4: Update `CLAUDE.md`**

Change the architecture invariant: the viewer no longer only replays prebaked JSON — it runs the shared projection algorithm (`viewer/project_route.js`) live; Python (`project_route.py`) is authoritative and a `tests/e2e/test_parity_py_js.py` parity test binds the two. Keep the `geo.py` single-place-for-datum-conversion and seeded-determinism invariants.

- [ ] **Step 5: Commit**

```bash
git add docs/project_route_function.md spec_design.md algorithm_description.md README.md CLAUDE.md
git commit -m "docs: document project_route function, strategies, fillet, and live-viewer invariant"
```

---

## Self-Review

**Spec coverage:**
- One portable function with clear I/O → Task 2 (`project_route`, dataclasses).
- Three switchable strategies, all params configurable → Task 2 (`ProjectConfig`), exposed live in Task 5.
- Arc-fillet corner smoothing (fast, drivable) → Task 1 + Task 2 (`smoothed`).
- Live browser compute, no precompute, algorithm selector + sliders → Tasks 4 + 5; JSON slimmed → Task 3.
- Python↔JS parity → Task 4.
- Grading/acceptance unchanged → Task 3 Step 5.
- Function description doc + invariant updates → Task 7.
- Web-visible e2e → Task 6.
All spec sections map to a task.

**Placeholder scan:** No TBD/TODO; every code step carries complete code; Task 7 doc steps enumerate exact required content (prose deliverable, not code).

**Type consistency:** `project_route(route, pose_e, pose_n, yaw, config, state, speed)` (Python) ↔ `projectRoute(route, pose{e,n,h}, cfg, state)` (JS) — the parity test bridges the two calling conventions via the harness. `ProjectConfig` field names match the JS `DEFAULT_CONFIG` keys exactly. `_match`/`match`, `smooth_corners`/`smoothCorners`, `rdp`, `resample` mirror across languages. `Projector` keeps `.step()/.reset()/.w_search/.ahead/.behind` and returns `ProjectionResult` so `test_search_window.py` and the acceptance suite are unaffected.
