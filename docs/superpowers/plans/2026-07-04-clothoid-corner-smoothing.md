# Clothoid Corner Smoothing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the `smoothed` strategy's circular-arc corner with a curvature-continuous clothoid whose transition length is calibrated from the human ego tracks, so the turn eases in like a real driver instead of snapping.

**Architecture:** A new `clothoid.py` builds a symmetric clothoid corner by integrating its linear-curvature profile (embeddable, no Fresnel table). `smoothing.smooth_corners` dispatches per corner to arc or clothoid via a `corner_style` param, falling back to arc when the clothoid can't fit. An offline `tools/calibrate_clothoid.py` derives the transition-length default from the datasets. The JS twin mirrors the clothoid (parity-tested), and the viewer exposes a corner-style selector + transition slider.

**Tech Stack:** Python 3.12 + numpy; static HTML/Canvas JS; pytest + Playwright.

## Global Constraints

- Clothoid = linear curvature ramp `0→1/R→0`; curvature is **continuous** (no jump), peak `κ = 1/min_turn_radius_m` (drivable). Computed by integrating the curvature profile at a fixed internal step (`INTERNAL_DS = 0.1 m`) — **identical loop in Python and JS** so parity holds.
- New config: `corner_style: "clothoid"` (default) | `"arc"`; `clothoid_transition_m` (calibrated default). `min_turn_radius_m`, `corner_angle_deg`, and `raw`/`centered` behavior unchanged.
- `smooth_corners` keeps `corner_style="arc"` as its own default so existing positional callers/tests are unchanged; `project_route` passes `corner_style="clothoid"` from config.
- Short-leg handling: try the clothoid at the transition, then at 0.5× and 0.25×; if it still needs a tangent length > ½ the shorter adjacent leg, **fall back to the arc fillet** for that corner (always fits, stays ≤ 1/R).
- Python authoritative; the Python↔JS parity test must pass (1e-3 m) including `corner_style="clothoid"` fixtures.
- Determinism preserved (no RNG). Calibration is offline; runtime has no data dependency.
- git hygiene: stage only named files with explicit `git add <paths>`; never `git add -A`/`.` (untracked `dataset/` must never be committed).

---

### Task 1: `clothoid.py` — curvature-continuous corner geometry

**Files:**
- Create: `src/parking_proj/clothoid.py`
- Test: `tests/test_clothoid.py`

**Interfaces:**
- Produces: `INTERNAL_DS = 0.1`; `clothoid_corner(delta, radius, transition, internal_ds=INTERNAL_DS) -> (pts, T)` — `pts` a fine canonical polyline from `(0,0)` heading `+x` to heading `+delta` (left turn), `T` the tangent length (start point to the incoming/outgoing tangent intersection).

- [ ] **Step 1: Write the failing tests**

Create `tests/test_clothoid.py`:

```python
import math
from parking_proj.clothoid import clothoid_corner


def _turn_rates(pts):
    # per-consecutive-segment heading change (proxy for curvature * ds)
    rates = []
    for i in range(1, len(pts)):
        dx, dy = pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1]
        rates.append(math.atan2(dy, dx))
    return rates


def test_turns_by_delta():
    delta = math.radians(90)
    pts, T = clothoid_corner(delta, radius=5.0, transition=3.0)
    # final heading (last segment bearing) ~ delta
    dx, dy = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
    assert math.atan2(dy, dx) == pytest_approx(delta, abs=0.03)
    assert T > 0


def test_curvature_is_continuous_no_jump():
    delta = math.radians(90)
    pts, _ = clothoid_corner(delta, radius=5.0, transition=3.0, internal_ds=0.1)
    rates = _turn_rates(pts)
    # consecutive change in bearing-rate (2nd difference of heading) has no big jump
    jumps = [abs(rates[i] - rates[i-1]) for i in range(1, len(rates))]
    # each internal step turns <= ds/R = 0.1/5 = 0.02 rad; step-to-step change is tiny
    assert max(jumps) < 0.01


def test_peak_curvature_within_min_radius():
    delta = math.radians(90)
    ds = 0.1
    pts, _ = clothoid_corner(delta, radius=5.0, transition=3.0, internal_ds=ds)
    rates = _turn_rates(pts)
    per_step = [abs((rates[i] - rates[i-1] + math.pi) % (2*math.pi) - math.pi) for i in range(1, len(rates))]
    # peak turn-rate per step <= ds/R + tolerance  => curvature <= 1/R
    assert max(per_step) <= ds / 5.0 + 0.005


def test_degenerate_inputs_return_trivial():
    assert clothoid_corner(0.0, 5.0, 3.0)[0] == [(0.0, 0.0)]
    assert clothoid_corner(math.radians(90), 5.0, 0.0)[0] == [(0.0, 0.0)]


def pytest_approx(*a, **k):
    import pytest
    return pytest.approx(*a, **k)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PYTHONPATH=src .venv/bin/python -m pytest tests/test_clothoid.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'parking_proj.clothoid'`.

- [ ] **Step 3: Implement `clothoid.py`**

Create `src/parking_proj/clothoid.py`:

```python
"""Clothoid (Euler spiral) corner: curvature ramps linearly, so the curve is
curvature-continuous (no jump at entry) — a smoother, more human-like turn than
a circular arc, while peak curvature is still 1/radius (drivable).

Computed by integrating the linear-curvature profile at a fixed internal step
(equivalent to Fresnel integrals but simpler and bit-identical across the
Python and JavaScript ports — no lookup table, no convergence loop).
"""
import math

INTERNAL_DS = 0.1


def clothoid_corner(delta, radius, transition, internal_ds=INTERNAL_DS):
    """Canonical symmetric clothoid corner turning LEFT by `delta` (radians, > 0).

    Curvature profile over arc length: 0 -> 1/radius (spiral in) over
    `transition`, an optional constant-1/radius arc, then 1/radius -> 0 (spiral
    out). If the two spirals alone already turn >= delta, they are shortened so
    they meet with no middle arc (peak curvature still 1/radius).

    Returns (pts, T): `pts` is the fine polyline from (0,0) heading +x to heading
    +delta; `T` is the tangent length (distance from the start point to where the
    incoming (+x axis) and outgoing tangent lines intersect).
    """
    if (delta <= 1e-9 or transition <= 1e-9 or radius <= 1e-9
            or abs(math.sin(delta)) < 1e-9):
        return [(0.0, 0.0)], 0.0
    theta_sp = transition / (2.0 * radius)          # heading each full spiral turns
    if 2.0 * theta_sp <= delta:
        lt = transition
        arc_len = radius * (delta - 2.0 * theta_sp)
    else:
        lt = radius * delta                         # each spiral turns delta/2, peak kappa = 1/R
        arc_len = 0.0
    total = 2.0 * lt + arc_len
    inv_r = 1.0 / radius

    def kappa(s):
        if s < lt:
            return (s / lt) * inv_r                  # ramp up
        if s <= lt + arc_len:
            return inv_r                              # constant arc
        return ((total - s) / lt) * inv_r            # ramp down

    n = max(2, int(math.ceil(total / internal_ds)))
    h = total / n
    x = y = theta = 0.0
    pts = [(0.0, 0.0)]
    s = 0.0
    for _ in range(n):
        k0 = kappa(s)
        k1 = kappa(s + h)
        theta_mid = theta + 0.5 * k0 * h             # midpoint heading for position
        x += math.cos(theta_mid) * h
        y += math.sin(theta_mid) * h
        theta += 0.5 * (k0 + k1) * h                 # trapezoid heading update
        s += h
        pts.append((x, y))
    xe, ye = pts[-1]
    T = xe - ye / math.tan(delta)
    return pts, T
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `PYTHONPATH=src .venv/bin/python -m pytest tests/test_clothoid.py -q`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/clothoid.py tests/test_clothoid.py
git commit -m "feat: clothoid.py — curvature-continuous corner via profile integration"
```

---

### Task 2: Offline calibration script

**Files:**
- Create: `tools/calibrate_clothoid.py`
- Create: `docs/clothoid_calibration.md` (written by the script)

**Interfaces:**
- Consumes: `parking_proj.realdata.load_dataset`, `is_dataset_dir`.
- Produces: prints `CLOTHOID_TRANSITION_M=<value>` (median human entry-ramp, clamped to [1,6] m) and writes the report. This value becomes the config default in Task 3.

- [ ] **Step 1: Implement the calibration script**

Create `tools/calibrate_clothoid.py`:

```python
"""Offline: estimate the clothoid transition length from human ego tracks.

Speed-gates out stationary RTK jitter, resamples to 1 m, segments genuine turns,
and measures each turn's curvature entry-ramp (10%->90% of peak). The median is
the calibrated transition length. Run: PYTHONPATH=src python tools/calibrate_clothoid.py
"""
import glob
import math
import os
import numpy as np
from parking_proj.realdata import load_dataset, is_dataset_dir

SPEED_MIN = 0.5      # m/s — drop stationary frames (jitter)
DS = 1.0             # resample spacing
KAPPA_TURN = 1.0 / 15.0


def _entry_ramps(ds):
    e = np.asarray(ds.meas_e); n = np.asarray(ds.meas_n); v = np.asarray(ds.speed)
    keep = v >= SPEED_MIN
    e, n = e[keep], n[keep]
    if len(e) < 10:
        return []
    seg = np.hypot(np.diff(e), np.diff(n)); s = np.concatenate([[0], np.cumsum(seg)])
    if s[-1] < DS * 5:
        return []
    su = np.arange(0, s[-1], DS); eu = np.interp(su, s, e); nu = np.interp(su, s, n)
    psi = np.arctan2(np.diff(nu), np.diff(eu))
    kap = np.abs((np.diff(psi) + np.pi) % (2 * np.pi) - np.pi) / DS
    turn = kap > KAPPA_TURN
    ramps, i = [], 0
    while i < len(turn):
        if turn[i]:
            j = i
            while j < len(turn) and turn[j]:
                j += 1
            pk = kap[i:j].max()
            peak = i + int(np.argmax(kap[i:j]))
            lo = i
            while lo < peak and kap[lo] < 0.1 * pk:
                lo += 1
            hi = lo
            while hi < peak and kap[hi] < 0.9 * pk:
                hi += 1
            ramps.append((hi - lo) * DS)
            i = j
        else:
            i += 1
    return ramps


def main():
    per, allr = [], []
    for d in sorted(glob.glob("dataset/*")):
        if not is_dataset_dir(d):
            continue
        r = _entry_ramps(load_dataset(d))
        allr += r
        per.append((os.path.basename(d), len(r), float(np.median(r)) if r else float("nan")))
    value = float(np.median(allr)) if allr else 3.0
    value = round(min(6.0, max(1.0, value)), 1)
    lines = ["# Clothoid transition-length calibration", "",
             f"Speed gate: >= {SPEED_MIN} m/s; resample {DS} m; turn threshold kappa > {KAPPA_TURN:.3f} (R<15 m).", "",
             "| dataset | turns | median entry ramp (m) |", "|---|---|---|"]
    for name, k, med in per:
        lines.append(f"| {name[:32]} | {k} | {med:.1f} |")
    lines += ["", f"**Calibrated `clothoid_transition_m` = {value} m** "
              f"(median entry ramp across all turns, clamped to [1, 6])."]
    with open("docs/clothoid_calibration.md", "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"CLOTHOID_TRANSITION_M={value}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the calibration script**

Run: `PYTHONPATH=src .venv/bin/python tools/calibrate_clothoid.py`
Expected: prints `CLOTHOID_TRANSITION_M=<value>` (a number in [1,6], expected ~2–4) and writes `docs/clothoid_calibration.md`. **Record the printed value — Task 3 uses it as the config default.**

- [ ] **Step 3: Commit**

```bash
git add tools/calibrate_clothoid.py docs/clothoid_calibration.md
git commit -m "feat: offline clothoid transition-length calibration from ego tracks"
```

---

### Task 3: Dispatch arc|clothoid in `smoothing.py`; wire config in `project_route.py`

**Files:**
- Modify: `src/parking_proj/smoothing.py` (extract arc helper; add clothoid helper; `smooth_corners` gains `corner_style` + `transition`)
- Modify: `src/parking_proj/project_route.py` (`ProjectConfig` gains `corner_style`, `clothoid_transition_m`; `smoothed` passes them)
- Test: `tests/test_smoothing.py` (add clothoid dispatch test), `tests/test_project_route.py` (add corner_style test)

**Interfaces:**
- Consumes: `clothoid.clothoid_corner`.
- Produces: `smooth_corners(pts, min_radius, corner_angle_deg, ds, eps, corner_style="arc", transition=3.0)`; `ProjectConfig.corner_style="clothoid"`, `ProjectConfig.clothoid_transition_m=<calibrated>`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_smoothing.py`:

```python
def test_clothoid_is_smoother_than_arc():
    import math
    from parking_proj.smoothing import smooth_corners
    pts = _l_shape()                       # existing helper: 90-deg L
    arc = smooth_corners(pts, 5.0, 10.0, 0.5, 0.2, corner_style="arc")
    clo = smooth_corners(pts, 5.0, 10.0, 0.5, 0.2, corner_style="clothoid", transition=3.0)

    def max_rate_jump(p):
        rates = [math.atan2(p[i][1]-p[i-1][1], p[i][0]-p[i-1][0]) for i in range(1, len(p))]
        return max(abs((rates[i]-rates[i-1]+math.pi) % (2*math.pi) - math.pi)
                   for i in range(1, len(rates)))
    # the clothoid has no single curvature jump; the arc snaps at entry
    assert max_rate_jump(clo) < max_rate_jump(arc)
```

Append to `tests/test_project_route.py`:

```python
def test_corner_style_default_is_clothoid_and_arc_selectable():
    from parking_proj.project_route import ProjectConfig
    assert ProjectConfig().corner_style == "clothoid"
    r = l_route()
    import math
    def max_rate_jump(path):
        rates = [math.atan2(path[i][1]-path[i-1][1], path[i][0]-path[i-1][0]) for i in range(1, len(path))]
        return max(abs((rates[i]-rates[i-1]+math.pi) % (2*math.pi) - math.pi)
                   for i in range(1, len(rates)))
    clo = project_route(r, 0.0, 0.0, 0.0, ProjectConfig(strategy="smoothed", corner_style="clothoid")).path
    arc = project_route(r, 0.0, 0.0, 0.0, ProjectConfig(strategy="smoothed", corner_style="arc")).path
    assert max_rate_jump(clo) < max_rate_jump(arc)
```

- [ ] **Step 2: Run to verify they fail**

Run: `PYTHONPATH=src .venv/bin/python -m pytest tests/test_smoothing.py::test_clothoid_is_smoother_than_arc tests/test_project_route.py::test_corner_style_default_is_clothoid_and_arc_selectable -q`
Expected: FAIL (`smooth_corners() got an unexpected keyword argument 'corner_style'` / `ProjectConfig` has no `corner_style`).

- [ ] **Step 3: Refactor `smoothing.py` to dispatch**

In `src/parking_proj/smoothing.py`, add the import at the top (after `import math`):

```python
from .clothoid import clothoid_corner
```

Replace the `smooth_corners` function (lines 64-116) with the dispatching version + two helpers:

```python
def _arc_world(ax, ay, vx, vy, bx, by, min_radius, delta, cross, ds):
    d1x, d1y = _unit(vx - ax, vy - ay)
    d2x, d2y = _unit(bx - vx, by - vy)
    tan_half = math.tan(delta / 2.0)
    if tan_half < 1e-9:
        return [(vx, vy)]
    T = min(min_radius * tan_half, 0.5 * math.hypot(vx - ax, vy - ay),
            0.5 * math.hypot(bx - vx, by - vy))
    if T < 1e-6:
        return [(vx, vy)]
    r_eff = T / tan_half
    p1x, p1y = vx - T * d1x, vy - T * d1y
    nx, ny = (-d1y, d1x) if cross >= 0 else (d1y, -d1x)
    cx, cy = p1x + r_eff * nx, p1y + r_eff * ny
    a1 = math.atan2(p1y - cy, p1x - cx)
    sign = 1.0 if cross >= 0 else -1.0
    steps = max(1, int(math.ceil(r_eff * delta / ds)))
    out = [(p1x, p1y)]
    for k in range(1, steps + 1):
        a = a1 + sign * delta * (k / steps)
        out.append((cx + r_eff * math.cos(a), cy + r_eff * math.sin(a)))
    return out


def _clothoid_world(ax, ay, vx, vy, bx, by, min_radius, transition, delta, cross):
    d1x, d1y = _unit(vx - ax, vy - ay)
    clamp = 0.5 * min(math.hypot(vx - ax, vy - ay), math.hypot(bx - vx, by - vy))
    for factor in (1.0, 0.5, 0.25):
        local, T = clothoid_corner(delta, min_radius, transition * factor)
        if 0.0 < T <= clamp:
            p1x, p1y = vx - T * d1x, vy - T * d1y
            nx, ny = (-d1y, d1x) if cross >= 0 else (d1y, -d1x)   # +y (left) -> turn side
            return [(p1x + lx * d1x + ly * nx, p1y + lx * d1y + ly * ny) for lx, ly in local]
    return None                                                    # doesn't fit -> caller uses arc


def smooth_corners(pts, min_radius, corner_angle_deg, ds, eps,
                   corner_style="arc", transition=3.0):
    """Replace sharp corners with fillets. corner_style="arc" uses a circular arc
    (curvature bounded by 1/min_radius, but jumps at entry); "clothoid" uses a
    curvature-continuous clothoid of the given transition length, falling back to
    the arc when the clothoid cannot fit the adjacent legs. Output is resampled at
    ds. Only corners sharper than corner_angle_deg are filleted; eps is the RDP
    corner-detection tolerance."""
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
        delta = math.acos(dot)
        if delta < thresh:
            out.append((vx, vy))
            continue
        cross = d1x * d2y - d1y * d2x
        corner = None
        if corner_style == "clothoid":
            corner = _clothoid_world(ax, ay, vx, vy, bx, by, min_radius, transition, delta, cross)
        if corner is None:
            corner = _arc_world(ax, ay, vx, vy, bx, by, min_radius, delta, cross, ds)
        out.extend(corner)
    out.append(verts[-1])
    return resample(out, ds)
```

- [ ] **Step 4: Add config fields in `project_route.py`**

In `ProjectConfig`, add two fields (after `simplify_eps_m`):

```python
    corner_style: str = "clothoid"       # "clothoid" | "arc"
    clothoid_transition_m: float = 3.0   # calibrated default (see docs/clothoid_calibration.md)
```

Set `clothoid_transition_m`'s default to the value `tools/calibrate_clothoid.py` printed in Task 2 (if that value is unavailable, keep 3.0). In `project_route`, change the smoothed call to pass the style + transition:

```python
    if cfg.strategy == "smoothed" and len(fwd_pts) >= 3:
        fwd_pts = smooth_corners(fwd_pts, cfg.min_turn_radius_m, cfg.corner_angle_deg,
                                 cfg.sample_ds_m, cfg.simplify_eps_m,
                                 corner_style=cfg.corner_style,
                                 transition=cfg.clothoid_transition_m)
```

- [ ] **Step 5: Run the tests + full suite**

Run: `PYTHONPATH=src .venv/bin/python -m pytest -q`
Expected: PASS — the two new tests pass; existing `test_smoothing.py` (arc default unchanged), `test_project_route.py`, `test_clothoid.py`, `test_search_window.py`, and the 14 acceptance cases all still pass.

- [ ] **Step 6: Commit**

```bash
git add src/parking_proj/smoothing.py src/parking_proj/project_route.py tests/test_smoothing.py tests/test_project_route.py
git commit -m "feat: corner_style arc|clothoid dispatch; clothoid default for smoothed"
```

---

### Task 4: JS clothoid twin + parity

**Files:**
- Modify: `viewer/project_route.js` (add `clothoidCorner`; `smoothCorners` dispatches; `DEFAULT_CONFIG` gains `corner_style`, `clothoid_transition_m`)
- Modify: `tests/e2e/test_parity_py_js.py` (add `corner_style` to fixtures)

**Interfaces:**
- Consumes: Python `project_route` fixtures with `corner_style`.
- Produces: `window.ProjectRoute.clothoidCorner(delta, radius, transition, internalDs)`; `smoothCorners(pts, R, angleDeg, ds, eps, cornerStyle, transition)`.

- [ ] **Step 1: Extend parity fixtures (failing)**

In `tests/e2e/test_parity_py_js.py` `_fixtures()`, iterate `corner_style` for the smoothed strategy. Change the strategy/config loop so that for `strat == "smoothed"` it emits BOTH `corner_style="arc"` and `corner_style="clothoid"` configs (other strategies keep the default). Concretely, where it builds `cfg = ProjectConfig(strategy=strat)`, replace with:

```python
                styles = ("arc", "clothoid") if strat == "smoothed" else ("clothoid",)
                for style in styles:
                    cfg = ProjectConfig(strategy=strat, corner_style=style)
                    out = project_route(r, pe, pn, yaw, cfg)
                    cases.append((rd, {"e": pe, "n": pn, "h": yaw}, asdict(cfg), out.path))
```

Ensure the harness passes `cfg` (which now includes `corner_style`/`clothoid_transition_m`) straight through to JS `projectRoute` (it already forwards the whole cfg object).

- [ ] **Step 2: Run parity to verify it fails**

Run: `PYTHONPATH=src .venv/bin/python -m pytest tests/e2e/test_parity_py_js.py -m e2e -q`
Expected: FAIL — JS ignores `corner_style` (no clothoid), so clothoid fixtures diverge from Python.

- [ ] **Step 3: Implement the clothoid in `viewer/project_route.js`**

Add near the other helpers (before `smoothCorners`):

```javascript
  const INTERNAL_DS = 0.1;
  function clothoidCorner(delta, radius, transition, internalDs) {
    internalDs = internalDs || INTERNAL_DS;
    if (delta <= 1e-9 || transition <= 1e-9 || radius <= 1e-9 || Math.abs(Math.sin(delta)) < 1e-9)
      return { pts: [[0, 0]], T: 0 };
    const thetaSp = transition / (2 * radius);
    let lt, arcLen;
    if (2 * thetaSp <= delta) { lt = transition; arcLen = radius * (delta - 2 * thetaSp); }
    else { lt = radius * delta; arcLen = 0; }
    const total = 2 * lt + arcLen, invR = 1 / radius;
    const kappa = (s) => s < lt ? (s / lt) * invR
                        : s <= lt + arcLen ? invR
                        : ((total - s) / lt) * invR;
    const n = Math.max(2, Math.ceil(total / internalDs)), h = total / n;
    let x = 0, y = 0, theta = 0, s = 0;
    const pts = [[0, 0]];
    for (let i = 0; i < n; i++) {
      const k0 = kappa(s), k1 = kappa(s + h);
      const thetaMid = theta + 0.5 * k0 * h;
      x += Math.cos(thetaMid) * h; y += Math.sin(thetaMid) * h;
      theta += 0.5 * (k0 + k1) * h; s += h;
      pts.push([x, y]);
    }
    const xe = pts[pts.length - 1][0], ye = pts[pts.length - 1][1];
    return { pts, T: xe - ye / Math.tan(delta) };
  }
```

Replace `smoothCorners` so it takes `(pts, R, angleDeg, ds, eps, cornerStyle, transition)` and, per corner, tries the clothoid first when `cornerStyle==="clothoid"`, else the arc. Mirror the Python dispatch exactly — factor the existing arc body into an `arcWorld(...)` inner and add a `clothoidWorld(...)` inner (fit factors `[1,0.5,0.25]`, clamp to ½ shorter leg, same placement `p1 + lx*d1 + ly*(±normal)`), returning `null` to fall back to arc. Keep the final `resample(out, ds)`.

Add to `DEFAULT_CONFIG`: `corner_style: "clothoid", clothoid_transition_m: 3.0` (use the calibrated value from Task 2 to match the Python default). Export `clothoidCorner` on `window.ProjectRoute`. In `projectRoute`, pass `cfg.corner_style` and `cfg.clothoid_transition_m` into `smoothCorners`.

- [ ] **Step 4: Run parity to verify it passes**

Run: `PYTHONPATH=src .venv/bin/python -m pytest tests/e2e/test_parity_py_js.py -m e2e -q`
Expected: PASS. Fix the JS until every clothoid fixture agrees with Python within 1e-3 m (the integration loop, dispatch, fit factors, and placement must match line-for-line).

- [ ] **Step 5: Commit**

```bash
git add viewer/project_route.js tests/e2e/test_parity_py_js.py
git commit -m "feat(project_route.js): clothoid twin + corner_style parity"
```

---

### Task 5: Viewer — corner-style selector + transition slider

**Files:**
- Modify: `viewer/index.html`, `viewer/viewer.js`

**Interfaces:**
- Consumes: `currentConfig()` (extends it), `#algo-params`.
- Produces: DOM `#corner-style`, `#p-transition` (+ readout `#p-transition-v`).

- [ ] **Step 1: Add controls to `index.html`**

Inside the `<div id="algo-params">` block, add (after the corner° slider label):

```html
        <label>corner
          <select id="corner-style">
            <option value="clothoid" selected>clothoid</option>
            <option value="arc">arc</option>
          </select></label>
        <label>transition <input type="range" id="p-transition" min="0.5" max="8" step="0.5" value="3"><span id="p-transition-v">3.0</span> m</label>
```

- [ ] **Step 2: Feed the controls into `currentConfig()`**

In `viewer.js` `currentConfig()`, add to the returned override object:

```javascript
    corner_style: document.getElementById("corner-style").value,
    clothoid_transition_m: parseFloat(document.getElementById("p-transition").value),
```

- [ ] **Step 3: Wire re-render**

In the `DOMContentLoaded` handler, add `#corner-style` to the change listeners and `#p-transition` to the slider loop:

```javascript
  document.getElementById("corner-style").onchange = () => renderFrame();
```
and include `"p-transition"` in the existing `for (const id of [...])` slider array (so its readout updates and it re-renders).

- [ ] **Step 4: Smoke check (headless)**

Run the same headless smoke as before (serve on a port, load a sim case, assert `#driver` non-blank + 0 JS errors), additionally switching `#corner-style` to `arc` and back:

```bash
PYTHONPATH=src .venv/bin/python -m http.server 8012 &
sleep 1
PYTHONPATH=src .venv/bin/python - <<'PY'
from playwright.sync_api import sync_playwright
with sync_playwright() as p:
    b=p.chromium.launch(); pg=b.new_page(); errs=[]
    pg.on("pageerror", lambda e: errs.append(str(e)))
    pg.goto("http://127.0.0.1:8012/viewer/index.html")
    pg.wait_for_selector("#case-list li"); pg.click("#tab-sim")
    pg.locator("#case-list li:not(.group-header)").first.click()
    pg.wait_for_function("(s)=>{const c=document.querySelector(s);if(!c)return false;const d=c.getContext('2d').getImageData(0,0,c.width,c.height).data;for(let i=3;i<d.length;i+=4)if(d[i]>0)return true;return false;}", arg="#driver", timeout=8000)
    pg.select_option("#corner-style","arc"); pg.wait_for_timeout(80)
    pg.select_option("#corner-style","clothoid"); pg.wait_for_timeout(80)
    print("driver rendered, errors:", errs); b.close()
PY
kill %1
```
Expected: `driver rendered, errors: []`.

- [ ] **Step 5: Commit**

```bash
git add viewer/index.html viewer/viewer.js
git commit -m "feat(viewer): corner-style selector + clothoid transition slider"
```

---

### Task 6: Web-visible e2e + docs

**Files:**
- Modify: `tests/e2e/test_viewer_e2e.py`
- Modify: `docs/project_route_function.md`, `algorithm_description.md`, `README.md`

**Interfaces:**
- Consumes: the live viewer (`computeBodyPath`, `#corner-style`, `#p-transition`).

- [ ] **Step 1: Add the e2e test**

Add to `tests/e2e/test_viewer_e2e.py`:

```python
def test_clothoid_smoother_than_arc(viewer):
    page, _ = viewer
    _select(page, "Near-90 corner (low)")
    page.eval_on_selector("#scrubber",
        "el => { el.value = Math.floor(el.max*0.15); el.dispatchEvent(new Event('input')); }")

    def max_rate_jump(style):
        page.select_option("#corner-style", style)
        page.wait_for_timeout(80)
        return page.evaluate("""() => {
          const p = computeBodyPath(STATE.case, STATE.frame).filter(q => q.x >= 0);
          const r = [];
          for (let i = 1; i < p.length; i++) r.push(Math.atan2(p[i].y-p[i-1].y, p[i].x-p[i-1].x));
          let m = 0;
          for (let i = 1; i < r.length; i++)
            m = Math.max(m, Math.abs(((r[i]-r[i-1]+Math.PI)%(2*Math.PI))-Math.PI));
          return m;
        }""")

    page.select_option("#algo-select", "smoothed")
    arc = max_rate_jump("arc")
    clo = max_rate_jump("clothoid")
    assert clo < arc, (clo, arc)          # clothoid has no entry snap


def test_transition_slider_changes_path(viewer):
    page, _ = viewer
    _select(page, "Near-90 corner (low)")
    page.eval_on_selector("#scrubber",
        "el => { el.value = Math.floor(el.max*0.15); el.dispatchEvent(new Event('input')); }")
    page.select_option("#algo-select", "smoothed")
    page.select_option("#corner-style", "clothoid")
    page.eval_on_selector("#p-transition", "el => { el.value = 1; el.dispatchEvent(new Event('input')); }")
    page.wait_for_timeout(80)
    s1 = page.evaluate(_SIGNATURE, "#driver")
    page.eval_on_selector("#p-transition", "el => { el.value = 6; el.dispatchEvent(new Event('input')); }")
    page.wait_for_timeout(80)
    s6 = page.evaluate(_SIGNATURE, "#driver")
    assert s1 != s6, "transition length must change the clothoid path"
```

- [ ] **Step 2: Run the full e2e suite (3× for order-independence)**

Run: `for i in 1 2 3; do PYTHONPATH=src .venv/bin/python -m pytest -m e2e -q 2>&1 | tail -2; done`
Expected: all pass each run (parity incl. clothoid + the two new tests + prior coverage).

- [ ] **Step 3: Update docs**

- `docs/project_route_function.md`: add a subsection on `corner_style` (arc vs clothoid), the clothoid's linear-curvature profile, curvature-continuity, the calibrated `clothoid_transition_m` (reference `docs/clothoid_calibration.md`), and the arc fallback for tight corners.
- `algorithm_description.md`: add the clothoid corner math (curvature profile, `θ_sp = Lt/(2R)`, spiral+arc+spiral, profile integration, T tangent length) alongside the arc.
- `README.md`: note the clothoid default for `smoothed`, the corner-style selector + transition slider, and that the transition length was calibrated from the human ego tracks.

- [ ] **Step 4: Commit**

```bash
git add tests/e2e/test_viewer_e2e.py docs/project_route_function.md algorithm_description.md README.md
git commit -m "test(e2e)+docs: clothoid smoother than arc; document data-calibrated clothoid"
```

---

## Self-Review

**Spec coverage:**
- Offline calibration (speed-gate, resample, entry ramp, median) → Task 2.
- Clothoid corner (linear curvature, spiral+arc+spiral, continuity, peak ≤ 1/R_min) → Task 1.
- `corner_style` config + arc fallback for short legs → Task 3.
- Python↔JS parity incl. clothoid → Task 4.
- Viewer corner-style selector + transition slider → Task 5.
- Web-visible "clothoid smoother than arc" + docs → Task 6.
All spec sections map to a task.

**Placeholder scan:** No TBD/TODO; every code step carries complete code; the one calibrated constant is produced by Task 2 and consumed by Tasks 3–4 with an explicit fallback (3.0).

**Type consistency:** Python `clothoid_corner(delta, radius, transition, internal_ds)` ↔ JS `clothoidCorner(delta, radius, transition, internalDs)`; both return the fine polyline + `T`. `smooth_corners(..., corner_style, transition)` ↔ JS `smoothCorners(..., cornerStyle, transition)`. `ProjectConfig.corner_style`/`clothoid_transition_m` ↔ JS `DEFAULT_CONFIG.corner_style`/`clothoid_transition_m`. The arc path is byte-preserved (extracted verbatim into `_arc_world`), so existing arc tests and the 14 acceptance verdicts are unaffected.
