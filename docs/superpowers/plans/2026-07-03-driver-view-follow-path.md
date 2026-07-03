# Driver-View Follow-Path Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a lateral-offset-removed body-frame follow-path as a first-class algorithm output baked into the prebaked JSON, and render it in the driver view behind a default-on toggle.

**Architecture:** The Python core (`projection.py`) gains a pure `follow_path()` function; the two generators (`generate.py`, `generate_real.py`) emit its result per frame. The static viewer draws the emitted path directly when the toggle is on, or the raw route slice when off — no matching logic moves into the viewer.

**Tech Stack:** Python 3.12 + numpy (core), static HTML/Canvas JS (viewer), pytest + Playwright (tests).

## Global Constraints

- Body frame is `+x` forward, `+y` left (matches `transform.to_body_frame`).
- `follow_path` window is **forward-only** `[cursor_s, cursor_s + 70 m]`, **truncated at route end**, sampled at **0.5 m**.
- The re-centering is a rigid lateral shift in the **car-heading** frame: subtract the car-frame lateral of the anchor (`lat_shift`) from every point; forward coords unchanged.
- The viewer toggle "remove lateral offset" defaults to **checked** (on).
- Grading is unaffected — all 14 simulation cases must keep identical verdicts.
- Regeneration stays deterministic/bit-identical (no randomness added).
- CLAUDE.md mandate: update `spec_design.md` for any change.

---

### Task 1: Core `follow_path()` function

**Files:**
- Modify: `src/parking_proj/projection.py` (add import at top; add constants + function at end)
- Test: `tests/test_follow_path.py` (create)

**Interfaces:**
- Consumes: `parking_proj.transform.to_body_frame(de, dn, yaw) -> (body_x, body_y)`; `Route.point_at_s(s) -> np.ndarray([e, n])`; `Route.length -> float`.
- Produces: `follow_path(route, pose_e, pose_n, yaw, cursor_s, ahead=FOLLOW_AHEAD, ds=FOLLOW_DS) -> (list[[float, float]], float)` returning `(points, lat_shift)`; module constants `FOLLOW_AHEAD = 70.0`, `FOLLOW_DS = 0.5`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_follow_path.py`:

```python
import math
import numpy as np
import pytest
from parking_proj.geometry import route_from_waypoints
from parking_proj.projection import follow_path


def straight_route():
    # due-east centerline, 100 m long, resampled at 0.1 m
    return route_from_waypoints([[0.0, 0.0], [100.0, 0.0]], ["1", "2"])


def test_follow_path_nulls_constant_lateral_offset():
    route = straight_route()
    # car 2 m north (= left) of the east-going route, heading due east
    pts, lat_shift = follow_path(route, 10.0, 2.0, 0.0, 10.0)
    # anchor is 2 m to the car's right -> car-frame lateral is -2 m
    assert lat_shift == pytest.approx(-2.0, abs=1e-6)
    assert max(abs(y) for _, y in pts) < 1e-6          # offset removed
    xs = [x for x, _ in pts]
    assert xs[0] == pytest.approx(0.0, abs=1e-9)
    assert xs[-1] == pytest.approx(70.0, abs=1e-6)     # full 0..70 m window
    assert np.allclose(np.diff(xs), 0.5, atol=1e-6)    # 0.5 m spacing


def test_follow_path_shows_heading_error():
    route = straight_route()
    yaw = math.radians(10.0)
    # on the line laterally, but pointed 10 deg off the route tangent
    pts, lat_shift = follow_path(route, 10.0, 0.0, yaw, 10.0)
    assert lat_shift == pytest.approx(0.0, abs=1e-9)
    far = next(p for p in pts if p[0] > 4.5)
    assert far[1] / far[0] == pytest.approx(-math.tan(yaw), abs=1e-3)


def test_follow_path_truncates_at_route_end():
    route = straight_route()                            # length 100 m
    pts, _ = follow_path(route, 60.0, 0.0, 0.0, 60.0)
    xs = [x for x, _ in pts]
    assert xs[-1] == pytest.approx(40.0, abs=1e-6)      # 100 - 60, not 70
    assert xs[-1] <= 70.0 + 1e-9
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv/bin/python -m pytest tests/test_follow_path.py -v`
Expected: FAIL with `ImportError: cannot import name 'follow_path'`.

- [ ] **Step 3: Add the import at the top of `projection.py`**

In `src/parking_proj/projection.py`, after the existing `import numpy as np` (line 4), add:

```python
from .transform import to_body_frame
```

- [ ] **Step 4: Add constants + function at the end of `projection.py`**

Append to `src/parking_proj/projection.py`:

```python
FOLLOW_AHEAD = 70.0
FOLLOW_DS = 0.5


def follow_path(route, pose_e, pose_n, yaw, cursor_s, ahead=FOLLOW_AHEAD, ds=FOLLOW_DS):
    """Re-anchored body-frame path with the lateral offset removed.

    Anchor P = route point at cursor_s. Each sampled route point ahead is
    expressed in the car-heading body frame (+x fwd, +y left), then shifted
    laterally by the car-frame lateral of P so P lands on y=0. Forward-only
    window [cursor_s, cursor_s+ahead], truncated at route end, sampled at ds.
    Returns (points, lat_shift): points is a list of [x, y]; lat_shift is the
    meters subtracted (car-frame lateral of the anchor).
    """
    px, py = route.point_at_s(cursor_s)
    _, lat_shift = to_body_frame(px - pose_e, py - pose_n, yaw)
    end_s = min(cursor_s + ahead, route.length)
    n = int((end_s - cursor_s) / ds) + 1
    pts = []
    for k in range(n):
        qx, qy = route.point_at_s(cursor_s + k * ds)
        bx, by = to_body_frame(qx - pose_e, qy - pose_n, yaw)
        pts.append([bx, by - lat_shift])
    return pts, lat_shift
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `.venv/bin/python -m pytest tests/test_follow_path.py -v`
Expected: PASS (3 passed).

- [ ] **Step 6: Commit**

```bash
git add src/parking_proj/projection.py tests/test_follow_path.py
git commit -m "feat: follow_path() — lateral-offset-removed body-frame path"
```

---

### Task 2: Emit `follow_path` from both generators

**Files:**
- Modify: `src/parking_proj/generate.py` (import line 4-6; frame loop ~28-44; config ~57)
- Modify: `src/parking_proj/generate_real.py` (import line 8; frame loop ~38-50; config ~58)
- Test: `tests/test_generate_follow_path.py` (create)

**Interfaces:**
- Consumes: `follow_path`, `FOLLOW_AHEAD`, `FOLLOW_DS` from Task 1; `build_case_dict(scenario)` and `build_scenarios()` (existing).
- Produces: each frame dict gains `"follow_path": [[x,y],…] | None` and `"lat_shift": float | None`; each case `config` gains `"follow_ahead"` and `"follow_ds"`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_generate_follow_path.py`:

```python
from parking_proj.generate import build_case_dict
from parking_proj.scenarios import build_scenarios


def test_case_dict_emits_follow_path():
    case = build_case_dict(build_scenarios()[0])
    assert case["config"]["follow_ahead"] == 70.0
    assert case["config"]["follow_ds"] == 0.5
    fr = case["frames"][5]
    assert isinstance(fr["follow_path"], list) and len(fr["follow_path"]) > 1
    assert all(len(p) == 2 for p in fr["follow_path"])
    assert fr["lat_shift"] is not None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.venv/bin/python -m pytest tests/test_generate_follow_path.py -v`
Expected: FAIL with `KeyError: 'follow_ahead'`.

- [ ] **Step 3: Update `generate.py` import**

In `src/parking_proj/generate.py`, change line 5:

```python
from .projection import Projector, follow_path, FOLLOW_AHEAD, FOLLOW_DS
```

- [ ] **Step 4: Update `generate.py` frame loop**

Replace the frame loop body (currently lines 28-44, the `for f, r in zip(frames, results):` block) with:

```python
    for f, r in zip(frames, results):
        if r is None:
            fp, lat_shift = None, None
        else:
            fp, lat_shift = follow_path(route, f.meas_e, f.meas_n, f.meas_yaw, r.cursor_s)
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
            "follow_path": None if fp is None else [[round(x, 3), round(y, 3)] for x, y in fp],
            "lat_shift": None if lat_shift is None else round(lat_shift, 4),
            "gt_seg": f.gt_seg,
            "gt_s": round(f.gt_s, 3),
        })
```

- [ ] **Step 5: Update `generate.py` config**

In the returned dict, replace the `"config"` line with:

```python
        "config": {"ahead": proj.ahead, "behind": proj.behind,
                   "follow_ahead": FOLLOW_AHEAD, "follow_ds": FOLLOW_DS},
```

- [ ] **Step 6: Run test to verify it passes**

Run: `.venv/bin/python -m pytest tests/test_generate_follow_path.py -v`
Expected: PASS.

- [ ] **Step 7: Mirror the change in `generate_real.py`**

Change the import (line 8) to:

```python
from .projection import Projector, follow_path, FOLLOW_AHEAD, FOLLOW_DS
```

Replace the frame loop (currently lines 38-50) with:

```python
    for i in range(len(ds.meas_e)):
        r = proj.step(float(ds.meas_e[i]), float(ds.meas_n[i]), float(ds.meas_yaw[i]))
        fp, lat_shift = follow_path(route, float(ds.meas_e[i]), float(ds.meas_n[i]),
                                    float(ds.meas_yaw[i]), r.cursor_s)
        frames.append({
            "t": _r(ds.t_us[i] / 1e6),
            "speed": _r(ds.speed[i]),
            "meas_pose": {"e": _r(ds.meas_e[i]), "n": _r(ds.meas_n[i]),
                          "h": _r(ds.meas_yaw[i], 5)},
            "meas_ll": {"lat": _r(ds.ego_llh[i, 0], 7), "lon": _r(ds.ego_llh[i, 1], 7)},
            "cursor_s": _r(r.cursor_s),
            "matched_seg": r.matched_seg,
            "est_lat_dev": _r(r.est_lat_dev, 4),
            "follow_path": [[_r(x), _r(y)] for x, y in fp],
            "lat_shift": _r(lat_shift, 4),
            "end_flag": bool(r.end_flag),
        })
```

Replace the `"config"` line (line 58) with:

```python
        "config": {"ahead": proj.ahead, "behind": proj.behind,
                   "follow_ahead": FOLLOW_AHEAD, "follow_ds": FOLLOW_DS},
```

- [ ] **Step 8: Verify grading unchanged + regenerate**

Run: `.venv/bin/python -m pytest -q` (full unit + acceptance suite)
Expected: all pass, including the existing acceptance tests (14 cases, 0 mismatches) — proving grading is unaffected.

Run: `.venv/bin/python -m parking_proj.generate`
Expected: writes `out/*.json` without error.

- [ ] **Step 9: Commit**

```bash
git add src/parking_proj/generate.py src/parking_proj/generate_real.py tests/test_generate_follow_path.py
git commit -m "feat: emit follow_path + lat_shift per frame (sim + real)"
```

---

### Task 3: Viewer — toggle + draw the follow-path

**Files:**
- Modify: `viewer/index.html:23` (add the checkbox)
- Modify: `viewer/viewer.js` (add `bodyRoutePoints` helper; use it in `drawDriver` and `drawWindshield`; wire the toggle)
- Test: `tests/e2e/test_viewer_e2e.py` (add one e2e test)

**Interfaces:**
- Consumes: per-frame `f.follow_path` and `c.config` from Task 2; existing `worldToBody(pe,pn,ex,ny,yaw) -> {x,y}`, `renderFrame()`.
- Produces: DOM `#recenter-toggle` (checked by default); `bodyRoutePoints(c, f, behind, ahead) -> [{x,y},…]`.

- [ ] **Step 1: Add the checkbox to `index.html`**

In `viewer/index.html`, replace line 23:

```html
      <label><input type="checkbox" id="persp-toggle"> perspective</label></h3>
```

with:

```html
      <label><input type="checkbox" id="recenter-toggle" checked> remove lateral offset</label>
      <label><input type="checkbox" id="persp-toggle"> perspective</label></h3>
```

- [ ] **Step 2: Add the `bodyRoutePoints` helper to `viewer.js`**

In `viewer/viewer.js`, immediately after `worldToBody` (ends at line 278), add:

```javascript
// Body-frame route points to draw this frame. When "remove lateral offset" is
// on and the algorithm emitted a follow_path, replay it directly (already
// body-frame, offset removed); otherwise derive the raw slice from the route
// and the measured pose. `behind`/`ahead` are body-forward metres.
function bodyRoutePoints(c, f, behind, ahead) {
  const recenter = document.getElementById("recenter-toggle");
  if (recenter && recenter.checked && f.follow_path) {
    return f.follow_path
      .map(([x, y]) => ({ x, y }))
      .filter((p) => p.x >= behind && p.x <= ahead);
  }
  const s = c.route.s, e = c.route.points_e, n = c.route.points_n, out = [];
  const loS = f.cursor_s + behind, hiS = f.cursor_s + ahead;
  for (let i = 0; i < s.length; i++) {
    if (s[i] < loS || s[i] > hiS) continue;
    out.push(worldToBody(f.meas_pose.e, f.meas_pose.n, e[i], n[i], f.meas_pose.h));
  }
  return out;
}
```

- [ ] **Step 3: Use the helper in `drawDriver` (top-down)**

In `viewer/viewer.js`, replace the route-slice block of `drawDriver` — currently:

```javascript
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
```

with:

```javascript
  // route slice in the body frame (offset-removed follow_path, or raw)
  const pts = bodyRoutePoints(c, f, behind, ahead);
  ctx.strokeStyle = "#2e9e5b"; ctx.lineWidth = 3; ctx.beginPath();
  pts.forEach((b, i) => {
    const x = toX(b.y), y = toY(b.x);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
```

- [ ] **Step 4: Use the helper in `drawWindshield` (perspective)**

In `viewer/viewer.js`, replace the ribbon-sampling block of `drawWindshield` — currently:

```javascript
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
```

with:

```javascript
  const HW = PERSP.half_width;
  const pts = bodyRoutePoints(STATE.case, f, 0, XMAX);
  const left = [], right = [], mid = [];
  for (const b of pts) {
    if (b.x <= 0.05) continue;
    const pl = project(b.x, b.y + HW), pr = project(b.x, b.y - HW), pm = project(b.x, b.y);
    if (pl) left.push(pl);
    if (pr) right.push(pr);
    if (pm) mid.push(pm);
  }
```

- [ ] **Step 5: Wire the toggle to re-render**

In `viewer/viewer.js`, immediately after line 512 (`document.getElementById("persp-toggle").onchange = () => renderFrame();`), add:

```javascript
  document.getElementById("recenter-toggle").onchange = () => renderFrame();
```

- [ ] **Step 6: Add the e2e test**

In `tests/e2e/test_viewer_e2e.py`, add:

```python
def test_recenter_toggle_default_and_changes_view(viewer):
    page, _ = viewer
    page.click("#tab-sim")
    _select(page, "X-crossing (high)")
    # move into the run so a lateral offset is present
    page.eval_on_selector(
        "#scrubber",
        "el => { el.value = Math.floor(el.max/2); el.dispatchEvent(new Event('input')); }",
    )
    assert page.is_checked("#recenter-toggle")            # default ON
    sig_on = page.evaluate(_SIGNATURE, "#driver")
    page.uncheck("#recenter-toggle")
    page.wait_for_timeout(100)
    sig_off = page.evaluate(_SIGNATURE, "#driver")
    assert sig_on != sig_off, "recenter toggle did not change the driver view"
    page.check("#recenter-toggle")
    page.click("#tab-real")
```

- [ ] **Step 7: Run the e2e suite**

Run: `.venv/bin/python -m pytest -m e2e -v`
Expected: all pass, including the new `test_recenter_toggle_default_and_changes_view` and the existing driver/perspective coverage tests.

- [ ] **Step 8: Commit**

```bash
git add viewer/index.html viewer/viewer.js tests/e2e/test_viewer_e2e.py
git commit -m "feat(viewer): default-on 'remove lateral offset' toggle draws follow_path"
```

---

### Task 4: Documentation

**Files:**
- Modify: `spec_design.md` (mandated by CLAUDE.md)
- Modify: `algorithm_description.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: the transform, output contract, and toggle behavior from Tasks 1-3.
- Produces: no code; documentation only.

- [ ] **Step 1: Update `spec_design.md`**

Add a subsection under the extensions/outputs area describing: the follow-path re-anchoring (anchor at `cursor_s`, rigid lateral shift by the car-frame lateral of the anchor, car-heading frame, forward coords unchanged); the JSON output contract (`follow_path` `[[x,y],…]` body frame `+x` fwd `+y` left, forward-only 0–70 m, 0.5 m spacing, truncated at route end; `lat_shift`); that `est_lat_dev` is unchanged and `lat_shift ≈ −est_lat_dev` at zero heading error; that grading is unaffected; and the viewer's default-on "remove lateral offset" toggle applying to both the top-down and perspective driver views.

- [ ] **Step 2: Update `algorithm_description.md`**

Add the math: anchor `P = route(cursor_s)`; `lat_shift = to_body_frame(P − pose, yaw).y`; sampled points `Q(s')` for `s' ∈ [cursor_s, min(cursor_s+70, L)]` step 0.5; `follow(s') = ( to_body_frame(Q−pose,yaw).x , to_body_frame(Q−pose,yaw).y − lat_shift )`. State that this removes cross-track offset while preserving heading error and curvature, and that it does not alter matching (`cursor_s`, `matched_seg`).

- [ ] **Step 3: Update `README.md`**

In the "Real data" section and/or "Using the viewer" section, note the new `follow_path`/`lat_shift` per-frame output and the "remove lateral offset" driver-view toggle (default on), and that the exported follow-path is the offset-free body-frame path (0–70 m ahead) intended for downstream consumers.

- [ ] **Step 4: Commit**

```bash
git add spec_design.md algorithm_description.md README.md
git commit -m "docs: document follow_path output and lateral-offset removal"
```

---

## Self-Review

**Spec coverage:**
- Transform (rigid lateral shift, car-heading frame, forward unchanged) → Task 1.
- Output contract (`follow_path` 0–70 m @ 0.5 m truncated; `lat_shift`; `est_lat_dev` kept; config fields) → Tasks 1 & 2.
- Grading unaffected → Task 2 Step 8 (full suite).
- Viewer toggle default-on, both views, BEV unchanged → Task 3.
- Docs (spec_design, algorithm_description, README) → Task 4.
All spec sections map to a task.

**Placeholder scan:** No TBD/TODO; every code step shows complete code and exact commands.

**Type consistency:** `follow_path(...)` returns `(list[[x,y]], lat_shift)` in Task 1 and is consumed with that exact shape in Task 2. `bodyRoutePoints(c,f,behind,ahead)` returns `[{x,y}]` and both call sites consume `.x`/`.y`. `FOLLOW_AHEAD`/`FOLLOW_DS` defined in Task 1, imported in Task 2. `#recenter-toggle` id consistent across HTML, helper, wiring, and the e2e test.
