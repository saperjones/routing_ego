# Portable Route-Projection Function + Live Demo — Design

**Date:** 2026-07-04
**Branch:** feature/parking-route-projection (main)

## Goal

Extract the core "project the global planned route into the vehicle body frame
(driver view)" logic into **one clearly-specified, portable function** that will
be ported to the vehicle. Give it **three switchable strategies** — keep-offset,
remove-offset, and remove-offset-plus-corner-smoothing — with **all parameters
configurable**. Port the function to JavaScript so the demo viewer computes the
result **live** (no precompute) with a left-panel algorithm selector and
parameter sliders. Ship a function-description document and a web-visible e2e
test.

## Architecture change (called out deliberately)

Today the viewer is a **pure replayer** of prebaked JSON; the algorithm lives
only in Python. This feature moves the algorithm into a **DOM-free JS module
that both the viewer and a parity test import**, and the viewer runs it live.

- **Python remains authoritative** — the reference implementation, the tested
  one, and the one ported to the vehicle.
- The prebaked JSON carries **only raw inputs** (global route + per-frame
  poses) — no precomputed paths. `follow_path` / `lat_shift` / the per-frame
  path fields are removed.
- A **Python↔JS parity test** guarantees the two implementations agree, so they
  cannot silently drift.
- `CLAUDE.md` and `spec_design.md` are updated to record the new invariant:
  *the viewer runs the shared projection algorithm live; Python is authoritative
  and a parity test binds the two.*

## 1. The function (I/O contract)

Pure function; the caller owns a small state object and passes it back each
frame. No hidden state → deterministic, testable frame-by-frame, portable to C.

```
project_route(route, pose_x, pose_y, pose_yaw, config, state=None, speed=None) -> ProjectOutput
```

**Inputs**
- `route`: prepared global polyline in a **local metric frame (ENU, metres)**:
  `points` (N×2), `s` (N cumulative arc-length), `tangents` (N×2 unit). Built
  once; datum conversion (GCJ/WGS→ENU) stays outside in `geo.py` so the function
  is pure geometry.
- `pose_x, pose_y, pose_yaw`: current vehicle pose in the same frame
  (`yaw` = CCW from +E, radians).
- `config`: `ProjectConfig` (below).
- `state`: `ProjectState` from the previous frame, or `None` on the first call.
- `speed`: optional (m/s); accepted but unused today — reserved for future
  look-ahead scaling. Keeps the signature stable.

**`ProjectConfig`** (all configurable)
```
strategy          = "smoothed"   # "raw" | "centered" | "smoothed"
behind_m          = 5.0          # output window back
ahead_m           = 70.0         # output window front
sample_ds_m       = 0.5          # output sample spacing
# matching (monotonic progress) — fixed during a run, not live sliders
search_ahead_m    = 15.0         # forward search window ("endurable offset")
search_back_m     = 0.3
heading_gate_deg  = 60.0
# smoothing (strategy = "smoothed")
min_turn_radius_m = 5.0          # R_min — vehicle kinematic limit
corner_angle_deg  = 10.0         # only fillet turns sharper than this
simplify_eps_m    = 0.20         # corner-vertex detection tolerance (RDP)
```

**`ProjectState`**: `cursor_s: float|None`, `initialized: bool`. The monotonic
`cursor_s` is the compact summary of "progress so far" — it replaces needing to
pass the whole driven history.

**`ProjectOutput`**: `path: list[[x,y]]` (body frame **+x fwd, +y left**, from
`−behind_m` to `+ahead_m`), `cursor_s`, `lat_dev`, `matched_seg`, `end_flag`,
`state` (updated).

## 2. Matching (internal)

Same monotonic-cursor matcher as today, now inside the function and driven by
`state`:
- first call (`state is None` or not initialized): global nearest heading-gated
  point → `cursor_s`.
- later calls: search `[cursor_s − search_back_m, cursor_s + search_ahead_m]`,
  heading-gated (`heading_gate_deg`, widened one frame if nothing passes),
  nearest point; advance `cursor_s = max(cursor_s, matched_s)` (monotonic).
- `lat_dev` = signed cross-track deviation at the matched point;
  `end_flag = cursor_s + ahead_m ≥ route.length`.

## 3. Three strategies (compose; `config.strategy`)

Sample the route over `[cursor_s − behind_m, cursor_s + ahead_m]` at `ds`,
transform to the body frame by pose (`+x` fwd, `+y` left):
- **`raw`** — body-frame slice as-is (keeps the lateral offset).
- **`centered`** — `raw` minus `lat_shift` (car-frame lateral of the anchor at
  `cursor_s`), a rigid lateral shift; forward coords and heading error
  preserved. The anchor lands at `y = 0`.
- **`smoothed`** — `centered`, then arc-fillet the sharp corners in the forward
  look-ahead (§4).

`lat_shift ≈ −lat_dev` at ~zero heading error (car→route vs route→car).

## 4. Arc-fillet smoothing (fast, provably drivable)

Applied per frame to the **forward** part (`x ∈ [0, ahead_m]`) of the centered
path; the behind part passes through unchanged.

1. **Corner vertices** — Ramer–Douglas–Peucker (`eps = simplify_eps_m`) collapses
   the dense (0.1 m) polyline into a few straight legs + vertices. Uniform for
   sim waypoint-routes and dense real routes.
2. **Fillet** each interior vertex `V` whose signed turn angle
   `δ` (between incoming dir `d₁` and outgoing dir `d₂`) exceeds
   `corner_angle_deg`:
   - tangent length `T = R·tan(|δ|/2)` with `R = min_turn_radius_m`;
   - clamp `T ≤ 0.5·min(|leg_in|, |leg_out|)`; if clamped, effective radius
     `R_eff = T / tan(|δ|/2)` (**≥ R_min**, still feasible);
   - tangent points `P₁ = V − T·d₁`, `P₂ = V + T·d₂`;
   - centre `C = P₁ + R_eff·n`, `n` = `d₁` rotated +90° (left turn, `cross(d₁,d₂)>0`)
     or −90° (right turn);
   - arc `P₁→P₂` about `C` sweeping `|δ|`, sampled at angular step `ds/R_eff`.
3. **Resample** the resulting straights+arcs at `ds`.

**Feasibility guarantee:** curvature `κ ≤ 1/R_min` everywhere → the vehicle can
always drive the path. All closed-form (`atan2`, `tan`, `sin/cos`); a 75 m
look-ahead has 0–2 corners, RDP is ~O(n) on ~150 points → tens–hundreds of
flops per frame.

**"Revert to straight after the turn" is automatic:** each frame re-fillets
whatever corner is still ahead; once `cursor_s` passes it, the forward window is
straight.

**Clothoid-ready:** the fillet is a `corner_curve(V, d₁, d₂, δ, R, ds)`
helper behind a small interface; a clothoid generator can replace it later
without touching the rest of the function.

**Edge cases:** `|δ| < corner_angle_deg` → straight (skip); legs too short →
T-clamp (radius grows, still feasible); adjacent corners can't overlap because
`T ≤ ½ leg`; corner at the near origin → fillet begins near the car.

## 5. JS port + live demo

- **`viewer/project_route.js`** — a **DOM-free** JS module mirroring the Python
  function: `projectRoute(route, pose, config, state)` plus `rdp`, `arcFillet`,
  and `DEFAULT_CONFIG`. Imported by both the viewer and the parity test.
- **Viewer** loads `route` (points/s, tangents computed in JS) + per-frame
  `meas_pose` (already in the JSON) and computes the path **live**:
  - **Left-panel controls:** a 3-way **algorithm selector** (Raw / Centered /
    Smoothed) and **live sliders** for `min_turn_radius_m`, `behind_m`,
    `ahead_m`, `corner_angle_deg`. Changing any of these recomputes the *current
    frame's output only* (matching cursor is unaffected) → instant.
  - **State on scrub/playback:** a `cursorByFrame[]` memo; matching is stepped
    sequentially 0→k and cached. Playback is incremental; scrubbing is instant
    after the first pass. Matching params (`search_ahead_m`, gate) are **fixed**
    in the demo (not sliders) so the cursor memo never needs rebuilding on a
    slider drag.
  - Both the top-down and windshield driver views draw the live path. BEV
    unchanged.
- **Prebaked JSON slims down:** only the baked *path* fields (`follow_path`,
  `lat_shift`) are removed. Everything the other panels need stays —
  `meas_pose`, `true_pose`, `speed`, `cursor_s`, `matched_seg`, `est_lat_dev`,
  grading `verdict`, `gt_*`, the route, LLH/basemap for real cases, etc.

## 6. Performance (embedded)

Per frame: windowed nearest search (O(window/ds)), a rigid shift, RDP + a couple
of closed-form arcs. No transcendental beyond `atan2/tan/sin/cos`, no
allocation-heavy structures, no iteration-to-converge. Suitable for a low-power
vehicle target; the Python and JS versions are both straight-line ports of the
same steps.

## 7. Files

- Create: `src/parking_proj/project_route.py` (function, `ProjectConfig`,
  `ProjectState`, `ProjectOutput`).
- Create: `src/parking_proj/smoothing.py` (`rdp`, `arc_fillet`, `corner_curve`).
- Create: `viewer/project_route.js` (DOM-free JS twin).
- Modify: `viewer/viewer.js`, `viewer/index.html` (selector + sliders + live
  compute; remove the recenter checkbox).
- Modify: `src/parking_proj/generate.py`, `generate_real.py` (stop baking paths).
- Fold: `Projector` / `follow_path` become thin wrappers over `project_route`
  (or are removed and callers updated). Grading is unaffected — matching output
  (`cursor_s`, `matched_seg`) is strategy-independent, so `generate.py` obtains
  it from `project_route` and the 14 acceptance verdicts stay identical.
- Docs: create `docs/project_route_function.md` (the "核心设计思路" deliverable);
  update `spec_design.md`, `algorithm_description.md`, `README.md`, `CLAUDE.md`.

## 8. Tests

**Python unit** (`tests/test_project_route.py`)
- `raw`: straight route + lateral offset ⇒ path retains the offset.
- `centered`: same ⇒ path `y ≈ 0` (offset nulled), `lat_dev` still reports it.
- `smoothed`: a 90° corner route ⇒ every consecutive-sample heading change
  corresponds to `κ ≤ 1/R_min + tol` (no sharp vertex), and the path stays
  within the corner region (bounded deviation); `raw`/`centered` on the same
  route DO contain a sharp vertex.
- window `[−behind, +ahead]`, `ds` spacing, truncation at route end.
- matching: monotonic cursor; along-track jump caught within `search_ahead_m`.

**Parity** (`tests/test_parity_py_js.py`) — Python dumps `project_route` outputs
for a set of (route, pose, config) fixtures; the JS module runs the same inputs
(via headless Chromium loading a tiny harness that imports `project_route.js`);
assert paths agree within a small tolerance (e.g. 1e-3 m) across all three
strategies.

**e2e web-visible** (`tests/e2e/test_viewer_e2e.py`)
- The 3-way selector switches the driver canvas; each choice renders.
- On a sharp-corner case with **Smoothed** selected, the drawn path has bounded
  per-sample heading change (rounded), whereas **Centered** shows the sharp
  corner — pixel/data proof of the fillet.
- Moving the `min_turn_radius_m` slider changes the smoothed path (larger R →
  wider arc).
- No JS errors; existing sim/real coverage still passes. Existing
  follow_path/recenter-toggle tests are updated to the new controls.

## 9. Non-goals / future

- Clothoid/spiral transitions (interface is ready; not built now).
- Kinematic feasibility from an arbitrary current heading (Dubins/Reeds–Shepp).
- Speed-scaled look-ahead; reverse-gear / multi-segment parking maneuvers.
