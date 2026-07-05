# `project_route` — global route → driver-view projection

This is the delivery note for the one function you port to the vehicle. It turns
a globally-planned route into the path **as seen from the driver's seat** (the
vehicle-body frame), one frame at a time, so a downstream low-speed parking
controller can follow it. Everything below matches the reference code in
`src/parking_proj/project_route.py` and `src/parking_proj/smoothing.py`.

The function is **pure**: it holds no hidden state. You keep a small state object
and hand it back to the function on the next frame. Same inputs always give the
same output, so it is easy to test and to port.

---

## 1. The call

```
project_route(route, pose_e, pose_n, yaw, config, state=None, speed=None) -> ProjectOutput
```

All geometry is done in a **local metric frame (ENU, meters)** — East `e`, North
`n`. Converting GPS/WGS-84 or GCJ-02 to ENU happens elsewhere; this function
never touches lat/lon.

The **body frame** it outputs is: `+x` forward (the nose direction), `+y` left.
`yaw` is the counter-clockwise angle of `+x` (body) measured from ENU East, in
radians.

### Inputs

| Parameter | What it is |
|-----------|-----------|
| `route` | The planned route as a prepared polyline object: `.points` (an N×2 array of `(e, n)`), `.s` (cumulative arc-length), `.tangents` (unit direction per point), `.length`, and lookups `.point_at_s(s)` / `.index_at_s(s)` / `.seg_of_index`. Built once; treated as immutable. |
| `pose_e`, `pose_n` | The vehicle's current position in ENU (meters). |
| `yaw` | The vehicle heading (radians, CCW from East). |
| `config` | A `ProjectConfig` (all tuning knobs, see below). |
| `state` | The `ProjectState` returned last frame. Pass `None` on the first frame — that triggers a one-time global search to seed the cursor. |
| `speed` | Vehicle speed (m/s). Accepted but not used by the current strategies; reserved for future use. |

### `ProjectConfig` (all fields configurable; these are the code defaults)

| Field | Default | Meaning |
|-------|---------|---------|
| `strategy` | `"smoothed"` | Output style: `"raw"` / `"centered"` / `"smoothed"` / `"human"` / `"human_centered"`. (The demo viewer defaults to `"human_centered"`.) |
| `behind_m` | `5.0` | How far behind the car the output path starts (m). |
| `ahead_m` | `40.0` | How far ahead the output path ends (m). |
| `sample_ds_m` | `0.5` | Spacing between output path points (m). |
| `search_ahead_m` | `15.0` | Forward reach of the per-frame match window (m). Same value as the module constant `SEARCH_AHEAD = 15.0`. |
| `search_back_m` | `0.3` | Backward tolerance of the match window (m). |
| `heading_gate_deg` | `60.0` | Heading-agreement gate (degrees) for matching. |
| `min_turn_radius_m` | `8.0` | Corner radius used for smoothing (m); applies to `smoothed` arc/clothoid. |
| `corner_angle_deg` | `10.0` | Only corners sharper than this are treated. |
| `simplify_eps_m` | `0.20` | Corner-detection tolerance (RDP) (m). |
| `corner_style` | `"clothoid"` | `smoothed` corner shape: `"clothoid"` / `"arc"` / `"driver"`. (The demo viewer defaults to `"driver"`.) |
| `clothoid_transition_m` | `8.0` | Clothoid transition length; also the Gaussian smoothing width (σ) for `"driver"` and the `human` strategies. The human ego-track calibration measured `1.5` m; the default is raised to `8.0` for a smoother feel. |
| `human_cut_m` | `2.2` | `"human"` strategy only: how far the corner is cut toward the inside at a 90° turn (m), calibrated from ego tracks. |

`ProjectState` carries `cursor_s` (the progress along the route, in meters of
arc-length) and `initialized`. You do not build it yourself — take it from the
previous `ProjectOutput.state`.

### Output — `ProjectOutput`

| Field | What it is |
|-------|-----------|
| `path` | The result: a list of `[x, y]` points in the body frame (`+x` forward, `+y` left), spanning `[cursor_s − behind_m, cursor_s + ahead_m]`. This is what the controller follows. |
| `cursor_s` | Where the car is along the route now (arc-length, meters). Never decreases across frames. |
| `lat_dev` | Signed cross-track deviation (m). **positive = vehicle is to the left of the route.** |
| `matched_seg` | Index of the matched route segment. |
| `end_flag` | `True` when `cursor_s + ahead_m` has reached the end of the route. |
| `state` | The `ProjectState` to hand back next frame. |

---

## 2. Core idea

Two steps every frame: (a) find where on the route the car is, then (b) build
the path in the driver's frame.

### (a) Match — where are we on the route

The function keeps a **progress cursor** `cursor_s` (how far along the route we
are). Each frame:

1. On the first frame (`state is None` or not initialized) it does one global
   nearest-point search over the whole route.
2. After that it only searches a **bounded forward window** in arc-length:
   `[cursor_s − search_back_m, cursor_s + search_ahead_m]`, and picks the closest
   route point to the measured position, subject to a **heading-agreement gate**
   (the route's direction must be within `heading_gate_deg` of the vehicle
   heading). If no candidate passes the gate, the gate is widened for that one
   frame so a frame is **never dropped**.
3. The cursor advances **monotonically**: `cursor_s = max(cursor_s, matched_s)`
   — it never decreases. This is what keeps the match on the right stroke where
   a route **crosses itself**: on the first pass the other stroke is far outside
   the window, and on the later pass the cursor has already moved past the
   crossing. It relies on the product rule that a road is never re-walked and
   there are no U-turns.

`lat_dev`, `matched_seg`, and `end_flag` come out of this step.

### (b) Build the driver-view path — five strategies

The route slice over `[cursor_s − behind_m, cursor_s + ahead_m]` is rotated into
the body frame. What differs between strategies is how the lateral offset and the
corners are handled. `strategy` selects one of:

| Strategy | Lateral offset | Corner treatment |
|----------|---------------|------------------|
| `"raw"` | kept as-is | none |
| `"centered"` | removed (matched point put at `y = 0`) | none |
| `"smoothed"` | removed | corner fillet per `corner_style` |
| `"human"` | removed at the anchor | corner **cut** toward the inside + smoothing |
| `"human_centered"` | minimised (vehicle projected onto the curve) | same as `"human"`, plus pointed straight forward |

Details:

- **`"raw"`** — body-frame rotation only. The path shows the real cross-track
  offset.
- **`"centered"`** — subtract the car-frame lateral of the matched point
  (`lat_shift`) from every point, so the matched point sits on `y = 0`. Lateral
  offset removed; heading error and curvature preserved.
- **`"smoothed"`** — `"centered"`, then the corners are rounded. The rounding
  runs on the whole route **once in world space** (result cached on the route),
  and each frame just re-windows that fixed curve — so the corner does **not**
  jitter frame to frame. The shape is set by `corner_style`:
  - `"arc"`: a circular fillet; tangent length `T = min(min_turn_radius_m · tan(δ/2), ½ leg)`, effective radius `R_eff = T / tan(δ/2)`. Curvature jumps from 0 to `1/R` at the entry. For non-degenerate legs, curvature `κ ≤ 1 / min_turn_radius_m`.
  - `"clothoid"`: an Euler spiral whose curvature ramps linearly `0 → 1/R → 0`, so there is no curvature jump; falls back to an arc when the legs are too short.
  - `"driver"`: a Gaussian low-pass of the whole route (width σ = `clothoid_transition_m`); it starts turning **before** the corner and is very smooth.
- **`"human"`** — `"centered"`, then each sharp corner vertex is shifted toward
  the **inside** of the turn by `human_cut_m · (δ / 90°)` and the route is
  Gaussian-smoothed. This mimics a driver taking a corner early and wide.
- **`"human_centered"`** — the `"human"` curve, but the vehicle is **projected
  onto that curve** (nearest point, searched within `±search_ahead_m`) and the
  frame is oriented along the **curve tangent** at that point. Result: the path
  leaves the car pointing **straight forward** with the near cross-track offset
  ≈ 0 (both the lateral offset and the heading angle are neutralised).

The part of the path **behind** the car is never smoothed.

The corner geometry lives in `src/parking_proj/smoothing.py`: `rdp` (corner-vertex
detection), `resample` (uniform spacing), `smooth_corners(pts, min_radius,
corner_angle_deg, ds, eps, corner_style="arc", transition=3.0)` for arc/clothoid/
driver, and `human_corners(pts, cut_gain, sigma, ds, eps, corner_angle_deg)` for
the corner-cut. (Note the `smooth_corners` function's own defaults are
`corner_style="arc", transition=3.0`; the product default `"clothoid"` / `8.0` is
supplied by `ProjectConfig` and passed in.)

---

## 3. Practical notes for porting

- **Deterministic:** no randomness anywhere. Same inputs → same output, bit for
  bit. This is what makes the JavaScript twin able to match the Python.
- **Cheap:** the match scans only the small forward window (not the whole route);
  the corner math is closed-form (arc/clothoid) or a short fixed-step loop, with
  no convergence iteration. The once-in-world smoothing is cached, so per frame
  you only re-window a fixed curve.
- **Reference vs. twin:** the Python `project_route.py` is authoritative. A
  DOM-free JavaScript twin, `viewer/project_route.js` (exposed as
  `window.ProjectRoute`), mirrors it, and `tests/e2e/test_parity_py_js.py`
  checks the two agree to within `1e-3` m across all strategies and corner
  styles. If you write another port, hold it to the same parity check.

---

*Self-check: (a) input signature, ENU/body-frame conventions, every `ProjectConfig`
field with its default, `ProjectState`, and every `ProjectOutput` field are
covered; the two-step core idea (monotonic-cursor match with the forward window +
heading gate + "never drop a frame" + monotonic non-decrease) and all five
strategies (with the three `smoothed` corner styles and the `human` / `human_centered`
behavior) are covered; determinism, cost, and the JS parity note are covered.
(b) Exact identifiers, defaults (`ahead_m = 40.0`, `clothoid_transition_m = 8.0`,
`min_turn_radius_m = 8.0`, `human_cut_m = 2.2`, `SEARCH_AHEAD = 15.0`, gate `60`,
`1e-3` m, etc.), and constraint wording ("pure", "never dropped", "never
decreases", "positive = vehicle is to the left", "behind … never smoothed") are
preserved verbatim; nothing was invented beyond the cited source files.*
