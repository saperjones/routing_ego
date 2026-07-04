# 核心设计思路 — `project_route` Function Description

This document describes the portable `project_route` function: its I/O contract,
the three output strategies, the arc and clothoid corner-smoothing math, the
per-frame behavior contract, and performance characteristics. Every symbol is
verified against `src/parking_proj/project_route.py`
and `src/parking_proj/smoothing.py`.

---

## 1. Entry Point

```python
project_route(route, pose_e, pose_n, yaw, config, state=None, speed=None) -> ProjectOutput
```

**The function is pure and stateless.** The caller holds the tiny `ProjectState`
object and passes it back each frame. There is no hidden mutation.

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `route` | `Route` | Dense polyline with arc-length `s`, unit tangents, and segment index. Immutable. |
| `pose_e` | `float` | ENU East of the vehicle (meters). |
| `pose_n` | `float` | ENU North of the vehicle (meters). |
| `yaw` | `float` | Heading: CCW angle of body `+x` from ENU East (radians). |
| `config` | `ProjectConfig` | All tuning parameters (see §2). |
| `state` | `ProjectState \| None` | Monotonic-cursor state from the previous frame. `None` on the first frame triggers a global search. |
| `speed` | `float \| None` | Vehicle speed (m/s); accepted but not used by the current strategies. Reserved for future use. |

### Return value — `ProjectOutput`

| Field | Type | Description |
|-------|------|-------------|
| `path` | `list[list[float,float]]` | `[[x, y], …]` in body frame (`+x` forward, `+y` left, meters). Spans `[cursor_s − behind_m, cursor_s + ahead_m]`, clipped to route ends. |
| `cursor_s` | `float` | Arc-length progress (m) at this frame. Non-decreasing across frames. |
| `lat_dev` | `float` | Signed lateral deviation (m). Positive = vehicle is to the **left** of the route. |
| `matched_seg` | `int` | Index of the matched route segment. |
| `end_flag` | `bool` | True when `cursor_s + ahead_m >= route.length`. |
| `state` | `ProjectState` | Ready to pass back on the next frame. Contains `cursor_s` and `initialized=True`. |

### Body-frame convention

```
+x forward (vehicle nose direction)
+y left
```

Rotation from ENU offset `(d_e, d_n)` at heading `yaw`:

```
x_forward =  d_e · cos(yaw) + d_n · sin(yaw)
y_left    = −d_e · sin(yaw) + d_n · cos(yaw)
```

---

## 2. Configuration — `ProjectConfig`

```python
@dataclass
class ProjectConfig:
    strategy: str = "smoothed"       # "raw" | "centered" | "smoothed"
    behind_m: float = 5.0            # look-behind window (m)
    ahead_m: float = 70.0            # look-ahead window (m)
    sample_ds_m: float = 0.5         # path sampling step (m)
    search_ahead_m: float = 15.0     # forward search window for matching (m)
    search_back_m: float = 0.3       # back-tolerance for matching (m)
    heading_gate_deg: float = 60.0   # heading-agreement gate (degrees)
    min_turn_radius_m: float = 8.0   # corner radius used for smoothing (m); "smoothed" only
    corner_angle_deg: float = 10.0   # minimum corner angle to fillet (degrees); "smoothed" only
    simplify_eps_m: float = 0.20     # RDP simplification tolerance (m); "smoothed" only
    corner_style: str = "clothoid"   # "arc" | "clothoid"; "smoothed" only
    clothoid_transition_m: float = 4.0  # clothoid spiral transition length (m); "smoothed"+"clothoid" only
```

`SEARCH_AHEAD = 15.0` is the module-level default, matched by `search_ahead_m`.

**Smooth-by-default (updated 2026-07-04).** The defaults `min_turn_radius_m = 8.0`
and `clothoid_transition_m = 4.0` are chosen so corners look smooth without any
tuning (they engage on 100% of the real planned-route legs, which are ≥ 21 m).
The human ego-track calibration (`docs/clothoid_calibration.md`) measured a 1.5 m
entry ramp; the shipped default transition is raised to 4.0 m for a gentler feel.
Both remain live sliders.

**Corner is smoothed ONCE in world space (no per-frame jump).** For the
`"smoothed"` strategy the whole route is smoothed one time in the world frame and
cached (`_get_smoothed`); each frame re-windows and re-projects that fixed curve
rather than re-filleting a sliding window — so the corner shape is stable
frame-to-frame instead of jittering. `raw`/`centered` sample the original route
unchanged. Matching (`cursor_s`) is unchanged, so grading is unaffected.

---

## 3. Monotonic Matching — `_match` and `_best_in_range`

### Per-frame matching (not frame 0)

The candidate index set covers arc-lengths `[cursor_s − search_back_m, cursor_s + search_ahead_m]`.
Within that window, the point with the minimum squared Euclidean distance to the
measured position is selected, subject to a heading-agreement gate:

```
Δθ[i] = | ((θ_tangent[i] − yaw + π) mod 2π) − π |   ∈ [0, π]
```

The gate passes index `i` iff `Δθ[i] ≤ radians(heading_gate_deg)`. If no
candidate passes, the gate is widened to all candidates in the window (a frame is
never dropped).

### Frame 0 (cold start)

When `state is None` or `not state.initialized`, the search window is the whole
route. This one-time global search seeds the cursor.

### Monotonic cursor update

```python
cursor_s = max(state.cursor_s, route.s[matched_index])
```

The `max` guarantees `cursor_s` is non-decreasing regardless of measurement
noise, which resolves self-crossing ambiguity (see `algorithm_description.md §6`).

### Lateral deviation

Measured at the cursor point using the route's left normal `n_left = (−t_n, t_e)`:

```
lat_dev = (pose − matched_point) · n_left
```

Positive = vehicle to the left of the route.

---

## 4. The Three Strategies

All three strategies share the same monotonic match. They differ only in how the
output path is constructed.

### 4.1 `"raw"` — Keep the lateral offset

The route points in `[cursor_s − behind_m, cursor_s + ahead_m]` are rotated into
the body frame with no further adjustment. The matched route point appears at the
car's actual lateral offset, reflecting cross-track error visually.

### 4.2 `"centered"` — Remove the lateral offset (straight-path anchor)

The body-`y` coordinate of the matched anchor point (`lat_shift`) is subtracted
from every sample's `by`:

```python
_, lat_shift = to_body_frame(ax - pose_e, ay - pose_n, yaw)   # body-y of anchor
...
by -= lat_shift     # applied to every sample when strategy != "raw"
```

This re-anchors the path so the matched route point sits on the `y = 0` (forward)
axis, removing the translational offset while preserving heading error and
curvature. The behind-stub and the forward portion are both shifted.

### 4.3 `"smoothed"` — Remove offset + corner smoothing

Same lateral shift as `"centered"`, then the **forward portion** (`fwd_pts`, i.e.
points at `s ≥ cursor_s`) is passed through `smooth_corners` if it contains at
least 3 points:

```python
if cfg.strategy == "smoothed" and len(fwd_pts) >= 3:
    fwd_pts = smooth_corners(fwd_pts, cfg.min_turn_radius_m, cfg.corner_angle_deg,
                             cfg.sample_ds_m, cfg.simplify_eps_m,
                             corner_style=cfg.corner_style,
                             transition=cfg.clothoid_transition_m)
```

The corner shape is selected by `corner_style`:

- `"arc"` (circular-arc fillet) — constant curvature `1/R`; curvature jumps
  discontinuously at the entry tangent point.
- `"clothoid"` (**default**) — curvature-continuous Euler spiral (clothoid); curvature
  ramps linearly from 0 to `1/R` over `clothoid_transition_m` metres, holds at
  `1/R` through any remaining constant-curvature arc, then ramps back to 0. The
  `clothoid_transition_m = 1.5 m` default was calibrated from real human-driven
  ego tracks (see `docs/clothoid_calibration.md`). The clothoid falls back to an
  arc when the adjacent legs are too short to accommodate the full transition.

The behind-stub is **not** smoothed — it reflects where the car actually came
from. Only the actionable look-ahead is smoothed into a drivable path.

---

## 5. Corner Smoothing (`smoothing.py`)

### 5.1 Pipeline

`smooth_corners(pts, min_radius, corner_angle_deg, ds, eps, corner_style="arc", transition=3.0)` applies three steps.
(Note: the function's own defaults are `corner_style="arc", transition=3.0` for
direct callers; the user-facing default is `ProjectConfig.corner_style="clothoid"`
with `clothoid_transition_m=1.5`, which `project_route` passes in — see §2/§5.5.)

1. **RDP simplification** (`rdp(pts, eps)`) — reduces the densely sampled
   polyline to its geometric skeleton, so that arc fitting operates on real
   vertices rather than interpolated midpoints.
2. **Corner fillet** — replaces each sharp vertex with either a circular-arc
   fillet (`corner_style="arc"`) or a clothoid (Euler spiral) transition
   (`corner_style="clothoid"`). See §5.3 and §5.5.
3. **Uniform resample** (`resample(out, ds)`) — restores the `ds`-spaced
   sampling so the consumer always gets a uniform step.

### 5.2 RDP — Ramer-Douglas-Peucker

Standard recursive algorithm: for each sub-polyline, find the point of maximum
perpendicular distance from the chord. If `> eps`, split and recurse; otherwise,
keep only the endpoints. Tolerance `eps = simplify_eps_m` (default 0.20 m).

### 5.3 Arc-Fillet Math

For each interior vertex `V` with predecessor `A` and successor `B`:

**Turn angle and direction:**

```
d1 = unit(V − A)
d2 = unit(B − V)
δ  = acos(clamp(d1 · d2, −1, 1))       (unsigned turn angle)
cross = d1 × d2 = d1.x·d2.y − d1.y·d2.x   (> 0 → left turn)
```

If `δ < corner_angle_deg` (in radians), the vertex is not sharp enough to fillet;
it is passed through unchanged.

**Tangent length T:**

The standard geometric identity for a circular arc of radius `R` fitting a turn
of angle `δ` is `T = R · tan(δ/2)`. To guarantee the arc fits inside the
available legs:

```
T = min(R_min · tan(δ/2),
        0.5 · |V − A|,          # half the incoming leg length
        0.5 · |B − V|)          # half the outgoing leg length
```

The half-leg clamp prevents the arc from consuming more than half of any
adjacent leg, so consecutive fillets never collide.

**Effective radius:**

When the clamp is active (short leg), `T < R_min · tan(δ/2)`, and the effective
radius is reduced accordingly:

```
R_eff = T / tan(δ/2)
```

`R_eff ≥ R_min` when the half-leg clamp is inactive (normal case). In the
degenerate case where a leg is shorter than `2 · R_min · tan(δ/2)`, `R_eff` can
fall below `R_min` — this is documented as the expected short-leg caveat.

**Arc construction:**

Tangent point `p1` on the incoming leg:

```
p1 = V − T · d1
```

Arc center `C` (perpendicular from `p1` toward the inside of the turn):

```
n = (−d1.y, d1.x)   if cross >= 0  (left turn)
    ( d1.y, −d1.x)  if cross < 0   (right turn)
C = p1 + R_eff · n
```

The arc sweeps from angle `a1 = atan2(p1.y − C.y, p1.x − C.x)` through `δ`
radians (CCW for left turns, CW for right turns):

```
steps = max(1, ceil(R_eff · δ / ds))
a_k   = a1 + sign · δ · (k / steps),   k = 1 … steps
arc_k = (C.x + R_eff·cos(a_k),  C.y + R_eff·sin(a_k))
```

### 5.4 Curvature Guarantee

The curvature of each arc is `κ = 1 / R_eff`. Because `R_eff = T / tan(δ/2)`:

- When the half-leg clamp is **not** active: `T = R_min · tan(δ/2)`, so
  `R_eff = R_min` and `κ = 1 / R_min` — curvature is exactly at the limit.
- When the half-leg clamp **is** active (short leg): `R_eff < R_min` and
  `κ > 1 / R_min` — curvature can exceed the limit in this degenerate case.

For non-degenerate legs (length > `2 · R_min · tan(δ/2)`), the curvature of every
arc is guaranteed `κ ≤ 1 / R_min`, making the entire path drivable at a minimum
turning radius of `R_min = min_turn_radius_m` (default 5.0 m).

Straight segments between arcs have `κ = 0`.

### 5.5 Clothoid (Euler Spiral) Corner — `corner_style="clothoid"`

The default corner style is `"clothoid"`. A clothoid (Euler spiral) is a curve
whose curvature varies **linearly with arc length**, giving a jerk-continuous (C2)
path that matches natural human steering behaviour. The curvature profile for one
corner is:

```
κ(s) = s / (R · L_t)           for s ∈ [0, L_t]       (entry spiral, curvature ramps up)
κ(s) = 1 / R                   for s ∈ [L_t, L_t+L_a]  (optional constant-curvature arc)
κ(s) = (L_t+L_a + L_t − s) / (R · L_t)  for s ∈ [L_t+L_a, 2L_t+L_a]  (exit spiral)
```

where `L_t = clothoid_transition_m` is the spiral length (one side) and `L_a ≥ 0`
is any remaining constant-curvature arc. The turn angle of one spiral is
`θ_sp = L_t / (2R)`.

The `clothoid_transition_m = 1.5 m` default was determined by measuring the
**entry ramp length** of real vehicle cornering from human-driven ego tracks: for
each detected turn (κ > 1/15 m⁻¹), the arc length from the straight to the peak
curvature was measured and the median taken across all resolved turns. See
`docs/clothoid_calibration.md` for the full calibration table. The value is 1.5 m
at the median across seven datasets.

**Fit procedure.** For a corner with signed turn angle `δ` and legs of length
`l₁`, `l₂`, the clothoid is attempted at transition factors 1.0, 0.5, 0.25
(`L_t = factor · clothoid_transition_m`). The tangent length is taken from the
integrated spiral endpoint as `T = xe − ye / tan(δ)` (larger than the arc's
`R · tan(δ/2)` because the spirals bow the curve outward). The attempt succeeds
when `T ≤ 0.5 · min(l₁, l₂)` (the same half-leg clamp as the arc). If no factor
succeeds, the corner falls back to a circular-arc fillet (§5.3).

**Curvature continuity.** The clothoid entry begins at κ = 0 (straight) and ramps
to `κ = 1/R`; the exit ramps back to 0. There is no curvature jump at the entry or
exit tangent points. The peak curvature is `1/R = 1/min_turn_radius_m`, the same
bound as the arc.

### 5.6 "Revert to Straight After the Turn" Per-Frame Behavior

Because matching is re-run each frame and the forward window advances with the
car, the corner shape is **not frozen**: once the car moves past a corner, the
next frame's `fwd_pts` begins past that corner's tangent point. The smoothed
corner stops appearing in the output — the path reverts to a straight (or
next-corner) segment naturally without any explicit state machine. This is a
consequence of the pure-function design: no corner memory, no hysteresis.

---

## 6. Performance Characteristics

The function is designed for an embedded target running at 10 Hz:

| Property | Guarantee |
|----------|-----------|
| **Windowed search** | Match scans at most `(search_ahead_m + search_back_m) / route_density` points (≈ 153 points at default 0.1 m route density) — O(W/Δs), not O(L). |
| **Closed-form arcs** | No iterative convergence. `T`, `R_eff`, `C`, and arc points are all direct algebraic evaluations. |
| **Clothoid integration** | The clothoid spiral is integrated from its linear-curvature profile by a fixed-step loop (midpoint rule for position, trapezoid for heading) at a fine internal step (`INTERNAL_DS = 0.1 m`); output is resampled at `ds`. No `scipy`; the identical loop runs in Python and JS. |
| **No convergence loops** | RDP is O(N log N) in typical cases; arc fitting is O(V) where V = number of RDP vertices (typically small: 2–10 per 70 m segment). |
| **Deterministic timing** | The matching window has a fixed upper bound; the only variable-cost step is RDP whose depth is bounded by the polyline vertex count. |

---

## 7. JavaScript Twin — `viewer/project_route.js`

A DOM-free port of the same algorithm is exposed as `window.ProjectRoute`:

```
ProjectRoute.DEFAULT_CONFIG       // mirrors ProjectConfig defaults exactly
ProjectRoute.projectRoute(route, pose, cfg, state)   // main entry (pose = {e, n, h})
ProjectRoute.match(route, pe, pn, yaw, cfg, state)
ProjectRoute.rdp(pts, eps)
ProjectRoute.smoothCorners(pts, R, angleDeg, ds, eps, cornerStyle, transition)
ProjectRoute.clothoidCorner(delta, radius, transition, internalDs)
ProjectRoute.resample(pts, ds)
ProjectRoute.toBody(de, dn, yaw)
ProjectRoute.indexAtS(route, s)
ProjectRoute.pointAtS(route, s)
ProjectRoute.bestInRange(route, pe, pn, yaw, loS, hiS, gate)
ProjectRoute.buildRoute(points_e, points_n, s_opt, waypoint_indices_opt)
```

`DEFAULT_CONFIG` includes `corner_style: "clothoid"` and
`clothoid_transition_m: 1.5`, mirroring `ProjectConfig` defaults.

The JS twin uses a true-modulo helper `mod(a, m) = ((a % m) + m) % m` to match
Python/numpy's `%` behaviour in the heading-gate angular difference computation.
The clothoid is integrated by the same fixed-step loop in both Python and JS
(midpoint position, trapezoid heading, `INTERNAL_DS = 0.1 m`, `n = ceil(total /
INTERNAL_DS)`, `h = total / n`), so the two agree bit-for-bit.

**Parity guarantee:** `tests/e2e/test_parity_py_js.py` runs 60 cases (3 routes ×
5 poses × 4 strategy/corner-style combos) through both Python and the JS twin in
a headless Chromium browser, asserting path length, per-point coordinates
(tolerance 1e-3 m), `matched_seg`, and `end_flag` all agree.
