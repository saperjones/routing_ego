# Algorithm Description — Parking Route Projection

> **Update 2026-07-04:** smoothing is applied **once in world space** to the whole route (not per-frame on the body-frame window), giving a temporally stable corner; shipped defaults `min_turn_radius_m = 8.0`, `clothoid_transition_m = 8.0` (calibration value 1.5 m).


This document leads with the **main algorithm function** and its input/output
format (§1), then describes the **auxiliary functions** and the **mathematics**
it relies on: the coordinate transforms, the route parameterization, the
stateful matching algorithm, the self-crossing guarantee, the simulation noise
model, and the grading metric. It reflects the code in `src/parking_proj/`.
Companion documents: `spec_design.md` (requirements and design in plain
language) and `function_intro.md` (a plain-language delivery note for the same
function).

Notation: vectors are lower-case bold in prose, written as pairs `(·, ·)` in
formulas. Positions are in a local ENU frame (East `e`, North `n`), meters.
Angles are radians unless a degree symbol is shown.

---

## 1. Main algorithm function — `project_route`

The single entry point is one **pure** function in `src/parking_proj/project_route.py`:

```
project_route(route, pose_e, pose_n, yaw, config, state=None, speed=None) -> ProjectOutput
```

It converts a globally-planned route (local ENU frame) into the path **as seen
from the driver's seat** (the vehicle-body frame) for the current frame. It
holds no hidden state: the caller keeps `state` and passes it back each frame,
so the same inputs always produce the same output. All auxiliary functions
below (matching, transforms, smoothing) exist only to serve this call.

### 1.1 Input format

| Argument | Type / format | Meaning |
|----------|---------------|---------|
| `route` | prepared polyline object: `.points` (N×2 array of `(e, n)`), `.s` (cumulative arc-length), `.tangents` (unit direction per point), `.length`, and lookups `.point_at_s(s)` / `.index_at_s(s)` / `.seg_of_index` | the planned route in local ENU meters (built once, immutable) — see §4 |
| `pose_e`, `pose_n` | float (meters, ENU) | the vehicle position |
| `yaw` | float (radians, CCW from ENU East) | the vehicle heading — see §2 |
| `config` | `ProjectConfig` | all tuning knobs: `strategy`, `behind_m` (5.0), `ahead_m` (40.0), `sample_ds_m` (0.5), `search_ahead_m` (15.0), `search_back_m` (0.3), `heading_gate_deg` (60.0), `min_turn_radius_m` (8.0), `corner_angle_deg` (10.0), `simplify_eps_m` (0.20), `corner_style` (`"clothoid"`), `clothoid_transition_m` (8.0), `human_cut_m` (2.2) |
| `state` | `ProjectState(cursor_s, initialized)` or `None` | progress carried between frames; pass `None` on the first frame to seed a global search |
| `speed` | float (m/s) or `None` | accepted but unused by current strategies; reserved |

### 1.2 Output format — `ProjectOutput`

| Field | Format | Meaning |
|-------|--------|---------|
| `path` | list of `[x, y]` in the body frame (`+x` forward, `+y` left), spanning `[cursor_s − behind_m, cursor_s + ahead_m]`, sampled at `sample_ds_m` | the driver-view path the controller follows — see §10.4 |
| `cursor_s` | float (arc-length, meters) | current progress along the route; **never decreases** across frames — see §6.3 |
| `lat_dev` | float (meters) | signed cross-track deviation, **positive = vehicle to the left of the route** — see §6.4 |
| `matched_seg` | int | index of the matched route segment |
| `end_flag` | bool | `True` when `cursor_s + ahead_m` reaches the route end |
| `state` | `ProjectState` | the state to hand back next frame |

### 1.3 Core idea and auxiliary functions

Each frame does two steps: **(a) match** — advance the monotonic cursor `cursor_s`
by a windowed, heading-gated nearest-point search (§6); **(b) build** the
body-frame path under the selected `ProjectConfig.strategy` — one of `raw` /
`centered` / `smoothed` / `human` / `human_centered` (§10.2), with corner
smoothing done **once in world space** so the corner is temporally stable
(§10.3). The auxiliary functions this call relies on:

- `geo.py` — WGS84 ⇄ ENU conversion (§3)
- `route.py` — arc-length, tangents, segment lookup (§4)
- `transform.py` — `to_body_frame` world→body rotation (§5)
- `projection.py` — the stateful windowed match and telemetry (§6)
- `smoothing.py` / `clothoid.py` — `rdp`, `resample`, `smooth_corners`, `human_corners`, `clothoid_corner` corner geometry (§10.3)
- `viewer/project_route.js` — the DOM-free JavaScript twin, held to the Python output by parity tests (§10.6)

The sections below describe each of these in detail.

---

## 2. Frames and conventions

- **World:** WGS84 geodetic `(lat, lon)`.
- **Working frame (ENU):** East `e`, North `n`, Up `u`; planar `(e, n)` used.
- **Body frame:** `+x` forward, `+y` left, `+z` up (right-handed).
- **Heading** `h`: the angle of the body `+x` axis measured **counter-clockwise
  from ENU East**. So the unit forward vector is `(cos h, sin h)`.

---

## 3. WGS84 ⇄ ENU (`geo.py`)

A local equirectangular tangent-plane projection about a fixed origin
`(lat₀, lon₀)` (a Hefei point). With Earth radius `R = 6 378 137 m` and
`deg = π/180`:

**Forward (geodetic → ENU):**

```
e = (lon − lon₀) · deg · R · cos(lat₀ · deg)
n = (lat − lat₀) · deg · R
```

**Inverse (ENU → geodetic):**

```
lat = lat₀ + n / (R · deg)
lon = lon₀ + e / (R · cos(lat₀ · deg) · deg)
```

The inverse is the exact algebraic inverse of the forward map, so the
round-trip error is at machine precision (≪ 1 mm) at parking-lot scale. This
approximation is valid because the working region spans only a few hundred
meters, where the meridian/parallel scale is effectively constant.

**Heading convention conversion.** A compass heading `hₙ` (degrees clockwise
from North) maps to the internal yaw (CCW from East) by

```
h = (90° − hₙ) · deg          (radians)
```

which is its own inverse in form (`hₙ = 90° − h/deg`).

---

## 4. Route parameterization (`route.py`)

A route is a dense polyline of points `P[0..N−1]`, spaced ≈ `0.1 m`.

**Arc-length.** With segment vectors `Δ[i] = P[i+1] − P[i]`:

```
s[0] = 0,   s[i] = Σ_{j<i} ‖Δ[j]‖,   L = s[N−1]  (route length)
```

`s` is strictly increasing for a non-degenerate polyline, so it is invertible;
`index_at_s(σ)` returns the last index with `s[i] ≤ σ` (a binary search),
clamped to `[0, N−1]`.

**Unit tangents.** Using a combined forward/backward difference (central in the
interior):

```
τ̃[i] = Δ[i−1] + Δ[i]   (with the missing term dropped at the ends)
t[i]  = τ̃[i] / ‖τ̃[i]‖
```

**Segments.** Waypoints (the human-numbered corners `1,2,…`) sit at indices
`w₀ < w₁ < … < w_M`. Point `i` belongs to segment `k` where
`w_k ≤ i < w_{k+1}`; a point exactly on an interior waypoint belongs to the
**entering** (next) segment. `segment_at_s(σ) = seg_of_index(index_at_s(σ))`.

---

## 5. Body-frame transform (`transform.py`)

Rotating a world offset into the body frame is a rotation by `−h` composed with
the axis convention `+x` forward / `+y` left. For a world offset
`d = P − pose = (d_e, d_n)` and heading `h`:

```
x_forward =  d_e · cos h + d_n · sin h
y_left    = −d_e · sin h + d_n · cos h
```

Equivalently, with the orthonormal body axes
`forward = (cos h, sin h)` and `left = (−sin h, cos h)`,
`x_forward = d · forward` and `y_left = d · left`. This is an orthonormal
(distance-preserving) rotation; it is the fixed display transform the viewer
also uses. Sanity values: at `h = 0`, East→forward and North→left; at
`h = 90°`, North→forward and East→right (`y_left = −1`).

---

## 6. The stateful projection algorithm (`projection.py`)

State carried between frames: the scalar **progress cursor** `σ_c` (`cursor_s`),
an arc-length position. Parameters (defaults): look-ahead `A = +20 m`,
look-behind `B = −5 m`, forward search `W = 15 m`, back tolerance
`ε_b = 0.3 m`, heading gate `γ = 60°`.

### 6.1 Angular gate

For a candidate index `i`, the heading disagreement is the wrapped absolute
difference between the route tangent angle and the vehicle heading:

```
θ[i] = atan2(t[i].n, t[i].e)
Δθ[i] = | ((θ[i] − h + π) mod 2π) − π |          ∈ [0, π]
```

The candidate passes the gate iff `Δθ[i] ≤ γ`.

### 6.2 Windowed match

Given the measured pose `(p, h)` with `p = (p_e, p_n)`, define the candidate
index set from the arc-length window

```
I = { i : index_at_s(σ_c − ε_b) ≤ i ≤ index_at_s(σ_c + W) }
```

(For frame 0 — the **seed** — the window is the whole route, `I = {0..N−1}`.)
Among gated candidates, pick the nearest point by squared Euclidean distance:

```
G = { i ∈ I : Δθ[i] ≤ γ }
i* = argmin_{i ∈ G} ‖P[i] − p‖²
```

If `G = ∅` (no candidate passes the gate — e.g. a hard corner), the gate is
**widened to include all of `I` for that frame only**, a flag `gate_widened`
is set, and `i*` is chosen over `I`. A frame is never dropped.

### 6.3 Monotonic cursor update

Let `σ* = s[i*]`. The cursor advances but never retreats:

```
σ_c ← max(σ_c, σ*)
```

This single `max` is what guarantees acceptance criterion "0 backward jumps":
`σ_c` is non-decreasing by construction regardless of measurement noise.

### 6.4 Outputs (telemetry)

Let `c = index_at_s(σ_c)`, matched point `m = P[c]`, tangent `t_c = t[c]`, and
the **left normal** `nˡ = (−t_c.n, t_c.e)`.

- **Signed lateral deviation** (positive = vehicle to the *left* of the route):

  ```
  est_lat_dev = (p − m) · nˡ
  ```

- **Matched segment:** `matched_seg = seg_of_index(c)`.
- **End-of-route flag:** `end_flag = (σ_c + A ≥ L)`.
- **Emitted slice** (rendered downstream): the route points with arc-length in
  `[σ_c + B, σ_c + A]`, clipped to `[0, L]` (no extrapolation), each mapped to
  the body frame by §5 using the measured pose.

The lateral deviation is measured at the **cursor point** (current progress),
not at an unconstrained nearest point. This is deliberate: a free nearest-point
distance would be ambiguous exactly at a self-crossing, reintroducing the
problem the cursor solves.

---

## 7. Why self-crossings resolve

Let a route cross itself at a world point `X` that is traversed twice, at
arc-lengths `σ₁ < σ₂` (`|σ₂ − σ₁|` is large — tens of meters in every scenario,
e.g. ≈ 91 m for the X-crossing). Suppose at some frame the true progress is near
`σ₁`, so `σ_c ≈ σ₁`.

The search window covers arc-lengths `[σ_c − ε_b, σ_c + W]` with `W = 15 m`
(the **endurable offset** — the largest along-track jump the projection can
absorb before the anchor falls behind the vehicle; configurable, default 15 m).
Because `σ₂ − σ₁ ≫ W` (tens of metres — 64–104 m across the scenarios), the
*second* occurrence of `X` (arc-length `σ₂`) is
**outside the window**, so it cannot be selected — even though it is the same
world point and thus at zero Euclidean distance. On the later pass, `σ_c` has
advanced past the intervening waypoints to near `σ₂`, and now the first
occurrence at `σ₁` is far *behind* the window. The heading gate (§6.1) is a
secondary safeguard for the handful of frames adjacent to `X` where both
strokes momentarily fall in-window but point in different directions.

Two numeric facts make this robust: the longitudinal localization error is
capped at `2 m < W`, so noise cannot push the match across the ≫ `W` gap; and
`σ_c` is monotone, so once past `X` it cannot snap back.

---

## 8. Simulation model (`simulate.py`)

All randomness comes from a seeded generator `rng = default_rng(seed)`, making
every case bit-reproducible. Speed `v = v_kmh / 3.6` m/s, rate `f = 10 Hz`, step
`ds = v/f` (≈ 0.222 m at 8 km/h).

### 8.1 Ground-truth ordinal walk

Frame count `F = ⌊L/ds⌋ + 1`. The intended (ground-truth) progress and segment
are exact by construction:

```
gt_s[k] = min(k · ds, L)
nom[k]  = P(gt_s[k])                 (centerline point at that arc-length)
gt_seg[k] = segment_at_s(gt_s[k])
```

`gt_s` is monotone non-decreasing; `gt_seg` is unambiguous even at crossings
because it comes from the ordinal walk, not a nearest-point search.

### 8.2 Corner rounding (low-pass of the centerline)

A normalized Gaussian kernel of odd width `w` (σ_k = w/5) is convolved with the
nominal path coordinates to round corners (the car cannot turn instantaneously):

```
smooth_e = (nom_e ∗ g_w),   smooth_n = (nom_n ∗ g_w)
```

The convolution returns an array the same length as its input. The first and
last `a = min(w, ⌊F/2⌋)` samples are re-anchored to the raw nominal values so
the endpoints are not pulled inward (and the head/tail anchors never overlap).

### 8.3 Colored (low-pass) noise primitive

For a target standard deviation `σ` and cap `c`, `lowpass_noise(F, w, σ, c)`:

```
draw white  ω ~ N(0,1)^F
s = ω ∗ g_w                  (Gaussian-smoothed → temporally correlated)
s ← s · (σ / std(s))         (rescale to target σ)
return clip(s, −c, c)
```

This yields a smooth, slowly-drifting, bounded sequence — the realistic shape of
RTK drift, not per-frame white jitter.

### 8.4 True trajectory (imperfect tracking)

The unit left-normal of the smoothed centerline, from its numerical gradient
`(ė, ṅ)`, is `perp = (−ṅ, ė)/‖(ė,ṅ)‖`. A lateral offset
`o = lowpass_noise(F, 4w, 0.15, 0.4)` is applied along it:

```
true[k] = (smooth_e[k], smooth_n[k]) + o[k] · perp[k]
true_yaw[k] = atan2( d/dk true_n , d/dk true_e )
```

So the true heading follows the tangent of the actual (offset, rounded) path,
and the tracking error has 1σ ≈ 0.15 m, hard-capped at 0.4 m, identical across
tiers.

### 8.5 Measured pose (RTK error)

Per tier, lateral 1σ ∈ {0.10, 0.50, 1.50} m and angular 1σ ∈ {0.01, 0.03,
0.05}°. Independent low-pass biases on each axis plus small white noise:

```
bias_e = lowpass_noise(F, 6w, σ_lat, 2.0),   bias_n likewise
white_e, white_n ~ N(0, 0.02²)
err = (bias_e + white_e, bias_n + white_n)
```

The **magnitude cap** enforces the 2 m position bound while preserving
direction:

```
ρ = ‖err‖;   scale = min(1, 2.0 / max(ρ, 10⁻⁹))
meas_pos[k] = true[k] + scale · err[k]
```

Heading is perturbed by a low-pass bias clipped to the angular cap:

```
meas_yaw[k] = true_yaw[k] + clip( lowpass_noise(F, 6w, σ_ang, c_ang), −c_ang, c_ang ),   c_ang = 0.05° in rad
```

Pitch and roll are generated the same way (capped ±0.05°) for display only; the
algorithm never reads them. The algorithm sees only `(meas_pos, meas_yaw)`.

---

## 9. Grading metric (`grade.py`)

Ground truth per frame is `(gt_s, gt_seg)`; the algorithm produces
`(σ_c, matched_seg)`. Three quantities are computed over a run:

**Backward jumps** — count of frames where the cursor decreased:
`#{ k : σ_c[k] < σ_c[k−1] − 10⁻⁶ }`. Must be `0`.

**Dropouts** — frames with no valid result. Must be `0`.

**Along-track-aware branch mismatches.** A frame is a mismatch iff the result
is missing, or the segment is wrong **and** the along-track error exceeds a
tolerance `BRANCH_TOL = 3.0 m`:

```
mismatch(k) = (result[k] is None)  OR
              ( matched_seg[k] ≠ gt_seg[k]  AND  |σ_c[k] − gt_s[k]| > 3.0 )
```

Rationale. A genuine *wrong-stroke jump* puts the cursor on a different stroke,
which is tens of meters away in arc-length (§7) → `|σ_c − gt_s|` is huge → it is
counted. A benign *segment-boundary timing* error — the cursor sitting on the
physically adjacent segment for a few frames because injected longitudinal noise
(≤ 2 m) offset it across a corner — has `|σ_c − gt_s| ≤ 2 m < 3 m` → it is not
counted. The threshold sits above the 2 m noise band and far below the
inter-stroke gap, so it discriminates cleanly. (This refines the spec's original
literal "≤ 3 frames" segment-equality rule; see `spec_design.md` §7 note.)

**True lateral deviation** (a reference the noise-free tracking error), computed
at the true arc-length with the same left-normal convention as §6.4:

```
true_lat_dev[k] = ( true_pos[k] − P(gt_s[k]) ) · nˡ(gt_s[k])
```

**Pass condition** for a case:

```
mismatches ≤ 3   AND   backward_jumps = 0   AND   dropouts = 0
```

All 14 test cases pass with 0 mismatches, 0 backward jumps, and 0 dropouts.

---

## 10. Output strategies and corner smoothing (`project_route.py`)

This section details the path-building step of the main function `project_route`
(§1) — after the match of §6 fixes `cursor_s`, the body-frame output path is
built. **Five** output strategies are selectable via `ProjectConfig.strategy`
(§10.2).

### 10.1 Lateral shift (common to `"centered"` and `"smoothed"`)

Let `P = route.point_at_s(cursor_s)`. Compute the body-`y` of the anchor
relative to the measured pose using the §5 rotation:

```
(_, lat_shift) = to_body_frame(P_e − pose_e, P_n − pose_n, yaw)
lat_shift = −(P_e − pose_e)·sin(yaw) + (P_n − pose_n)·cos(yaw)
```

For every sampled point `Q = route.point_at_s(s')` the raw body coordinates are:

```
(bx, by) = to_body_frame(Q_e − pose_e, Q_n − pose_n, yaw)
```

For `"centered"` and `"smoothed"`, `by − lat_shift` is used instead of `by`.
This places the anchor at `body-y = 0`, removing the translational cross-track
offset while preserving heading error and curvature.

### 10.2 Strategies

| Strategy | Lateral shift | Corner smoothing |
|----------|--------------|-----------------|
| `"raw"` | no | none |
| `"centered"` | yes (§10.1) | none |
| `"smoothed"` | yes (§10.1) | forward portion filleted: `driver` / clothoid (default) / arc, per `corner_style` (§10.3) |
| `"human"` | yes (§10.1) | inside corner-cut by `human_cut_m · (δ / 90°)` then Gaussian-smoothed (`smoothing.human_corners`) |
| `"human_centered"` | vehicle **projected onto** the `human` curve (nearest point within `±search_ahead_m`), frame oriented along the curve tangent → path points straight forward (offset **and** heading neutralized) | same as `"human"` |

The **behind-stub** (samples with `s' < cursor_s`) is never smoothed regardless
of strategy. The behind-stub reflects where the car actually came from; smoothing
it would misrepresent history.

### 10.3 Corner smoothing (`smoothing.smooth_corners`)

The forward portion of `"smoothed"` is processed by
`smooth_corners(pts, min_radius, corner_angle_deg, ds, eps, corner_style, transition)` in three steps:

**Step 1: RDP simplification.** `rdp(pts, eps)` (Ramer-Douglas-Peucker, tolerance
`simplify_eps_m = 0.20 m` default) reduces the `ds`-spaced polyline to its
geometric skeleton. This ensures corner fitting operates on real geometric vertices
rather than interpolated midpoints.

**Step 2: Corner fillet** — shape selected by `corner_style`.

#### 10.3.1 Circular-arc fillet (`corner_style="arc"`)

For each interior RDP vertex `V` with predecessor `A` and successor `B`:

```
d1 = unit(V − A),   d2 = unit(B − V)
δ  = acos(clamp(d1 · d2, −1, 1))          (unsigned turn angle)
```

If `δ < radians(corner_angle_deg)`, the vertex is skipped (not sharp enough).
Otherwise:

```
T = min(R_min · tan(δ/2),
        0.5 · |V − A|,           # half incoming leg
        0.5 · |B − V|)           # half outgoing leg
R_eff = T / tan(δ/2)
```

Tangent point on incoming leg: `p1 = V − T · d1`.
Arc center: `C = p1 + R_eff · n` where `n` is the inward perpendicular
(`n = (−d1.y, d1.x)` for left turns, `(d1.y, −d1.x)` for right turns).
Arc sweep from `a1 = atan2(p1.y − C.y, p1.x − C.x)` through angle `δ`
with `steps = max(1, ceil(R_eff · δ / ds))` arc points.

**Curvature guarantee.** When neither half-leg clamp is active,
`T = R_min · tan(δ/2)` and `R_eff = R_min`, so `κ = 1/R_eff = 1/R_min`.
Curvature on all non-degenerate arcs satisfies `κ ≤ 1/min_turn_radius_m`.
Degenerate case: if a leg is shorter than `2 · R_min · tan(δ/2)`, the
half-leg clamp reduces `R_eff` below `R_min`.

Note: the circular arc has a **curvature discontinuity** at the tangent points
`p1` and `p2` — curvature jumps from 0 (straight) to `1/R` (arc) in one step.

#### 10.3.2 Clothoid (Euler spiral) corner (`corner_style="clothoid"`, default)

A clothoid is a curve whose curvature is **linear in arc length**, giving a
jerk-continuous (C2) path. Curvature profile for one corner:

```
κ(s) = s / (R · L_t)                                  s ∈ [0, L_t]          (entry spiral)
κ(s) = 1/R                                             s ∈ [L_t, L_t+L_a]   (arc, if any)
κ(s) = (2L_t + L_a − s) / (R · L_t)                   s ∈ [L_t+L_a, 2L_t+L_a]  (exit spiral)
```

where `L_t = clothoid_transition_m` (one-sided transition length, default 1.5 m,
data-calibrated — see `docs/clothoid_calibration.md`) and `L_a ≥ 0`.

The half-turn angle of one spiral:

```
θ_sp = L_t / (2R)
```

The total turn angle is `δ = 2θ_sp + L_a/R`. Given `δ` and `R`, the arc length
is `L_a = max(0, R·δ − 2·θ_sp·R) = max(0, R·δ − L_t)`.

The local clothoid polyline is built by integrating the linear-curvature profile
`κ(s)` with a **fixed-step loop** (midpoint rule for position, trapezoid for
heading) at a fine internal step `INTERNAL_DS = 0.1 m` — `n = ceil(total /
INTERNAL_DS)`, `h = total / n`. There is **no** `scipy`, Simpson's rule, or
Fresnel table; the identical loop runs in Python (`clothoid.py`) and JS
(`project_route.js`), which is what makes the two agree bit-for-bit.

The tangent length is taken from the integrated endpoint `(xe, ye)` as the
intersection of the incoming (+x axis) and outgoing tangent lines:
`T = xe − ye / tan(δ)`. For a full spiral–arc–spiral corner this is **larger**
than the arc's `R · tan(δ/2)` (e.g. ~15% larger for a 90° turn at R = 5 m),
because the spirals push the curve outward. The resulting local polyline is
rotated and translated to the world frame at `p1 = V − T · d1`.

**Fit procedure.** The clothoid is attempted at transition factors 1.0, 0.5, 0.25
(`L_t = factor · clothoid_transition_m`). The attempt succeeds when
`T ≤ 0.5 · min(l₁, l₂)` (the same half-leg clamp as the arc). If no factor
succeeds, the corner falls back to a circular-arc fillet (§10.3.1).

**Curvature continuity.** The clothoid begins and ends at κ = 0 (tangent to the
straight legs). There is no curvature jump at entry or exit. Peak curvature is
`1/R = 1/min_turn_radius_m`, the same bound as the arc.

**Calibrated default.** The `clothoid_transition_m = 1.5 m` was measured from
real ego tracks: the median entry-ramp length (arc length from straight to peak
curvature) across resolved turns in seven human-driven parking datasets.

**Step 3: Uniform resample.** `resample(out, ds)` restores `ds`-spaced sampling
so the consumer always receives a uniform step.

### 10.4 Per-frame path structure

`ProjectOutput.path` is a flat list `[[x, y], …]` covering:

```
s ∈ [max(cursor_s − behind_m, 0),  min(cursor_s + ahead_m, route.length)]
```

sampled at `sample_ds_m` (default 0.5 m). Points with `s < cursor_s` form the
behind-stub (unsmoothed); points with `s ≥ cursor_s` form the forward portion
(smoothed for `"smoothed"` strategy). No extrapolation beyond route ends.

### 10.5 Relationship to telemetry

`est_lat_dev`, `cursor_s`, and `matched_seg` are computed by the matching step
before path construction and are independent of the chosen strategy. Changing
`strategy`, `corner_style`, or any path parameter never affects the matching
decisions.

### 10.6 JavaScript twin

`viewer/project_route.js` is a DOM-free port exposed as `window.ProjectRoute`.
It implements the same matching (`bestInRange`, `match`), body-frame rotation
(`toBody`), clothoid integral (`clothoidCorner`), and smoothing
(`rdp`, `smoothCorners`, `resample`) as the Python reference. The heading-gate
angular difference uses a true-modulo helper to match Python/numpy `%` semantics.
Parity is enforced by `tests/e2e/test_parity_py_js.py`
(60 cases: 3 routes × 5 poses × 4 strategy/corner-style combos; tolerance 1e-3 m
per point).

---

## 11. Summary of guarantees

- **Monotone progress** is algebraically guaranteed by the `max` in §6.3.
- **Correct stroke at self-crossings** follows from the window width `W` being
  far smaller than the inter-stroke arc-length gap, plus the heading gate and
  the 2 m error cap (§7).
- **Bounded output** — the emitted slice is always clipped to the real route
  (§6.4); no frame is dropped (§6.2).
- **Reproducibility** — a single seeded generator drives all noise (§8), so
  regenerating any case is bit-identical.
- **Drivable corners** — for non-degenerate legs, the `"smoothed"` strategy
  produces a forward path with peak curvature `κ ≤ 1/min_turn_radius_m` (§10.3).
  With `corner_style="clothoid"` (default), the path is additionally
  curvature-continuous (no entry/exit snap).
- **Python↔JS parity** — `viewer/project_route.js` produces numerically
  identical output to `project_route.py` (tolerance 1e-3 m) for all
  strategy/corner-style combinations, verified by `tests/e2e/test_parity_py_js.py`.
