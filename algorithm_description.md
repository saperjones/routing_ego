# Algorithm Description — Parking Route Projection

This document describes the **mathematics** of the implemented system: the
coordinate transforms, the route parameterization, the stateful matching
algorithm, the self-crossing guarantee, the simulation noise model, and the
grading metric. It reflects the code in `src/parking_proj/`. Companion
document: `spec_design.md` (requirements and design in plain language).

Notation: vectors are lower-case bold in prose, written as pairs `(·, ·)` in
formulas. Positions are in a local ENU frame (East `e`, North `n`), meters.
Angles are radians unless a degree symbol is shown.

---

## 1. Frames and conventions

- **World:** WGS84 geodetic `(lat, lon)`.
- **Working frame (ENU):** East `e`, North `n`, Up `u`; planar `(e, n)` used.
- **Body frame:** `+x` forward, `+y` left, `+z` up (right-handed).
- **Heading** `h`: the angle of the body `+x` axis measured **counter-clockwise
  from ENU East**. So the unit forward vector is `(cos h, sin h)`.

---

## 2. WGS84 ⇄ ENU (`geo.py`)

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

## 3. Route parameterization (`route.py`)

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

## 4. Body-frame transform (`transform.py`)

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

## 5. The stateful projection algorithm (`projection.py`)

State carried between frames: the scalar **progress cursor** `σ_c` (`cursor_s`),
an arc-length position. Parameters (defaults): look-ahead `A = +20 m`,
look-behind `B = −5 m`, forward search `W = 3.5 m`, back tolerance
`ε_b = 0.3 m`, heading gate `γ = 60°`.

### 5.1 Angular gate

For a candidate index `i`, the heading disagreement is the wrapped absolute
difference between the route tangent angle and the vehicle heading:

```
θ[i] = atan2(t[i].n, t[i].e)
Δθ[i] = | ((θ[i] − h + π) mod 2π) − π |          ∈ [0, π]
```

The candidate passes the gate iff `Δθ[i] ≤ γ`.

### 5.2 Windowed match

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

### 5.3 Monotonic cursor update

Let `σ* = s[i*]`. The cursor advances but never retreats:

```
σ_c ← max(σ_c, σ*)
```

This single `max` is what guarantees acceptance criterion "0 backward jumps":
`σ_c` is non-decreasing by construction regardless of measurement noise.

### 5.4 Outputs (telemetry)

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
  the body frame by §4 using the measured pose.

The lateral deviation is measured at the **cursor point** (current progress),
not at an unconstrained nearest point. This is deliberate: a free nearest-point
distance would be ambiguous exactly at a self-crossing, reintroducing the
problem the cursor solves.

---

## 6. Why self-crossings resolve

Let a route cross itself at a world point `X` that is traversed twice, at
arc-lengths `σ₁ < σ₂` (`|σ₂ − σ₁|` is large — tens of meters in every scenario,
e.g. ≈ 91 m for the X-crossing). Suppose at some frame the true progress is near
`σ₁`, so `σ_c ≈ σ₁`.

The search window covers arc-lengths `[σ_c − ε_b, σ_c + W]` with `W = 3.5 m`.
Because `σ₂ − σ₁ ≫ W`, the *second* occurrence of `X` (arc-length `σ₂`) is
**outside the window**, so it cannot be selected — even though it is the same
world point and thus at zero Euclidean distance. On the later pass, `σ_c` has
advanced past the intervening waypoints to near `σ₂`, and now the first
occurrence at `σ₁` is far *behind* the window. The heading gate (§5.1) is a
secondary safeguard for the handful of frames adjacent to `X` where both
strokes momentarily fall in-window but point in different directions.

Two numeric facts make this robust: the longitudinal localization error is
capped at `2 m < W`, so noise cannot push the match across the ≫ `W` gap; and
`σ_c` is monotone, so once past `X` it cannot snap back.

---

## 7. Simulation model (`simulate.py`)

All randomness comes from a seeded generator `rng = default_rng(seed)`, making
every case bit-reproducible. Speed `v = v_kmh / 3.6` m/s, rate `f = 10 Hz`, step
`ds = v/f` (≈ 0.222 m at 8 km/h).

### 7.1 Ground-truth ordinal walk

Frame count `F = ⌊L/ds⌋ + 1`. The intended (ground-truth) progress and segment
are exact by construction:

```
gt_s[k] = min(k · ds, L)
nom[k]  = P(gt_s[k])                 (centerline point at that arc-length)
gt_seg[k] = segment_at_s(gt_s[k])
```

`gt_s` is monotone non-decreasing; `gt_seg` is unambiguous even at crossings
because it comes from the ordinal walk, not a nearest-point search.

### 7.2 Corner rounding (low-pass of the centerline)

A normalized Gaussian kernel of odd width `w` (σ_k = w/5) is convolved with the
nominal path coordinates to round corners (the car cannot turn instantaneously):

```
smooth_e = (nom_e ∗ g_w),   smooth_n = (nom_n ∗ g_w)
```

The convolution returns an array the same length as its input. The first and
last `a = min(w, ⌊F/2⌋)` samples are re-anchored to the raw nominal values so
the endpoints are not pulled inward (and the head/tail anchors never overlap).

### 7.3 Colored (low-pass) noise primitive

For a target standard deviation `σ` and cap `c`, `lowpass_noise(F, w, σ, c)`:

```
draw white  ω ~ N(0,1)^F
s = ω ∗ g_w                  (Gaussian-smoothed → temporally correlated)
s ← s · (σ / std(s))         (rescale to target σ)
return clip(s, −c, c)
```

This yields a smooth, slowly-drifting, bounded sequence — the realistic shape of
RTK drift, not per-frame white jitter.

### 7.4 True trajectory (imperfect tracking)

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

### 7.5 Measured pose (RTK error)

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

## 8. Grading metric (`grade.py`)

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
which is tens of meters away in arc-length (§6) → `|σ_c − gt_s|` is huge → it is
counted. A benign *segment-boundary timing* error — the cursor sitting on the
physically adjacent segment for a few frames because injected longitudinal noise
(≤ 2 m) offset it across a corner — has `|σ_c − gt_s| ≤ 2 m < 3 m` → it is not
counted. The threshold sits above the 2 m noise band and far below the
inter-stroke gap, so it discriminates cleanly. (This refines the spec's original
literal "≤ 3 frames" segment-equality rule; see `spec_design.md` §6 note.)

**True lateral deviation** (a reference the noise-free tracking error), computed
at the true arc-length with the same left-normal convention as §5.4:

```
true_lat_dev[k] = ( true_pos[k] − P(gt_s[k]) ) · nˡ(gt_s[k])
```

**Pass condition** for a case:

```
mismatches ≤ 3   AND   backward_jumps = 0   AND   dropouts = 0
```

All 14 test cases pass with 0 mismatches, 0 backward jumps, and 0 dropouts.

---

## 9. Summary of guarantees

- **Monotone progress** is algebraically guaranteed by the `max` in §5.3.
- **Correct stroke at self-crossings** follows from the window width `W` being
  far smaller than the inter-stroke arc-length gap, plus the heading gate and
  the 2 m error cap (§6).
- **Bounded output** — the emitted slice is always clipped to the real route
  (§5.4); no frame is dropped (§5.2).
- **Reproducibility** — a single seeded generator drives all noise (§7), so
  regenerating any case is bit-identical.
