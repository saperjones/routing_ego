# Design Spec: Parking Navigation Route Projection Algorithm

**Status:** finalized (requirements + hardened design + confirmed tests)
**Deliverables:** Python projection algorithm + simulation harness + static HTML visualization tool
**Implementation handoff:** `superpowers:writing-plans` (this spec contains no implementation code)

---

## 1. Requirements Summary

In a low-speed parking auto-driving system, a globally-planned navigation route (WGS84, sampled every 0.1 m along street centerlines) must be re-expressed **from the vehicle-body / driver's viewpoint** each frame so a downstream path-following controller can drive it. The core challenge is doing this **robustly** — correctly tracking *where on the route the car is* — even when the route **crosses itself** and under **RTK localization error**.

**Operating envelope:** above-ground parking lots with GPS/RTK, 6–10 km/h, Hefei (local ENU anchored at a Hefei origin).

**Explicit non-goals:**
- No pose/heading **smoothing or filtering** (a prior module handles that; this project must not add smoothing).
- Not building the downstream controller.
- No underground/no-GPS, no non-RTK localization.
- No re-walking a road and no U-turn-and-repeat routes (disallowed by product definition; the monotonic-progress design relies on this guarantee).
- No camera-calibrated perspective; a **default pinhole** model powers a nice-to-have windshield view only.

**Coordinate conventions (authoritative):**
- **Global:** WGS84.
- **Working frame:** local **ENU** (East = e, North = n, meters), anchored at a chosen Hefei lat/lon. All algorithm math happens here; WGS84 ⇄ ENU conversion is isolated in `geo.py`.
- **Vehicle body frame (matches `JSON_FIELD_DESCRIPTION_V7` ego(curr)):** **`+x` forward, `+y` left, `+z` up**, right-handed, meters. This is the output frame consumed downstream.
- **Vehicle heading `h`:** CCW angle of the vehicle's forward (`+x` body) axis from ENU East. Any compass-heading-from-north is converted to this in `geo.py`.
- 2D planar model (e, n, h); pitch/roll are negligible/cosmetic (±0.05° cap).

---

## 2. Chosen Approach & Rationale

**Two clean halves meeting at one JSON artifact:**
- A **Python generator** (numpy) that runs the algorithm + simulation **offline** and prebakes every test case to JSON.
- A **static HTML viewer** that plays back the JSON. It contains **no matching-algorithm logic**, which structurally enforces the "no recompute on click / no randomness between views" requirement.

**Why:** the projection algorithm lives in testable Python and is exercised against ground truth; the viewer can't diverge from it or re-randomize because the stateful algorithm simply doesn't exist in JS. The only viewer-side computation is a **fixed, stateless display rotation** (rendering math, not matching).

**Core algorithmic idea — stateful monotonic progress:** the algorithm carries a **progress cursor** (`cursor_s`, arc-length along the route). Each frame it matches only within a **bounded forward window** near the previous cursor, advances the cursor **monotonically** (never decreasing — valid because no road is re-walked and there are no U-turns), and uses a **heading-agreement gate** as a secondary tie-breaker. This resolves self-crossing ambiguity that a stateless nearest-point search cannot.

**Alternatives rejected:**
- Stateless global nearest-point search — genuinely ambiguous at self-crossings (the exact failure this project targets).
- Re-implementing the algorithm in JS — two diverging implementations, harder to test.
- Baking the full body-frame slice into JSON every frame — redundant and heavy; superseded by storing the algorithm's *decisions* and rendering the slice viewer-side.

---

## 3. Architecture / Components / Data Flow

### 3.1 File layout

```
routing_ego/
├── src/                         # Python core (numpy)
│   ├── geo.py                   # WGS84 ⇄ local ENU (Hefei origin); heading→math-yaw
│   ├── route.py                 # Route value object: dense 0.1 m polyline, arc-length, tangents, waypoint labels
│   ├── projection.py            # THE algorithm: stateful projector (progress cursor)
│   ├── simulate.py              # two-layer sim: tracking layer + RTK error layer, 10 Hz
│   ├── scenarios.py             # test-case matrix: geometry + tier configs + fixed seeds
│   └── generate.py              # runs all scenarios → JSON per case + index.json; computes verdicts
├── viewer/
│   ├── index.html               # single-page viewer
│   ├── viewer.js                # loads JSON; panorama/BEV/driver-view; playback
│   └── viewer.css
├── out/                         # generated JSON (git-ignored): index.json + <case_id>.json
└── tests/                       # pytest: algorithm correctness vs ground truth
```

### 3.2 Component responsibilities (each has one job, testable in isolation)

- **`geo.py`** — WGS84 ⇄ ENU (sub-mm round-trip); the single place conventions are converted. Nothing else knows about lat/lon.
- **`route.py`** — immutable Route value object: the dense 0.1 m polyline `P[i]`, cumulative arc-length `s[i]`, unit tangents `t[i]` (finite-difference), arc-length↔index lookup, and the **waypoint labels** (human-meaningful vertices 1,2,3,… for the panorama). Shared by algorithm and sim.
- **`projection.py`** — the product under test. Pure: input = (measured pose, Route), plus carried state; output = matching decisions + telemetry. Knows nothing about simulation or rendering.
- **`simulate.py`** — the only home of randomness (seeded). Produces true trajectory + measured poses + ground-truth labels.
- **`scenarios.py`** — declares the 14-case matrix with fixed seeds.
- **`generate.py`** — orchestrates, grades against ground truth, writes JSON + verdicts.
- **`viewer/`** — dumb playback; fixed display rotation only.

### 3.3 The algorithm (`projection.py`) in detail

**Carried state:** `cursor_s` (monotonic arc-length), `initialized`.

**Constants (all configurable):** `AHEAD = +20 m`, `BEHIND = −5 m` (projection window); `W_SEARCH = 3.5 m`, `EPS_BACK = 0.3 m` (forward search window); `GATE = 60°` (heading agreement).

**Per-frame procedure:**
1. **Seed (frame 0 only):** global nearest-point search over the whole route, gated by heading agreement (pick the candidate whose route tangent best aligns with vehicle heading). The one allowed global search.
2. **Forward-windowed match:** over arc-length `[cursor_s − EPS_BACK, cursor_s + W_SEARCH]`, pick the point minimizing lateral distance to the measured position, subject to the heading gate.
3. **Heading-agreement gate:** reject candidates whose route tangent differs from vehicle heading by > `GATE`. If **no** candidate passes, widen the gate for that frame, log it, and still emit (never drop a frame).
4. **Update cursor:** `cursor_s = max(cursor_s, matched_s)` (monotonic non-decrease).
5. **Emit window slice (viewer renders it):** the algorithm records `cursor_s`; the body-frame slice is route points in `[cursor_s + BEHIND, cursor_s + AHEAD]`, clipped at route ends (**no extrapolation**), rotated into body frame via §3.4. Near start, behind-stub is shorter; near end, ahead clips and `end_flag` is set.
6. **Telemetry:** `est_lat_dev` (signed perpendicular offset of the vehicle from the matched route tangent, **positive = vehicle left of the route**), progress %, `matched_seg`, `end_flag`.

**Self-crossing resolution:** on the first pass through a crossing, the other stroke's arc-length is far outside `[cursor_s − EPS_BACK, cursor_s + W_SEARCH]`, so it cannot be matched; on the later pass, the cursor has advanced past the intervening waypoints, so the current stroke is where the cursor is. The heading gate is the backstop in the few frames adjacent to the exact crossing.

### 3.4 Body-frame transform (`+x` forward, `+y` left)

Working in ENU, with heading `h` = CCW angle of `+x` body from East, for a global offset `d = P_route − pose`:

```
forward (+x_body) = ( cos h,  sin h )
left    (+y_body) = ( −sin h, cos h )

body_x (forward) =  d.e·cos h + d.n·sin h
body_y (left)    = −d.e·sin h + d.n·cos h
```

Verified by `test_projection_frame.py` at h = 0/90/180/−90°. In the driver-view drawing, `+x` (forward) is **up** on screen and `+y` (left) is **to the left**.

### 3.5 Simulation (`simulate.py`) — two layers over the planned route

**Layer 1 — planned centerline:** dense 0.1 m polyline from scenario waypoints. Straight/L/X segments are straight between vertices; smooth/S/figure-eight use arcs/splines sampled at 0.1 m. **Sharp planned vertices stay sharp.**

**Layer 2 — true trajectory (imperfect tracking):**
- Walk the centerline at constant speed (default 8 km/h) at 10 Hz (≈0.22 m/frame).
- Add a **smooth low-frequency lateral offset** (slow random-walk, low-pass filtered), 1σ ≈ **0.15 m**, hard cap **0.4 m**, applied perpendicular to the centerline tangent. Same across all localization tiers.
- **Round sharp corners** with a fixed turn radius (~2.5 m, configurable) so true heading is continuous and the car realistically **cuts corners** (true lateral deviation legitimately spikes there).
- **True heading** = tangent of the resulting (offset, rounded) trajectory.

**Layer 3 — measured pose (RTK error, the only thing the algorithm sees):**
- `bias(t)` = low-passed random-walk, scaled to the tier's 1σ, isotropic in-plane (independent equal-σ e/n walks), hard cap **2 m**.
- `white(t)` = small per-frame Gaussian.
- `meas_pos = true_pos + bias(t) + white(t)`; `meas_heading = true_heading + heading_bias + heading_white`, cap **±0.05°**.
- Pitch/roll: cosmetic ±0.05°-capped noise for the info panel; unused by the algorithm.

**Localization tiers (1σ):**

| Tier | Lateral & longitudinal pos | Heading/pitch/roll |
|------|----------------------------|--------------------|
| low | 0.10 m | 0.01° |
| medium | 0.50 m | 0.03° |
| high | 1.50 m | 0.05° |

Position cap 2 m; angle cap ±0.05°.

**Determinism:** each case has a fixed integer seed (`numpy.random.default_rng(seed)`); regeneration is bit-identical. Tiers of the same base geometry share identical geometry/speed but use different seeds so their error realizations look distinct.

**Ground truth for grading:** per frame the sim records true arc-length `gt_s` and true segment `gt_seg`. `generate.py` compares algorithm output against these.

### 3.6 Per-frame JSON record (decisions, not the full slice)

```
frame: {
  t, speed,
  true_pose: {e, n, h},
  meas_pose: {e, n, h, pitch, roll},
  cursor_s, matched_seg, est_lat_dev, true_lat_dev, end_flag,
  gt_seg, gt_s
}
```
Plus per case: the Route polyline + waypoint labels, tier/seed metadata, and the graded **verdict** (PASS/FAIL + metric values). The viewer renders the driver-view slice from `Route`, `cursor_s`, and `meas_pose` via the fixed §3.4 rotation.

### 3.7 Viewer (`viewer/`)

```
┌────────────┬───────────────────────────────┬──────────────────┐
│ LEFT       │ CENTER                         │ RIGHT            │
│ case list  │ ┌───────────────────────────┐  │ heading °        │
│ (grouped   │ │ BEV top-down              │  │ speed km/h       │
│  by        │ │ route + driven traj + car │  │ pos (e,n)        │
│  scenario, │ └───────────────────────────┘  │ est lat dev*     │
│  PASS/FAIL │ ┌───────────────────────────┐  │ true lat dev(ref)│
│  badges)   │ │ driver-view (+x up,+y left)│  │ progress %       │
│ ┌────────┐ │ │ +pinhole overlay (toggle) │  │ matched seg      │
│ │panorama│ │ └───────────────────────────┘  │ frame k/N        │
│ │1,2,3…  │ │ [◀ ▮ ▶  ●━━━━━ scrubber  ×spd]│  │ PASS/FAIL, %     │
│ └────────┘ │                                │                  │
└────────────┴───────────────────────────────┴──────────────────┘
```

- **Panorama (left):** whole planned route; **waypoints numbered 1,2,3,…** with direction arrowheads; small dot at car's current position.
- **BEV (center-top):** planned route (light) + accumulated driven trajectory (bold) + oriented car marker at true pose. Fixed transform, north-up.
- **Driver-view (center-bottom):** body-frame slice, car at origin, `+x` up (forward), `+y` left; projected route ahead + behind-stub. Nice-to-have **pinhole perspective overlay** (toggleable): default camera ~1.4 m height, slight downward pitch, ~60° HFOV, principal point centered — exposed as config constants.
- **Controls:** play/pause, step ±1, scrubber, speed ×0.5/×1/×2. Selecting a case loads its JSON, resets to frame 0.
- **Anti-flicker:** each figure is a `<canvas>` with a **fixed world→screen transform** computed once from route bounds (no mid-play pan/zoom); the static layer (route, panorama) is drawn once to an offscreen canvas and blitted; only the car marker + slice redraw per frame via `requestAnimationFrame` throttled to 10 Hz.

---

## 4. Error Handling

| Situation | Handling |
|-----------|----------|
| Route start (frame 0) | behind-stub naturally shorter than 5 m; valid slice; no error |
| Route end | ahead slice clips at last point; `end_flag` set; **no extrapolation** |
| High-tier error at 2 m cap | search window (`W_SEARCH = 3.5 m ≥ 2 m cap + advance + margin`) still contains true match; cursor monotonic; a correlated longitudinal bias may leave the cursor slightly ahead of true and not fully retreat — **expected** under no-smoothing + monotonic progress |
| No candidate passes heading gate (sharp corner) | widen gate for that frame, **log** it, still emit — never drop a frame |
| Frame-0 seed under high error | global search + heading gate lands on the correct starting segment (geometrically the wrong stroke is far away in every scenario) |
| Self-crossing ambiguity | bounded forward window + monotonic cursor keep the match on the current stroke both passes; heading gate is backstop |

---

## 5. Test Methodology (confirmed)

### 5.1 Unit tests (algorithm mechanics — clean, deterministic)

| ID | Precondition | Action | Expected |
|----|--------------|--------|----------|
| U1 | Hefei ENU origin | WGS84→ENU→WGS84 on 100 points | error < 1 mm |
| U2 | route with straight + arc | build arc-length + tangents | arc-length monotonic = Euclidean sum; tangent matches analytic to 1e-6 |
| U3 | known route point at known offset from pose | body-frame transform at h = 0/90/180/−90° | body (x,y) exactly equals hand-computed values (pins `+x` forward / `+y` left) |
| U4 | synthetic forward-only pose stream | run projector | `cursor_s` non-decreasing every frame |
| U5 | scenario E geometry, **zero-error** poses | run full pass | matched segment correct on both passes; never flips at crossing |
| U6 | frame where wrong stroke is geometrically nearer but heading-misaligned | single-frame match | gate rejects wrong stroke; matches correct stroke |

### 5.2 Scenario acceptance (integration — graded vs ground truth, all 14 cases)

| ID | Precondition | Action | Expected |
|----|--------------|--------|----------|
| S1 (happy path) | A/straight, low tier | full run | correct-branch 100%; est vs true lat-dev within ~1σ; 0 backward jumps; 0 dropouts |
| S2 | B smooth turn; D S-shape | full run | correct branch 100%; deviation bounded through curvature |
| S3 (corner stress) | C near-90°, high tier | full run | correct branch 100%; **true** lat-dev spikes at corner (car cuts it) but algorithm stays matched; no dropout |
| S4 (crossing E) | E, all 3 tiers | full run | correct-branch ≥ N−3 frames (3-frame tolerance) per tier; both crossing passes on correct stroke; 0 backward jumps |
| S5 (figure-eight F) | F, medium | full run | correct branch through curved transverse crossing within tolerance |
| S6 (two-crossing G) | G, medium + high | full run | correct stroke at **both** crossings within tolerance; monotonic cursor holds against snap-back to the early diagonal |

### 5.3 Edge / exception tests

| ID | Precondition | Action | Expected |
|----|--------------|--------|----------|
| E1 (route start) | frame 0 at route start | run | behind-stub < 5 m; no error; valid slice |
| E2 (route end) | car reaches destination | run final frames | `end_flag` set; ahead clips at last point; **no extrapolation** |
| E3 (max error) | any scenario, high tier hitting 2 m cap | run | 0 backward jumps, 0 dropouts, cursor monotonic despite cap-sized jump |
| E4 (gate fallback) | sharp corner, no candidate passes 60° gate | that frame | gate widened, frame logged, **not** dropped; valid slice |
| E5 (seed under error) | E start, high tier, frame-0 global search | seed cursor | seeds onto correct segment (1-2), not the far 3-4 stroke |

### 5.4 Viewer smoke tests (scripted/manual)

| ID | Action | Expected |
|----|--------|----------|
| V1 | load `index.json` + each case | all 14 cases load; panorama shows numbered waypoints with direction arrows |
| V2 | play / step / scrub | frame-step + continuous play work; telemetry (heading, speed, pos, est+true lat-dev, progress, seg, frame k/N) updates |
| V3 | continuous play | fixed transform → no pan/zoom flicker; static layer blitted; only car+slice redraw |
| V4 | click between cases | no regeneration; identical data each time (frozen JSON) |

### 5.5 Test-case matrix (14 cases)

| # | Scenario | Geometry | Tiers | Approx frames |
|---|----------|----------|-------|---------------|
| A | Straight | 40 m straight segment | low, medium, high | ~180 |
| B | Smooth turn | 90° arc r=15 m + 8 m lead-in/out | low, medium | ~180 |
| C | Near-90° corner | two 25 m legs at 85° sharp vertex | low, high | ~225 |
| D | S-shape | two opposing arcs r=12 m | medium | ~180 |
| E | 又 / X crossing | 1(bl)→2(tr)→3(tl)→4(br), ≈40×30 m; 1-2 & 3-4 cross once mid-route (traversed twice) | low, medium, high | ~400 |
| F | Figure-eight (∞) | two r≈10 m loops sharing one central crossing, traversed twice transversely | medium | ~400 |
| G | Two crossing points | `1(0,0)→2(60,30)→3(20,40)→4(30,-10)→5(55,30)` scaled to ~40×30 m; segments 3-4 and 4-5 each cross the early diagonal 1-2 | medium, high | ~500–600 |

### 5.6 Acceptance-criteria coverage map

| Acceptance criterion (§6) | Verified by |
|---------------------------|-------------|
| Correct-branch association (≤3-frame tolerance) | U5, S1–S6, E5 |
| Monotonic progress (0 backward jumps) | U4, S1/S4/S6, E3 |
| Bounded projection accuracy | S1–S3 |
| No dropouts | S1, E1–E4 |
| Viewer behavior | V1–V4 |

---

## 6. Acceptance Criteria & Success Metrics

1. **Correct branch/segment association (headline):** every frame's matched point lies on the correct route segment (vs ground truth), including through self-crossings, with a tolerance of **at most 3 frames** per run. Everywhere else: correct.
2. **Monotonic progress:** `cursor_s` non-decreasing across each run; **0** backward jumps.
3. **Bounded projection accuracy:** estimated vs true lateral deviation within **~1σ** of the active tier's lateral error on straights, with more allowance on sharp corners.
4. **No dropouts:** every frame yields a valid body-frame slice filling `[BEHIND, AHEAD]`, except naturally clipped at route start/end.
5. **Viewer:** all 14 cases load; panorama shows numbered waypoints; BEV + driver-view + telemetry render per frame; frame-step + continuous play with minimal flicker; nothing regenerates on click.

---

## 7. TBD

- Exact default pinhole camera parameters (height/pitch/HFOV/principal point) for the nice-to-have perspective overlay — set as config constants at implementation time; not on the acceptance path.
- Exact Hefei ENU origin lat/lon — pick a concrete Hefei coordinate at implementation time (does not affect algorithm behavior).
