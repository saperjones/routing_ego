# Design Spec: Parking Navigation Route Projection Algorithm

> **Update 2026-07-04 (smoothing):** the `smoothed` strategy now smooths the route **once in world space** (cached) and re-windows that fixed curve each frame, so the corner no longer jitters frame-to-frame. Defaults raised for a smooth-by-default feel: `min_turn_radius_m = 8.0`, `clothoid_transition_m = 4.0` (calibration measured 1.5 m). The driver view overlays the real driven trajectory for comparison.


**Status:** finalized (requirements + hardened design + confirmed tests)
**Deliverables:** Python projection algorithm + simulation harness + static HTML visualization tool
**Implementation handoff:** `superpowers:writing-plans` (this spec contains no implementation code)

---

## 1. Requirements Summary

We have a low-speed parking auto-driving system. It is given a navigation route planned in world coordinates (WGS84, with a point every `0.1 m` along the street centerline). Every frame, that route has to be re-expressed **from the vehicle-body / driver's viewpoint** so the downstream path-following controller can drive along it.

The hard part is doing this **robustly**. "Robustly" here means: always know *where on the route the car currently is*, even when the route **crosses itself**, and even under **RTK localization error** (RTK = the GPS/positioning technique in use).

**Operating envelope:** above-ground parking lots with GPS/RTK, 6–10 km/h, in Hefei. We work in a local ENU frame anchored at a Hefei origin. (ENU = a flat local map frame with axes East, North, Up.)

**Explicit non-goals** (things this project deliberately does not do):
- No pose/heading **smoothing or filtering** (a prior module handles that; this project must not add smoothing).
- Not building the downstream controller.
- No underground/no-GPS, no non-RTK localization.
- No re-walking a road and no U-turn-and-repeat routes (disallowed by product definition; the monotonic-progress design relies on this guarantee).
- No camera-calibrated perspective; a **default pinhole** camera model powers the windshield view only. ("Pinhole" = the simplest camera model.) *(Implemented as the driver-view perspective toggle — see §3.7.)*

**Coordinate conventions (authoritative):**
- **Global:** WGS84.
- **Working frame:** local **ENU** (East = e, North = n, meters), anchored at a chosen Hefei lat/lon. All algorithm math happens here. The WGS84 ⇄ ENU conversion is isolated in `geo.py` — nothing else touches lat/lon.
- **Vehicle body frame (matches `JSON_FIELD_DESCRIPTION_V7` ego(curr)):** **`+x` forward, `+y` left, `+z` up**, right-handed, meters. This is the output frame the downstream controller consumes.
- **Vehicle heading `h`:** the counter-clockwise angle of the vehicle's forward axis (`+x` body) measured from ENU East. If a heading arrives as a compass bearing (measured from North), it is converted to this convention in `geo.py`.
- The model is 2D and planar (e, n, h). Pitch and roll are treated as negligible/cosmetic (capped at ±0.05°).

---

## 2. Chosen Approach & Rationale

The system is **two clean halves that meet at a JSON file:**
- A **Python generator** (using numpy) runs the stateful projection algorithm over a seeded simulation and pre-computes every test case into JSON. ("Prebakes" = computes ahead of time and stores.) The JSON carries raw inputs — route geometry, ego poses, and matching decisions (`cursor_s`, `matched_seg`, telemetry) — but **not** a precomputed body-frame path.
- A **static HTML viewer** loads JSON and calls `ProjectRoute.projectRoute` (a DOM-free JS twin of the Python function) live each frame to compute the body-frame path under the currently selected algorithm and slider values.

**Why this design:** the matching cursor (`cursor_s`) is computed once, deterministically, in Python and stored. The body-frame path rendering is computed live in the browser, so changing the algorithm selector (`raw` / `centered` / `smoothed`) or any parameter slider (`R_min`, `behind`, `ahead`, `corner°`) takes effect instantly without a server round-trip. Because the JS twin is parity-tested against Python (`tests/e2e/test_parity_py_js.py`), what you see in the browser is mathematically identical to what the Python algorithm produces.

**The split point is the monotonic cursor, not the body-frame path.** The JSON stores decisions; the viewer re-renders paths on-the-fly from those decisions. Regeneration (re-randomization) is still impossible — the JSON is frozen — but the viewer can explore different output strategies over the same run.

**Core algorithmic idea — stateful monotonic progress:** the algorithm keeps a **progress cursor** (`cursor_s`, the arc-length distance travelled along the route). Each frame it only looks for a match inside a **bounded forward window** near the previous cursor, and it advances the cursor **monotonically** (it never decreases). Monotonic advance is valid because no road is re-walked and there are no U-turns. A **heading-agreement gate** is used as a secondary tie-breaker (it checks the route's direction against the car's heading). Together these resolve the ambiguity at a self-crossing that a memoryless nearest-point search cannot.

**The portable function (`project_route.py`):** the pure function `project_route` encapsulates both the match and the path construction in a single, stateless call. The caller holds `ProjectState(cursor_s, initialized)` and passes it back each frame. Three output strategies are selectable via `ProjectConfig.strategy`:

| Strategy | Offset | Corner smoothing |
|----------|--------|-----------------|
| `"raw"` | as-is (shows lateral offset) | none |
| `"centered"` | removed (anchor at `y = 0`) | none |
| `"smoothed"` | removed | corner fillet: `corner_style="clothoid"` (default, Euler spiral, curvature-continuous) or `corner_style="arc"` (circular arc); radius ≥ `min_turn_radius_m` |

**Alternatives rejected:**
- Stateless global nearest-point search — genuinely ambiguous at self-crossings (the exact failure this project targets).
- Baking the full body-frame slice into JSON every frame — redundant and precludes live algorithm/parameter exploration in the viewer.
- Two independent implementations without a parity test — risks silent divergence.

---

## 3. Architecture / Components / Data Flow

### 3.1 File layout

```
routing_ego/
├── src/                         # Python core (numpy)
│   ├── geo.py                   # WGS84 ⇄ local ENU (Hefei origin); heading→math-yaw
│   ├── route.py                 # Route value object: dense 0.1 m polyline, arc-length, tangents, waypoint labels
│   ├── projection.py            # THE algorithm: stateful projector (progress cursor)
│   ├── project_route.py         # portable pure function: ProjectConfig/State/Output + 3 strategies (raw/centered/smoothed)
│   ├── smoothing.py             # RDP corner detection + circular-arc fillet (fast, closed-form)
│   ├── simulate.py              # two-layer sim: tracking layer + RTK error layer, 10 Hz
│   ├── scenarios.py             # test-case matrix: geometry + tier configs + fixed seeds
│   └── generate.py              # runs all scenarios → JSON per case + index.json; computes verdicts
├── viewer/
│   ├── index.html               # single-page viewer; algorithm selector + parameter sliders
│   ├── project_route.js         # DOM-free JS twin of project_route.py + smoothing.py (window.ProjectRoute)
│   ├── viewer.js                # loads JSON; panorama/BEV/driver-view; playback; calls ProjectRoute live
│   └── viewer.css
├── out/                         # generated JSON (git-ignored): index.json + <case_id>.json
└── tests/                       # pytest: algorithm correctness vs ground truth; tests/e2e/ browser parity
```

### 3.2 Component responsibilities (each has one job, testable in isolation)

Each of these does one job and can be tested on its own:

- **`geo.py`** — converts WGS84 ⇄ ENU with a sub-millimeter round-trip error. It is the single place conventions are converted; nothing else knows about lat/lon.
- **`route.py`** — an immutable Route value object. It holds the dense `0.1 m` polyline `P[i]`, the cumulative arc-length `s[i]`, the unit tangents `t[i]` (computed by finite differences), an arc-length↔index lookup, and the **waypoint labels** (the human-meaningful corner numbers 1,2,3,… shown in the panorama). Shared by the algorithm and the simulation.
- **`projection.py`** — the stateful projector (monotonic cursor + heading gate). Pure: input is (measured pose, Route, state) and output is matching decisions plus telemetry. Knows nothing about simulation or rendering.
- **`project_route.py`** — the portable pure function `project_route(route, pose_e, pose_n, yaw, config, state, speed) → ProjectOutput`. Carries the full output-path logic: monotonic match, body-frame rotation, and the three strategies (raw / centered / smoothed). Python is authoritative; the JS twin must match it frame-for-frame.
- **`smoothing.py`** — fast, embedded-friendly polyline smoothing: RDP corner detection followed by closed-form circular-arc fillet. No iteration. Curvature bounded by `1/min_radius` on non-degenerate legs.
- **`simulate.py`** — the only place randomness lives (and it is seeded). It produces the true trajectory, the measured poses, and the ground-truth labels.
- **`scenarios.py`** — declares the 14-case matrix with fixed seeds.
- **`generate.py`** — the orchestrator: runs the cases, grades them against ground truth, and writes the JSON plus verdicts.
- **`viewer/project_route.js`** — DOM-free JS twin of `project_route.py` + `smoothing.py`. Exposed as `window.ProjectRoute`. Runs the projection algorithm live in the browser each frame, so the viewer's driver view always reflects the current slider values without a regeneration round-trip. Parity with the Python reference is enforced by `tests/e2e/test_parity_py_js.py`.
- **`viewer/viewer.js`** + **`viewer/index.html`** — load JSON (raw inputs: route geometry, ego poses, telemetry decisions), call `ProjectRoute.projectRoute` each frame, render the result. The JSON carries no precomputed body-frame path.

### 3.3 The algorithm (`projection.py`) in detail

**Carried state:** `cursor_s` (the monotonic arc-length) and `initialized`.

**Constants (all configurable):** `AHEAD = +20 m`, `BEHIND = −5 m` (the projection window); `W_SEARCH = 15 m`, `EPS_BACK = 0.3 m` (the forward search window — `W_SEARCH` is the **endurable offset**, the largest along-track jump the projection can absorb before the anchor falls behind the vehicle; kept well below the 64–104 m self-crossing stroke separation so crossing disambiguation still holds); `GATE = 60°` (the heading-agreement threshold).

**Per-frame procedure:**
1. **Seed (frame 0 only):** do one global nearest-point search over the whole route, gated by heading agreement (pick the candidate whose route tangent best lines up with the vehicle heading). This is the one allowed global search.
2. **Forward-windowed match:** over the arc-length range `[cursor_s − EPS_BACK, cursor_s + W_SEARCH]`, pick the point that minimizes the lateral distance to the measured position, subject to the heading gate.
3. **Heading-agreement gate:** reject any candidate whose route tangent differs from the vehicle heading by more than `GATE`. If **no** candidate passes, widen the gate for that one frame, log it, and still emit a result (never drop a frame).
4. **Update cursor:** `cursor_s = max(cursor_s, matched_s)` — this enforces monotonic non-decrease (the cursor can't move backward).
5. **Emit window slice (the viewer renders it):** the algorithm records `cursor_s`. The body-frame slice is the route points in `[cursor_s + BEHIND, cursor_s + AHEAD]`, clipped at the route ends (**no extrapolation**), rotated into the body frame per §3.4. Near the start the behind-stub is shorter; near the end the ahead portion clips and `end_flag` is set.
6. **Telemetry:** `est_lat_dev` (the signed perpendicular offset of the vehicle from the matched route tangent, where **positive = vehicle left of the route**), progress %, `matched_seg`, and `end_flag`.

**Self-crossing resolution:** on the first pass through a crossing, the other stroke's arc-length is far outside `[cursor_s − EPS_BACK, cursor_s + W_SEARCH]`, so it cannot be matched. On the later pass, the cursor has already advanced past the intervening waypoints, so the current stroke is where the cursor sits. The heading gate is the backstop for the few frames right next to the exact crossing point.

### 3.4 Body-frame transform (`+x` forward, `+y` left)

Working in ENU, with heading `h` = the CCW angle of `+x` body from East, for a global offset `d = P_route − pose`:

```
forward (+x_body) = ( cos h,  sin h )
left    (+y_body) = ( −sin h, cos h )

body_x (forward) =  d.e·cos h + d.n·sin h
body_y (left)    = −d.e·sin h + d.n·cos h
```

This is verified by `test_projection_frame.py` at h = 0/90/180/−90°. In the driver-view drawing, `+x` (forward) points **up** on screen and `+y` (left) points **to the left**.

### 3.5 Simulation (`simulate.py`) — two layers over the planned route

**Layer 1 — planned centerline:** a dense `0.1 m` polyline built from the scenario waypoints. Straight, L-shaped, and X-shaped segments run straight between vertices; smooth, S-shaped, and figure-eight cases use arcs/splines sampled at `0.1 m`. **Sharp planned vertices stay sharp.**

**Layer 2 — true trajectory (imperfect tracking):** this is where the car actually drives, which is close to but not exactly the centerline.
- Walk the centerline at constant speed (default `8 km/h`) at `10 Hz` (about `0.22 m` of travel per frame).
- Add a **smooth low-frequency lateral offset** — a slow, low-pass-filtered random walk — with 1σ ≈ **0.15 m** and a hard cap of **0.4 m**, applied perpendicular to the centerline tangent. This offset is the same across all localization tiers.
- **Round sharp corners** with a fixed turn radius (~2.5 m, configurable) so the true heading changes continuously and the car realistically **cuts corners** (which makes the true lateral deviation legitimately spike there).
- **True heading** = the tangent of the resulting (offset and rounded) trajectory.

**Layer 3 — measured pose (the RTK error; this is the only thing the algorithm sees):**
- `bias(t)` = a low-pass-filtered random walk, scaled to the tier's 1σ, isotropic in the plane (independent equal-σ walks on e and n), with a hard cap of **2 m**.
- `white(t)` = a small per-frame Gaussian noise.
- `meas_pos = true_pos + bias(t) + white(t)`. And `meas_heading = true_heading + heading_bias + heading_white`, capped at **±0.05°**.
- Pitch/roll: cosmetic noise capped at ±0.05° for the info panel; the algorithm does not use it.

**Localization tiers (1σ):**

| Tier | Lateral & longitudinal pos | Heading/pitch/roll |
|------|----------------------------|--------------------|
| low | 0.10 m | 0.01° |
| medium | 0.50 m | 0.03° |
| high | 1.50 m | 0.05° |

Position cap 2 m; angle cap ±0.05°.

**Determinism:** each case has a fixed integer seed (`numpy.random.default_rng(seed)`), so regenerating it is bit-identical. Tiers of the same base geometry share identical geometry and speed but use different seeds, so their error patterns look distinct from each other.

**Ground truth for grading:** each frame the simulation records the true arc-length `gt_s` and the true segment `gt_seg`. `generate.py` compares the algorithm's output against these.

### 3.6 Per-frame JSON record (decisions + raw inputs; no precomputed body-frame path)

```
frame: {
  t, speed,
  true_pose: {e, n, h},
  meas_pose: {e, n, h, pitch, roll},
  cursor_s, matched_seg, est_lat_dev, true_lat_dev, end_flag,
  gt_seg, gt_s
}
```

Along with each case: the Route polyline (`points_e`, `points_n`, `s`, `waypoint_indices`, `waypoint_labels`) plus tier/seed metadata and the graded **verdict** (PASS/FAIL plus metric values).

Real-data frames (`mode:"real"`) omit `true_pose`, `true_lat_dev`, `gt_seg`, and `gt_s` (no simulation ground truth).

**The JSON carries no precomputed body-frame path.** The viewer calls `ProjectRoute.projectRoute` live each frame to build the driver-view path from `meas_pose`, the Route, and the current `cursor_s`. This means the algorithm selector and parameter sliders in the viewer take effect instantly, and the output is mathematically identical to what the Python function would produce (guaranteed by the parity test).

#### 3.6.1 The `project_route` function and three output strategies

The portable function `project_route(route, pose_e, pose_n, yaw, config, state=None, speed=None) → ProjectOutput` is the single authority for body-frame path construction. Its key dataclasses:

**`ProjectConfig`** (all fields configurable, defaults match viewer defaults):

| Field | Default | Description |
|-------|---------|-------------|
| `strategy` | `"smoothed"` | `"raw"` / `"centered"` / `"smoothed"` |
| `behind_m` | 5.0 | look-behind window (m) |
| `ahead_m` | 70.0 | look-ahead window (m) |
| `sample_ds_m` | 0.5 | path sampling step (m) |
| `search_ahead_m` | 15.0 | forward search window (m) |
| `search_back_m` | 0.3 | back-tolerance (m) |
| `heading_gate_deg` | 60.0 | heading-agreement gate (degrees) |
| `min_turn_radius_m` | 5.0 | arc-fillet min radius (m); `smoothed` only |
| `corner_angle_deg` | 10.0 | min angle to fillet (degrees); `smoothed` only |
| `simplify_eps_m` | 0.20 | RDP tolerance (m); `smoothed` only |
| `corner_style` | `"clothoid"` | `"arc"` or `"clothoid"`; `smoothed` only |
| `clothoid_transition_m` | 1.5 | clothoid spiral transition length (m); `smoothed`+`clothoid` only |

**`ProjectState`**: `cursor_s`, `initialized` — the caller holds this between frames.

**`ProjectOutput`**: `path`, `cursor_s`, `lat_dev`, `matched_seg`, `end_flag`, `state`.

**Strategies:**

- **`"raw"`** — body-frame rotation only; lateral offset preserved.
- **`"centered"`** — lateral shift subtracted so the matched anchor sits at `y = 0`; preserves heading error and curvature.
- **`"smoothed"`** — same shift as `"centered"`, then the forward portion is passed through `smooth_corners`: RDP simplification (tolerance `simplify_eps_m`), followed by a corner fillet. When `corner_style="clothoid"`, each corner uses a symmetric clothoid (Euler spiral) with curvature ramping linearly 0→1/R over `clothoid_transition_m`, an optional constant-curvature arc, then back to 0; the fit is tried at factors 1, 0.5, 0.25 of the transition length and falls back to an arc if none fit. When `corner_style="arc"` (or clothoid fallback), a circular-arc fillet with `T = R_min · tan(δ/2)` clamped to half the adjacent leg, `R_eff = T / tan(δ/2)`. For non-degenerate legs, peak curvature `κ ≤ 1 / min_turn_radius_m`; the behind-stub is not smoothed.

See `docs/project_route_function.md` for the full math derivation.

### 3.7 Viewer (`viewer/`)

("BEV" below = bird's-eye view, i.e. a top-down map view.)

```
┌────────────┬───────────────────────────────┬──────────────────┐
│ LEFT       │ CENTER                         │ RIGHT            │
│ case list  │ ┌───────────────────────────┐  │ heading °        │
│ (grouped   │ │ BEV top-down              │  │ speed km/h       │
│  by        │ │ route + driven traj + car │  │ pos (e,n)        │
│  scenario, │ └───────────────────────────┘  │ est lat dev*     │
│  PASS/FAIL │ ┌───────────────────────────┐  │ true lat dev(ref)│
│  badges)   │ │ driver-view (+x up,+y left)│  │ progress %       │
│ ┌────────┐ │ │ +perspective 3D (toggle)  │  │ matched seg      │
│ │panorama│ │ └───────────────────────────┘  │ frame k/N        │
│ │1,2,3…  │ │ [◀ ▮ ▶  ●━━━━━ scrubber  ×spd]│  │ PASS/FAIL, %     │
│ └────────┘ │                                │                  │
└────────────┴───────────────────────────────┴──────────────────┘
```

- **Panorama (left):** the whole planned route, with **waypoints numbered 1,2,3,…** and direction arrowheads, plus a small dot at the car's current position.
- **BEV (center-top):** the planned route (light) plus the accumulated driven trajectory (bold) plus an oriented car marker at the true pose. It uses a fixed transform and is drawn north-up.
- **Driver-view (center-bottom):** the body-frame path computed live by `ProjectRoute.projectRoute` — car at the origin, `+x` up (forward), `+y` left. Ticking the **perspective** checkbox switches the panel into a **windshield 3D view**: the route is projected through a forward-looking pinhole camera onto the road plane ahead, drawn with a sky and horizon line, a perspective ground grid for depth, and the trajectory as a ribbon that is wide near the car and **narrows toward the vanishing point** on the horizon (with a dashed centerline). The driver's position is anchored at bottom-center by a **hood band, a forward center line, and an ego arrow**. The default camera is ~1.4 m high, pitched down ~10°, ~70° horizontal field of view, principal point centered; these plus the ribbon half-width are exposed as `PERSP` constants in `viewer.js`.
- **Algorithm selector (`#algo-select`):** `Raw (keep offset)` / `Centered (no offset)` / `Smoothed (drivable corners)` — maps directly to `ProjectConfig.strategy`. Changing it re-renders the current frame instantly (no reload).
- **Corner-style selector (`#corner-style`):** `arc` (circular-arc fillet) or `clothoid` (**default**, Euler spiral with linear curvature ramp); visible when `Smoothed` is active. Maps to `ProjectConfig.corner_style`.
- **Parameter sliders:** six range inputs update `ProjectConfig` fields live:
  - `#p-radius` (`R_min`, 3–12 m, step 0.5) → `min_turn_radius_m`
  - `#p-behind` (behind, 0–10 m, step 1) → `behind_m`
  - `#p-ahead` (ahead, 20–100 m, step 5) → `ahead_m`
  - `#p-corner` (corner°, 5–45°, step 5) → `corner_angle_deg`
  - `#p-transition` (transition, 0.5–8 m, step 0.5) → `clothoid_transition_m`; active with `clothoid` corner style
- **Live compute architecture:** the monotonic cursor (`cursor_s`) is memoized frame-by-frame at the default matching parameters. Changing a slider or the algo selector recomputes only the body-frame path from the already-known cursor — the match does not re-run. This makes slider interaction instant even on long cases.
- **Controls:** play/pause, step ±1, scrubber, speed ×0.5/×1/×2. Selecting a case loads its JSON and resets to frame 0.
- **Anti-flicker:** each figure is a `<canvas>` with a **fixed world→screen transform** computed once from the route bounds (so there is no mid-play pan or zoom). The static layer (route, panorama) is drawn once onto an offscreen canvas and then copied in ("blitted"); only the car marker and the slice are redrawn each frame, via `requestAnimationFrame` throttled to 10 Hz.

---

## 4. Error Handling

| Situation | Handling |
|-----------|----------|
| Route start (frame 0) | behind-stub naturally shorter than 5 m; valid slice; no error |
| Route end | ahead slice clips at last point; `end_flag` set; **no extrapolation** |
| High-tier error at 2 m cap | search window (`W_SEARCH = 15 m ≫ 2 m cap + advance + margin`) still contains true match; cursor monotonic; a correlated longitudinal bias may leave the cursor slightly ahead of true and not fully retreat — **expected** under no-smoothing + monotonic progress |
| Real-data along-track jump (localization gap) | wide search window (`W_SEARCH = 15 m`) lets the cursor catch up so the projected anchor stays beside the vehicle instead of lagging behind (which would show as a spurious lateral offset in the recentred driver view) |
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
| V5 | tick the perspective checkbox | driver-view switches to the windshield 3D view (sky/horizon/grid + tapering route ribbon); no JS errors; unticking returns to the top-down slice |
| V6 | switch algo selector (`#algo-select`) between raw / centered / smoothed | driver-view path updates instantly; smoothed corners are visually rounded vs. raw/centered; no JS errors |
| V7 | adjust parameter sliders (`#p-radius`, `#p-behind`, `#p-ahead`, `#p-corner`) | driver-view updates instantly; wider radius → gentler arcs; wider ahead → longer path; no reload required |
| V8 | switch corner-style selector (`#corner-style`) between arc and clothoid | driver-view path changes (clothoid has a smoother curvature-jump profile than arc); `test_clothoid_smoother_than_arc` verifies this with a curvature-jump metric |
| V9 | adjust transition slider (`#p-transition`) with clothoid active | driver-view canvas signature changes; longer transition → more gradual curvature build-up; `test_transition_slider_changes_path` verifies |

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
| Viewer behavior | V1–V7 |
| Python↔JS parity (path, matched_seg, end_flag) | `tests/e2e/test_parity_py_js.py` (60 cases: 3 routes × 5 poses × 4 strategy/corner-style combos) |

---

## 6. Acceptance Criteria & Success Metrics

1. **Correct branch/segment association (headline):** every frame's matched point lies on the correct route segment (vs ground truth), including through self-crossings, with a tolerance of **at most 3 frames** per run. Everywhere else: correct.
2. **Monotonic progress:** `cursor_s` non-decreasing across each run; **0** backward jumps.
3. **Bounded projection accuracy:** estimated vs true lateral deviation within **~1σ** of the active tier's lateral error on straights, with more allowance on sharp corners.
4. **No dropouts:** every frame yields a valid body-frame slice filling `[BEHIND, AHEAD]`, except naturally clipped at route start/end.
5. **Viewer:** all 14 cases load; panorama shows numbered waypoints; BEV + driver-view + telemetry render per frame; frame-step + continuous play with minimal flicker; nothing regenerates on click.

> Implementation note: during the build the headline metric in criterion 1 was refined to be **along-track-aware** — a frame counts as a wrong-branch mismatch only when the matched segment differs from ground truth **and** the along-track error `|cursor_s − gt_s|` exceeds `3 m`. This keeps the "≤ 3 frames" tolerance for real wrong-stroke jumps while not penalizing benign ~1–2 m segment-boundary timing under injected noise. See `algorithm_description.md` §6.

---

## Extensions (shipped)

- **Real-data ingestion + OSM basemap:** Simulation / Real-data tabs; the same
  algorithm runs on `dataset/` packages (ego GCJ-02→WGS-84, `yaw_boot`+boot→ENU
  offset, planned route as the Route); the real-data BEV shows an OSM basemap
  with route + ego track + direction arrows in WGS-84. Design:
  `docs/superpowers/specs/2026-07-03-real-data-osm-design.md`; plan:
  `docs/superpowers/plans/2026-07-03-real-data-osm.md`.

## 7. TBD

- ~~Exact default pinhole camera parameters~~ — *resolved:* the perspective view uses `PERSP` in `viewer.js` (height 1.4 m, pitch 10°, HFOV 70°, principal point centered, ribbon half-width 0.7 m); still off the acceptance path.
- Exact Hefei ENU origin lat/lon — pick a concrete Hefei coordinate at implementation time (does not affect algorithm behavior). *(Implemented as `31.8206, 117.2290` in `geo.py`.)*

---

*Self-check: (a) every information point from the original spec — requirements, non-goals, coordinate conventions, the two-halves approach and rejected alternatives, the full component/algorithm/transform/simulation/JSON/viewer description, the error-handling table, all test tables and the case matrix, the acceptance criteria, and the TBDs — is carried over here. (b) Constraint wording ("must not add smoothing", "never drop a frame", "no extrapolation", "only", "default", the tier/window/cap numbers) and technical identifiers (file names, `cursor_s`, `AHEAD`/`BEHIND`/`W_SEARCH`/`EPS_BACK`/`GATE`, `end_flag`, `est_lat_dev`, `gt_s`/`gt_seg`, etc.) are preserved verbatim; the added along-track-metric note is flagged explicitly as an implementation note, not part of the original text.*
