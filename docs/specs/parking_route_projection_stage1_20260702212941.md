# Product Requirements Document: Parking Navigation Route Projection Algorithm

## 1. Background & Goals

**Problem to solve.** In a low-speed parking system, an auto-driving module must follow a globally-planned navigation route. The planned route is expressed in a global coordinate frame (WGS84), but the controller needs the route expressed **from the driver's / vehicle-body viewpoint** (an ego-centric slice of the path ahead of and around the car) in order to drive along it. Producing that ego-centric projection robustly — including through geometrically ambiguous route shapes (self-crossing paths) and under RTK localization error — is the core problem.

**Goals / expected value.**
- A robust algorithm that projects a global-frame planned route into the vehicle-body frame each frame, correctly tracking *where on the route the car is* even when the route crosses itself.
- A reproducible simulation harness that exercises the algorithm across representative and adversarial route shapes with realistic localization error.
- An HTML visualization tool to inspect each test case (route panorama, BEV, driver-view projection, per-frame telemetry) with frame-by-frame and continuous playback.

**Non-goals.** The algorithm does **not** smooth or filter pose/heading — a prior module is assumed to have done that. This project produces the projection algorithm, the simulation, and the viewer; it does **not** produce the downstream controller.

## 2. Target Users & Usage Scenarios

- **Target users:** the engineer(s) developing and validating the parking auto-driving system; the projection algorithm's direct consumer is the downstream path-following controller.
- **Operating scenario:** low-speed parking, **6–10 km/h**, currently only above-ground parking lots **with GPS/RTK signal**. Global frame is **WGS84**. Vehicle-body frame: **Y** = rear→front (forward), **X** = left→right (perpendicular to Y in-plane), **Z** = up.
- **Typical flow:** a planned route is given as a global-frame polyline sampled every **0.1 m**, from a start to a destination. As the car drives, each frame the algorithm consumes the car's measured pose + the route and emits the body-frame projection of the route slice around the car, which the controller follows.

## 3. Requirement Scope

**In scope this round:**
1. Projection algorithm (global route → vehicle-body-frame route slice), stateful with monotonic progress tracking.
2. Simulation harness generating deterministic, prebaked test cases (route + true trajectory + measured poses + per-frame algorithm output) as JSON.
3. HTML visualization tool (static, playback-only) reading the prebaked JSON.
4. A defined test-case matrix (scenarios × localization-error tiers) including at least one self-crossing route.

**Out of scope this round:**
- Any pose/heading smoothing or filtering (assumed done upstream).
- The downstream path-following controller itself.
- Underground / no-GPS parking, and any non-RTK localization.
- Re-walking the same road or U-turn-and-repeat routes (disallowed by product definition; not simulated).
- Real camera-calibrated perspective rendering (a default pinhole model is used for a *nice-to-have* windshield view only).

## 4. Feature Details

### Feature 1: Route projection algorithm (core)

**Output (per frame):** the planned route resampled/clipped to an arc-length window around the vehicle's projected position, expressed in vehicle-body-frame (x, y) points at the original 0.1 m spacing, plus:
- the matched point / progress cursor (arc-length along the route),
- the **estimated lateral deviation** (measured pose vs matched route point),
- a **destination-reached / end-of-window** flag when the route end falls inside the window.

**Projection window:** default **[−5 m behind, +20 m ahead]** of arc length around the vehicle's projected position. Both bounds are **configurable**. Near the route start the behind-stub is naturally shorter; near the destination the ahead portion clips at the final point and the display indicates the route ends (no extrapolation).

**Statefulness (key rule):** the algorithm maintains a running **progress cursor** — an arc-length "how far along the route am I." Each frame it searches for the match only in a bounded forward window near the previous cursor; progress is **monotonically non-decreasing** (guaranteed valid by the product rule that no road is walked twice and there are no U-turns). A **heading-agreement gate** (vehicle heading vs local route-segment tangent, with a threshold) is a secondary tie-breaker / sanity check and prevents snapping to a wrong branch. A bounded search window ensures a large localization error cannot snap the cursor backward or across to a wrong stroke.

**Flow (per frame):** receive measured pose (x, y, heading) + route → search forward window from current cursor for the nearest valid route point (gated by heading agreement) → advance cursor (non-decreasing) → transform the [−5, +20] route slice into the body frame → compute estimated lateral deviation and end-of-route flag → emit.

**Key rules:**
- 2D planar model (x, y, heading); pitch/roll are treated as negligible/cosmetic.
- No smoothing of the input pose or output points.
- Cursor never decreases.

**Edge cases & exception handling:**
- **Self-crossing routes (又 / X shape):** the car passes the crossing point twice. Monotonic cursor + bounded forward search keeps the match on the correct stroke each pass (see Feature 2, scenario E).
- **Route start:** behind-stub shorter than 5 m.
- **Route end:** ahead window clips; end-of-route flag set; no extrapolation.
- **Large localization error (high tier):** search window sized so error cannot cause backward/cross-branch snapping.

### Feature 2: Simulation harness

**Two-layer model:**
1. **Planned centerline** — the global route, a dense polyline sampled at 0.1 m. (Real routes are typically near-straight street centerlines; test geometries also include curves and crossings, all represented as dense 0.1 m polylines.)
2. **True vehicle trajectory (imperfect tracking)** — centerline + a smooth, low-frequency lateral offset (slow random-walk, not jittery); heading = tangent of the car's *own* true trajectory; **constant speed** per test case along its own arc length. Tracking lateral error **1σ ≈ 0.15 m, capped ≈ 0.4 m**, and is **the same across all localization tiers**.
3. **Measured pose** = true pose + RTK localization error (the tier under test). This is all the algorithm sees.

**Localization error model (RTK):** at each frame the true pose is perturbed by a slowly-drifting random-walk bias plus small per-frame white noise (temporally correlated, realistic for RTK), capped at the RTK bounds (lateral ≤ 2 m, angles ≤ ±0.05°).

- **Lateral position error (1σ):** low = 0.10 m, medium = 0.50 m, high = 1.50 m. Cap 2 m.
- **Longitudinal position error:** same magnitude as lateral (isotropic in-plane).
- **Heading/pitch/roll (1σ):** low = 0.01°, medium = 0.03°, high = 0.05°. Cap ±0.05°. (Pitch/roll cosmetic only.)

**Timing / motion:** constant speed per test case (default 8 km/h, configurable, within 6–10 km/h); **10 Hz** frame rate (0.1 s/frame ⇒ ≈0.22 m/frame at 8 km/h).

**Determinism:** each test case is generated **once** with a fixed random seed and prebaked to JSON. Different test cases use fixed, distinct configurations (not re-randomized on view), so differences between cases are meaningful and stable.

**Geo origin:** local ENU frame anchored at a point in **Hefei** (a chosen Hefei lat/lon as origin); WGS84 ⇄ local ENU conversion for global-frame consistency.

**Proposed test-case matrix** (to be confirmed in review; all include the two-layer model above):

| # | Scenario | Geometry | Error tiers |
|---|----------|----------|-------------|
| A | Straight line | single straight segment | low, medium, high (3 cases) |
| B | Smooth turn | gentle constant-radius arc | low, medium (or medium only) |
| C | Near-90° corner | sharp L-shaped turn | low, high |
| D | S-shape | two opposing curves | medium |
| E | 又 / X crossing | route 1→2→3→4 where segment 1-2 and segment 3-4 physically cross once (self-crossing) | low, medium, high (3 cases) |
| F | Second self-crossing variant | a different multi-segment route with one or more crossing points (e.g. figure-eight-like or an added crossing) | medium |

*(Exact count/tiers finalized at review. Scenario E geometry per the user's whiteboard: 1 bottom-left → 2 top-right → 3 top-left → 4 bottom-right, with 1-2 and 3-4 crossing in the middle; the crossing point is traversed twice.)*

### Feature 3: HTML visualization tool

Static, single-page, playback-only; reads prebaked JSON; **no recompute on click**.

**Left panel:** list of test cases; and a **route panorama** for the selected case. The planned route's segments must be labeled with **Arabic-numeral sequence numbers at each segment's start/end vertex** so the drawing order/direction of travel is legible.

**Center (two stacked figures):**
- **Top — BEV top-down view:** planned route + the car's actual driven trajectory.
- **Bottom — driver-view projection:** the body-frame projection result (primary), with the default-pinhole windshield perspective view as a nice-to-have, toggleable overlay.
- **Bottom-most — playback control:** a progress bar / scrubber supporting frame-by-frame stepping and continuous play. Rendering must minimize flicker (stable, incremental redraw).

**Right panel (per-frame telemetry):** current heading, speed, position coordinates, current-frame lateral deviation — **estimated (primary)** and **true (faint reference)** — progress along route, and other relevant per-frame info.

## 5. Non-Functional Requirements

- **Performance / data volume:** each case is 10 Hz over its route length; JSON must stay small enough to load and scrub smoothly in a browser. Playback must be smooth and **low-flicker**.
- **Determinism / reproducibility:** fixed seeds; identical output on regeneration; stable across views.
- **Permissions & roles:** single-user developer tool; no auth.
- **Data & privacy:** synthetic data only; no real user or vehicle data.

## 6. Dependencies & Constraints

- **Python + numpy** for the algorithm and simulation (offline JSON generation).
- **Static HTML/JS** viewer (self-contained, opens in a browser; consumes prebaked JSON).
- WGS84 ⇄ local ENU conversion.
- Constraint: **no pose/heading smoothing** in this project (upstream responsibility).
- Constraint: routes never re-walk a road or U-turn-and-repeat.

## 7. Acceptance Criteria & Success Metrics

1. **Correct branch/segment association (headline):** at every frame the matched point lies on the *correct* route segment (verified against ground truth), including through self-crossings — with a tolerance of **at most 3 frames** (e.g. right at a crossing). Everywhere else: correct.
2. **Monotonic progress:** progress cursor arc-length is non-decreasing across each run; **0** backward jumps.
3. **Bounded projection accuracy:** estimated lateral deviation vs true lateral deviation stays within ~1σ of the active tier's lateral error on straights, with more allowance on sharp corners (quality bound).
4. **No dropouts:** every frame yields a valid body-frame projection filling the configured window, except naturally clipped at route start/end.
5. **Viewer:** all prebaked cases load; panorama shows numbered segment endpoints; BEV + driver-view + telemetry render per frame; frame-step and continuous play work with minimal flicker; nothing regenerates on click.

## TBD

- Final test-case matrix count and exact error-tier assignment per scenario (proposed in §4 Feature 2; confirm at review).
- Exact default pinhole camera parameters for the nice-to-have perspective view (height/pitch/FOV defaults to be set as config constants).
