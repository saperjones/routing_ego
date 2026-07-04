# Parking Route Projection

Projects a globally-planned navigation route into the **vehicle-body frame**
each frame, so a low-speed parking auto-driving controller can follow it — the
route as seen from the driver's viewpoint. The hard part is doing this
**robustly** under RTK localization error and on routes that **cross
themselves**, without ever locking onto the wrong branch.

The project is two halves that meet at a JSON file:

- a **Python core** (numpy) that runs the stateful projection algorithm over a
  seeded two-layer simulation and prebakes 14 graded test cases to `out/*.json`;
  the JSON carries route geometry, ego poses, and matching decisions —
  **not** a precomputed body-frame path;
- a **static HTML/Canvas viewer** that loads JSON and calls a DOM-free JS twin
  of the projection algorithm (`viewer/project_route.js`, `window.ProjectRoute`)
  live each frame, so changing the algorithm selector or parameter sliders updates
  the driver view instantly without a server round-trip.

Python (`project_route.py`) is authoritative; `tests/e2e/test_parity_py_js.py`
binds the two implementations to within 1e-3 m.

---

## Quick start

```bash
./run.sh            # create venv, generate cases, serve viewer, open browser
./run.sh test       # unit + acceptance suite
./run.sh e2e        # headless-browser end-to-end suite
./run.sh gen        # regenerate out/*.json only
./run.sh gen-real   # prebake dataset/ real cases (+ OSM tiles) -> out/real/
./run.sh setup      # just create the venv + install deps
PORT=9000 ./run.sh  # override the port (default 8000)
```

`./run.sh` serves from the **repo root** and opens
`http://localhost:8000/viewer/index.html`. It stays up until you Ctrl-C.

> Serve from the repo root, not from inside `viewer/`: the viewer fetches
> `../out/...`, which only resolves when the server root is the repo root.

---

## Using the viewer

- **Left** — test cases grouped by scenario, each with a PASS/FAIL badge; below,
  the full route *panorama* with waypoints numbered `1,2,3,…` and direction
  arrows so the travel order is legible.
- **Center top — BEV (top-down):** the planned route plus the car's actual
  driven trajectory and an oriented car marker.
- **Center bottom — Driver view (`+x` up, `+y` left):** the body-frame path
  computed live by `ProjectRoute.projectRoute` each frame. Tick the
  **perspective** checkbox to switch into a windshield 3D view (horizon, ground
  grid, route as a ribbon narrowing toward the vanishing point). Camera constants
  live in `PERSP` in `viewer.js`.
  - **Algorithm selector** — `Raw (keep offset)` / `Centered (no offset)` /
    `Smoothed (drivable corners)` — maps to `ProjectConfig.strategy`. Raw keeps
    the lateral cross-track offset visible; Centered removes it (anchor at
    `y = 0`); Smoothed additionally replaces sharp corners with circular-arc
    fillets (curvature ≤ 1/R_min on non-degenerate legs).
  - **Parameter sliders** — `R_min` (3–12 m), `behind` (0–10 m), `ahead`
    (20–100 m), `corner°` (5–45°) — update `ProjectConfig` fields live. Wider
    radius means gentler arcs; all changes are instant, no reload.
- **Bottom — playback:** step ◀ ▶, play/pause, scrubber, speed (0.5×/1×/2×).
- **Right — telemetry:** heading, speed, position, estimated & true lateral
  deviation, progress, matched segment, frame index, and the case verdict.

Test cases are **prebaked and deterministic** — clicking a case never
regenerates or re-randomizes it, so differences between cases are meaningful.

---

## How it works

### The projection algorithm (`src/parking_proj/project_route.py`)

The portable pure function `project_route(route, pose_e, pose_n, yaw, config, state=None, speed=None) → ProjectOutput`
is the single entry point for body-frame path computation. The caller holds a
tiny `ProjectState(cursor_s, initialized)` and passes it back each frame. Each
frame the function:

1. **Matches** the measured pose within a bounded forward window
   `[cursor_s − 0.3 m, cursor_s + 15 m]` gated by **heading agreement**
   (60° gate; widened if nothing passes, so a frame is never dropped). Frame 0
   uses a global search to seed the cursor.
2. **Advances the cursor monotonically** (`cursor_s = max(cursor_s, matched_s)`)
   — valid because re-walking and U-turns are forbidden by product definition.
3. **Constructs the body-frame path** over `[cursor_s − behind_m, cursor_s + ahead_m]`
   (defaults: 5 m behind, 70 m ahead, 0.5 m step) according to the chosen strategy:

| Strategy | Lateral offset | Corner smoothing |
|----------|---------------|-----------------|
| `"raw"` | kept | none |
| `"centered"` | removed (anchor at `y = 0`) | none |
| `"smoothed"` | removed | circular-arc fillet on forward portion |

4. **Returns** `ProjectOutput`: `path` (`[[x,y],…]` body frame), `cursor_s`,
   `lat_dev`, `matched_seg`, `end_flag`, and the next `state`.

**Smoothed corners — arc-fillet math.** The forward portion of the `"smoothed"`
path is processed by `smooth_corners` in `smoothing.py`:

1. **RDP simplification** (tolerance `simplify_eps_m`, default 0.20 m) detects
   the real geometric vertices.
2. **Circular-arc fillet** — for each vertex with unsigned turn angle `δ`, the
   tangent length is `T = min(R_min · tan(δ/2), half each adjacent leg)` and
   the effective radius is `R_eff = T / tan(δ/2)`. For non-degenerate legs
   (length > `2 · R_min · tan(δ/2)`), curvature `κ ≤ 1/R_min`. The behind-stub
   is never smoothed.
3. **Uniform resample** at `sample_ds_m` restores even spacing.

The JS twin (`viewer/project_route.js`, `window.ProjectRoute`) implements the
same math; parity is verified to 1e-3 m by `tests/e2e/test_parity_py_js.py`.

**Self-crossing routes** (the "又"/X case) are resolved by construction: on the
first pass through a crossing the other stroke is far outside the forward window;
on the later pass the cursor has already advanced past the intervening waypoints.
The heading gate is a backstop. No pose/heading smoothing is done anywhere in the
algorithm — an upstream module is assumed to own that.

**Self-crossing routes** (the "又"/X case) are resolved by construction: on the
first pass through a crossing the other stroke is far outside the forward
window; on the later pass the cursor has already advanced past the intervening
waypoints. The heading gate is a backstop. No pose/heading smoothing is done
anywhere in the algorithm — an upstream module is assumed to own that.

### The simulation (`src/parking_proj/simulate.py`)

Two layers over the planned centerline:

1. **True trajectory (imperfect tracking):** the centerline with smooth corner
   rounding and a slow, capped lateral offset (1σ 0.15 m, cap 0.4 m).
2. **Measured pose (RTK error):** true pose + correlated bias + white noise, by
   tier.

| Tier   | Lateral 1σ | Heading 1σ |
|--------|-----------|------------|
| low    | 0.10 m    | 0.01°      |
| medium | 0.50 m    | 0.03°      |
| high   | 1.50 m    | 0.05°      |

Position error is capped at 2 m, angles at ±0.05°. Constant 8 km/h, 10 Hz.
Everything is seeded (`numpy.random.default_rng`), so regeneration is
byte-identical.

### Test-case matrix (14 cases)

| Group | Scenario           | Tiers                | Note |
|-------|--------------------|----------------------|------|
| A     | Straight           | low, medium, high    | error-tier comparison |
| B     | Smooth turn        | low, medium          | constant-radius arc |
| C     | Near-90° corner    | low, high            | car cuts the corner |
| D     | S-shape            | medium               | opposing arcs |
| E     | 又 / X-crossing    | low, medium, high    | strokes cross once, traversed twice |
| F     | Figure-eight       | medium               | curved self-crossing |
| G     | Two crossing points| medium, high         | two later legs cross an early diagonal |

### Coordinate conventions

- **Global:** WGS84. **Working frame:** local **ENU** (meters), anchored at a
  Hefei origin (`31.8206, 117.2290`); WGS84 ⇄ ENU is isolated in `geo.py`.
- **Body frame:** `+x` forward, `+y` left, `+z` up (right-handed).
- **Heading:** CCW angle of `+x` from ENU East, radians internally.
- **Lateral deviation sign:** positive = vehicle is to the **left** of the route.

---

## Acceptance criteria

Each case is graded against simulation ground truth. A case **passes** when:

- **correct-branch mismatches ≤ 3 frames** — where a mismatch is the matched
  segment differing from ground truth **and** the along-track error
  `|cursor_s − gt_s|` exceeding 3 m. This detects a real wrong-stroke jump
  (crossing strokes are tens of meters apart) while tolerating benign
  ~1–2 m segment-boundary timing under injected noise;
- **0 backward cursor jumps** (monotonic progress);
- **0 dropouts** (every frame yields a valid projection).

All 14 cases pass (0 mismatches, 0 backward jumps, 0 dropouts).

---

## Testing

```bash
pytest -v            # 56 unit + acceptance tests (fast, no browser)
```

**End-to-end (headless browser):**

```bash
pip install -e ".[e2e]"
python -m playwright install chromium
pytest -m e2e -v     # 17 tests
```

The e2e suite serves the repo root, drives the viewer in headless Chromium, and
asserts the canvases render with meaningful coverage (the route spans the
canvas, not just a stray pixel), telemetry populates, playback steps, the
scrubber seeks, the BEV layer rebuilds on case switch, and **no JS errors**
occur. It also includes `test_parity_py_js.py`, which runs 30 cases (2 routes ×
5 poses × 3 strategies) through both the Python `project_route` function and the
JS twin (`window.ProjectRoute`) and asserts path coordinates, `matched_seg`, and
`end_flag` all agree to within 1e-3 m. It's excluded from the default `pytest`
run via the `e2e` marker.

---

## Project layout

```
src/parking_proj/
  geo.py         WGS84 <-> local ENU; heading conventions
  route.py       Route value object: dense polyline, arc-length, tangents, segments
  geometry.py    builders: resample, arc, route-from-waypoints
  transform.py   ENU -> body-frame rotation (+x fwd, +y left)
  projection.py  the algorithm: stateful monotonic-cursor projector + heading gate
  simulate.py    two-layer simulation (tracking + RTK error), seeded
  scenarios.py   the 14-case matrix (geometry + tier + seed)
  grade.py       grade output vs ground truth (correct-branch, monotonic, dropouts)
  generate.py    run all cases -> out/<case>.json + out/index.json (+ verdicts)
viewer/          static HTML/CSS/JS playback tool (no algorithm logic)
tests/           pytest unit + acceptance; tests/e2e/ browser suite
docs/            design spec and implementation plan
run.sh           convenience runner (serve/gen/test/e2e/setup)
```

## Manual setup

```bash
python3.12 -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
python -m parking_proj.generate     # writes out/
python -m http.server 8000          # from repo root; open /viewer/index.html
```

## Real data

The viewer's **Real data** tab (default) plays back real vehicle datasets from
`dataset/<pkg>/` (each with `ego_route_llh.json` + `route_generation_result/planned_route.json`).
`./run.sh gen-real` runs the same projection algorithm on them (ego `llh`
GCJ-02→WGS-84→ENU, heading `yaw_boot`+boot→ENU offset, planned route as the
Route) and prebakes `out/real/<id>.json` plus an OSM basemap. The BEV shows the
region map with the planned route + ego track + direction arrows, all in WGS-84.
Real cases have no PASS/FAIL or true-lat-dev (no ground truth); `est_lat_dev` is
still shown. Real frames carry the same raw inputs (route, `meas_pose`,
`cursor_s`, `matched_seg`) as simulation frames; the viewer computes the
body-frame path live from them using the selected algorithm and sliders.

**Tile source (configurable).** By default tiles come from OpenStreetMap
(`https://tile.openstreetmap.org/{z}/{x}/{y}.png`). Override via env vars to use
a compliant provider / API key:

```bash
PARKING_TILE_URL="https://tile.example.com/{z}/{x}/{y}.png?apikey={key}" \
PARKING_TILE_KEY="your-key" \
PARKING_TILE_UA="your-app/1.0 (contact@you)" \
  ./run.sh gen-real
```

`{z}/{x}/{y}` and optional `{key}` are substituted per tile. If the tile server
is unreachable or refuses scripted access (returns an identical "blocked" tile),
the BEV falls back to a gray grid and the route/track/arrows still render.

---

## Design docs

- Spec: `docs/specs/parking_route_projection_all_checked_20260702213925.md`
- Plan: `docs/superpowers/plans/2026-07-02-parking-route-projection.md`
