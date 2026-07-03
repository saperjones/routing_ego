# Parking Route Projection

Projects a globally-planned navigation route into the **vehicle-body frame**
each frame, so a low-speed parking auto-driving controller can follow it — the
route as seen from the driver's viewpoint. The hard part is doing this
**robustly** under RTK localization error and on routes that **cross
themselves**, without ever locking onto the wrong branch.

The project is two halves that meet at one JSON artifact:

- a **Python core** (numpy) that runs the projection algorithm over a seeded
  two-layer simulation and prebakes 14 graded test cases to `out/*.json`;
- a **static HTML/Canvas viewer** that plays those cases back (route panorama,
  BEV, driver-view, telemetry, frame-by-frame + continuous playback).

The viewer holds **no matching logic** — it only replays prebaked results and
applies a fixed display rotation — so what you see is exactly what the
algorithm produced.

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
- **Center bottom — Driver view (`+x` up, `+y` left):** the projected route
  slice in the body frame, car at the origin. Tick the **perspective** checkbox
  to switch it into a windshield 3D view — a pinhole projection of the route
  onto the road plane ahead (horizon, ground grid, and the route as a ribbon
  that narrows toward the vanishing point). Camera constants live in `PERSP` in
  `viewer.js`.  
  The **"remove lateral offset"** checkbox (default ON) controls whether the
  driver view draws the prebaked `follow_path` (cross-track offset removed,
  heading error and curvature preserved) or the raw route slice. It applies to
  both the top-down and the perspective driver views; the BEV is unaffected.
- **Bottom — playback:** step ◀ ▶, play/pause, scrubber, speed (0.5×/1×/2×).
- **Right — telemetry:** heading, speed, position, estimated & true lateral
  deviation, progress, matched segment, frame index, and the case verdict.

Test cases are **prebaked and deterministic** — clicking a case never
regenerates or re-randomizes it, so differences between cases are meaningful.

---

## How it works

### The projection algorithm (`src/parking_proj/projection.py`)

A stateful projector carries a **progress cursor** `cursor_s` (arc-length along
the route). Each frame it:

1. matches the measured pose only within a bounded **forward window**
   `[cursor_s − 0.3 m, cursor_s + 15 m]` (the forward reach is the configurable
   **endurable offset**, default 15 m — wide enough to catch the vehicle after a
   localization jump, still far below the ≥64 m self-crossing stroke gap), gated
   by **heading agreement**
   (route tangent vs. vehicle heading, 60° gate; widened for a single frame if
   nothing passes, so a frame is never dropped);
2. advances the cursor **monotonically** (`cursor_s = max(cursor_s, matched_s)`)
   — valid because the product rule forbids re-walking a road or U-turning;
3. emits the route slice in `[cursor_s − 5 m, cursor_s + 20 m]` transformed into
   the body frame, plus telemetry (lateral deviation, matched segment,
   end-of-route flag);
4. computes a **follow-path** — a forward-only, 0–70 m look-ahead, 0.5 m
   sampled body-frame path with the cross-track offset removed (see
   [Follow-path output](#follow-path-output-follow_path--lat_shift)), emitted
   as `follow_path` (`[[x,y],…]`) and `lat_shift` (scalar) per frame.

### Follow-path output (`follow_path` / `lat_shift`)

Each frame the algorithm also emits:

| Field | Type | Description |
|-------|------|-------------|
| `follow_path` | `[[x,y],…]` body frame | Route look-ahead with lateral offset removed. `+x` forward, `+y` left. Forward-only window `[cursor_s, cursor_s + 70 m]`, truncated at route end, sampled every 0.5 m. |
| `lat_shift` | scalar (m) | The meters subtracted from every point's lateral coordinate. Equals the car-frame body-`y` of the anchor (route point at `cursor_s`). At zero heading error `lat_shift ≈ −est_lat_dev`. |

The anchor point (`s = cursor_s`) lands at `y = 0` in `follow_path`; all forward coordinates (`x`) are identical to the raw body-frame values, so heading error and curvature are preserved. `est_lat_dev`, `cursor_s`, and `matched_seg` are **unaffected** — `follow_path` is a read-only post-processing step.

The per-case `config` object includes `follow_ahead: 70.0` and `follow_ds: 0.5` so downstream consumers can interpret the window without reading source code. The exported `follow_path` is intended as the deliverable for downstream path-following controllers: an offset-free, 70 m look-ahead, body-frame path at 10 Hz.

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
pytest -v            # 47 unit + acceptance tests (fast, no browser)
```

**End-to-end (headless browser):**

```bash
pip install -e ".[e2e]"
python -m playwright install chromium
pytest -m e2e -v     # 15 tests
```

The e2e suite serves the repo root, drives the viewer in headless Chromium, and
asserts the canvases render with meaningful coverage (the route spans the
canvas, not just a stray pixel), telemetry populates, playback steps, the
scrubber seeks, the BEV layer rebuilds on case switch, and **no JS errors**
occur. It also verifies the follow-path **web effect** at the pixel level: with
"remove lateral offset" on, the driver-view route stroke renders at the car's
centreline (directly ahead); with it off, the stroke is displaced sideways, and
the two renders differ horizontally by exactly `lat_shift × pixels-per-metre`.
It's excluded from the default `pytest` run via the `e2e` marker.

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
still shown. Each real frame includes `follow_path` and `lat_shift` (same
contract as simulation; see [Follow-path output](#follow-path-output-follow_path--lat_shift)).

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
