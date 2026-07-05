# `offline_processing_routing_projection` — offline interface

Post-process a pre-processed bag into a per-frame **ego-frame (body-frame)
routing path** by driving the authoritative Python `project_route`. No browser,
no config file — every parameter is passed on the command line.

Module: `src/parking_proj/offline_processing_routing_projection.py`

---

## How to invoke

### Command line

```bash
python -m parking_proj.offline_processing_routing_projection \
  --ego-json   <ego_route_llh.json> \
  --route-json <planned_route.json> \
  --out        <out.json> \
  [--strategy ... and any ProjectConfig flag]
```

Exit code `0` = results generated, non-zero = failure (also reflected in the
output `status` object).

### As a library

```python
from parking_proj.offline_processing_routing_projection import run
from parking_proj.project_route import ProjectConfig

result = run(ego_path, route_path, ProjectConfig(strategy="human_centered"))
# result is the dict described under "Output" below
```

- `run(ego_path, route_path, config) -> dict` — in-memory core (raises on load error).
- `main(argv=None) -> int` — the CLI wrapper (writes `--out`, returns the exit code).

---

## Input

Two files (paths may live anywhere):

| Arg | File | Provides |
|-----|------|----------|
| `--ego-json` | `ego_route_llh.json` | the **ego pose stream**: `points[]` with `timestamp_us`, `llh` (GCJ-02 lat/lon), `yaw_boot`, `position_boot`, `v` |
| `--route-json` | `route_generation_result/planned_route.json` | the **global routing path** to follow: `planned_route` (WGS-84 `[lat,lon]`) + `waypoints` |

Loading (via `realdata.load_dataset_files`) converts ego `llh` GCJ-02 → WGS-84 →
local ENU, computes the boot→ENU heading offset `θ` so `yaw = yaw_boot + θ`, and
builds the planned route as an ENU `Route`. This matches the viewer's real-data
pipeline exactly.

---

## Output

A single JSON file (`--out`). **One record per input ego frame**, in input
order, with `timestamp_us` synced to the input:

```jsonc
{
  "status": { "generated": true, "n_frames": 4953, "message": "" },
  "meta": {
    "ego_json": "...", "route_json": "...",
    "frame": "body: +x forward, +y left, meters",
    "config": { /* every ProjectConfig field actually used */ },
    "generated_by": "offline_processing_routing_projection.py"
  },
  "frames": [
    {
      "timestamp_us": 1782098347256492,          // == input point's timestamp_us
      "pose": { "e": .., "n": .., "yaw": .., "lat": .., "lon": .. },
      "speed": 3.12,                              // m/s
      "path": [[x, y], ...],                      // EGO-FRAME path: +x forward, +y left, meters
                                                  //   spans [cursor_s - behind_m, cursor_s + ahead_m]
      "cursor_s": .., "lat_dev": .., "matched_seg": .., "end_flag": false
    }
    // ... one per frame
  ]
}
```

- **`path`** is the deliverable: the routing path in the driver/ego frame for
  that timestamp. `+x` forward, `+y` left, meters.
- **`lat_dev`**: signed cross-track deviation (positive = vehicle left of route).
- **Success indicator**: the process **exit code** (`0` / non-zero) *and* the
  top-level `status.generated` boolean (+ `n_frames`, `message`). On failure the
  file still gets written with `generated: false` and a `message`.

---

## Configuration

Every field of `ProjectConfig` is an optional CLI flag; omit to use its default.

| Flag | Default | Meaning |
|------|---------|---------|
| `--strategy` | `smoothed` | `raw` / `centered` / `smoothed` / `human` / `human_centered` |
| `--behind-m` | `5.0` | path start behind the car (m) |
| `--ahead-m` | `40.0` | path end ahead of the car (m) |
| `--sample-ds-m` | `0.5` | spacing between path points (m) |
| `--search-ahead-m` | `15.0` | forward match-window reach (m) |
| `--search-back-m` | `0.3` | backward match tolerance (m) |
| `--heading-gate-deg` | `60.0` | heading-agreement gate (deg) |
| `--min-turn-radius-m` | `8.0` | corner radius for `smoothed` arc/clothoid (m) |
| `--corner-angle-deg` | `10.0` | only corners sharper than this are treated (deg) |
| `--simplify-eps-m` | `0.20` | corner-detection tolerance (m) |
| `--corner-style` | `clothoid` | `clothoid` / `arc` / `driver` — **only affects `smoothed`** |
| `--clothoid-transition-m` | `8.0` | transition length; **also the Gaussian smoothing width for `human`/`human_centered`** — bigger = smoother |
| `--human-cut-m` | `2.2` | `human`/`human_centered` inside corner-cut at a 90° turn (m), calibrated from ego tracks |

---

## Recommended: best `human_centered` configuration

`human_centered` gives a smooth, human-like line that is also **centered** on the
car and **pointing straight forward** (both lateral offset and heading error
neutralized). These are the smooth-by-default values used in the viewer — a good
starting point that needs no tuning:

```bash
python -m parking_proj.offline_processing_routing_projection \
  --ego-json   dataset/<id>/ego_route_llh.json \
  --route-json dataset/<id>/route_generation_result/planned_route.json \
  --out        out/offline_<id>.json \
  --strategy human_centered \
  --ahead-m 40 \
  --behind-m 5 \
  --sample-ds-m 0.5 \
  --clothoid-transition-m 8 \
  --human-cut-m 2.2
```

Equivalent library call:

```python
from parking_proj.project_route import ProjectConfig
cfg = ProjectConfig(
    strategy="human_centered",
    ahead_m=40.0,
    behind_m=5.0,
    sample_ds_m=0.5,
    clothoid_transition_m=8.0,   # smoothness knob for human_centered (bigger = smoother)
    human_cut_m=2.2,             # inside corner-cut at 90°
)
```

Notes for `human_centered`:
- **`--clothoid-transition-m` is the one smoothness knob** (it sets the Gaussian
  smoothing width). Increase it for gentler, earlier turns; decrease for tighter.
- **`--corner-style` and `--min-turn-radius-m` do not apply** to `human_centered`
  (they only affect the `smoothed` strategy) — leave them at defaults.
- Since the path is centered and forward-pointing, `path[0]` starts near
  `[−behind_m, 0]` and the near points have `y ≈ 0`.
