# Offline Routing Projection — Design

**Date:** 2026-07-05
**Goal:** A command-line interface function that, offline (post-processing a
pre-processed bag), turns a generated global route + an ego pose stream into a
per-frame **ego-frame (body-frame) routing path** by driving the authoritative
Python `project_route`. Plus a viewer button that runs it live and overlays the
result against the in-browser JS twin for parity checking.

## Components

### 1. `src/parking_proj/offline_processing_routing_projection.py`

The interface function. Two entry points sharing one code path:

- **`run(ego_path, route_path, config) -> dict`** — in-memory core.
  1. Load via `realdata.load_dataset_files(ego_path, route_path)` (see §4):
     planned route → ENU `Route`; ego `llh` GCJ-02→WGS-84→ENU; per-frame
     `yaw = yaw_boot + θ`; `timestamp_us`, `v`.
  2. Loop every ego frame calling
     `project_route(route, e, n, yaw, config, state, speed)`, carrying `state`
     across frames (monotonic cursor). Corner smoothing is cached on the route
     (world-once), so it is computed once, not per frame.
  3. Return the output dict (§3).
- **`main(argv=None) -> int`** — argparse CLI; builds `config`, calls `run`,
  writes `--out`, prints the status, returns the exit code. Run via
  `python -m parking_proj.offline_processing_routing_projection`.

**CLI flags.** Required: `--ego-json`, `--route-json`, `--out`. Optional — every
`ProjectConfig` field, each defaulting to its `ProjectConfig` default:
`--strategy`, `--behind-m`, `--ahead-m`, `--sample-ds-m`, `--search-ahead-m`,
`--search-back-m`, `--heading-gate-deg`, `--min-turn-radius-m`,
`--corner-angle-deg`, `--simplify-eps-m`, `--corner-style`,
`--clothoid-transition-m`, `--human-cut-m`. No config file.

### 2. Output JSON (§3)

One record per input ego frame, in input order, timestamps synced to the input:

```jsonc
{
  "status": { "generated": true, "n_frames": 4953, "message": "" },
  "meta": {
    "ego_json": "...", "route_json": "...",
    "frame": "body: +x forward, +y left, meters",
    "config": { /* all ProjectConfig fields used */ },
    "generated_by": "offline_processing_routing_projection.py"
  },
  "frames": [
    { "timestamp_us": 1782098347256492,
      "pose": { "e": .., "n": .., "yaw": .., "lat": .., "lon": .. },
      "speed": 3.12,
      "path": [[x, y], ...],
      "cursor_s": .., "lat_dev": .., "matched_seg": .., "end_flag": false }
  ]
}
```

**Success indicator:** exit code `0` on success / non-zero on failure, AND the
top-level `status` object. On failure (missing/invalid file, empty route, parse
error) write `status.generated=false` + `message`, print to stderr, exit
non-zero.

### 3. Backend: `src/parking_proj/viewer_server.py`

A stdlib `SimpleHTTPRequestHandler` subclass serving static files from the repo
root (as `http.server` does today) plus one route:

- `POST /api/offline` body `{ "dataset_id": "...", "config": {...} }` →
  resolve `dataset/<dataset_id>/ego_route_llh.json` +
  `route_generation_result/planned_route.json`, call
  `offline_processing_routing_projection.run(...)`, respond `200` with the dict.
  On error respond `500` with `{status:{generated:false,message}}`.

`run.sh serve` runs `python -m parking_proj.viewer_server $PORT` instead of
`python -m http.server`. The e2e `base_url` fixture launches the same module so
the endpoint exists under test.

### 4. `realdata.py` refactor

Extract `load_dataset_files(ego_path, planned_path, dataset_id=None) ->
RealDataset` holding the current parsing; `load_dataset(dir)` becomes a thin
wrapper that resolves the two paths and calls it. No behavior change.

### 5. Viewer UI

On the **Real-data tab only** (sim cases have no dataset files), a **"Test
offline (Python)"** button + a status line near the driver view:

- Click → status "Start processing…", POST `{dataset_id, config}` (config =
  selected `strategy` + the live slider values). On success → "Done — you can
  check the results." and store the per-frame offline result on `STATE.offline`,
  keyed by the (dataset, config) it was computed for.
- While an offline result is loaded, the bottom driver view renders, per frame
  (scrubbable): the **Python offline path (solid green)** + the **live JS-twin
  path (dashed blue)** for the same selected algorithm + the usual **orange
  real-trajectory**. Top-down draws both; perspective draws the offline path as
  the ribbon plus the live path as a dashed line.
- **Only the selected algorithm** — the button is **disabled while compare-all
  is on** and on the **Simulation tab**.
- Changing the algorithm or any slider, switching case/tab, or toggling
  compare **clears** `STATE.offline` (reverts to the normal live view), so a
  shown offline path always matches the settings it was computed with.
- Frame alignment: offline `frames[i]` ↔ viewer frame `i` (same input order),
  sanity-checked by `timestamp_us`.

## Testing

- **Python unit** (`tests/`): `run(...)` on a real `dataset/` case → exit-0
  shape: `status.generated=true`, `n_frames == points count`, timestamps equal
  input `timestamp_us`, every `path` a non-empty list of `[x,y]` pairs;
  `--strategy raw` vs `centered` change the output; determinism (same inputs →
  identical dict). Failure: missing file → `main` non-zero + `generated=false`.
- **Server integration** (`tests/`): start `viewer_server` on a free port in a
  thread, `POST /api/offline` for a real dataset → 200 + `generated=true`;
  unknown dataset → 500 + `generated=false`.
- **e2e**: Real tab, click "Test offline", wait for the "done" status, assert
  the driver view renders (offline + live overlay) and the two agree closely
  (parity through the UI); assert the button is disabled in compare-all mode.

## Scope (YAGNI)

No config file, no `run.sh` subcommand for the offline tool (it's `python -m`),
no plotting. Reuses the existing `realdata` + `project_route` + JS-twin stack.
