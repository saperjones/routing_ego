# CLAUDE.md
* always update spec_design.md for any change 

## Commands

```bash
./run.sh gen        # regenerate simulation cases -> out/*.json
./run.sh gen-real   # prebake real datasets from dataset/ -> out/real/ (+ OSM tiles)
./run.sh test       # unit + acceptance suite
./run.sh e2e        # headless-browser end-to-end suite

# offline post-processing: bag -> per-frame ego-frame path via Python project_route
python -m parking_proj.offline_processing_routing_projection \
  --ego-json <ego_route_llh.json> --route-json <planned_route.json> --out <out.json> [--strategy ...]
```

## Architecture / invariants

- The system splits at JSON: the Python core (numpy) runs the stateful matching
  algorithm and prebakes decisions (`cursor_s`, `matched_seg`, telemetry) plus
  raw route geometry and ego poses. The JSON carries **no** precomputed
  body-frame path.
- The viewer (`viewer/project_route.js`, `window.ProjectRoute`) runs the shared
  projection algorithm live each frame — same math as `src/parking_proj/project_route.py`
  — so the algorithm selector (`#algo-select`: raw / centered / smoothed) and
  parameter sliders (`#p-radius`, `#p-behind`, `#p-ahead`, `#p-corner`) update
  the driver view instantly. Python (`project_route.py`) is authoritative;
  `tests/e2e/test_parity_py_js.py` binds the two (30 cases, tolerance 1e-3 m).
- Real datasets are prebaked through the same Python algorithm into
  `out/real/<id>.json` with `mode:"real"` (no ground truth → no verdict or
  true_lat_dev). The viewer's real-data BEV renders an OSM basemap in Web-Mercator
  with route+track+arrows; all coordinates are converted to WGS-84 (ego `llh` is
  GCJ-02). When a compliant tile source is not reachable, the BEV falls back to a
  gray graticule — the route/track/arrows always render. Tile source is
  configurable via env: `PARKING_TILE_URL` (template with `{z}/{x}/{y}`, optional
  `{key}`), `PARKING_TILE_KEY`, `PARKING_TILE_UA`. Default is the OSM community
  server `tile.openstreetmap.de` — the main `tile.openstreetmap.org` blocks
  scripted/bulk access (serves an identical placeholder tile), so baking against
  it yields `has_map:false`. Set `PARKING_TILE_URL` for a keyed provider or
  satellite imagery (e.g. Esri World Imagery uses `.../{z}/{y}/{x}`).
- `geo.py` is the single place WGS-84 ⇄ ENU conversion happens; nothing else
  touches lat/lon.
- `offline_processing_routing_projection.py` is the offline CLI interface: it
  loads a bag's `ego_route_llh.json` + `planned_route.json` (via
  `realdata.load_dataset_files`) and drives the authoritative Python
  `project_route` per frame, writing one ego-frame path record per input frame
  (timestamps synced) plus a `status`/exit-code indicator. All params are CLI
  flags — no config file. `run.sh serve` runs `parking_proj.viewer_server`
  (static files + a `POST /api/offline` endpoint) so the viewer's "Test offline"
  button can run it live and overlay the Python path against the JS twin.
- All randomness is seeded (`numpy.random.default_rng`), so regeneration is
  bit-identical.
