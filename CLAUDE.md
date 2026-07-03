# CLAUDE.md
* always update spec_design.md for any change 

## Commands

```bash
./run.sh gen        # regenerate simulation cases -> out/*.json
./run.sh gen-real   # prebake real datasets from dataset/ -> out/real/ (+ OSM tiles)
./run.sh test       # unit + acceptance suite
./run.sh e2e        # headless-browser end-to-end suite
```

## Architecture / invariants

- The system splits cleanly at JSON: the Python core (numpy) runs the projection
  algorithm and prebakes results; the static HTML viewer only replays prebaked JSON
  — it contains no matching logic.
- Real datasets are prebaked through the same Python algorithm into
  `out/real/<id>.json` with `mode:"real"` (no ground truth → no verdict or
  true_lat_dev). The viewer's real-data BEV renders an OSM basemap in Web-Mercator
  with route+track+arrows; all coordinates are converted to WGS-84 (ego `llh` is
  GCJ-02). When a compliant tile source is not reachable, the BEV falls back to a
  gray graticule — the route/track/arrows always render.
- `geo.py` is the single place WGS-84 ⇄ ENU conversion happens; nothing else
  touches lat/lon.
- All randomness is seeded (`numpy.random.default_rng`), so regeneration is
  bit-identical.
