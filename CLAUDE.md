# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Projects a globally-planned parking route into the vehicle **body frame** each
frame, robustly under RTK localization error and on self-crossing routes. Two
halves meet at one JSON artifact: a **Python core** (`src/parking_proj/`) that
runs the algorithm over a seeded simulation and prebakes 14 graded test cases to
`out/*.json`, and a **static HTML/Canvas viewer** (`viewer/`) that replays them.

## Commands

The project uses a venv at `.venv` and a `run.sh` wrapper. `run.sh` always
operates from the repo root and creates the venv on first use.

```bash
./run.sh            # generate cases + serve viewer + open browser (default)
./run.sh gen        # regenerate out/*.json only
./run.sh test       # unit + acceptance suite
./run.sh e2e        # headless-browser end-to-end suite (installs playwright+chromium)
./run.sh setup      # create venv + install deps
PORT=9000 ./run.sh  # override port (default 8000)
```

Direct invocations (activate `.venv` first, or prefix with `./.venv/bin/`):

```bash
pytest -v                                  # 31 unit + acceptance tests (e2e excluded by marker)
pytest tests/test_projection.py::test_crossing_correct_branch_zero_error -v   # a single test
pytest -m e2e -v                           # 9 browser tests (needs: pip install -e ".[e2e]" && playwright install chromium)
PYTHONPATH=src python -m parking_proj.generate   # writes out/
```

Gotchas that will bite you:
- **`python -m parking_proj.generate` needs `PYTHONPATH=src`** unless you ran
  `pip install -e .` (the package lives under `src/`; pytest already sets this
  via `pyproject.toml`).
- **Serve the viewer from the repo ROOT**, not from `viewer/`. The viewer
  fetches `../out/...`, which only resolves when the server root is the repo
  root: `python -m http.server 8000` then open `/viewer/index.html`.
- `out/` is git-ignored; regenerate it with `./run.sh gen`.
- The `e2e` suite is excluded from the default `pytest` run via
  `addopts = "-m 'not e2e'"`; run it explicitly with `-m e2e`.

## Architecture (big picture)

Data flow: `scenarios.py` (geometry + tier + seed) → `route.py` (dense polyline)
→ `simulate.py` (true trajectory + measured poses + ground truth) →
`projection.py` (the algorithm, fed the *measured* poses) → `grade.py` (vs
ground truth) → `generate.py` (bundles to JSON + verdict) → `viewer/` (playback).

**The viewer holds NO matching logic** — it only replays prebaked decisions and
applies the fixed display rotation (`worldToBody` in `viewer.js`, mirroring
`transform.py`). This is a structural invariant, not a style choice: it enforces
"no recompute / no re-randomize on click." Do not move algorithm logic into JS.

**The algorithm (`projection.py`) is a stateful matcher.** It carries a
monotonic **progress cursor** `cursor_s` (arc-length along the route). Each frame
it matches only within a bounded forward window `[cursor_s − eps_back,
cursor_s + w_search]`, gated by heading agreement, then advances
`cursor_s = max(cursor_s, matched_s)`. The `max` is what guarantees "0 backward
jumps"; the small window (`w_search=3.5 m`, far below the tens-of-meters
inter-stroke gap) is what prevents wrong-stroke locks at self-crossings. Lateral
deviation and `matched_seg` are read **at the cursor point**, not a free
nearest-point — deliberately, since a free nearest point is ambiguous at
crossings.

**The simulation (`simulate.py`) is two layers over the centerline:** (1) a true
trajectory = smoothed/corner-rounded centerline + a capped lateral offset; (2) a
measured pose = true + correlated RTK bias + white noise, capped. Ground truth
(`gt_s`, `gt_seg`) comes from the **ordinal walk**, never a nearest-point search,
so it stays unambiguous at crossings. All randomness is from one seeded
`default_rng` → regeneration is bit-identical.

## Invariants to preserve when editing

- **No pose/heading smoothing anywhere in `projection.py`** — an upstream module
  owns that; this project must not add it. (Simulation-side smoothing for corner
  rounding is separate and fine.)
- **Coordinate conventions** (see `spec_design.md` / `algorithm_description.md`):
  working frame is local ENU (meters, Hefei origin); body frame is `+x` forward,
  `+y` left, `+z` up; heading is CCW radians from ENU East. Lateral deviation is
  positive = vehicle **left** of the route. These signs are consistent across
  `transform.py`, `projection.py`, `grade.py`, and the viewer — change one, fix
  all four (there are unit tests pinning the transform at h = 0/90/180/−90°).
- **Acceptance metric is along-track-aware** (`grade.py`): a correct-branch
  mismatch requires `matched_seg != gt_seg` AND `|cursor_s − gt_s| > 3.0 m`
  (`BRANCH_TOL`). This distinguishes real wrong-stroke jumps from benign ~1–2 m
  boundary-timing under noise. A case passes iff `mismatches <= 3 and
  backward_jumps == 0 and dropouts == 0`. All 14 cases currently pass at 0/0/0;
  if a change regresses this, fix the code or the geometry — do not loosen the
  bar. (This refines the spec's original literal "≤3 frame" rule.)
- `route_from_waypoints` **fails loudly** if waypoints collapse to the same
  resampled index — that signals bad geometry, not something to catch.

## Reference docs

- `README.md` — user-facing overview and run instructions.
- `spec_design.md` — plain-language design spec.
- `algorithm_description.md` — the mathematics (transforms, matching, sim, metric).
- `docs/specs/` and `docs/superpowers/plans/` — the original spec and the
  task-by-task implementation plan.
