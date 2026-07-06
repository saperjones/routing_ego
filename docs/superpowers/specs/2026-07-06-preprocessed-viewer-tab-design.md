# Design Spec: "Pre-processed" viewer tab

**Date:** 2026-07-06
**Status:** approved (ready for implementation plan)
**Scope:** viewer only (`viewer/*`) + one small e2e fixture + docs. No change to the
Python core (`src/parking_proj/*`) or to real/sim data generation.

---

## 1. Problem

A colleague wrapped the routing-projection algorithm into their own data pipeline
and produced results, but has no way to see whether those results are correct.
Their output is exactly the JSON that `offline_processing_routing_projection`
emits (`interface_offline.md`). We need a **visual verification** path in the
existing viewer: point it at a folder of the colleague's files and see the
projected path rendered, with nothing else in the tool changing.

Example folder (`data3in1/`) contains three files:

| File | Meaning |
|------|---------|
| `ego_route_llh.json` | ego pose stream, same as `dataset/<id>/ego_route_llh.json`. `llh` is **GCJ-02**. |
| `planned_route.json` | global routing path, same as the dataset file. `planned_route` is **WGS-84** `[lat,lon]`. |
| `routing_projection.json` | **new** — the per-frame driver-frame path produced by `offline_processing_routing_projection` (one of the five strategies; here `human_centered`). |

## 2. Requirements

- Add a **third left-panel tab** `Pre-processed`, alongside `Real data` and `Simulation`.
- Selecting it lets the user **pick a folder** (native browser folder picker).
- The viewer **renders the result** reusing the existing center views (BEV +
  Driver view). No algorithm is re-run; the tool only displays what the files contain.
- **Everything else is unchanged.** This is a pure visualization/verification feature.

## 3. Key data fact (removes a class of bugs)

`routing_projection.json` is the verbatim output of
`offline_processing_routing_projection` and is **self-contained per frame**:

```jsonc
{ "status": {...}, "meta": { "config": { "strategy": "human_centered", "behind_m": 5.0, "ahead_m": 40.0, ... } },
  "frames": [ { "timestamp_us": .., "pose": { "e": .., "n": .., "yaw": .., "lat": .., "lon": .. },
               "speed": .., "path": [[x,y],...], "cursor_s": .., "lat_dev": .., "matched_seg": .., "end_flag": false } ] }
```

Verified against `data3in1/`:
- frame[0] `pose.lat/lon = (31.8364909, 117.139121)` matches `planned_route[0] =
  (31.8364992, 117.1391112)` → **`pose.lat/lon` is WGS-84**.
- `ego_route_llh.json` `llh = (31.8346, 117.1447)` carries the classic GCJ-02
  offset → GCJ-02.

Consequences:
- The BEV needs **no GCJ-02 conversion in the browser**: both the route
  (`planned_route`) and the ego track (per-frame `pose.lat/lon`) are already WGS-84
  and align in the viewer's existing WGS-84 Web-Mercator BEV.
- Only **two** of the three files are used:
  - `routing_projection.json` — **required** (path + pose + telemetry per frame).
  - `planned_route.json` — used for the route polyline / waypoints / progress length.
  - `ego_route_llh.json` — **redundant** for these views (its GCJ-02 track is a
    lower-fidelity copy of the WGS-84 pose already in the projection file). Read
    only to validate the folder; not otherwise consumed.

## 4. Design decisions (locked)

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Folder selection | **Browser folder picker** (`<input type="file" webkitdirectory>`), files read client-side | Literal "select a folder"; no new server endpoint; works in the existing static viewer. |
| Views rendered | **BEV (top-down) + Driver view** | User asked for the BEV map plus the lower-middle driver view. |
| BEV basemap | **Gray graticule only** (no OSM tiles) | No prebaked tiles exist for a picked folder; matches real-data mode's graceful fallback; fully offline. |
| Right-panel controls in pre mode | **Disabled**, with a read-only config caption | The path comes from the file; live sliders would mislead. |
| Coordinate handling | Use WGS-84 `pose.lat/lon` + `planned_route` for BEV; use ENU `pose.e/n/yaw` only for body-frame overlay | No GCJ-02 port; ENU used only for relative transforms (origin cancels). |

## 5. Architecture

### 5.1 State & tab
- Add `#tab-pre` ("Pre-processed") to `#tabs` in `viewer/index.html`.
- New `STATE.mode = "pre"`. `selectTab("pre")` shows the folder-picker UI instead
  of fetching an index; clears `STATE.case`, offline, playback.
- Switching **away** from `pre` to `real`/`sim` clears the synthetic case and
  reloads that mode's index (regression: no stale pre case / stale BEV static layer).

### 5.2 Folder picker
- A hidden `<input type="file" webkitdirectory id="pre-folder">` plus a
  "Choose folder…" button rendered into the left panel (replacing the case list)
  when `mode==="pre"`.
- On `change`: collect `event.target.files`, match by **basename**
  (`f.name` / last path segment), ignoring `.DS_Store` and any nesting.
- Read `routing_projection.json` (required) and `planned_route.json` (if present)
  via `File.text()` → `JSON.parse`. Show a status/validation line
  (folder name + n_frames, or an error).

### 5.3 `buildPreCase(projection, plannedRoute)` — pure builder
On `window` (like `window.ProjectRoute`) so it is unit-testable in the page
context. Returns a synthetic "case" shaped so the **existing real-data renderers
accept it unchanged**:

```jsonc
{
  "mode": "pre",
  "case_id": "<folder name>",
  "route_llh":     [[lat,lon], ...],        // = plannedRoute.planned_route (WGS-84); [] if absent
  "route_waypoints": [[lat,lon], ...],      // = plannedRoute.waypoints (for panorama numbering); optional
  "ego_track_llh": [[pose.lat, pose.lon], ...],   // one per frame
  "route_arrow_idx": [..], "ego_arrow_idx": [..], // evenly spaced indices, like real-data
  "basemap": null,                          // -> gray graticule
  "route_total_len_m": <number|null>,       // WGS-84 cumulative arc length of route; null if no route
  "config": projection.meta.config,         // strategy + window, for the read-only caption
  "status_message": projection.status.message,
  "frames": [
    { "meas_pose": { "e": pose.e, "n": pose.n, "h": pose.yaw },
      "meas_ll":   { "lat": pose.lat, "lon": pose.lon },
      "speed": .., "est_lat_dev": lat_dev, "true_lat_dev": null,
      "cursor_s": .., "matched_seg": .., "end_flag": .., "path": [[x,y],...] }
  ],
  "verdict": null
}
```

Notes:
- `route_total_len_m` computed by summing great-circle (equirectangular is fine at
  this scale) distances along `planned_route`. Used only for the progress %.
- If `plannedRoute` is missing: `route_llh=[]`, `route_total_len_m=null`.

### 5.4 Rendering (branch on `mode==="pre"`)
- **BEV**: reuse the real-data renderer (`buildBevRealStatic` / `drawBevReal`).
  It already handles `basemap:null` → gray graticule and draws `route_llh` (blue) +
  `ego_track_llh` (black) + arrows + a per-frame yellow car marker from
  `meas_ll` + `meas_pose.h`. The synthetic case satisfies it as-is. If
  `route_llh` is empty, it renders track-only (graceful).
- **Driver view**: new `drawDriverPre(cv, f)` (top-down):
  - draws the file's `f.path` directly (green) — the deliverable;
  - overlays the real driven trajectory (orange dashed) by transforming every
    frame's `pose.(e,n)` into the current frame's body frame via `worldToBody`
    using **`f.meas_pose.h` (= pose.yaw)**;
  - car glyph at origin;
  - window (`behind_m`/`ahead_m`) taken from `case.config`, **not** the sliders.
  - Perspective toggle reuses `drawWindshield`, fed `f.path` as the ribbon path.
- **Panorama**: small helper draws `route_llh` in Web-Mercator (reusing
  `mercatorGlobalPx` + a fit transform) + numbered `route_waypoints` + a
  current-frame position dot from `meas_ll`. If no route, panorama is blank.
- **Telemetry**: heading from `pose.yaw`; speed; pos `(e,n)`; est lat dev =
  `lat_dev`; true lat dev = "–"; progress = `cursor_s / route_total_len_m`
  (or "–"); matched seg; frame; verdict = "— (pre-processed)".

### 5.5 Controls in pre mode
- Right-panel algorithm selector + all sliders + corner-style: **disabled**; add a
  read-only caption summarizing `case.config`
  (e.g. `pre-processed output — strategy=human_centered, ahead=40 m, behind=5 m`).
- **Compare-all** toggle: disabled (file has a single strategy).
- **Test offline (Python)** button: disabled (extend `updateOfflineButton` to
  require `mode==="real"`).
- Playback / scrubber / step / speed / perspective toggle: **work normally**.

## 6. Edge cases

| Case | Behavior |
|------|----------|
| `routing_projection.json` absent | Error line ("routing_projection.json not found in folder"); no render. |
| `status.generated == false` | Show `status.message`; still render `frames` if present, else stop with the message. |
| `planned_route.json` absent | BEV renders **track-only** + a warning; route/panorama omitted; progress shows "–". |
| Empty `frames` | Message "no frames in routing_projection.json"; no render. |
| `path[0].x > 0` (near route start, behind clipped) | Draw as-is; not an error. |
| `.DS_Store` / nested folders from `webkitdirectory` | Ignored (basename match on the three names). |
| Switch tab pre → real/sim | Clear synthetic case + BEV static caches; reload that mode's index. |

## 7. Testing

### 7.1 Unit (pure builder, via page context)
The repo tests JS logic through headless Chromium (no node runner). In the e2e
harness, `page.evaluate` calls `window.buildPreCase(projStub, routeStub)` with
tiny synthetic inputs and asserts:
- `route_llh.length == plannedRoute.planned_route.length`;
- `ego_track_llh.length == frames.length` and equals each `pose.lat/lon`;
- frame mapping (`meas_pose.h == pose.yaw`, `est_lat_dev == lat_dev`, `path` copied);
- `route_total_len_m > 0` for a multi-point route; `null` when route omitted.

### 7.2 E2E (playwright, mirrors `tests/e2e/test_viewer_e2e.py`)
1. Serve repo root; open the viewer; no JS errors.
2. Click `#tab-pre`; the "Choose folder…" button appears.
3. `set_input_files` the fixture's files onto `#pre-folder`.
4. Assert: `#bev` non-blank; `#driver` non-blank; telemetry populated (frame label,
   progress); `#scrubber` max == n_frames − 1.
5. Step frames (step-fwd / scrubber) → `#bev` signature changes (car marker moves);
   `#driver` stays non-blank.
6. Assert `#compare-toggle`, `#btn-offline`, `#algo-select`, and the sliders are
   **disabled** in pre mode.
7. Switch to `#tab-real` → pre case cleared; real index reloads (no stale render).

### 7.3 Fixture
A small **committed** 3-file dataset under `tests/e2e/fixtures/preproc/`
(~5 frames), derived from `data3in1/` but truncated, so e2e is fast and
deterministic. Avoids committing the 8 MB `data3in1/`. It must include a
`routing_projection.json` (required), a `planned_route.json`, and an
`ego_route_llh.json` (to mimic the real folder shape, even though unused).

### 7.4 Existing suites
`./run.sh test` and `./run.sh e2e` must still pass unchanged (this feature adds
tests; it does not alter existing render paths for real/sim modes — verified by
the case-switch regression guard already in `test_viewer_e2e.py`).

## 8. Docs

- Update **`spec_design.md`** (CLAUDE.md mandate) with a "Pre-processed viewer tab"
  subsection: what it is, which files it reads, the WGS-84 self-containment fact,
  and that it re-uses the real-data BEV renderer with a gray graticule.
- No change to `interface_offline.md` (the offline function is unchanged); optionally
  cross-reference the new tab from it.

## 9. Non-goals

- No re-running of the projection algorithm in pre mode (no JS twin, no Python call).
- No OSM tile fetching for pre mode.
- No GCJ-02 conversion in the browser.
- No editing of the offline function or its output format.
- No panorama/BEV parity with sim mode's ENU rendering (pre mode is WGS-84 like real).
