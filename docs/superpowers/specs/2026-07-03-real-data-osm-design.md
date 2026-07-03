# Design: Real-Data Ingestion + OSM Basemap for the Route-Projection Viewer

**Date:** 2026-07-03
**Status:** approved (brainstorming) — input to `superpowers:writing-plans`
**Extends:** the existing simulation tool (`spec_design.md`, `algorithm_description.md`).

## 1. Goal & scope

Run the **existing** projection algorithm on **real vehicle data** (the
`dataset/` packages) alongside the synthetic simulation, and visualize it:

- Left panel gains two tabs: **Simulation** (existing) and **Real data** (new),
  **defaulting to Real data**.
- Real data lists the **first-level** `dataset/` subdirectories that qualify
  (contain `ego_route_llh.json` **and**
  `route_generation_result/planned_route.json`).
- Clicking a dataset loads a **prebaked** result (algorithm run offline) and
  plays it back exactly like a simulation case.
- For real data, the center **BEV** shows an **OSM basemap** of the region
  (Hefei, < 5 km²), with the planned route + the actual ego trajectory overlaid,
  **direction arrows at fixed intervals**, everything in **WGS-84**.
- All other displays (driver-view / windshield perspective, telemetry, playback)
  are **unchanged**, just fed from real data.
- **No heavy screen flicker** when the map is loaded.

**Non-goals:** no live/interactive slippy map; no re-implementation of the
algorithm in JS; no change to the simulation cases; no special reverse/loop
handling beyond what the algorithm already does.

## 2. Architecture — same invariant

**Python computes, the viewer replays.** Real datasets are preprocessed offline
into the **same per-frame JSON schema** the viewer already consumes (plus a
`mode:"real"` flag and a basemap reference, minus the ground-truth fields). The
viewer additions are UI-only: the tabs and a real-data BEV renderer. The
`Projector` algorithm is **not modified**.

New/changed units:
- `src/parking_proj/realdata.py` — adapter: read a dataset dir → build the
  `Route` + the per-frame measured-pose stream for the algorithm.
- `src/parking_proj/geo.py` — add GCJ-02 ⇄ WGS-84 (the WGS→ENU already exists).
- `src/parking_proj/osm.py` — prep-time tile fetch + stitch + geo-reference.
- `src/parking_proj/generate_real.py` (or extend `generate.py`) — run all
  dataset dirs → `out/real/<id>.json` + `out/real/<id>/basemap.png|json` +
  `out/real/index.json`.
- `viewer/viewer.js` — tabs + real-data BEV renderer (static/dynamic layers).

## 3. Coordinate handling (authoritative)

- **Global datum: WGS-84 everywhere in this feature.**
- `ego_route_llh.json` `llh` is **GCJ-02** → convert **GCJ-02→WGS-84 first**
  (see `docs/ego_route_llh_format.md`).
- `route_generation_result/planned_route.json` is already **WGS-84**, as
  `[lat, lon]` pairs.
- **Algorithm frame:** local **ENU meters**, origin = mean of the dataset's
  WGS-84 route points. Planned route and ego poses both → this ENU.
- **BEV display frame:** **Web Mercator** pixel space defined by the basemap
  image (§6); overlays are placed by `lon/lat → mercator → pixel`.
- **Heading:** `yaw_enu = yaw_boot + θ`, where `θ` is the per-dataset boot→ENU
  rotation (§5).

## 4. Real-data → algorithm input mapping (the adapter)

| Algorithm needs | Real-data field | Processing |
|---|---|---|
| **Route** (navigation path) | `planned_route.json` `planned_route` (2067 `[lat,lon]`, WGS-84) | dense polyline → WGS-84→ENU; `waypoints` (11) mapped to nearest indices → `Route.waypoint_indices`/labels 1..11 |
| **Per-frame position** | `ego_route_llh.json` `points[].llh` (GCJ-02) | GCJ-02→WGS-84→ENU meters |
| **Per-frame heading** | `points[].yaw_boot` | `+ θ` (per-dataset boot→ENU offset, §5) |
| **Speed** | `points[].v` | m/s, as-is |
| **Timestamp** | `points[].timestamp_us` | playback cadence (~20 Hz) |

- **No ground truth.** A real drive has no known "true segment," so real cases
  produce **no PASS/FAIL verdict, no `gt_s`/`gt_seg`, no `true_lat_dev`, and no
  correct-branch metric.** `est_lat_dev` (ego's signed lateral offset from the
  planned route, at the cursor) is still computed and shown.
- The `Projector` runs **unchanged** on `(measured pose, Route)`; the driver-view
  and windshield perspective project the planned route into the ego body frame
  exactly as in simulation.

## 5. boot→ENU heading offset θ (verified method)

The `ego_route_llh.json` "boot" frame is a **rigid rotation of ENU** by a
per-session constant (the vehicle's orientation at power-on). There is **no
`heading` field**; the only orientation fields (`yaw`==`yaw_boot`,
`rotation_boot`, `yaw_rate`) live in the boot frame and are internally exact
(radians). Estimate θ **from positions only** (independent of any yaw field):

1. `a_i = position_boot[i+k] − position_boot[i]` (boot-frame displacements);
   `b_i = ENU[i+k] − ENU[i]` (from `llh` GCJ→WGS→ENU). Stride `k≈10`; keep frames
   with `|a_i| > 0.3 m`.
2. `θ = atan2( Σ(a_x·b_y − a_y·b_x), Σ(a_x·b_x + a_y·b_y) )` — the least-squares
   2D rotation; magnitude-weighted, no wrap-around, robust.
3. Validate `scale = Σ|b| / Σ|a| ≈ 1` (confirms pure rotation / same metric).
4. `yaw_enu[i] = yaw_boot[i] + θ`.

**Verified on the sample dataset:** θ = 33.48°, scale = 1.003, bootstrap over
half-samples = 33.48 ± 0.10° (per-sample scatter ~7° averages down to a ±0.1°
constant). `yaw_boot` ≡ boot-frame velocity direction (−0.05°), so `yaw_boot+θ`
equals the ENU motion heading but is smoother and defined even when slow.

**Fallback:** if too few moving frames (`|a_i|>0.3 m`) exist to estimate θ
reliably, fall back to the motion-derived heading directly and log a warning.

## 6. OSM basemap (prep-time, static)

Per dataset, at prep time (network used **once**, here only):

1. Compute the WGS-84 bbox of (planned route ∪ ego track) + margin (~40 m).
2. Choose slippy zoom `z` = the max zoom whose covering tile count ≤ a cap
   (~25 tiles); for < 5 km² this is z ≈ 16–17.
3. Fetch the covering XYZ tiles from an OSM raster source (with a `User-Agent`,
   tiles cached locally), stitch into one **`basemap.png`**.
4. Write **`basemap.json`** = `{ z, tile_x0, tile_y0, width_px, height_px,
   lon/lat bounds, and the mercator↔pixel mapping }`.
5. **Fallback:** if fetching fails (offline/sandbox), write `basemap:null`; the
   viewer draws a plain gray background with a light graticule and still renders
   all overlays. (Algorithm results are unaffected.)

Tiles are EPSG:3857 (Web Mercator); `lon/lat→mercator→pixel` is linear given
`basemap.json`, so overlays align exactly with the raster.

## 7. Viewer changes

- **Tabs** `#tab-sim` / `#tab-real`; default **real**. Switching loads that
  index (`out/index.json` for sim, `out/real/index.json` for real) into the
  existing case-list.
- **Real-data BEV renderer** (mirrors the existing anti-flicker split):
  - **Static offscreen layer** (drawn once per case): basemap image (or gray
    fallback) + planned route polyline (color A) + ego full trajectory (color B)
    + **direction arrows every ~20 m arc-length on both paths**, oriented along
    the local tangent.
  - **Dynamic layer** (per frame): oriented ego car marker at the current pose +
    current progress dot; each frame = blit static + draw dynamic.
  - **Fixed** mercator-pixel→canvas transform; no pan/zoom during playback (no
    reflow, no re-fetch) → no flicker.
- **Sim BEV renderer unchanged.** A per-case `mode` flag selects the renderer.
- **Driver-view / windshield perspective + telemetry:** same code; for real
  cases the telemetry hides/greys the **verdict** and **true lat dev** rows.
- Panorama (left) for real cases shows the planned route with numbered waypoints
  as today (optional; can reuse the sim panorama drawing on the ENU route).

## 8. Per-case JSON schema (real)

Same per-frame record as simulation **minus** `gt_seg` / `gt_s` /
`true_lat_dev`, **plus**, at the top level:
- `mode: "real"`, `dataset_id`, `theta_deg`, ENU `origin` (lat0/lon0),
- `basemap`: `{png, z, bounds, pixel-mapping}` or `null`,
- `route`: ENU polyline (`points_e/n/s/waypoint_indices/labels`) — drives
  driver-view + slice (as sim) — **and** `route_llh` (`[lat,lon]`) for the BEV,
- `ego_track_llh`: `[lat,lon]` full driven track for the BEV static layer.

Per frame: `t, speed, meas_pose{e,n,h}` (ENU, for driver-view) + `meas_ll`
(`lat,lon`, for BEV), `cursor_s, matched_seg, est_lat_dev, end_flag`.

## 9. Error handling

| Situation | Handling |
|---|---|
| Dataset dir missing a required file | excluded from the Real-data list (noted in `out/real/index.json`) |
| OSM tiles unavailable at prep | `basemap:null` → gray graticule fallback; overlays still drawn |
| Too few moving frames for θ | fall back to motion-derived heading; log a warning |
| Ego drive diverges from planned route | Projector's forward window handles it; cursor stalls → `end_flag` |
| Very large frame count | playback throttled to the data cadence; static layer drawn once |

## 10. Testing

**Python unit:**
- `geo`: GCJ-02↔WGS-84 round-trip (< ~1 m) and a known reference offset.
- θ estimator: recovers a synthetic rotation applied to a track; `scale≈1`.
- `osm`: `lon/lat↔mercator↔pixel` round-trip; XYZ tile-bounds math; zoom
  selection respects the tile cap.
- `realdata` adapter: builds a valid `Route` (waypoint indices monotonic) and a
  finite pose stream from a dataset dir; GCJ→WGS applied.

**Integration:** each real case builds; `cursor_s` monotonic (0 backward
jumps); 0 dropouts; `est_lat_dev` finite for all frames. (No branch metric —
no ground truth.)

**e2e (playwright):** Real-data tab is default and lists the dataset dir(s);
clicking loads; BEV shows basemap-or-gray + planned route + ego track + arrows +
car (coverage assertions); switching to Simulation tab still works; no JS
errors; no per-frame rescale/flicker (fixed transform; static layer blitted).

## 11. Acceptance criteria

1. Left panel has **Simulation / Real data** tabs; **Real data is default** and
   lists qualifying `dataset/` subdirs; clicking plays one back.
2. The algorithm runs on real data via §4's field mapping; `cursor_s` monotonic,
   0 dropouts.
3. Real BEV shows the region map (or gray fallback) with **planned route + ego
   trajectory + direction arrows**, all WGS-84, aligned to the basemap.
4. Real cases show **no PASS/FAIL and no true-lat-dev**; `est_lat_dev` is shown.
5. **No heavy flicker** during playback (static basemap/route/arrows drawn once;
   only car + progress redraw).
6. Simulation tab and all other displays (driver-view, perspective, telemetry
   layout) are unchanged.

## 12. TBD

- OSM tile source/policy for the prep step (dev uses a standard OSM raster; may
  swap for a permitted/self-hosted source). Not on the acceptance path — the
  gray fallback keeps everything working offline.
- Exact arrow interval and the two path colors (default ~20 m; refine in impl).
