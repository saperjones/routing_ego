# Format Spec: `ego_route_llh.json`

Ego-vehicle **localization / egomotion**, sampled at the output frames of an AVP
(automated valet parking) annotation package. One point per output frame, in
time order, carrying the vehicle's global position (WGS84), local metric pose
and orientation, velocities, yaw and yaw-rate, and speed.

> **This is NOT the planned routing path.** The planned route lives in
> `route_generation_result/planned_route.json` (fields `planned_route`,
> `waypoints`, `ego_track`, as `[lat, lon]` pairs). `ego_route_llh.json` is the
> *measured ego trajectory / attitude* record. Do not confuse the two.

Observed from: `dev_CHERY_M32T_46651_ALL_MANUAL_2026-06-22-14-08-25_20260625_101425_annotation/ego_route_llh.json`
(`schema_version: avp_annotation_schema_2026-06-23_v6`).

---

## 1. Purpose and relationships

- **What it holds:** the complete global ego route for the clip — every output
  frame's pose/attitude/velocity, produced by the iflytek localization
  (`iflytek_localization_egomotion`) aligned to the global output-frame
  selection.
- **How other files reference it:** per-frame annotation JSONs contain a
  `data_route` whose `route_index` values map into `points[].route_index` in
  this file. Per `data_route_policy`, each frame's `data_route` is *downsampled*
  by `data_route_stride`; **this file keeps the complete (un-downsampled) global
  route**, and the first and last global route points are always included.
- **Relationship to `planned_route.json`:** `ego_route_llh.json` is the raw
  measured trajectory + attitude; `planned_route.json` is the *derived, planned*
  centerline the vehicle should follow. They share the same drive but differ in
  content, sampling, and coordinate layout (see §6).

---

## 2. Top-level object

Root is a JSON object with metadata followed by the `points` array.

| Field | Type | Meaning | Example / observed |
|-------|------|---------|--------------------|
| `schema_version` | string | Annotation schema id/version. | `avp_annotation_schema_2026-06-23_v6` |
| `producer` | object | Generator provenance: `{name, version}`. | name `build_json_data_all_sample_annotations_slots.py`, version `bag_to_annotation_2026-06-23_route_global` |
| `source` | string | Origin of the pose data. | `iflytek_localization_egomotion aligned with the global output frame selection` |
| `coordinate` | string | Human-readable note on the coordinate systems used (WGS84 lon/lat/height + boot/global pose & velocity; the `route_index` mapping). | see file |
| `output_frame_stride` | int | Stride used when selecting output frames from the source. `1` = every source frame kept. | `1` |
| `frame_sampling_enabled` | bool | Whether additional frame sampling was applied. | `false` |
| `data_route_stride` | int | Downsample stride applied to the per-frame `data_route` in *other* files (not to this file). | `10` |
| `data_route_policy` | string | Text describing the per-frame `data_route` downsampling policy and that this file retains the full route. | see file |
| `fields` | string[] | The canonical per-point field names (see §3.1). | 10 names |
| `point_count` | int | Number of entries in `points` (`== len(points)`). | `5047` |
| `points` | object[] | The per-frame ego records, time-ordered (see §3). | length `5047` |

---

## 3. `points[]` — per-frame ego record

One object per output frame. Frames are ordered by increasing time; in the
observed file `route_index == source_frame_index` for every point and both run
`0 … point_count-1` contiguously. Sampling interval ≈ **50 ms (~20 Hz)**. Every
observed point has `is_interpolated: true` (poses interpolated to the output
frame time). All points share one identical key set.

The declared `fields` array lists the 10 **canonical** fields; each point object
additionally carries convenience/duplicate keys (timestamp variants, flattened
`longitude/latitude/height`, `rotation_boot`, `yaw`, and interpolation
metadata). All keys observed are documented below.

### 3.1 Canonical fields (from `fields`)

| Field | Type | Units | Meaning |
|-------|------|-------|---------|
| `route_index` | int | — | Index of this point within the global route; the value other files' `data_route.route_index` maps to. |
| `source_frame_index` | int | — | Index of the source frame this point came from. |
| `timestamp_us` | int | microseconds | Frame timestamp (epoch, µs). |
| `llh` | object | — | Geodetic position, see §3.3. |
| `position_boot` | object | meters | Position in the local **boot** frame `{x, y, z}`, see §4. |
| `velocity_body` | object | m/s | Velocity in the **vehicle body** frame `{x, y, z}` (`x` ≈ longitudinal/forward; `z` observed `0`). |
| `velocity_boot` | object | m/s | Same velocity expressed in the **boot** frame `{x, y, z}`. |
| `yaw_boot` | float | radians | Vehicle heading (yaw) in the boot frame; observed range within `(−π, π]`. |
| `yaw_rate` | float | rad/s | Yaw angular rate. |
| `v` | float | m/s | Scalar speed; equals `‖velocity_body‖` (and `‖velocity_boot‖`) in observed data. |

### 3.2 Additional keys present on each point

| Field | Type | Units | Meaning |
|-------|------|-------|---------|
| `timestamp_ns` | int | nanoseconds | `timestamp_us × 1000`. |
| `timestamp_ns_str` | string | nanoseconds | Same value as a decimal **string** (avoids 64-bit/float precision loss in JS). |
| `reference_timestamp_us` / `_ns` / `_ns_str` | int / int / string | µs / ns / ns | Reference-clock timestamp; equals `timestamp_*` in observed data. |
| `pose_timestamp_us` / `_ns` / `_ns_str` | int / int / string | µs / ns / ns | Timestamp of the pose used; equals `timestamp_*` in observed data. |
| `longitude`, `latitude`, `height` | float | deg, deg, m | Flattened copy of `llh.longitude/latitude/height`. |
| `rotation_boot` | object | — | Orientation quaternion in the boot frame `{w, x, y, z}`; `yaw_boot` is its planar yaw. |
| `yaw` | float | radians | Duplicate of `yaw_boot` in observed data. |
| `is_interpolated` | bool | — | Whether the pose was interpolated to the frame time (all `true` observed). |
| `interpolation_source_timestamp_us` | int[2] | microseconds | The two source pose timestamps bracketing the interpolation. |

### 3.3 `llh` sub-object

| Field | Type | Units | Meaning |
|-------|------|-------|---------|
| `available` | bool | — | Whether a geodetic fix is present (all `true` observed). |
| `type` | int | — | Fix/solution type enum. Observed value `2`. Exact enum meaning **(TBD)**. |
| `longitude` | float | degrees | WGS84 longitude. |
| `latitude` | float | degrees | WGS84 latitude. |
| `height` | float | meters | Ellipsoidal/geoid height **(datum TBD)**. |

---

## 4. Coordinate frames and conventions

- **Geodetic (global):** WGS84 `longitude` / `latitude` (degrees) and `height`
  (meters). This is the global position of the vehicle.
- **Boot frame (`*_boot`):** a local right-handed metric frame (meters) anchored
  at system **boot** (session start). Holds `position_boot {x,y,z}`,
  `velocity_boot {x,y,z}`, `rotation_boot {w,x,y,z}`, and `yaw_boot`. Observed
  `position_boot.z` is large-negative (≈ −208 … −212) and varies slowly, so it
  is a full 3D frame, not planar. The exact axis convention and origin/datum of
  this frame are **(TBD)** — confirm z-axis sign/direction before using it.
- **Body frame (`velocity_body`):** the vehicle-fixed frame. `x` is the
  longitudinal/forward component (in observed data `velocity_body.x ≈ v`), `y`
  is lateral, `z` is `0` (planar). Expected to match the program's ego(curr)
  convention `+x` forward / `+y` left / `+z` up (cf. `JSON_FIELD_DESCRIPTION_V7`),
  but the `y`/`z` sign convention here is **(TBD)** — confirm against a known
  turn before relying on the sign.
- **Heading:** `yaw_boot` (radians) is the vehicle heading in the boot frame and
  equals the planar yaw of `rotation_boot`. (Sanity-checked: for point 0,
  `2·atan2(z, w)` of `rotation_boot` reduces to `yaw_boot`.)

---

## 5. Data semantics and observed invariants

- **Ordering:** `points` are time-ordered by `timestamp_us` (strictly
  increasing).
- **Indexing:** `route_index` is contiguous `0 … point_count-1`; in the observed
  file it equals `source_frame_index` (do not assume this always holds — treat
  `route_index` as the authoritative cross-file key).
- **Rate:** ≈ 20 Hz (Δt ≈ 49–51 ms).
- **Interpolation:** all observed poses are interpolated
  (`is_interpolated: true`) to the output-frame times, with the source
  timestamps in `interpolation_source_timestamp_us`.
- **Speed consistency:** `v == ‖velocity_body‖ == ‖velocity_boot‖`
  (velocity_body and velocity_boot are the same vector in two frames).
- **Timestamps in three forms:** integer µs, integer ns, and ns-as-string.
  Prefer `timestamp_us` for arithmetic; use `*_ns_str` when a consumer cannot
  hold a 64-bit integer exactly (e.g. JavaScript).
- **Duplicated fields:** `longitude/latitude/height` (top-level) duplicate
  `llh.*`; `yaw` duplicates `yaw_boot`. They are provided for convenience;
  treat `llh.*` and `yaw_boot` as canonical.

---

## 6. Contrast with `planned_route.json` (do not confuse)

| Aspect | `ego_route_llh.json` | `route_generation_result/planned_route.json` |
|--------|----------------------|----------------------------------------------|
| Content | Measured ego pose/attitude/velocity per frame | Planned route + waypoints + source ego track |
| Key arrays | `points[]` (rich objects) | `planned_route[]`, `waypoints[]`, `ego_track[]` (coordinate pairs) |
| Coordinate layout | explicit `longitude`,`latitude`,`height` + boot-frame `x,y,z` | `[lat, lon]` pairs (latitude first) |
| Yaw units | radians (`yaw_boot`) | degrees (`waypoint_yaws`, `start_yaw_deg`) |
| Extras | velocities, yaw-rate, quaternion, timestamps, interpolation | `params{}`, `ego_length_m`, `planned_length_m` |
| Role | "where/what attitude the car actually was" | "where the car is supposed to go" |

The cross-file key is `route_index`: per-frame `data_route.route_index` →
`ego_route_llh.json points[].route_index`.

---

## 7. Open items (TBD)

- `llh.type` enum meaning (observed `2`).
- Boot-frame exact origin/datum and axis convention (notably the sign/direction
  of `z`, observed large-negative).
- `velocity_body` `y`/`z` sign convention (forward is confirmed via `x ≈ v`).
- `height` datum (ellipsoidal vs geoid/MSL).
- Whether `route_index == source_frame_index` always holds, or only when
  `output_frame_stride == 1` and `frame_sampling_enabled == false`.
