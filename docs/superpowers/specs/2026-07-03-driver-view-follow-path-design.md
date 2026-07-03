# Driver-View Follow-Path (lateral-offset removal) — Design

**Date:** 2026-07-03
**Branch:** feature/parking-route-projection

## Problem

The driver view is drawn in the car's body frame, centered on the car's
measured pose. When the car sits laterally off the nav route — which it often
does, because the driver does not track the line exactly and the nav line is
not necessarily a street centerline — the route renders shifted to the left or
right. That is confusing: the driver sees "the route is 1 m to my right"
instead of "drive this way."

We want the route to emanate from **directly ahead of the car marker**, with
the left/right offset removed, so it shows *which direction to drive*, not
*how far off-center the route happens to be*.

This is not only a display concern. The re-centered path in the car's body
frame is an **algorithm output** that will be transferred to downstream
consumers (e.g. a low-speed follow controller), so it must be produced by the
Python core and written into the prebaked JSON — the viewer only replays it.

## The transform

Each frame the projector already commits to a point on the route at `cursor_s`.
Call the route point there the **anchor** `P` — the projection of the car onto
the nav route.

Today each route point renders in the body frame as
`b = to_body_frame(pt_e − pose_e, pt_n − pose_n, heading)` (`+x` forward,
`+y` left). The re-centering is a **rigid lateral shift** in that same
car-heading frame:

```
lat_shift = to_body_frame(P_e − pose_e, P_n − pose_n, heading).y   # car-frame lateral of the anchor
follow_x  = b.x                                                    # forward: unchanged
follow_y  = b.y − lat_shift                                        # subtract the same y from every point
```

Properties:

- **Forward coordinates are untouched.** Only the left/right offset is removed —
  exactly the "偏左偏右的 offset," not longitudinal position.
- **Frame stays car-heading-up.** Heading error survives: if the car points 10°
  off the route, `follow_path` visibly angles 10° ahead — that angle *is* the
  steering cue.
- **Route shape is preserved** (it is a rigid translation), so curvature ahead
  still shows where the road turns.
- The anchor's lateral goes to 0, so the path emanates from directly ahead of
  the car marker (car stays at the origin).

`lat_shift` is the car-frame lateral of the anchor (car→route vector), whereas
`est_lat_dev` is the route-normal deviation (route→car vector). They are the
same magnitude with **opposite sign** when heading error is ~0 (`lat_shift ≈
−est_lat_dev`). Both are reported, so no information is lost.

## Output contract (Python core)

`projection.py` gains a helper that, given the route, pose, heading, and
`cursor_s`, returns the re-anchored body-frame path over a **forward-only**
window, sampled at a fixed spacing:

- Window: `[cursor_s, cursor_s + 70 m]` along the route, **truncated at route
  end** (fewer points near the end).
- Spacing: **0.5 m** (≤ 141 points/frame).
- Frame: body frame, `+x` forward, `+y` left, car at origin, lateral offset
  removed.

`generate.py` and `generate_real.py` emit per frame:

- `follow_path`: `[[x, y], …]` (meters, rounded to 3 dp). **The transferable
  deliverable.**
- `lat_shift`: meters removed this frame (traceability, 4 dp).
- `est_lat_dev`: unchanged, still emitted.

Case-level `config` gains `follow_ahead: 70.0` and `follow_ds: 0.5`.

For real-data frames the same fields are emitted (real mode already runs the
same projector; no ground truth is involved, so this is unaffected by the
no-verdict rule).

## Grading is unaffected

`follow_path` is a derived output. Matching (`cursor_s`, `matched_seg`),
monotonicity, and dropout grading are unchanged. All 14 simulation cases grade
byte-identically to before.

## Viewer changes

- New checkbox beside the perspective toggle: **"remove lateral offset"**,
  default **checked**.
- `drawDriver` (top-down) and `drawWindshield` (perspective):
  - **checked** → draw the emitted `follow_path` directly (both views share the
    one field), clipped to each view's existing display window.
  - **unchecked** → draw the raw slice from `route` + `meas_pose` exactly as
    today.
  - Car marker stays at the origin in both states.
- The **display windows are unchanged** (top-down `[−5 m, +20 m]`, windshield
  `[0, config.ahead]`). The full 0–70 m export lives in the JSON; the viewer
  shows the near part. In re-centered mode the top-down view shows only the
  forward part (the 70 m export has no "behind" data); raw mode keeps the 5 m
  of behind-context.
- **BEV is unchanged.** It shows true geography, so the raw lateral offset stays
  visible there and in the `est_lat_dev` telemetry — nothing is hidden.

## Docs

- `spec_design.md` (mandated by CLAUDE.md): document the re-anchoring transform
  and the `follow_path` output contract.
- `algorithm_description.md`: add the math (anchor, `lat_shift`, rigid shift,
  car-heading frame, forward-only 0–70 m window).
- `README.md`: note the new output field, `lat_shift`, and the toggle.

## Tests

Python:

- Straight route, constant lateral offset injected ⇒ every `follow_path` point
  has `|y| ≈ 0` (offset nulled) while `est_lat_dev` still reports the offset.
- Constant heading error, on-line laterally ⇒ `follow_path` is a straight line
  angled by the heading error (forward coords increase, `y` grows linearly).
- Window/sampling: points spaced 0.5 m; last point ≤ `cursor_s + 70 m`;
  truncated cleanly at route end.

e2e:

- The toggle is checked by default.
- Toggling changes the `#driver` canvas signature (re-centered ≠ raw).
- No JS errors; canvases still render with meaningful coverage.
