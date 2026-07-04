# Clothoid Corner Smoothing (data-calibrated) — Design

**Date:** 2026-07-04
**Branch:** feature/portable-projection-function

## Problem

The `smoothed` strategy rounds route corners with a **circular-arc fillet**. An
arc bounds curvature (`κ ≤ 1/R_min`) but its curvature **jumps** from 0 to
`1/R` at the tangent point — the steering "snaps." That jump is what reads as
"not smooth enough."

Analysis of the 7 human-driven ego tracks in `dataset/` (resampled to 1 m and
speed-gated to remove RTK jitter and frequent stops) shows real turns cluster
around `R ≈ 2–4 m` and, crucially, that drivers **ease into** turns — curvature
ramps up gradually over roughly a **2–4 m entry**, not instantaneously. The
mathematical curve that ramps curvature linearly is the **clothoid (Euler
spiral)**. So the fix is to replace the arc with a clothoid transition, with its
one shape parameter (transition length) **calibrated from the human data**.

The data is too noisy/sparse (7 logs, many stops) for a learned/ML curve
template — it would overfit noise. Calibrating a single physically-meaningful
parameter is the robust way to "learn how the curve looks."

## 1. Offline calibration (learn once, bake a constant)

New script `tools/calibrate_clothoid.py`:

1. Load each dataset's ego track (ENU) and per-frame speed.
2. **Speed-gate:** drop frames below a speed threshold (default 0.5 m/s) — this
   removes the stationary jitter that produced impossible sub-metre "turns."
3. Resample the kept track to a fixed spacing (1.0 m).
4. Curvature per segment = heading change per metre; **segment turns** as
   contiguous runs with `κ > 1/15 m⁻¹`.
5. For each turn, measure the **entry ramp length**: arc-length from where κ
   first exceeds 10% of the turn's peak to where it reaches 90% of peak.
6. Report the distribution and take the **median entry ramp** as the calibrated
   `clothoid_transition_m` (expected ~2–4 m).

Output: a human-readable `docs/clothoid_calibration.md` (per-turn stats +
chosen value) and the single number used as the config default. **Runtime is
unaffected — the vehicle receives a constant; no data dependency at run time.**

## 2. Clothoid corner generator

Behind the existing `corner_curve(...)` seam (so nothing else changes), a
**symmetric clothoid–arc–clothoid** transition per corner. Given corner turn
angle `δ`, end radius `R = min_turn_radius_m`, and transition length `Lt`:

- Each Euler-spiral leg ramps curvature linearly `0 → 1/R` over `Lt`, turning
  the heading by `θ_sp = Lt / (2R)`.
- If `2·θ_sp < δ`: two spirals + a **middle circular arc** turning the
  remaining `δ − 2·θ_sp` at radius `R`.
- If `2·θ_sp ≥ δ`: no middle arc; use two shorter symmetric spirals that
  together turn `δ` (scale the spiral so it ends before reaching `1/R`).
- Sample the spiral positions via **Fresnel integrals**. For parking geometry
  `θ_sp = Lt/(2R) ≈ 0.3 rad` is small, so a short Taylor series for the Fresnel
  `C/S` converges immediately — no lookup table, no convergence loop
  (embeddable).

**Guarantees:** curvature is **continuous** (no entry jump) and peak `κ = 1/R`
(still drivable). **Short-leg handling:** the spiral+arc needs a tangent length
on each leg; if it exceeds half the available leg, shrink `Lt` (and, if still
too tight, fall back toward the plain arc as `Lt → 0`) so the corner always
fits — curvature stays ≤ 1/R (feasible) throughout.

## 3. Configuration (all configurable)

`ProjectConfig` / JS `DEFAULT_CONFIG` gain:

- `corner_style: "clothoid"` (default) | `"arc"` — the arc stays available for
  comparison and tests.
- `clothoid_transition_m` — calibrated default from step 1.

`min_turn_radius_m` and `corner_angle_deg` are unchanged. `smoothed` dispatches
to arc or clothoid per `corner_style`; `raw`/`centered` are untouched.

## 4. Python↔JS parity

Port the clothoid generator + Fresnel approximation to
`viewer/project_route.js`. Extend `tests/e2e/test_parity_py_js.py` to include
`corner_style="clothoid"` fixtures (multi-segment/corner routes) and confirm the
Python and JS paths agree within 1e-3 m.

## 5. Viewer

The left panel gains a **corner-style selector** (`#corner-style`: arc /
clothoid, default clothoid) and a **transition-length slider**
(`#p-transition`, default = calibrated value). Changing either recomputes the
current frame's path live (output-only params — cursor memo untouched). Both
driver views use it; BEV unchanged.

## 6. Tests + docs

- Python (`tests/test_smoothing.py` / a clothoid test): the clothoid path's
  **curvature is continuous** — the max change in per-sample turn-rate is far
  smaller than the arc's single jump — while peak curvature stays ≤ 1/R_min;
  short-leg case still fits and stays feasible; `corner_style="arc"` reproduces
  today's arc behavior.
- e2e (web-visible): on the corner case, the **clothoid** path's max
  consecutive curvature change is clearly smaller than the **arc** path's
  (proving it is smoother), and the transition slider changes the curve.
- Docs: `docs/clothoid_calibration.md` (calibration result); update
  `docs/project_route_function.md`, `algorithm_description.md`, `README.md`.

## Non-goals / future

- Learned/ML curve templates (data insufficient; revisit with more/cleaner logs).
- Speed-scaled transition length (calibrate a single value now; scaling by
  approach speed is a future refinement).
- Curvature-rate (steering-rate) bound as an explicit constraint — the clothoid
  gives continuity; an explicit rate limit is out of scope here.
