"""Offline: estimate the clothoid transition length from human ego tracks.

Speed-gates out stationary RTK jitter, resamples to 0.5 m, smooths per-step
curvature with a 5-sample moving average, segments genuine turns, and measures
each turn's curvature entry-ramp (10%->90% of peak). Turns whose measured ramp
is 0 m are excluded as unresolvable sharp/jitter artifacts. The median over
resolved (ramp > 0) turns is the calibrated transition length.
Run: PYTHONPATH=src python tools/calibrate_clothoid.py
"""
import glob
import os
import numpy as np
from parking_proj.realdata import load_dataset, is_dataset_dir

SPEED_MIN = 0.5      # m/s — drop stationary frames (jitter)
DS = 0.5             # resample spacing (m)
KAPPA_TURN = 1.0 / 15.0
SMOOTH_WIN = 5       # curvature moving-average window


def _entry_ramps(ds):
    e = np.asarray(ds.meas_e); n = np.asarray(ds.meas_n); v = np.asarray(ds.speed)
    keep = v >= SPEED_MIN
    e, n = e[keep], n[keep]
    if len(e) < 10:
        return []
    seg = np.hypot(np.diff(e), np.diff(n)); s = np.concatenate([[0], np.cumsum(seg)])
    if s[-1] < DS * 5:
        return []
    su = np.arange(0, s[-1], DS); eu = np.interp(su, s, e); nu = np.interp(su, s, n)
    psi = np.arctan2(np.diff(nu), np.diff(eu))
    kap = np.abs((np.diff(psi) + np.pi) % (2 * np.pi) - np.pi) / DS
    # Smooth curvature to reduce RTK jitter before turn detection and ramp measurement
    kap = np.convolve(kap, np.ones(SMOOTH_WIN) / SMOOTH_WIN, mode="same")
    turn = kap > KAPPA_TURN
    ramps, i = [], 0
    while i < len(turn):
        if turn[i]:
            j = i
            while j < len(turn) and turn[j]:
                j += 1
            pk = kap[i:j].max()
            peak = i + int(np.argmax(kap[i:j]))
            lo = i
            while lo < peak and kap[lo] < 0.1 * pk:
                lo += 1
            hi = lo
            while hi < peak and kap[hi] < 0.9 * pk:
                hi += 1
            ramps.append((hi - lo) * DS)
            i = j
        else:
            i += 1
    return ramps


def main():
    per, allr = [], []
    for d in sorted(glob.glob("dataset/*")):
        if not is_dataset_dir(d):
            continue
        r = _entry_ramps(load_dataset(d))
        allr += r
        resolved = [x for x in r if x > 0]
        med_resolved = float(np.median(resolved)) if resolved else float("nan")
        per.append((os.path.basename(d), len(r), len(resolved), med_resolved))
    all_resolved = [x for x in allr if x > 0]
    value = float(np.median(all_resolved)) if all_resolved else 3.0
    value = round(min(6.0, max(1.0, value)), 1)
    lines = ["# Clothoid transition-length calibration", "",
             f"Speed gate: >= {SPEED_MIN} m/s; resample {DS} m; "
             f"curvature smoothed (moving average window {SMOOTH_WIN}); "
             f"turn threshold kappa > {KAPPA_TURN:.3f} (R<15 m).", "",
             "Turns with a measured entry ramp of 0 m are excluded as unresolvable "
             "sharp/jitter artifacts. The calibrated value is the median of the "
             "remaining resolved ramps.", "",
             "| dataset | turns detected | resolved turns | median entry ramp (m) |",
             "|---|---|---|---|"]
    for name, k, kr, med in per:
        med_str = f"{med:.1f}" if not (isinstance(med, float) and med != med) else "nan"
        lines.append(f"| {name[:32]} | {k} | {kr} | {med_str} |")
    lines += ["", f"**Calibrated `clothoid_transition_m` = {value} m** "
              f"(median entry ramp over resolved turns, clamped to [1, 6]; "
              f"0-ramp jitter turns excluded)."]
    with open("docs/clothoid_calibration.md", "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"CLOTHOID_TRANSITION_M={value}")


if __name__ == "__main__":
    main()
