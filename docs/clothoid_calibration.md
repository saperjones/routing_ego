# Clothoid transition-length calibration

Speed gate: >= 0.5 m/s; resample 0.5 m; curvature smoothed (moving average window 5); turn threshold kappa > 0.067 (R<15 m).

Turns with a measured entry ramp of 0 m are excluded as unresolvable sharp/jitter artifacts. The calibrated value is the median of the remaining resolved ramps.

| dataset | turns detected | resolved turns | median entry ramp (m) |
|---|---|---|---|
| dev_CHERY_M32T_46651_ALL_MANUAL_ | 73 | 56 | 1.5 |
| dev_CHERY_M32T_46651_ALL_MANUAL_ | 54 | 45 | 1.5 |
| dev_CHERY_M32T_46651_ALL_MANUAL_ | 80 | 62 | 1.0 |
| dev_CHERY_M32T_46651_ALL_MANUAL_ | 72 | 64 | 1.8 |
| dev_CHERY_M32T_46651_ALL_MANUAL_ | 69 | 57 | 1.5 |
| dev_CHERY_M32T_46651_ALL_MANUAL_ | 68 | 55 | 1.0 |
| dev_CHERY_M32T_46651_ALL_MANUAL_ | 72 | 57 | 1.5 |

**Calibrated `clothoid_transition_m` = 1.5 m** (median entry ramp over resolved turns, clamped to [1, 6]; 0-ramp jitter turns excluded).
