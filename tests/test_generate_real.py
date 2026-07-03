import math
import numpy as np
from pathlib import Path
from parking_proj.generate_real import build_real_case_dict, arrow_indices

DATASET = ("dataset/dev_CHERY_M32T_46651_ALL_MANUAL_2026-06-22-14-08-25_"
           "20260625_101425_annotation")


def test_arrow_indices_spacing():
    # a straight 100 m E-W line at lat0
    lat0, lon0 = 31.834, 117.14
    import parking_proj.geo as geo
    lls = []
    for e in np.linspace(0, 100, 500):
        # invert enu_about: lon = lon0 + e/(DEG*R*cos), lat=lat0
        lls.append([lat0, lon0 + e / (geo._DEG * geo._EARTH_R * math.cos(lat0 * geo._DEG))])
    idx = arrow_indices(np.array(lls), lat0, lon0, step_m=20.0)
    assert 4 <= len(idx) <= 6           # ~5 arrows over 100 m at 20 m
    assert idx[0] == 0


def test_real_case_dict_schema_and_monotonic():
    if not Path(DATASET).exists():
        import pytest
        pytest.skip("sample dataset not present")
    case = build_real_case_dict(DATASET, basemap=None)
    assert case["mode"] == "real"
    assert "route" in case and "route_llh" in case and "ego_track_llh" in case
    assert case["basemap"] is None
    assert "theta_deg" in case and "origin" in case
    f0 = case["frames"][0]
    for k in ("t", "speed", "meas_pose", "meas_ll", "cursor_s", "matched_seg",
              "est_lat_dev", "end_flag"):
        assert k in f0
    assert "true_lat_dev" not in f0 and "gt_seg" not in f0
    cs = [f["cursor_s"] for f in case["frames"]]
    assert all(cs[i + 1] >= cs[i] - 1e-6 for i in range(len(cs) - 1))  # monotonic
    assert all(math.isfinite(f["est_lat_dev"]) for f in case["frames"])
    assert case["route_arrow_idx"] and case["ego_arrow_idx"]
