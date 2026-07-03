import math
import numpy as np
from pathlib import Path
from parking_proj import realdata

DATASET = ("dataset/dev_CHERY_M32T_46651_ALL_MANUAL_2026-06-22-14-08-25_"
           "20260625_101425_annotation")


def test_theta_recovers_synthetic_rotation():
    rng = np.random.default_rng(0)
    enu = np.cumsum(rng.normal(0, 1, (500, 2)), axis=0)  # a wandering path
    th = math.radians(33.0)
    R = np.array([[math.cos(-th), -math.sin(-th)], [math.sin(-th), math.cos(-th)]])
    boot = enu @ R.T           # boot = ENU rotated by -th, so recovered theta = +th
    est, scale = realdata.estimate_boot_to_enu_theta(boot, enu)
    assert abs(math.degrees(est) - 33.0) < 0.5
    assert abs(scale - 1.0) < 0.02


def test_is_dataset_dir_and_load():
    d = Path(DATASET)
    if not d.exists():
        import pytest
        pytest.skip("sample dataset not present")
    assert realdata.is_dataset_dir(str(d))
    ds = realdata.load_dataset(str(d))
    assert ds.route.length > 100.0
    assert ds.meas_e.shape == ds.meas_n.shape == ds.meas_yaw.shape
    assert len(ds.meas_e) > 1000
    # theta near the measured ~33 deg for this dataset
    assert 20.0 < math.degrees(ds.theta_rad) < 45.0
    # measured poses are finite
    assert np.all(np.isfinite(ds.meas_e)) and np.all(np.isfinite(ds.meas_yaw))
