import math
import numpy as np
from parking_proj import geometry as g
from parking_proj.simulate import simulate, SimConfig, TIERS


def _corner():
    return g.route_from_waypoints([(0, 0), (25, 0), (25, 25)], [1, 2, 3], ds=0.1)


def test_deterministic_same_seed():
    r = _corner()
    a = simulate(r, SimConfig(tier="medium", seed=7))
    b = simulate(r, SimConfig(tier="medium", seed=7))
    assert len(a) == len(b)
    assert all(abs(x.meas_e - y.meas_e) < 1e-12 for x, y in zip(a, b))


def test_tracking_cap_respected():
    r = g.route_from_waypoints([(0, 0), (40, 0)], [1, 2], ds=0.1)
    frames = simulate(r, SimConfig(tier="low", seed=1))
    # true lateral offset from the y=0 centerline stays within cap+margin
    assert max(abs(f.true_n) for f in frames) <= 0.4 + 0.05


def test_localization_cap_respected():
    r = g.route_from_waypoints([(0, 0), (40, 0)], [1, 2], ds=0.1)
    frames = simulate(r, SimConfig(tier="high", seed=2))
    for f in frames:
        d = math.hypot(f.meas_e - f.true_e, f.meas_n - f.true_n)
        assert d <= 2.0 + 1e-6
        assert abs(f.meas_yaw - f.true_yaw) <= math.radians(0.05) + 1e-9


def test_heading_continuous_through_corner():
    r = _corner()
    frames = simulate(r, SimConfig(tier="low", seed=3))
    dyaws = [abs((frames[i + 1].true_yaw - frames[i].true_yaw + math.pi)
                 % (2 * math.pi) - math.pi) for i in range(len(frames) - 1)]
    # rounded corner => no single-frame ~90deg jump
    assert max(dyaws) < math.radians(20.0)


def test_gt_progress_monotonic():
    r = _corner()
    frames = simulate(r, SimConfig(tier="medium", seed=4))
    gs = [f.gt_s for f in frames]
    assert all(gs[i + 1] >= gs[i] - 1e-9 for i in range(len(gs) - 1))
