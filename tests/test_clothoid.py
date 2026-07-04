import math
from parking_proj.clothoid import clothoid_corner


def _turn_rates(pts):
    # per-consecutive-segment heading change (proxy for curvature * ds)
    rates = []
    for i in range(1, len(pts)):
        dx, dy = pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1]
        rates.append(math.atan2(dy, dx))
    return rates


def test_turns_by_delta():
    delta = math.radians(90)
    pts, T = clothoid_corner(delta, radius=5.0, transition=3.0)
    # final heading (last segment bearing) ~ delta
    dx, dy = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
    assert math.atan2(dy, dx) == pytest_approx(delta, abs=0.03)
    assert T > 0


def test_curvature_is_continuous_no_jump():
    delta = math.radians(90)
    pts, _ = clothoid_corner(delta, radius=5.0, transition=3.0, internal_ds=0.1)
    rates = _turn_rates(pts)
    # consecutive change in bearing-rate (2nd difference of heading) has no big jump
    jumps = [abs(rates[i] - rates[i-1]) for i in range(1, len(rates))]
    # each internal step turns <= ds/R = 0.1/5 = 0.02 rad; step-to-step change is tiny
    assert max(jumps) < 0.01


def test_peak_curvature_within_min_radius():
    delta = math.radians(90)
    ds = 0.1
    pts, _ = clothoid_corner(delta, radius=5.0, transition=3.0, internal_ds=ds)
    rates = _turn_rates(pts)
    per_step = [abs((rates[i] - rates[i-1] + math.pi) % (2*math.pi) - math.pi) for i in range(1, len(rates))]
    # peak turn-rate per step <= ds/R + tolerance  => curvature <= 1/R
    assert max(per_step) <= ds / 5.0 + 0.005


def test_degenerate_inputs_return_trivial():
    assert clothoid_corner(0.0, 5.0, 3.0)[0] == [(0.0, 0.0)]
    assert clothoid_corner(math.radians(90), 5.0, 0.0)[0] == [(0.0, 0.0)]


def pytest_approx(*a, **k):
    import pytest
    return pytest.approx(*a, **k)
