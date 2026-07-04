import math
import numpy as np
import pytest
from parking_proj.geometry import route_from_waypoints
from parking_proj.project_route import project_route, ProjectConfig, ProjectState


def straight():
    return route_from_waypoints([[0.0, 0.0], [100.0, 0.0]], ["1", "2"])


def l_route():
    # 30 m east, then 30 m north — a 90 deg left corner
    return route_from_waypoints([[0.0, 0.0], [30.0, 0.0], [30.0, 30.0]], ["1", "2", "3"])


def _max_heading_step(path):
    m = 0.0
    for i in range(2, len(path)):
        a = math.atan2(path[i - 1][1] - path[i - 2][1], path[i - 1][0] - path[i - 2][0])
        b = math.atan2(path[i][1] - path[i - 1][1], path[i][0] - path[i - 1][0])
        m = max(m, abs((b - a + math.pi) % (2 * math.pi) - math.pi))
    return m


def test_raw_keeps_lateral_offset():
    r = straight()
    cfg = ProjectConfig(strategy="raw")
    out = project_route(r, 10.0, 2.0, 0.0, cfg)      # 2 m left of an east route
    fwd = [p for p in out.path if p[0] >= 0]
    assert min(p[1] for p in fwd) < -1.9             # route sits ~2 m to the right (kept)
    assert out.lat_dev == pytest.approx(2.0, abs=1e-6)


def test_centered_removes_offset():
    r = straight()
    out = project_route(r, 10.0, 2.0, 0.0, ProjectConfig(strategy="centered"))
    assert max(abs(p[1]) for p in out.path) < 1e-6   # offset nulled
    assert out.lat_dev == pytest.approx(2.0, abs=1e-6)


def test_window_bounds_and_spacing():
    r = straight()
    out = project_route(r, 20.0, 0.0, 0.0, ProjectConfig(strategy="centered",
                                                         behind_m=5.0, ahead_m=70.0))
    xs = [p[0] for p in out.path]
    assert min(xs) == pytest.approx(-5.0, abs=0.6)
    assert max(xs) == pytest.approx(70.0, abs=0.6)


def test_monotonic_cursor_catches_alongtrack_jump():
    r = straight()
    st = ProjectState()
    o1 = project_route(r, 0.0, 0.0, 0.0, ProjectConfig(), st)
    o2 = project_route(r, 10.0, 0.0, 0.0, ProjectConfig(), o1.state)
    assert o2.cursor_s == pytest.approx(10.0, abs=0.2)   # 15 m window catches the jump


def test_smoothed_rounds_the_corner():
    r = l_route()
    # sit at the start heading east so the corner is in the look-ahead
    sharp = project_route(r, 0.0, 0.0, 0.0, ProjectConfig(strategy="centered")).path
    smooth = project_route(r, 0.0, 0.0, 0.0,
                           ProjectConfig(strategy="smoothed", min_turn_radius_m=5.0,
                                         sample_ds_m=0.5)).path
    assert _max_heading_step(sharp) > 1.0            # centered has the sharp 90 deg
    assert _max_heading_step(smooth) <= 0.5 / 5.0 + 0.05   # smoothed is bounded (<= ds/R)
