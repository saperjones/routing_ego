import math
import numpy as np
import pytest
from parking_proj.geometry import route_from_waypoints
from parking_proj.projection import follow_path


def straight_route():
    # due-east centerline, 100 m long, resampled at 0.1 m
    return route_from_waypoints([[0.0, 0.0], [100.0, 0.0]], ["1", "2"])


def test_follow_path_nulls_constant_lateral_offset():
    route = straight_route()
    # car 2 m north (= left) of the east-going route, heading due east
    pts, lat_shift = follow_path(route, 10.0, 2.0, 0.0, 10.0)
    # anchor is 2 m to the car's right -> car-frame lateral is -2 m
    assert lat_shift == pytest.approx(-2.0, abs=1e-6)
    assert max(abs(y) for _, y in pts) < 1e-6          # offset removed
    xs = [x for x, _ in pts]
    assert xs[0] == pytest.approx(0.0, abs=1e-9)
    assert xs[-1] == pytest.approx(70.0, abs=1e-6)     # full 0..70 m window
    assert np.allclose(np.diff(xs), 0.5, atol=1e-6)    # 0.5 m spacing


def test_follow_path_shows_heading_error():
    route = straight_route()
    yaw = math.radians(10.0)
    # on the line laterally, but pointed 10 deg off the route tangent
    pts, lat_shift = follow_path(route, 10.0, 0.0, yaw, 10.0)
    assert lat_shift == pytest.approx(0.0, abs=1e-9)
    far = next(p for p in pts if p[0] > 4.5)
    assert far[1] / far[0] == pytest.approx(-math.tan(yaw), abs=1e-3)


def test_follow_path_truncates_at_route_end():
    route = straight_route()                            # length 100 m
    pts, _ = follow_path(route, 60.0, 0.0, 0.0, 60.0)
    xs = [x for x, _ in pts]
    assert xs[-1] == pytest.approx(40.0, abs=1e-6)      # 100 - 60, not 70
    assert xs[-1] <= 70.0 + 1e-9
