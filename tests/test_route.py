import numpy as np
from parking_proj.route import Route


def _straight(n=401, ds=0.1):
    xs = np.arange(n) * ds
    pts = np.column_stack([xs, np.zeros(n)])
    return pts


def test_arclength_monotonic_and_total():
    pts = _straight(401, 0.1)  # 40 m
    r = Route(pts, waypoint_indices=[0, 400], waypoint_labels=[1, 2])
    assert np.all(np.diff(r.s) > 0)
    assert abs(r.length - 40.0) < 1e-6
    assert abs(r.s[-1] - 40.0) < 1e-6


def test_tangent_on_straight_is_unit_east():
    pts = _straight(401, 0.1)
    r = Route(pts, [0, 400], [1, 2])
    np.testing.assert_allclose(r.tangents[10], [1.0, 0.0], atol=1e-9)
    assert abs(np.linalg.norm(r.tangents[10]) - 1.0) < 1e-9


def test_lookups_by_arclength():
    pts = _straight(401, 0.1)
    r = Route(pts, [0, 400], [1, 2])
    assert r.index_at_s(10.0) == 100
    np.testing.assert_allclose(r.point_at_s(10.0), [10.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(r.tangent_at_s(10.0), [1.0, 0.0], atol=1e-9)


def test_segment_labels_split_by_waypoint():
    # two 20 m legs: indices 0..200 seg0, 200..400 seg1
    pts = _straight(401, 0.1)
    r = Route(pts, [0, 200, 400], [1, 2, 3])
    assert r.segment_at_s(5.0) == 0
    assert r.segment_at_s(30.0) == 1
