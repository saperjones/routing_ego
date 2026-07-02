import numpy as np
from parking_proj import geometry as g


def test_resample_uniform_spacing():
    fine = np.column_stack([np.linspace(0, 10, 37), np.zeros(37)])
    out = g.resample(fine, ds=0.1)
    d = np.linalg.norm(np.diff(out, axis=0), axis=1)
    assert np.allclose(d, 0.1, atol=1e-6)
    assert abs(out[-1, 0] - 10.0) < 0.05


def test_arc_radius_constant():
    pts = g.arc(center=(0, 0), radius=5.0, a0=0.0, a1=np.pi / 2, n=100)
    r = np.linalg.norm(pts, axis=1)
    assert np.allclose(r, 5.0, atol=1e-9)


def test_route_from_waypoints_labels():
    r = g.route_from_waypoints([(0, 0), (20, 0), (20, 20)], labels=[1, 2, 3], ds=0.1)
    assert r.waypoint_labels == [1, 2, 3]
    assert abs(r.length - 40.0) < 0.05
    # corner point present near (20,0)
    i = r.index_at_s(20.0)
    np.testing.assert_allclose(r.points[i], [20.0, 0.0], atol=0.1)
