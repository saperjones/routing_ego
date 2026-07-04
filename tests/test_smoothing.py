import math
import numpy as np
from parking_proj.smoothing import rdp, smooth_corners


def _l_shape(step=0.1):
    up = [(0.0, round(y, 3)) for y in np.arange(0.0, 10.0 + step / 2, step)]
    right = [(round(x, 3), 10.0) for x in np.arange(step, 10.0 + step / 2, step)]
    return up + right


def _max_heading_step(pts):
    m = 0.0
    for i in range(2, len(pts)):
        a = math.atan2(pts[i - 1][1] - pts[i - 2][1], pts[i - 1][0] - pts[i - 2][0])
        b = math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0])
        m = max(m, abs((b - a + math.pi) % (2 * math.pi) - math.pi))
    return m


def test_rdp_collapses_l_shape_to_three_vertices():
    verts = rdp(_l_shape(), eps=0.2)
    assert len(verts) == 3                       # start, corner, end
    assert verts[0] == (0.0, 0.0)


def test_raw_corner_is_sharp():
    assert _max_heading_step(_l_shape()) > 1.5   # ~pi/2 jump at the corner


def test_smooth_corners_bounds_curvature():
    R, ds = 5.0, 0.5
    out = smooth_corners(_l_shape(), min_radius=R, corner_angle_deg=10.0, ds=ds, eps=0.2)
    # arc sampled at ds on radius R turns by ~ds/R per step; allow tolerance
    assert _max_heading_step(out) <= ds / R + 0.05
    # endpoints preserved (roughly): starts near origin, ends near (10,10)
    assert abs(out[0][0]) < 1e-6 and abs(out[0][1]) < 1e-6


def test_smooth_corners_keeps_straight_line_straight():
    line = [(0.0, y) for y in np.arange(0.0, 10.01, 0.1)]
    out = smooth_corners(line, min_radius=5.0, corner_angle_deg=10.0, ds=0.5, eps=0.2)
    assert _max_heading_step(out) < 1e-6
