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


def _z_shape(step=0.1):
    """Z/S polyline: (0,0)->(0,10)->(10,10)->(10,20) — two ~90 degree corners with 10 m legs."""
    up1 = [(0.0, round(y, 3)) for y in np.arange(0.0, 10.0 + step / 2, step)]
    right = [(round(x, 3), 10.0) for x in np.arange(step, 10.0 + step / 2, step)]
    up2 = [(10.0, round(y, 3)) for y in np.arange(10.0 + step, 20.0 + step / 2, step)]
    return up1 + right + up2


def test_smooth_corners_two_consecutive_corners():
    """Regression: the straight middle leg between two filleted corners must be preserved."""
    R, ds = 5.0, 0.5
    out = smooth_corners(_z_shape(), min_radius=R, corner_angle_deg=10.0, ds=ds, eps=0.2)

    # Both corners bounded: max heading change per step <= ds/R + tolerance
    assert _max_heading_step(out) <= ds / R + 0.05

    # Middle leg present: some sample near x=0 (exit of corner 1) and near x=10 (entry of corner 2)
    xs = [p[0] for p in out]
    assert any(abs(x) < 0.1 for x in xs), "straight middle leg missing near x=0 (chorded across)"
    assert any(abs(x - 10.0) < 0.1 for x in xs), "straight middle leg missing near x=10 (chorded across)"


def test_clothoid_is_smoother_than_arc():
    import math
    from parking_proj.smoothing import smooth_corners

    # Use 20 m legs so R=5.0 clothoid (tangent ~7.9 m) fits without fallback to arc.
    step = 0.1
    up = [(0.0, round(y, 3)) for y in np.arange(0.0, 20.0 + step / 2, step)]
    right = [(round(x, 3), 20.0) for x in np.arange(step, 20.0 + step / 2, step)]
    pts = up + right

    arc = smooth_corners(pts, 5.0, 10.0, 0.5, 0.2, corner_style="arc")
    clo = smooth_corners(pts, 5.0, 10.0, 0.5, 0.2, corner_style="clothoid", transition=1.5)

    def curvature_jump(path):
        r = [math.atan2(path[i][1] - path[i-1][1], path[i][0] - path[i-1][0])
             for i in range(1, len(path))]
        wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
        tr = [wrap(r[i] - r[i-1]) for i in range(1, len(r))]   # per-step turn rate
        return max(abs(tr[i] - tr[i-1]) for i in range(1, len(tr)))  # max change = curvature jump

    # The arc snaps curvature at entry (large 2nd difference);
    # the clothoid ramps gradually (small 2nd difference).
    assert curvature_jump(clo) < curvature_jump(arc)
