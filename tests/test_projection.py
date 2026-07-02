import math
import numpy as np
from parking_proj import geometry as g
from parking_proj.projection import Projector


def _x_crossing_route():
    # 1(0,0)->2(40,30)->3(0,30)->4(40,0); segs 1-2 and 3-4 cross at (20,15)
    return g.route_from_waypoints([(0, 0), (40, 30), (0, 30), (40, 0)],
                                  labels=[1, 2, 3, 4], ds=0.1)


def test_cursor_is_monotonic():
    r = g.route_from_waypoints([(0, 0), (40, 0)], [1, 2], ds=0.1)
    p = Projector(r)
    last = -1.0
    for x in np.linspace(0, 40, 200):
        res = p.step(x, 0.0, 0.0)
        assert res.cursor_s >= last - 1e-9
        last = res.cursor_s


def test_cursor_does_not_retreat_on_backward_noise():
    r = g.route_from_waypoints([(0, 0), (40, 0)], [1, 2], ds=0.1)
    p = Projector(r)
    p.step(10.0, 0.0, 0.0)
    res = p.step(9.5, 0.0, 0.0)   # measured jumps backward
    assert res.cursor_s >= 10.0 - 1e-6


def test_crossing_correct_branch_zero_error():
    r = _x_crossing_route()
    p = Projector(r)
    # walk the true centerline exactly (no error), sampling by arc length
    mism = 0
    for i in range(len(r.points)):
        pt = r.points[i]
        tang = r.tangents[i]
        yaw = math.atan2(tang[1], tang[0])
        res = p.step(pt[0], pt[1], yaw)
        if res.matched_seg != int(r.seg_of_index[i]):
            mism += 1
    assert mism <= 3


def test_heading_gate_rejects_wrong_stroke():
    r = _x_crossing_route()
    p = Projector(r)
    # advance cursor onto seg 0 near the crossing along 1-2
    for s in np.arange(0.0, 24.0, 0.2):
        pt = r.point_at_s(s)
        t = r.tangent_at_s(s)
        p.step(pt[0], pt[1], math.atan2(t[1], t[0]))
    # now at crossing (20,15): stroke 1-2 heading ~atan2(30,40); assert still seg 0
    res = p.step(20.0, 15.0, math.atan2(30.0, 40.0))
    assert res.matched_seg == 0
