"""Geometry builders: resampling, arcs, and Route construction."""
import numpy as np
from .route import Route


def resample(points, ds=0.1):
    points = np.asarray(points, dtype=float)
    seg = np.diff(points, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = float(s[-1])
    n = max(int(round(total / ds)) + 1, 2)
    target = np.linspace(0.0, total, n)
    ex = np.interp(target, s, points[:, 0])
    ny = np.interp(target, s, points[:, 1])
    return np.column_stack([ex, ny])


def arc(center, radius, a0, a1, n=200):
    a = np.linspace(a0, a1, n)
    cx, cy = center
    return np.column_stack([cx + radius * np.cos(a), cy + radius * np.sin(a)])


def _nearest_index(points, xy):
    d = np.linalg.norm(points - np.asarray(xy, dtype=float), axis=1)
    return int(np.argmin(d))


def route_from_dense(dense, waypoints_xy, labels, ds=0.1):
    pts = resample(dense, ds=ds)
    idx = [_nearest_index(pts, wp) for wp in waypoints_xy]
    idx[0], idx[-1] = 0, len(pts) - 1
    idx = sorted(set(idx))
    return Route(pts, idx, labels[: len(idx)])


def route_from_waypoints(waypoints, labels, ds=0.1):
    if len(waypoints) < 2:
        raise ValueError("route_from_waypoints needs >= 2 waypoints")
    waypoints = [np.asarray(w, dtype=float) for w in waypoints]
    legs = []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        legs.append(np.column_stack([a, b]).T)  # 2 points per leg
    dense = np.vstack([legs[0]] + [leg[1:] for leg in legs[1:]])
    pts = resample(dense, ds=ds)
    idx = [_nearest_index(pts, wp) for wp in waypoints]
    idx[0], idx[-1] = 0, len(pts) - 1
    if any(idx[k] >= idx[k + 1] for k in range(len(idx) - 1)):
        raise ValueError("waypoints too close together for the given ds (indices collapsed)")
    return Route(pts, idx, labels)
