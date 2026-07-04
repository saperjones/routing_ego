"""Portable route -> body-frame projection: monotonic matching + 3 strategies.

The single entry point is project_route(). It is pure: the caller holds a small
ProjectState (the monotonic progress cursor) and passes it back each frame, so
the function is deterministic and portable (no hidden state). Body frame is
+x forward, +y left; yaw is CCW from +E in radians.
"""
import math
from dataclasses import dataclass
import numpy as np
from .transform import to_body_frame
from .smoothing import smooth_corners

SEARCH_AHEAD = 15.0


@dataclass
class ProjectConfig:
    strategy: str = "smoothed"          # "raw" | "centered" | "smoothed"
    behind_m: float = 5.0
    ahead_m: float = 70.0
    sample_ds_m: float = 0.5
    search_ahead_m: float = SEARCH_AHEAD
    search_back_m: float = 0.3
    heading_gate_deg: float = 60.0
    min_turn_radius_m: float = 5.0
    corner_angle_deg: float = 10.0
    simplify_eps_m: float = 0.20
    corner_style: str = "clothoid"       # "clothoid" | "arc"
    clothoid_transition_m: float = 1.5   # calibrated default (see docs/clothoid_calibration.md)


@dataclass
class ProjectState:
    cursor_s: float = None
    initialized: bool = False


@dataclass
class ProjectOutput:
    path: list          # list of [x, y] in the body frame, -behind_m .. +ahead_m
    cursor_s: float
    lat_dev: float
    matched_seg: int
    end_flag: bool
    state: ProjectState


def _best_in_range(route, pos_e, pos_n, yaw, lo_s, hi_s, gate_rad):
    lo = route.index_at_s(max(lo_s, 0.0))
    hi = route.index_at_s(min(hi_s, route.length))
    hi = max(hi, lo)
    idxs = np.arange(lo, hi + 1)
    pts = route.points[idxs]
    d2 = (pts[:, 0] - pos_e) ** 2 + (pts[:, 1] - pos_n) ** 2
    yaws = np.arctan2(route.tangents[idxs][:, 1], route.tangents[idxs][:, 0])
    dyaw = np.abs((yaws - yaw + math.pi) % (2 * math.pi) - math.pi)
    gated = dyaw <= gate_rad
    if not np.any(gated):
        gated = np.ones_like(d2, dtype=bool)
    masked = np.where(gated, d2, np.inf)
    return int(idxs[int(np.argmin(masked))])


def _match(route, pose_e, pose_n, yaw, cfg, state):
    gate = math.radians(cfg.heading_gate_deg)
    if state is None or not state.initialized:
        mi = _best_in_range(route, pose_e, pose_n, yaw, 0.0, route.length, gate)
        cursor_s = float(route.s[mi])
    else:
        mi = _best_in_range(route, pose_e, pose_n, yaw,
                            state.cursor_s - cfg.search_back_m,
                            state.cursor_s + cfg.search_ahead_m, gate)
        cursor_s = max(state.cursor_s, float(route.s[mi]))
    ci = route.index_at_s(cursor_s)
    mp = route.points[ci]
    tang = route.tangents[ci]
    normal_left = np.array([-tang[1], tang[0]])
    lat_dev = float(np.dot(np.array([pose_e, pose_n]) - mp, normal_left))
    matched_seg = int(route.seg_of_index[ci])
    end_flag = (cursor_s + cfg.ahead_m) >= route.length - 1e-9
    return cursor_s, matched_seg, lat_dev, end_flag


def project_route(route, pose_e, pose_n, yaw, config, state=None, speed=None):
    cfg = config
    cursor_s, matched_seg, lat_dev, end_flag = _match(route, pose_e, pose_n, yaw, cfg, state)
    ax, ay = route.point_at_s(cursor_s)
    _, lat_shift = to_body_frame(ax - pose_e, ay - pose_n, yaw)   # car-frame lateral of anchor
    lo = max(cursor_s - cfg.behind_m, 0.0)
    hi = min(cursor_s + cfg.ahead_m, route.length)
    n = int((hi - lo) / cfg.sample_ds_m) + 1
    behind_pts, fwd_pts = [], []
    for k in range(n):
        s = lo + k * cfg.sample_ds_m
        qx, qy = route.point_at_s(s)
        bx, by = to_body_frame(qx - pose_e, qy - pose_n, yaw)
        if cfg.strategy != "raw":
            by -= lat_shift
        (behind_pts if s < cursor_s else fwd_pts).append((bx, by))
    if cfg.strategy == "smoothed" and len(fwd_pts) >= 3:
        fwd_pts = smooth_corners(fwd_pts, cfg.min_turn_radius_m, cfg.corner_angle_deg,
                                 cfg.sample_ds_m, cfg.simplify_eps_m,
                                 corner_style=cfg.corner_style,
                                 transition=cfg.clothoid_transition_m)
    path = [[x, y] for x, y in (behind_pts + fwd_pts)]
    return ProjectOutput(path=path, cursor_s=cursor_s, lat_dev=lat_dev,
                         matched_seg=matched_seg, end_flag=end_flag,
                         state=ProjectState(cursor_s=cursor_s, initialized=True))
