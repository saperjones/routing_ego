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
import bisect
from .smoothing import smooth_corners, human_corners

SEARCH_AHEAD = 15.0


@dataclass
class ProjectConfig:
    strategy: str = "smoothed"          # "raw" | "centered" | "smoothed"
    behind_m: float = 5.0
    ahead_m: float = 40.0
    sample_ds_m: float = 0.5
    search_ahead_m: float = SEARCH_AHEAD
    search_back_m: float = 0.3
    heading_gate_deg: float = 60.0
    min_turn_radius_m: float = 8.0       # corner radius used for smoothing (m)
    corner_angle_deg: float = 10.0
    simplify_eps_m: float = 0.20
    corner_style: str = "clothoid"       # "clothoid" | "arc"
    clothoid_transition_m: float = 8.0   # smooth default; human ego-track calibration was 1.5 m (docs/clothoid_calibration.md)
    human_cut_m: float = 2.2             # "human" strategy: inside corner-cut at a 90° turn (m), calibrated from ego tracks


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


class _SmoothedRoute:
    """A world-frame polyline (the planned route with its corners pre-smoothed
    once) with arc-length lookup. Sampling this fixed curve each frame keeps the
    corner stable instead of re-filleting a sliding window (which jitters)."""
    __slots__ = ("pts", "s", "length")

    def __init__(self, pts):
        self.pts = pts
        acc = [0.0]
        for i in range(1, len(pts)):
            acc.append(acc[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
        self.s = acc
        self.length = acc[-1] if len(acc) > 1 else 0.0

    def point_at_s(self, s):
        if s <= 0.0 or len(self.pts) < 2:
            return self.pts[0]
        if s >= self.length:
            return self.pts[-1]
        i = min(bisect.bisect_right(self.s, s) - 1, len(self.pts) - 2)
        seg = self.s[i + 1] - self.s[i]
        t = 0.0 if seg < 1e-12 else (s - self.s[i]) / seg
        return (self.pts[i][0] + t * (self.pts[i + 1][0] - self.pts[i][0]),
                self.pts[i][1] + t * (self.pts[i + 1][1] - self.pts[i][1]))


def _get_world(route, cfg):
    """Build (and cache on the route) the world route transformed ONCE for cfg:
    'smoothed' rounds corners (arc/clothoid/driver); 'human' cuts corners inside
    (calibrated) then smooths — mimicking how a driver takes a corner early/wide."""
    is_human = cfg.strategy in ("human", "human_centered")
    key = ("human" if is_human else cfg.strategy, cfg.corner_style, round(cfg.min_turn_radius_m, 4),
           round(cfg.clothoid_transition_m, 4), round(cfg.corner_angle_deg, 4),
           round(cfg.simplify_eps_m, 4), round(cfg.sample_ds_m, 4), round(cfg.human_cut_m, 4))
    cache = getattr(route, "_sm_cache", None)
    if cache is None:
        cache = {}
        try:
            route._sm_cache = cache
        except AttributeError:
            pass
    sm = cache.get(key)
    if sm is None:
        world = [(float(p[0]), float(p[1])) for p in route.points]
        if is_human:
            pts = human_corners(world, cfg.human_cut_m, cfg.clothoid_transition_m,
                                cfg.sample_ds_m, cfg.simplify_eps_m, cfg.corner_angle_deg)
        else:
            pts = smooth_corners(world, cfg.min_turn_radius_m, cfg.corner_angle_deg,
                                 cfg.sample_ds_m, cfg.simplify_eps_m,
                                 corner_style=cfg.corner_style, transition=cfg.clothoid_transition_m)
        sm = _SmoothedRoute(pts)
        cache[key] = sm
    return sm


def _project_onto(geom, pe, pn, cs0, window):
    """Arc-length on geom of the point nearest to (pe,pn), searched within
    +/-window of cs0 (the projection of the vehicle onto the curve)."""
    lo = bisect.bisect_left(geom.s, max(cs0 - window, 0.0))
    hi = bisect.bisect_right(geom.s, min(cs0 + window, geom.length))
    best_s, best_d = cs0, float("inf")
    for i in range(max(0, lo), min(len(geom.pts), max(hi, lo + 1))):
        px, py = geom.pts[i]
        d = (px - pe) ** 2 + (py - pn) ** 2
        if d < best_d:
            best_d, best_s = d, geom.s[i]
    return best_s


def project_route(route, pose_e, pose_n, yaw, config, state=None, speed=None):
    cfg = config
    cursor_s, matched_seg, lat_dev, end_flag = _match(route, pose_e, pose_n, yaw, cfg, state)
    # "smoothed"/"human"/"human_centered" sample a route transformed ONCE in world
    # space (stable corner); "raw"/"centered" sample the original route.
    if cfg.strategy in ("smoothed", "human", "human_centered"):
        geom = _get_world(route, cfg)
        cs = cursor_s * (geom.length / route.length) if route.length > 1e-9 else cursor_s
        # "human_centered": project the vehicle onto the generated curve (nearest
        # point) and center there, minimising the residual offset, AND orient the
        # frame along the curve tangent at that point so the path leaves the car
        # pointing straight forward (heading angle neutralised, not just offset).
        if cfg.strategy == "human_centered":
            cs = _project_onto(geom, pose_e, pose_n, cs, cfg.search_ahead_m)
            tx0, ty0 = geom.point_at_s(cs)
            tx1, ty1 = geom.point_at_s(min(cs + max(cfg.sample_ds_m, 0.5), geom.length))
            if (tx1, ty1) != (tx0, ty0):
                yaw = math.atan2(ty1 - ty0, tx1 - tx0)   # frame forward = curve tangent
    else:
        geom = route
        cs = cursor_s
    ax, ay = geom.point_at_s(cs)
    _, lat_shift = to_body_frame(ax - pose_e, ay - pose_n, yaw)   # car-frame lateral of anchor
    lo = max(cs - cfg.behind_m, 0.0)
    hi = min(cs + cfg.ahead_m, geom.length)
    n = int((hi - lo) / cfg.sample_ds_m) + 1
    path = []
    for k in range(n):
        s = lo + k * cfg.sample_ds_m
        qx, qy = geom.point_at_s(s)
        bx, by = to_body_frame(qx - pose_e, qy - pose_n, yaw)
        if cfg.strategy != "raw":
            by -= lat_shift
        path.append([bx, by])
    return ProjectOutput(path=path, cursor_s=cursor_s, lat_dev=lat_dev,
                         matched_seg=matched_seg, end_flag=end_flag,
                         state=ProjectState(cursor_s=cursor_s, initialized=True))
