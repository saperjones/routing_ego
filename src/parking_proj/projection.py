"""Stateful route projector with monotonic progress cursor + heading gate."""
import math
from dataclasses import dataclass
import numpy as np
from .transform import to_body_frame


@dataclass
class ProjectionResult:
    cursor_s: float
    matched_index: int
    matched_seg: int
    est_lat_dev: float
    end_flag: bool
    gate_widened: bool


class Projector:
    def __init__(self, route, ahead=20.0, behind=-5.0,
                 w_search=3.5, eps_back=0.3, gate_deg=60.0):
        self.route = route
        self.ahead = ahead
        self.behind = behind  # emit-window bounds consumed downstream (viewer/generate), not used in matching
        self.w_search = w_search
        self.eps_back = eps_back
        self.gate = math.radians(gate_deg)
        self.reset()

    def reset(self):
        self.cursor_s = None
        self.initialized = False

    def _best_in_range(self, pos_e, pos_n, yaw, lo_s, hi_s):
        r = self.route
        lo = r.index_at_s(max(lo_s, 0.0))
        hi = r.index_at_s(min(hi_s, r.length))
        hi = max(hi, lo)
        idxs = np.arange(lo, hi + 1)
        pts = r.points[idxs]
        d2 = (pts[:, 0] - pos_e) ** 2 + (pts[:, 1] - pos_n) ** 2
        yaws = np.arctan2(r.tangents[idxs][:, 1], r.tangents[idxs][:, 0])
        dyaw = np.abs((yaws - yaw + math.pi) % (2 * math.pi) - math.pi)
        gated = dyaw <= self.gate
        widened = False
        if not np.any(gated):
            gated = np.ones_like(d2, dtype=bool)  # widen gate this frame
            widened = True
        masked = np.where(gated, d2, np.inf)
        j = int(np.argmin(masked))
        return int(idxs[j]), widened

    def step(self, pose_e, pose_n, yaw):
        r = self.route
        if not self.initialized:
            mi, widened = self._best_in_range(pose_e, pose_n, yaw, 0.0, r.length)
            self.cursor_s = float(r.s[mi])
            self.initialized = True
        else:
            lo_s = self.cursor_s - self.eps_back
            hi_s = self.cursor_s + self.w_search
            mi, widened = self._best_in_range(pose_e, pose_n, yaw, lo_s, hi_s)
            matched_s = float(r.s[mi])
            self.cursor_s = max(self.cursor_s, matched_s)

        ci = r.index_at_s(self.cursor_s)
        mp = r.points[ci]
        tang = r.tangents[ci]
        normal_left = np.array([-tang[1], tang[0]])
        dev = float(np.dot(np.array([pose_e, pose_n]) - mp, normal_left))
        end_flag = (self.cursor_s + self.ahead) >= r.length - 1e-9
        return ProjectionResult(
            cursor_s=self.cursor_s,
            matched_index=ci,
            matched_seg=int(r.seg_of_index[ci]),
            est_lat_dev=dev,
            end_flag=end_flag,
            gate_widened=widened,
        )


FOLLOW_AHEAD = 70.0
FOLLOW_DS = 0.5


def follow_path(route, pose_e, pose_n, yaw, cursor_s, ahead=FOLLOW_AHEAD, ds=FOLLOW_DS):
    """Re-anchored body-frame path with the lateral offset removed.

    Anchor P = route point at cursor_s. Each sampled route point ahead is
    expressed in the car-heading body frame (+x fwd, +y left), then shifted
    laterally by the car-frame lateral of P so P lands on y=0. Forward-only
    window [cursor_s, cursor_s+ahead], truncated at route end, sampled at ds.
    Returns (points, lat_shift): points is a list of [x, y]; lat_shift is the
    meters subtracted (car-frame lateral of the anchor).
    """
    px, py = route.point_at_s(cursor_s)
    _, lat_shift = to_body_frame(px - pose_e, py - pose_n, yaw)
    end_s = min(cursor_s + ahead, route.length)
    n = int((end_s - cursor_s) / ds) + 1
    pts = []
    for k in range(n):
        qx, qy = route.point_at_s(cursor_s + k * ds)
        bx, by = to_body_frame(qx - pose_e, qy - pose_n, yaw)
        pts.append([bx, by - lat_shift])
    return pts, lat_shift
