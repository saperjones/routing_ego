"""Back-compat matcher wrapper. The algorithm now lives in project_route.py;
Projector delegates to its shared matcher so there is a single implementation."""
import math
from dataclasses import dataclass
from .project_route import _match, ProjectConfig, ProjectState, SEARCH_AHEAD  # noqa: F401


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
                 w_search=SEARCH_AHEAD, eps_back=0.3, gate_deg=60.0):
        self.route = route
        self.ahead = ahead
        self.behind = behind
        self.w_search = w_search
        self._cfg = ProjectConfig(ahead_m=ahead, behind_m=abs(behind),
                                  search_ahead_m=w_search, search_back_m=eps_back,
                                  heading_gate_deg=gate_deg)
        self.reset()

    def reset(self):
        self._state = ProjectState()

    def step(self, pose_e, pose_n, yaw):
        cursor_s, matched_seg, lat_dev, end_flag = _match(
            self.route, pose_e, pose_n, yaw, self._cfg, self._state)
        self._state = ProjectState(cursor_s=cursor_s, initialized=True)
        ci = self.route.index_at_s(cursor_s)
        return ProjectionResult(cursor_s=cursor_s, matched_index=ci,
                                matched_seg=matched_seg, est_lat_dev=lat_dev,
                                end_flag=end_flag, gate_widened=False)
