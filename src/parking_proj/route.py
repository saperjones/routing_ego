"""Immutable-ish Route value object: dense polyline with arc-length index."""
import numpy as np


class Route:
    def __init__(self, points, waypoint_indices, waypoint_labels):
        self.points = np.asarray(points, dtype=float)
        n = len(self.points)
        seg = np.diff(self.points, axis=0)                      # (N-1, 2)
        seg_len = np.linalg.norm(seg, axis=1)                   # (N-1,)
        self.s = np.concatenate([[0.0], np.cumsum(seg_len)])    # (N,)
        self.length = float(self.s[-1])

        # unit tangents via forward/backward/central differences
        t = np.zeros((n, 2))
        t[:-1] = seg
        t[1:] += seg
        norms = np.linalg.norm(t, axis=1, keepdims=True)
        norms[norms == 0] = 1.0
        self.tangents = t / norms

        self.waypoint_indices = list(waypoint_indices)
        self.waypoint_labels = list(waypoint_labels)
        self.waypoint_s = [float(self.s[i]) for i in self.waypoint_indices]

        # segment id per point: seg k spans waypoint_indices[k]..[k+1]
        self.seg_of_index = np.zeros(n, dtype=int)
        for k in range(len(self.waypoint_indices) - 1):
            lo = self.waypoint_indices[k]
            hi = self.waypoint_indices[k + 1]
            self.seg_of_index[lo:hi] = k
        self.seg_of_index[self.waypoint_indices[-1]:] = len(self.waypoint_indices) - 2

    def index_at_s(self, s: float) -> int:
        s = min(max(s, 0.0), self.length)
        return int(np.searchsorted(self.s, s, side="right") - 1) if s < self.length else len(self.s) - 1

    def point_at_s(self, s: float) -> np.ndarray:
        return self.points[self.index_at_s(s)]

    def tangent_at_s(self, s: float) -> np.ndarray:
        return self.tangents[self.index_at_s(s)]

    def segment_at_s(self, s: float) -> int:
        return int(self.seg_of_index[self.index_at_s(s)])
