"""Fixed ENU->body-frame rotation. Body: +x forward, +y left."""
import math
import numpy as np


def to_body_frame(de: float, dn: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    body_x = de * c + dn * s     # forward
    body_y = -de * s + dn * c    # left
    return body_x, body_y


def world_to_body(points, pose_e, pose_n, yaw):
    points = np.asarray(points, dtype=float)
    de = points[:, 0] - pose_e
    dn = points[:, 1] - pose_n
    c, s = math.cos(yaw), math.sin(yaw)
    bx = de * c + dn * s
    by = -de * s + dn * c
    return np.column_stack([bx, by])
