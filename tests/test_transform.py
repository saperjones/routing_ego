import math
import numpy as np
from parking_proj.transform import to_body_frame, world_to_body


def test_transform_yaw0():
    # yaw=0: +x body points East. Point due East -> forward; due North -> left.
    bx, by = to_body_frame(1.0, 0.0, 0.0)
    assert math.isclose(bx, 1.0, abs_tol=1e-9) and math.isclose(by, 0.0, abs_tol=1e-9)
    bx, by = to_body_frame(0.0, 1.0, 0.0)
    assert math.isclose(bx, 0.0, abs_tol=1e-9) and math.isclose(by, 1.0, abs_tol=1e-9)


def test_transform_yaw90():
    # yaw=90deg: +x body points North. East is to the right (negative left).
    y = math.pi / 2
    bx, by = to_body_frame(0.0, 1.0, y)   # north -> forward
    assert math.isclose(bx, 1.0, abs_tol=1e-9) and math.isclose(by, 0.0, abs_tol=1e-9)
    bx, by = to_body_frame(1.0, 0.0, y)   # east -> right
    assert math.isclose(bx, 0.0, abs_tol=1e-9) and math.isclose(by, -1.0, abs_tol=1e-9)


def test_transform_yaw180_and_neg90():
    bx, by = to_body_frame(1.0, 0.0, math.pi)      # east, facing west -> behind
    assert math.isclose(bx, -1.0, abs_tol=1e-9) and math.isclose(by, 0.0, abs_tol=1e-9)
    bx, by = to_body_frame(1.0, 0.0, -math.pi / 2)  # facing south, east -> left
    assert math.isclose(bx, 0.0, abs_tol=1e-9) and math.isclose(by, 1.0, abs_tol=1e-9)


def test_world_to_body_vectorized():
    pts = np.array([[2.0, 0.0], [2.0, 1.0]])
    out = world_to_body(pts, 2.0, 0.0, 0.0)
    np.testing.assert_allclose(out, [[0.0, 0.0], [0.0, 1.0]], atol=1e-9)
