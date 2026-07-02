"""The 14-case test matrix: geometry + tier + fixed seed per case."""
from dataclasses import dataclass
import numpy as np
from . import geometry as g
from .route import Route


@dataclass
class Scenario:
    case_id: str
    name: str
    route: Route
    tier: str
    seed: int


def _straight():
    return g.route_from_waypoints([(0, 0), (40, 0)], [1, 2])


def _smooth_turn():
    s1 = np.column_stack([np.linspace(0, 8, 80), np.zeros(80)])
    a = g.arc((8, 15), 15.0, -np.pi / 2, 0.0, 200)
    s2 = np.column_stack([np.full(80, 23.0), np.linspace(15, 23, 80)])
    dense = np.vstack([s1, a, s2])
    return g.route_from_dense(dense, [(0, 0), (8, 0), (23, 15), (23, 23)],
                              [1, 2, 3, 4])


def _corner90():
    # near-90 deg: second leg turned ~95 deg from the first
    ang = np.radians(95.0)
    p2 = (25.0, 0.0)
    p3 = (p2[0] + 25.0 * np.cos(ang), p2[1] + 25.0 * np.sin(ang))
    return g.route_from_waypoints([(0, 0), p2, p3], [1, 2, 3])


def _s_shape():
    a1 = g.arc((0, 12), 12.0, -np.pi / 2, 0.0, 200)      # curve right
    a2 = g.arc((24, 12), 12.0, np.pi, np.pi / 2, 200)    # curve back
    dense = np.vstack([a1, a2])
    return g.route_from_dense(dense, [(0, 0), (12, 12), (24, 24)], [1, 2, 3])


def _x_crossing():
    return g.route_from_waypoints([(0, 0), (40, 30), (0, 30), (40, 0)],
                                  [1, 2, 3, 4])


def _figure_eight():
    t = np.linspace(0, 2 * np.pi, 2000, endpoint=True)
    x = 20.0 * np.sin(t)
    y = 15.0 * np.sin(2 * t)
    dense = np.column_stack([x, y])
    return g.route_from_dense(dense, [(0, 0), (20, 0), (-20, 0), (0, 0)],
                              [1, 2, 3, 4])


def _two_crossing():
    scale = 0.6
    wps = [(0, 0), (60, 30), (20, 40), (30, -10), (55, 30)]
    wps = [(x * scale, y * scale) for (x, y) in wps]
    return g.route_from_waypoints(wps, [1, 2, 3, 4, 5])


def build_scenarios() -> list[Scenario]:
    out = []
    out.append(Scenario("A_low", "Straight (low)", _straight(), "low", 101))
    out.append(Scenario("A_medium", "Straight (medium)", _straight(), "medium", 102))
    out.append(Scenario("A_high", "Straight (high)", _straight(), "high", 103))
    out.append(Scenario("B_low", "Smooth turn (low)", _smooth_turn(), "low", 201))
    out.append(Scenario("B_medium", "Smooth turn (medium)", _smooth_turn(), "medium", 202))
    out.append(Scenario("C_low", "Near-90 corner (low)", _corner90(), "low", 301))
    out.append(Scenario("C_high", "Near-90 corner (high)", _corner90(), "high", 302))
    out.append(Scenario("D_medium", "S-shape (medium)", _s_shape(), "medium", 401))
    out.append(Scenario("E_low", "X-crossing (low)", _x_crossing(), "low", 501))
    out.append(Scenario("E_medium", "X-crossing (medium)", _x_crossing(), "medium", 502))
    out.append(Scenario("E_high", "X-crossing (high)", _x_crossing(), "high", 503))
    out.append(Scenario("F_medium", "Figure-eight (medium)", _figure_eight(), "medium", 601))
    out.append(Scenario("G_medium", "Two-crossing (medium)", _two_crossing(), "medium", 701))
    out.append(Scenario("G_high", "Two-crossing (high)", _two_crossing(), "high", 702))
    return out
