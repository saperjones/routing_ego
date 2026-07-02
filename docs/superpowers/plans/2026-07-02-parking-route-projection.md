# Parking Route Projection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a stateful algorithm that projects a global (ENU) planned route into the vehicle-body frame each frame, plus a seeded simulation harness and a static HTML viewer that plays back prebaked test cases.

**Architecture:** Two halves meeting at one JSON artifact. A Python generator (numpy) runs the projection algorithm over a two-layer simulation (imperfect tracking + RTK error) offline and prebakes 14 test cases to JSON with graded verdicts. A static HTML/JS viewer plays back that JSON; it contains no matching logic, only a fixed display rotation.

**Tech Stack:** Python 3.11+, numpy, pytest for the core; vanilla HTML/CSS/JS (Canvas 2D) for the viewer. No build step for the viewer.

## Global Constraints

- Working frame is local **ENU** (East `e`, North `n`, meters); all algorithm math is in ENU. WGS84 ⇄ ENU lives only in `geo.py`.
- Vehicle body frame: **`+x` forward, `+y` left, `+z` up**, right-handed (matches `JSON_FIELD_DESCRIPTION_V7` ego(curr)).
- Heading `h` = CCW angle of the `+x` body axis from ENU East, in **radians** internally.
- **No pose/heading smoothing or filtering** anywhere in the projection algorithm.
- 2D planar model (`e`, `n`, `h`); pitch/roll are cosmetic noise only.
- Projection window defaults: `AHEAD = +20.0 m`, `BEHIND = -5.0 m` (configurable).
- Search window defaults: `W_SEARCH = 3.5 m`, `EPS_BACK = 0.3 m` (configurable).
- Heading gate default: `GATE = 60°` (configurable).
- Route point spacing: `DS = 0.1 m`.
- Simulation: constant speed default `8 km/h`, `10 Hz` frame rate.
- Localization tiers (1σ): low `(0.10 m, 0.01°)`, medium `(0.50 m, 0.03°)`, high `(1.50 m, 0.05°)`. Position cap `2.0 m`, angle cap `±0.05°`.
- Tracking layer: lateral 1σ `0.15 m`, cap `0.4 m`; corner-rounding radius `~2.5 m`.
- Determinism: fixed integer seed per case via `numpy.random.default_rng(seed)`; regeneration is bit-identical.
- Acceptance: correct-branch association with **≤ 3 frames** tolerance; **0** backward cursor jumps; **0** dropouts.
- Sign convention: lateral deviation **positive = vehicle is to the left** of the route.

---

### Task 1: Project scaffold + `geo.py` (WGS84 ⇄ ENU)

**Files:**
- Create: `pyproject.toml`
- Create: `.gitignore`
- Create: `src/parking_proj/__init__.py`
- Create: `src/parking_proj/geo.py`
- Create: `tests/__init__.py`
- Create: `tests/test_geo.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `HEFEI_LAT0: float`, `HEFEI_LON0: float` (module constants)
  - `wgs84_to_enu(lat: float, lon: float) -> tuple[float, float]` → `(e, n)` meters
  - `enu_to_wgs84(e: float, n: float) -> tuple[float, float]` → `(lat, lon)` degrees
  - `compass_to_yaw(heading_north_deg: float) -> float` → yaw radians CCW from East
  - `yaw_to_compass(yaw_rad: float) -> float` → compass degrees CW from North

- [ ] **Step 1: Create `pyproject.toml`**

```toml
[project]
name = "parking_proj"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = ["numpy>=1.26"]

[project.optional-dependencies]
dev = ["pytest>=8.0"]

[tool.pytest.ini_options]
pythonpath = ["src"]
testpaths = ["tests"]

[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"
```

- [ ] **Step 2: Create `.gitignore`**

```
__pycache__/
*.pyc
.pytest_cache/
out/
.venv/
```

- [ ] **Step 3: Create empty package files**

Create `src/parking_proj/__init__.py` and `tests/__init__.py` as empty files.

- [ ] **Step 4: Write the failing test** — `tests/test_geo.py`

```python
import math
from parking_proj import geo


def test_origin_maps_to_zero():
    e, n = geo.wgs84_to_enu(geo.HEFEI_LAT0, geo.HEFEI_LON0)
    assert abs(e) < 1e-6 and abs(n) < 1e-6


def test_roundtrip_under_1mm():
    # 100 points in a ~400 m box around the origin
    for i in range(100):
        de = -200.0 + 4.0 * i          # meters east
        dn = 150.0 - 3.0 * i           # meters north
        lat, lon = geo.enu_to_wgs84(de, dn)
        e2, n2 = geo.wgs84_to_enu(lat, lon)
        assert abs(e2 - de) < 1e-3 and abs(n2 - dn) < 1e-3


def test_compass_yaw_conversions():
    # Compass 0deg (north) -> yaw 90deg (CCW from east)
    assert math.isclose(geo.compass_to_yaw(0.0), math.pi / 2, abs_tol=1e-9)
    # Compass 90deg (east) -> yaw 0
    assert math.isclose(geo.compass_to_yaw(90.0), 0.0, abs_tol=1e-9)
    # Round trip
    for h in (0.0, 45.0, 90.0, 180.0, 270.0):
        back = geo.yaw_to_compass(geo.compass_to_yaw(h)) % 360.0
        assert math.isclose(back, h % 360.0, abs_tol=1e-9)
```

- [ ] **Step 5: Run test to verify it fails**

Run: `pytest tests/test_geo.py -v`
Expected: FAIL with `ModuleNotFoundError` / `AttributeError` (geo not implemented).

- [ ] **Step 6: Implement `src/parking_proj/geo.py`**

```python
"""WGS84 <-> local ENU and heading-convention conversions.

Local equirectangular tangent-plane projection anchored at a Hefei origin.
At parking-lot scale (<0.5 km) round-trip error is well under 1 mm and the
inverse is exact algebra.
"""
import math

# Hefei, Anhui — a point near the city center.
HEFEI_LAT0 = 31.8206
HEFEI_LON0 = 117.2290

_EARTH_R = 6378137.0  # WGS84 semi-major axis, meters
_DEG = math.pi / 180.0
_COS_LAT0 = math.cos(HEFEI_LAT0 * _DEG)


def wgs84_to_enu(lat: float, lon: float) -> tuple[float, float]:
    n = (lat - HEFEI_LAT0) * _DEG * _EARTH_R
    e = (lon - HEFEI_LON0) * _DEG * _EARTH_R * _COS_LAT0
    return e, n


def enu_to_wgs84(e: float, n: float) -> tuple[float, float]:
    lat = HEFEI_LAT0 + (n / (_EARTH_R * _DEG))
    lon = HEFEI_LON0 + (e / (_EARTH_R * _COS_LAT0 * _DEG))
    return lat, lon


def compass_to_yaw(heading_north_deg: float) -> float:
    """Compass heading (CW from North) -> yaw radians (CCW from East)."""
    return (90.0 - heading_north_deg) * _DEG


def yaw_to_compass(yaw_rad: float) -> float:
    """Yaw radians (CCW from East) -> compass heading degrees (CW from North)."""
    return 90.0 - yaw_rad / _DEG
```

- [ ] **Step 7: Run test to verify it passes**

Run: `pytest tests/test_geo.py -v`
Expected: PASS (3 tests).

- [ ] **Step 8: Commit**

```bash
git init
git add pyproject.toml .gitignore src/parking_proj/__init__.py src/parking_proj/geo.py tests/__init__.py tests/test_geo.py
git commit -m "feat: project scaffold + geo WGS84<->ENU conversions"
```

---

### Task 2: `route.py` — Route value object

**Files:**
- Create: `src/parking_proj/route.py`
- Test: `tests/test_route.py`

**Interfaces:**
- Consumes: nothing (numpy only).
- Produces: `class Route` with:
  - `Route(points: np.ndarray, waypoint_indices: list[int], waypoint_labels: list[int])` — `points` is `(N,2)` ENU meters, assumed ~`DS`-spaced.
  - `.points: np.ndarray (N,2)`, `.s: np.ndarray (N,)` cumulative arc-length, `.tangents: np.ndarray (N,2)` unit, `.length: float`
  - `.waypoint_indices: list[int]`, `.waypoint_labels: list[int]`, `.waypoint_s: list[float]`
  - `.seg_of_index: np.ndarray (N,)` int — which waypoint-segment (0-based) each point belongs to
  - `.index_at_s(s: float) -> int`
  - `.point_at_s(s: float) -> np.ndarray (2,)`
  - `.tangent_at_s(s: float) -> np.ndarray (2,)`
  - `.segment_at_s(s: float) -> int`

- [ ] **Step 1: Write the failing test** — `tests/test_route.py`

```python
import numpy as np
from parking_proj.route import Route


def _straight(n=401, ds=0.1):
    xs = np.arange(n) * ds
    pts = np.column_stack([xs, np.zeros(n)])
    return pts


def test_arclength_monotonic_and_total():
    pts = _straight(401, 0.1)  # 40 m
    r = Route(pts, waypoint_indices=[0, 400], waypoint_labels=[1, 2])
    assert np.all(np.diff(r.s) > 0)
    assert abs(r.length - 40.0) < 1e-6
    assert abs(r.s[-1] - 40.0) < 1e-6


def test_tangent_on_straight_is_unit_east():
    pts = _straight(401, 0.1)
    r = Route(pts, [0, 400], [1, 2])
    np.testing.assert_allclose(r.tangents[10], [1.0, 0.0], atol=1e-9)
    assert abs(np.linalg.norm(r.tangents[10]) - 1.0) < 1e-9


def test_lookups_by_arclength():
    pts = _straight(401, 0.1)
    r = Route(pts, [0, 400], [1, 2])
    assert r.index_at_s(10.0) == 100
    np.testing.assert_allclose(r.point_at_s(10.0), [10.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(r.tangent_at_s(10.0), [1.0, 0.0], atol=1e-9)


def test_segment_labels_split_by_waypoint():
    # two 20 m legs: indices 0..200 seg0, 200..400 seg1
    pts = _straight(401, 0.1)
    r = Route(pts, [0, 200, 400], [1, 2, 3])
    assert r.segment_at_s(5.0) == 0
    assert r.segment_at_s(30.0) == 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_route.py -v`
Expected: FAIL (`ModuleNotFoundError: parking_proj.route`).

- [ ] **Step 3: Implement `src/parking_proj/route.py`**

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_route.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/route.py tests/test_route.py
git commit -m "feat: Route value object with arc-length index and segment labels"
```

---

### Task 3: `geometry.py` — polyline builders

**Files:**
- Create: `src/parking_proj/geometry.py`
- Test: `tests/test_geometry.py`

**Interfaces:**
- Consumes: `Route` (Task 2) for `route_from_waypoints`.
- Produces:
  - `resample(points: np.ndarray, ds: float = 0.1) -> np.ndarray` — uniform arc-length resample of a fine `(M,2)` polyline
  - `arc(center, radius, a0, a1, n=200) -> np.ndarray (n,2)` — `a0..a1` radians
  - `route_from_waypoints(waypoints: list[tuple], labels: list[int], ds=0.1) -> Route` — straight legs between waypoints (sharp corners)
  - `route_from_dense(dense: np.ndarray, waypoints_xy: list[tuple], labels: list[int], ds=0.1) -> Route` — resample a fine polyline and locate waypoint indices by nearest point

- [ ] **Step 1: Write the failing test** — `tests/test_geometry.py`

```python
import numpy as np
from parking_proj import geometry as g


def test_resample_uniform_spacing():
    fine = np.column_stack([np.linspace(0, 10, 37), np.zeros(37)])
    out = g.resample(fine, ds=0.1)
    d = np.linalg.norm(np.diff(out, axis=0), axis=1)
    assert np.allclose(d, 0.1, atol=1e-6)
    assert abs(out[-1, 0] - 10.0) < 0.05


def test_arc_radius_constant():
    pts = g.arc(center=(0, 0), radius=5.0, a0=0.0, a1=np.pi / 2, n=100)
    r = np.linalg.norm(pts, axis=1)
    assert np.allclose(r, 5.0, atol=1e-9)


def test_route_from_waypoints_labels():
    r = g.route_from_waypoints([(0, 0), (20, 0), (20, 20)], labels=[1, 2, 3], ds=0.1)
    assert r.waypoint_labels == [1, 2, 3]
    assert abs(r.length - 40.0) < 0.05
    # corner point present near (20,0)
    i = r.index_at_s(20.0)
    np.testing.assert_allclose(r.points[i], [20.0, 0.0], atol=0.1)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_geometry.py -v`
Expected: FAIL (`ModuleNotFoundError`).

- [ ] **Step 3: Implement `src/parking_proj/geometry.py`**

```python
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
    waypoints = [np.asarray(w, dtype=float) for w in waypoints]
    legs = []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        legs.append(np.column_stack([a, b]).T)  # 2 points per leg
    dense = np.vstack([legs[0]] + [leg[1:] for leg in legs[1:]])
    pts = resample(dense, ds=ds)
    idx = [_nearest_index(pts, wp) for wp in waypoints]
    idx[0], idx[-1] = 0, len(pts) - 1
    return Route(pts, idx, labels)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_geometry.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/geometry.py tests/test_geometry.py
git commit -m "feat: geometry builders (resample, arc, route-from-waypoints)"
```

---

### Task 4: Body-frame transform

**Files:**
- Create: `src/parking_proj/transform.py`
- Test: `tests/test_transform.py`

**Interfaces:**
- Consumes: nothing (numpy).
- Produces:
  - `to_body_frame(de: float, dn: float, yaw: float) -> tuple[float, float]` → `(body_x_forward, body_y_left)`
  - `world_to_body(points: np.ndarray, pose_e: float, pose_n: float, yaw: float) -> np.ndarray (N,2)` → columns `[x_forward, y_left]`

- [ ] **Step 1: Write the failing test** — `tests/test_transform.py`

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_transform.py -v`
Expected: FAIL (`ModuleNotFoundError`).

- [ ] **Step 3: Implement `src/parking_proj/transform.py`**

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_transform.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/transform.py tests/test_transform.py
git commit -m "feat: ENU->body-frame transform (+x forward, +y left)"
```

---

### Task 5: `projection.py` — stateful Projector

**Files:**
- Create: `src/parking_proj/projection.py`
- Test: `tests/test_projection.py`

**Interfaces:**
- Consumes: `Route` (Task 2).
- Produces:
  - `@dataclass ProjectionResult`: `cursor_s: float`, `matched_index: int`, `matched_seg: int`, `est_lat_dev: float`, `end_flag: bool`, `gate_widened: bool`
  - `class Projector`:
    - `Projector(route: Route, ahead=20.0, behind=-5.0, w_search=3.5, eps_back=0.3, gate_deg=60.0)`
    - `.reset() -> None`
    - `.step(pose_e: float, pose_n: float, yaw: float) -> ProjectionResult`

- [ ] **Step 1: Write the failing test** — `tests/test_projection.py`

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_projection.py -v`
Expected: FAIL (`ModuleNotFoundError`).

- [ ] **Step 3: Implement `src/parking_proj/projection.py`**

```python
"""Stateful route projector with monotonic progress cursor + heading gate."""
import math
from dataclasses import dataclass
import numpy as np


@dataclass
class ProjectionResult:
    cursor_s: float
    matched_index: int
    matched_seg: int
    est_lat_dev: float
    end_flag: bool
    gate_widened: bool


def _ang_diff(a, b):
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return abs(d)


class Projector:
    def __init__(self, route, ahead=20.0, behind=-5.0,
                 w_search=3.5, eps_back=0.3, gate_deg=60.0):
        self.route = route
        self.ahead = ahead
        self.behind = behind
        self.w_search = w_search
        self.eps_back = eps_back
        self.gate = math.radians(gate_deg)
        self.reset()

    def reset(self):
        self.cursor_s = None
        self.initialized = False

    def _tangent_yaw(self, i):
        t = self.route.tangents[i]
        return math.atan2(t[1], t[0])

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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_projection.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/projection.py tests/test_projection.py
git commit -m "feat: stateful Projector (monotonic cursor + heading gate)"
```

---

### Task 6: `simulate.py` — two-layer simulation

**Files:**
- Create: `src/parking_proj/simulate.py`
- Test: `tests/test_simulate.py`

**Interfaces:**
- Consumes: `Route` (Task 2).
- Produces:
  - `TIERS: dict[str, tuple[float, float]]` = `{"low": (0.10, 0.01), "medium": (0.50, 0.03), "high": (1.50, 0.05)}` (lat 1σ m, angle 1σ deg)
  - `@dataclass SimConfig`: `tier: str`, `seed: int`, `speed_kmh: float = 8.0`, `hz: float = 10.0`, `track_sigma: float = 0.15`, `track_cap: float = 0.4`, `smooth_win: int = 25`, `pos_cap: float = 2.0`, `ang_cap_deg: float = 0.05`
  - `@dataclass Frame`: `t, speed, true_e, true_n, true_yaw, meas_e, meas_n, meas_yaw, pitch, roll, gt_s, gt_seg`
  - `simulate(route: Route, cfg: SimConfig) -> list[Frame]`

- [ ] **Step 1: Write the failing test** — `tests/test_simulate.py`

```python
import math
import numpy as np
from parking_proj import geometry as g
from parking_proj.simulate import simulate, SimConfig, TIERS


def _corner():
    return g.route_from_waypoints([(0, 0), (25, 0), (25, 25)], [1, 2, 3], ds=0.1)


def test_deterministic_same_seed():
    r = _corner()
    a = simulate(r, SimConfig(tier="medium", seed=7))
    b = simulate(r, SimConfig(tier="medium", seed=7))
    assert len(a) == len(b)
    assert all(abs(x.meas_e - y.meas_e) < 1e-12 for x, y in zip(a, b))


def test_tracking_cap_respected():
    r = g.route_from_waypoints([(0, 0), (40, 0)], [1, 2], ds=0.1)
    frames = simulate(r, SimConfig(tier="low", seed=1))
    # true lateral offset from the y=0 centerline stays within cap+margin
    assert max(abs(f.true_n) for f in frames) <= 0.4 + 0.05


def test_localization_cap_respected():
    r = g.route_from_waypoints([(0, 0), (40, 0)], [1, 2], ds=0.1)
    frames = simulate(r, SimConfig(tier="high", seed=2))
    for f in frames:
        d = math.hypot(f.meas_e - f.true_e, f.meas_n - f.true_n)
        assert d <= 2.0 + 1e-6
        assert abs(f.meas_yaw - f.true_yaw) <= math.radians(0.05) + 1e-9


def test_heading_continuous_through_corner():
    r = _corner()
    frames = simulate(r, SimConfig(tier="low", seed=3))
    dyaws = [abs((frames[i + 1].true_yaw - frames[i].true_yaw + math.pi)
                 % (2 * math.pi) - math.pi) for i in range(len(frames) - 1)]
    # rounded corner => no single-frame ~90deg jump
    assert max(dyaws) < math.radians(20.0)


def test_gt_progress_monotonic():
    r = _corner()
    frames = simulate(r, SimConfig(tier="medium", seed=4))
    gs = [f.gt_s for f in frames]
    assert all(gs[i + 1] >= gs[i] - 1e-9 for i in range(len(gs) - 1))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_simulate.py -v`
Expected: FAIL (`ModuleNotFoundError`).

- [ ] **Step 3: Implement `src/parking_proj/simulate.py`**

```python
"""Two-layer simulation: imperfect tracking + RTK localization error."""
import math
from dataclasses import dataclass
import numpy as np

TIERS = {"low": (0.10, 0.01), "medium": (0.50, 0.03), "high": (1.50, 0.05)}


@dataclass
class SimConfig:
    tier: str
    seed: int
    speed_kmh: float = 8.0
    hz: float = 10.0
    track_sigma: float = 0.15
    track_cap: float = 0.4
    smooth_win: int = 25
    pos_cap: float = 2.0
    ang_cap_deg: float = 0.05


@dataclass
class Frame:
    t: float
    speed: float
    true_e: float
    true_n: float
    true_yaw: float
    meas_e: float
    meas_n: float
    meas_yaw: float
    pitch: float
    roll: float
    gt_s: float
    gt_seg: int


def _gauss_kernel(win):
    win = max(int(win), 1)
    if win % 2 == 0:
        win += 1
    x = np.arange(win) - win // 2
    sig = win / 5.0
    k = np.exp(-0.5 * (x / sig) ** 2)
    return k / k.sum()


def _smooth(a, win):
    k = _gauss_kernel(win)
    return np.convolve(a, k, mode="same")


def _lowpass_noise(rng, n, win, target_sigma, cap):
    w = rng.standard_normal(n)
    s = _smooth(w, win)
    std = s.std()
    if std > 0:
        s = s * (target_sigma / std)
    return np.clip(s, -cap, cap)


def simulate(route, cfg: SimConfig) -> list[Frame]:
    rng = np.random.default_rng(cfg.seed)
    lat_sigma, ang_sigma_deg = TIERS[cfg.tier]
    speed_mps = cfg.speed_kmh / 3.6
    ds_frame = speed_mps / cfg.hz

    # 1) nominal ordinal walk along planned route -> gt_s per frame (exact)
    n_frames = max(int(route.length / ds_frame) + 1, 2)
    gt_s = np.minimum(np.arange(n_frames) * ds_frame, route.length)
    nom = np.array([route.point_at_s(s) for s in gt_s])          # (F,2)
    gt_seg = np.array([route.segment_at_s(s) for s in gt_s])

    # 2) corner rounding via low-pass on nominal positions (cuts corners inside)
    smooth_e = _smooth(nom[:, 0], cfg.smooth_win)
    smooth_n = _smooth(nom[:, 1], cfg.smooth_win)
    # keep endpoints anchored (convolution 'same' biases ends)
    smooth_e[:cfg.smooth_win] = nom[:cfg.smooth_win, 0]
    smooth_n[:cfg.smooth_win] = nom[:cfg.smooth_win, 1]
    smooth_e[-cfg.smooth_win:] = nom[-cfg.smooth_win:, 0]
    smooth_n[-cfg.smooth_win:] = nom[-cfg.smooth_win:, 1]

    # tangent of smoothed centerline -> perpendicular for tracking offset
    dse = np.gradient(smooth_e)
    dsn = np.gradient(smooth_n)
    tnorm = np.hypot(dse, dsn)
    tnorm[tnorm == 0] = 1.0
    perp_e, perp_n = -dsn / tnorm, dse / tnorm

    # 3) tracking lateral offset (smooth, capped), same construction all tiers
    off = _lowpass_noise(rng, n_frames, cfg.smooth_win * 4,
                         cfg.track_sigma, cfg.track_cap)
    true_e = smooth_e + off * perp_e
    true_n = smooth_n + off * perp_n

    # true heading = tangent of final true trajectory
    te = np.gradient(true_e)
    tn = np.gradient(true_n)
    true_yaw = np.arctan2(tn, te)

    # 4) RTK localization error (correlated bias + white), capped to 2 m
    bias_e = _lowpass_noise(rng, n_frames, cfg.smooth_win * 6, lat_sigma, cfg.pos_cap)
    bias_n = _lowpass_noise(rng, n_frames, cfg.smooth_win * 6, lat_sigma, cfg.pos_cap)
    white_e = rng.normal(0, 0.02, n_frames)
    white_n = rng.normal(0, 0.02, n_frames)
    err_e = bias_e + white_e
    err_n = bias_n + white_n
    mag = np.hypot(err_e, err_n)
    scale = np.where(mag > cfg.pos_cap, cfg.pos_cap / np.maximum(mag, 1e-9), 1.0)
    meas_e = true_e + err_e * scale
    meas_n = true_n + err_n * scale

    ang_sigma = math.radians(ang_sigma_deg)
    ang_cap = math.radians(cfg.ang_cap_deg)
    yaw_err = np.clip(_lowpass_noise(rng, n_frames, cfg.smooth_win * 6,
                                     ang_sigma, ang_cap), -ang_cap, ang_cap)
    meas_yaw = true_yaw + yaw_err
    pitch = np.clip(_lowpass_noise(rng, n_frames, cfg.smooth_win * 6,
                                   ang_sigma, ang_cap), -ang_cap, ang_cap)
    roll = np.clip(_lowpass_noise(rng, n_frames, cfg.smooth_win * 6,
                                  ang_sigma, ang_cap), -ang_cap, ang_cap)

    frames = []
    for i in range(n_frames):
        frames.append(Frame(
            t=i / cfg.hz, speed=speed_mps,
            true_e=float(true_e[i]), true_n=float(true_n[i]),
            true_yaw=float(true_yaw[i]),
            meas_e=float(meas_e[i]), meas_n=float(meas_n[i]),
            meas_yaw=float(meas_yaw[i]),
            pitch=float(pitch[i]), roll=float(roll[i]),
            gt_s=float(gt_s[i]), gt_seg=int(gt_seg[i]),
        ))
    return frames
```

Note on `test_tracking_cap_respected`: it uses a straight route on `y=0`, where the smoothed centerline is identical to nominal, so `true_n` equals the tracking offset and the cap check is exact.

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_simulate.py -v`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/simulate.py tests/test_simulate.py
git commit -m "feat: two-layer simulation (tracking + RTK error, seeded)"
```

---

### Task 7: `scenarios.py` — the 14-case matrix

**Files:**
- Create: `src/parking_proj/scenarios.py`
- Test: `tests/test_scenarios.py`

**Interfaces:**
- Consumes: `geometry` (Task 3), `route.Route` (Task 2).
- Produces:
  - `@dataclass Scenario`: `case_id: str`, `name: str`, `route: Route`, `tier: str`, `seed: int`
  - `build_scenarios() -> list[Scenario]` — exactly 14 cases (A×3, B×2, C×2, D×1, E×3, F×1, G×2)

- [ ] **Step 1: Write the failing test** — `tests/test_scenarios.py`

```python
from parking_proj.scenarios import build_scenarios


def test_fourteen_cases_with_unique_ids():
    sc = build_scenarios()
    assert len(sc) == 14
    ids = [s.case_id for s in sc]
    assert len(set(ids)) == 14


def test_tiers_present():
    sc = {s.case_id: s for s in build_scenarios()}
    assert sc["A_low"].tier == "low"
    assert sc["E_high"].tier == "high"
    assert all(s.route.length > 5.0 for s in sc.values())


def test_x_crossing_labels():
    sc = {s.case_id: s for s in build_scenarios()}
    assert sc["E_low"].route.waypoint_labels == [1, 2, 3, 4]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_scenarios.py -v`
Expected: FAIL (`ModuleNotFoundError`).

- [ ] **Step 3: Implement `src/parking_proj/scenarios.py`**

```python
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
    lead = g.arc((0, 15), 15.0, -np.pi / 2, -np.pi / 2, 2)  # placeholder start
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_scenarios.py -v`
Expected: PASS (3 tests). If `_smooth_turn`/`route_from_dense` produce a waypoint count mismatch, the `route_from_dense` helper already clamps labels via `labels[: len(idx)]`; the test only checks `E_low` labels, so B/D need not have all 4.

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/scenarios.py tests/test_scenarios.py
git commit -m "feat: 14-case scenario matrix (A-G) with fixed seeds"
```

---

### Task 8: `generate.py` — run, grade, write JSON

**Files:**
- Create: `src/parking_proj/grade.py`
- Create: `src/parking_proj/generate.py`
- Test: `tests/test_generate.py`

**Interfaces:**
- Consumes: `Route`, `Projector`+`ProjectionResult` (Task 5), `simulate`+`Frame`+`SimConfig` (Task 6), `build_scenarios`+`Scenario` (Task 7), `transform.world_to_body` (Task 4, used only by viewer — not here).
- Produces:
  - `grade.grade_case(route, frames, results) -> dict` → keys `correct_branch_frames, total_frames, mismatches, backward_jumps, dropouts, max_dev_gap, passed`
  - `grade.true_lat_dev(route, frame) -> float`
  - `generate.build_case_dict(scenario) -> dict` (full JSON payload for one case)
  - `generate.main(out_dir="out") -> None` (writes `out/index.json` + `out/<case_id>.json`)

- [ ] **Step 1: Write the failing test** — `tests/test_generate.py`

```python
from parking_proj.scenarios import build_scenarios
from parking_proj.generate import build_case_dict


def test_case_dict_schema_and_pass():
    scen = {s.case_id: s for s in build_scenarios()}
    case = build_case_dict(scen["E_low"])
    assert case["case_id"] == "E_low"
    assert "route" in case and "waypoints" in case["route"]
    assert case["route"]["waypoint_labels"] == [1, 2, 3, 4]
    assert len(case["frames"]) > 0
    f0 = case["frames"][0]
    for key in ("t", "true_pose", "meas_pose", "cursor_s", "matched_seg",
                "est_lat_dev", "true_lat_dev", "end_flag", "gt_seg", "gt_s"):
        assert key in f0
    v = case["verdict"]
    assert v["backward_jumps"] == 0
    assert v["dropouts"] == 0
    assert v["mismatches"] <= 3
    assert v["passed"] is True


def test_all_cases_pass_branch_and_monotonic():
    for s in build_scenarios():
        case = build_case_dict(s)
        v = case["verdict"]
        assert v["backward_jumps"] == 0, s.case_id
        assert v["mismatches"] <= 3, (s.case_id, v["mismatches"])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_generate.py -v`
Expected: FAIL (`ModuleNotFoundError`).

- [ ] **Step 3: Implement `src/parking_proj/grade.py`**

```python
"""Grade algorithm output against simulation ground truth."""
import numpy as np


def true_lat_dev(route, frame) -> float:
    mp = route.point_at_s(frame.gt_s)
    tang = route.tangent_at_s(frame.gt_s)
    normal_left = np.array([-tang[1], tang[0]])
    return float(np.dot(np.array([frame.true_e, frame.true_n]) - mp, normal_left))


def grade_case(route, frames, results) -> dict:
    total = len(frames)
    mismatches = sum(1 for f, r in zip(frames, results)
                     if r is None or r.matched_seg != f.gt_seg)
    dropouts = sum(1 for r in results if r is None)
    backward = 0
    prev = -1.0
    for r in results:
        if r is None:
            continue
        if r.cursor_s < prev - 1e-6:
            backward += 1
        prev = r.cursor_s
    # deviation gap on (near-)straight frames only
    gaps = []
    for f, r in zip(frames, results):
        if r is None:
            continue
        gaps.append(abs(r.est_lat_dev - true_lat_dev(route, f)))
    max_gap = float(max(gaps)) if gaps else 0.0
    passed = (mismatches <= 3) and (backward == 0) and (dropouts == 0)
    return {
        "total_frames": total,
        "correct_branch_frames": total - mismatches,
        "mismatches": mismatches,
        "backward_jumps": backward,
        "dropouts": dropouts,
        "max_dev_gap": round(max_gap, 4),
        "passed": bool(passed),
    }
```

- [ ] **Step 4: Implement `src/parking_proj/generate.py`**

```python
"""Run all scenarios, grade them, and write prebaked JSON for the viewer."""
import json
import os
import numpy as np
from .simulate import simulate, SimConfig
from .projection import Projector
from .scenarios import build_scenarios
from . import grade as grading


def _round2(a):
    return [round(float(x), 3) for x in a]


def build_case_dict(scenario) -> dict:
    route = scenario.route
    frames = simulate(route, SimConfig(tier=scenario.tier, seed=scenario.seed))
    proj = Projector(route)
    results = []
    for f in frames:
        try:
            results.append(proj.step(f.meas_e, f.meas_n, f.meas_yaw))
        except Exception:
            results.append(None)

    verdict = grading.grade_case(route, frames, results)

    frame_dicts = []
    for f, r in zip(frames, results):
        frame_dicts.append({
            "t": round(f.t, 3),
            "speed": round(f.speed, 3),
            "true_pose": {"e": round(f.true_e, 3), "n": round(f.true_n, 3),
                          "h": round(f.true_yaw, 5)},
            "meas_pose": {"e": round(f.meas_e, 3), "n": round(f.meas_n, 3),
                          "h": round(f.meas_yaw, 5),
                          "pitch": round(f.pitch, 6), "roll": round(f.roll, 6)},
            "cursor_s": None if r is None else round(r.cursor_s, 3),
            "matched_seg": None if r is None else r.matched_seg,
            "est_lat_dev": None if r is None else round(r.est_lat_dev, 4),
            "true_lat_dev": round(grading.true_lat_dev(route, f), 4),
            "end_flag": None if r is None else bool(r.end_flag),
            "gt_seg": f.gt_seg,
            "gt_s": round(f.gt_s, 3),
        })

    return {
        "case_id": scenario.case_id,
        "name": scenario.name,
        "tier": scenario.tier,
        "route": {
            "points_e": _round2(route.points[:, 0]),
            "points_n": _round2(route.points[:, 1]),
            "s": _round2(route.s),
            "waypoint_indices": route.waypoint_indices,
            "waypoint_labels": route.waypoint_labels,
        },
        "config": {"ahead": 20.0, "behind": -5.0},
        "frames": frame_dicts,
        "verdict": verdict,
    }


def main(out_dir="out"):
    os.makedirs(out_dir, exist_ok=True)
    index = []
    for scenario in build_scenarios():
        case = build_case_dict(scenario)
        with open(os.path.join(out_dir, f"{scenario.case_id}.json"), "w") as fh:
            json.dump(case, fh)
        index.append({
            "case_id": scenario.case_id,
            "name": scenario.name,
            "tier": scenario.tier,
            "group": scenario.case_id.split("_")[0],
            "verdict": case["verdict"],
        })
    with open(os.path.join(out_dir, "index.json"), "w") as fh:
        json.dump({"cases": index}, fh)
    print(f"Wrote {len(index)} cases to {out_dir}/")


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `pytest tests/test_generate.py -v`
Expected: PASS (2 tests). If any scenario reports `mismatches > 3`, that scenario's geometry has a crossing inside the search window — do NOT loosen the acceptance bar; instead re-check that scenario's arc-length gap between strokes exceeds `W_SEARCH` (3.5 m) and fix the geometry scale in `scenarios.py`.

- [ ] **Step 6: Commit**

```bash
git add src/parking_proj/grade.py src/parking_proj/generate.py tests/test_generate.py
git commit -m "feat: grading + JSON generation for all 14 cases"
```

---

### Task 9: Viewer shell — layout, case list, panorama, data loader

**Files:**
- Create: `viewer/index.html`
- Create: `viewer/viewer.css`
- Create: `viewer/viewer.js`

**Interfaces:**
- Consumes: `out/index.json`, `out/<case_id>.json` (Task 8 output).
- Produces (JS globals used by Task 10):
  - `loadIndex() -> Promise` populates the case list
  - `loadCase(caseId) -> Promise` sets global `STATE.case`, `STATE.frame = 0`
  - `fitTransform(canvas, minE, maxE, minN, maxN) -> {toX, toY}` — fixed world→screen mapping
  - `drawPanorama()` — static route with numbered waypoints + arrows
  - global `STATE = {case, frame, playing, speed}`

- [ ] **Step 1: Create `viewer/index.html`**

```html
<div id="app">
  <aside id="left">
    <h2>Test cases</h2>
    <ul id="case-list"></ul>
    <h2>Route panorama</h2>
    <canvas id="panorama" width="320" height="240"></canvas>
  </aside>
  <main id="center">
    <div class="fig"><h3>BEV (top-down)</h3><canvas id="bev" width="560" height="300"></canvas></div>
    <div class="fig"><h3>Driver view (+x up, +y left)
      <label><input type="checkbox" id="persp-toggle"> perspective</label></h3>
      <canvas id="driver" width="560" height="300"></canvas></div>
    <div id="controls">
      <button id="btn-step-back">◀</button>
      <button id="btn-play">▶ / ❚❚</button>
      <button id="btn-step-fwd">▶</button>
      <input type="range" id="scrubber" min="0" max="0" value="0" step="1">
      <select id="speed"><option value="0.5">0.5×</option>
        <option value="1" selected>1×</option><option value="2">2×</option></select>
      <span id="frame-label">0 / 0</span>
    </div>
  </main>
  <aside id="right">
    <h2>Telemetry</h2>
    <table id="telemetry">
      <tr><td>heading</td><td id="tm-heading">–</td></tr>
      <tr><td>speed</td><td id="tm-speed">–</td></tr>
      <tr><td>pos (e,n)</td><td id="tm-pos">–</td></tr>
      <tr><td>est lat dev</td><td id="tm-estdev">–</td></tr>
      <tr><td>true lat dev</td><td id="tm-truedev">–</td></tr>
      <tr><td>progress</td><td id="tm-progress">–</td></tr>
      <tr><td>matched seg</td><td id="tm-seg">–</td></tr>
      <tr><td>frame</td><td id="tm-frame">–</td></tr>
      <tr><td>verdict</td><td id="tm-verdict">–</td></tr>
    </table>
  </aside>
</div>
<script src="viewer.js"></script>
```

- [ ] **Step 2: Create `viewer/viewer.css`**

```css
* { box-sizing: border-box; }
body { margin: 0; font: 13px/1.4 system-ui, sans-serif; color: #222; }
#app { display: grid; grid-template-columns: 340px 1fr 240px; height: 100vh; }
#left, #right { padding: 10px; overflow-y: auto; background: #f6f7f9; }
#center { padding: 10px; display: flex; flex-direction: column; gap: 8px; }
canvas { background: #fff; border: 1px solid #ccd; display: block; }
.fig h3 { margin: 4px 0; font-size: 13px; font-weight: 600; }
#case-list { list-style: none; padding: 0; margin: 0 0 12px; }
#case-list li { padding: 5px 8px; cursor: pointer; border-radius: 4px; }
#case-list li:hover { background: #e6ebf2; }
#case-list li.active { background: #d3e0f5; font-weight: 600; }
.badge { float: right; font-size: 11px; padding: 0 6px; border-radius: 8px; color: #fff; }
.badge.pass { background: #2e9e5b; } .badge.fail { background: #cc3a3a; }
#controls { display: flex; align-items: center; gap: 8px; }
#scrubber { flex: 1; }
#telemetry { width: 100%; border-collapse: collapse; }
#telemetry td { padding: 3px 4px; border-bottom: 1px solid #e2e5ea; }
#telemetry td:last-child { text-align: right; font-variant-numeric: tabular-nums; }
```

- [ ] **Step 3: Create `viewer/viewer.js` (shell + loader + panorama)**

```javascript
const STATE = { case: null, frame: 0, playing: false, speed: 1 };

async function loadIndex() {
  const res = await fetch("../out/index.json");
  const data = await res.json();
  const ul = document.getElementById("case-list");
  ul.innerHTML = "";
  for (const c of data.cases) {
    const li = document.createElement("li");
    const pass = c.verdict.passed;
    li.innerHTML = `${c.name}<span class="badge ${pass ? "pass" : "fail"}">${pass ? "PASS" : "FAIL"}</span>`;
    li.onclick = () => selectCase(c.case_id, li);
    ul.appendChild(li);
  }
}

async function loadCase(caseId) {
  const res = await fetch(`../out/${caseId}.json`);
  STATE.case = await res.json();
  STATE.frame = 0;
  const sc = document.getElementById("scrubber");
  sc.max = STATE.case.frames.length - 1;
  sc.value = 0;
}

function routeXY(c) {
  return { e: c.route.points_e, n: c.route.points_n };
}

function fitTransform(canvas, minE, maxE, minN, maxN, pad = 20) {
  const w = canvas.width, h = canvas.height;
  const sx = (w - 2 * pad) / Math.max(maxE - minE, 1e-6);
  const sy = (h - 2 * pad) / Math.max(maxN - minN, 1e-6);
  const s = Math.min(sx, sy);
  const toX = (e) => pad + (e - minE) * s;
  const toY = (n) => h - pad - (n - minN) * s;  // north up
  return { toX, toY, s };
}

function routeBounds(c) {
  const { e, n } = routeXY(c);
  return {
    minE: Math.min(...e), maxE: Math.max(...e),
    minN: Math.min(...n), maxN: Math.max(...n),
  };
}

function drawPanorama() {
  const c = STATE.case, cv = document.getElementById("panorama");
  const ctx = cv.getContext("2d");
  ctx.clearRect(0, 0, cv.width, cv.height);
  if (!c) return;
  const b = routeBounds(c);
  const T = fitTransform(cv, b.minE, b.maxE, b.minN, b.maxN);
  const { e, n } = routeXY(c);
  // route polyline
  ctx.strokeStyle = "#4477cc"; ctx.lineWidth = 2; ctx.beginPath();
  ctx.moveTo(T.toX(e[0]), T.toY(n[0]));
  for (let i = 1; i < e.length; i++) ctx.lineTo(T.toX(e[i]), T.toY(n[i]));
  ctx.stroke();
  // numbered waypoints
  const wi = c.route.waypoint_indices, wl = c.route.waypoint_labels;
  ctx.fillStyle = "#cc3a3a"; ctx.font = "bold 13px sans-serif";
  for (let k = 0; k < wi.length; k++) {
    const x = T.toX(e[wi[k]]), y = T.toY(n[wi[k]]);
    ctx.beginPath(); ctx.arc(x, y, 4, 0, 2 * Math.PI); ctx.fill();
    ctx.fillText(String(wl[k]), x + 6, y - 6);
  }
  // direction arrows between waypoints
  ctx.strokeStyle = "#cc3a3a"; ctx.fillStyle = "#cc3a3a";
  for (let k = 0; k + 1 < wi.length; k++) {
    const mi = Math.floor((wi[k] + wi[k + 1]) / 2);
    drawArrow(ctx, T.toX(e[mi]), T.toY(n[mi]),
              Math.atan2(-(n[mi + 1] - n[mi]), e[mi + 1] - e[mi]));
  }
}

function drawArrow(ctx, x, y, ang) {
  const L = 8;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x - L * Math.cos(ang - 0.4), y - L * Math.sin(ang - 0.4));
  ctx.moveTo(x, y);
  ctx.lineTo(x - L * Math.cos(ang + 0.4), y - L * Math.sin(ang + 0.4));
  ctx.stroke();
}

async function selectCase(caseId, li) {
  document.querySelectorAll("#case-list li").forEach((x) => x.classList.remove("active"));
  if (li) li.classList.add("active");
  STATE.playing = false;
  await loadCase(caseId);
  drawPanorama();
  renderFrame();   // defined in Task 10
}

window.addEventListener("DOMContentLoaded", loadIndex);
```

- [ ] **Step 4: Manual smoke test (V1)**

Run:
```bash
python -m parking_proj.generate            # writes out/
cd viewer && python -m http.server 8000
```
Open `http://localhost:8000/index.html`. Expected: left panel lists 14 cases with PASS/FAIL badges; clicking a case draws the panorama with numbered waypoints 1,2,3(…) and direction arrows. (`renderFrame` is stubbed until Task 10 — a `ReferenceError` in console is expected here and resolved in Task 10.)

- [ ] **Step 5: Commit**

```bash
git add viewer/index.html viewer/viewer.css viewer/viewer.js
git commit -m "feat: viewer shell, case list, panorama, data loader"
```

---

### Task 10: Viewer — BEV, driver-view, playback, telemetry

**Files:**
- Modify: `viewer/viewer.js` (append rendering + playback + controls)

**Interfaces:**
- Consumes: `STATE`, `fitTransform`, `routeXY`, `routeBounds`, `drawPanorama` (Task 9).
- Produces: `renderFrame()`, `worldToBody(e,n,ex,ny,yaw)`, and wired controls (play/step/scrub/speed).

- [ ] **Step 1: Append BEV + driver-view + telemetry rendering to `viewer/viewer.js`**

```javascript
// ---- offscreen static layers (anti-flicker) ----
let BEV_STATIC = null, BEV_T = null;

function buildBevStatic() {
  const c = STATE.case, cv = document.getElementById("bev");
  const b = routeBounds(c);
  BEV_T = fitTransform(cv, b.minE - 3, b.maxE + 3, b.minN - 3, b.maxN + 3);
  BEV_STATIC = document.createElement("canvas");
  BEV_STATIC.width = cv.width; BEV_STATIC.height = cv.height;
  const ctx = BEV_STATIC.getContext("2d");
  const { e, n } = routeXY(c);
  ctx.strokeStyle = "#9db4d8"; ctx.lineWidth = 2; ctx.beginPath();
  ctx.moveTo(BEV_T.toX(e[0]), BEV_T.toY(n[0]));
  for (let i = 1; i < e.length; i++) ctx.lineTo(BEV_T.toX(e[i]), BEV_T.toY(n[i]));
  ctx.stroke();
}

function worldToBody(pe, pn, ex, ny, yaw) {
  const de = ex - pe, dn = ny - pn;
  const cs = Math.cos(yaw), sn = Math.sin(yaw);
  return { x: de * cs + dn * sn, y: -de * sn + dn * cs };  // x fwd, y left
}

function drawBev() {
  const c = STATE.case, cv = document.getElementById("bev");
  const ctx = cv.getContext("2d");
  if (!BEV_STATIC) buildBevStatic();
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.drawImage(BEV_STATIC, 0, 0);
  // driven trajectory up to current frame
  ctx.strokeStyle = "#222"; ctx.lineWidth = 2; ctx.beginPath();
  for (let i = 0; i <= STATE.frame; i++) {
    const f = c.frames[i];
    const x = BEV_T.toX(f.true_pose.e), y = BEV_T.toY(f.true_pose.n);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();
  // car marker
  const f = c.frames[STATE.frame];
  drawCar(ctx, BEV_T, f.true_pose.e, f.true_pose.n, f.true_pose.h);
}

function drawCar(ctx, T, e, n, yaw) {
  const L = 1.8, W = 0.9;  // meters (half-extents)
  const corners = [[L, W], [L, -W], [-L, -W], [-L, W]];
  ctx.fillStyle = "rgba(204,58,58,0.8)"; ctx.beginPath();
  corners.forEach(([fx, fy], i) => {
    const ex = e + fx * Math.cos(yaw) - fy * Math.sin(yaw);
    const ny = n + fx * Math.sin(yaw) + fy * Math.cos(yaw);
    const x = T.toX(ex), y = T.toY(ny);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.closePath(); ctx.fill();
}

function drawDriver() {
  const c = STATE.case, cv = document.getElementById("driver");
  const ctx = cv.getContext("2d");
  ctx.clearRect(0, 0, cv.width, cv.height);
  const f = c.frames[STATE.frame];
  if (f.cursor_s == null) return;
  const ahead = c.config.ahead, behind = c.config.behind;
  // body-frame fixed transform: x forward -> up, y left -> left
  const w = cv.width, h = cv.height, ppm = (h - 20) / (ahead - behind);
  const toX = (by) => w / 2 - by * ppm;   // +y left -> screen left
  const toY = (bx) => h - 10 - (bx - behind) * ppm; // +x forward -> up
  // slice of route in [cursor_s+behind, cursor_s+ahead]
  const s = c.route.s, e = c.route.points_e, n = c.route.points_n;
  const loS = f.cursor_s + behind, hiS = f.cursor_s + ahead;
  ctx.strokeStyle = "#2e9e5b"; ctx.lineWidth = 3; ctx.beginPath();
  let started = false;
  for (let i = 0; i < s.length; i++) {
    if (s[i] < loS || s[i] > hiS) continue;
    const b = worldToBody(f.meas_pose.e, f.meas_pose.n, e[i], n[i], f.meas_pose.h);
    const x = toX(b.y), y = toY(b.x);
    if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
  }
  ctx.stroke();
  // car at origin
  ctx.fillStyle = "#cc3a3a";
  ctx.fillRect(toX(0.9), toY(1.8), (0.9 * 2) * ppm, (1.8 + 1.8) * ppm * 0 + 3.6 * ppm);
  if (f.end_flag) {
    ctx.fillStyle = "#cc3a3a"; ctx.font = "12px sans-serif";
    ctx.fillText("route ends", 8, 16);
  }
  if (document.getElementById("persp-toggle").checked) drawPerspective(ctx, f);
}

function drawPerspective(ctx, f) {
  // nice-to-have default pinhole overlay: cam height 1.4m, pitch -2deg, hfov 60
  const cv = ctx.canvas, cx = cv.width * 0.75, cyv = cv.height * 0.35;
  const fpx = (cv.width * 0.25) / Math.tan((60 * Math.PI / 180) / 2);
  const H = 1.4, pitch = -2 * Math.PI / 180;
  ctx.strokeStyle = "#a0522d"; ctx.lineWidth = 2; ctx.beginPath();
  const s = STATE.case.route.s, e = STATE.case.route.points_e, n = STATE.case.route.points_n;
  let started = false;
  for (let i = 0; i < s.length; i++) {
    if (s[i] < f.cursor_s || s[i] > f.cursor_s + STATE.case.config.ahead) continue;
    const b = worldToBody(f.meas_pose.e, f.meas_pose.n, e[i], n[i], f.meas_pose.h);
    if (b.x <= 0.5) continue;
    const px = cx - (b.y / b.x) * fpx * 0.25;
    const py = cyv + (H / b.x + pitch) * fpx * 0.25;
    if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
  }
  ctx.stroke();
}

function renderFrame() {
  const c = STATE.case; if (!c) return;
  drawBev(); drawDriver(); updateTelemetry(); drawPanoramaDot();
  document.getElementById("scrubber").value = STATE.frame;
  document.getElementById("frame-label").textContent =
    `${STATE.frame} / ${c.frames.length - 1}`;
}

function drawPanoramaDot() {
  drawPanorama();  // static redraw (cheap) then dot
  const c = STATE.case, cv = document.getElementById("panorama");
  const ctx = cv.getContext("2d");
  const b = routeBounds(c);
  const T = fitTransform(cv, b.minE, b.maxE, b.minN, b.maxN);
  const f = c.frames[STATE.frame];
  ctx.fillStyle = "#111";
  ctx.beginPath(); ctx.arc(T.toX(f.true_pose.e), T.toY(f.true_pose.n), 3, 0, 2 * Math.PI); ctx.fill();
}

function updateTelemetry() {
  const c = STATE.case, f = c.frames[STATE.frame];
  const deg = (r) => (90 - r * 180 / Math.PI).toFixed(2);  // compass
  const set = (id, v) => (document.getElementById(id).textContent = v);
  set("tm-heading", `${deg(f.meas_pose.h)}° (N-CW)`);
  set("tm-speed", `${(f.speed * 3.6).toFixed(1)} km/h`);
  set("tm-pos", `(${f.meas_pose.e.toFixed(2)}, ${f.meas_pose.n.toFixed(2)})`);
  set("tm-estdev", f.est_lat_dev == null ? "–" : `${f.est_lat_dev.toFixed(3)} m`);
  set("tm-truedev", `${f.true_lat_dev.toFixed(3)} m`);
  const prog = c.route.s.length ? (f.cursor_s / c.route.s[c.route.s.length - 1] * 100) : 0;
  set("tm-progress", `${prog.toFixed(1)}%`);
  set("tm-seg", f.matched_seg == null ? "–" : String(f.matched_seg));
  set("tm-frame", `${STATE.frame} / ${c.frames.length - 1}`);
  const v = c.verdict;
  set("tm-verdict", `${v.passed ? "PASS" : "FAIL"} (mis ${v.mismatches})`);
}
```

- [ ] **Step 2: Append playback + controls to `viewer/viewer.js`**

```javascript
let lastTick = 0;
function tick(ts) {
  if (STATE.playing && STATE.case) {
    const dtMs = 1000 / (10 * STATE.speed);   // frames stored at 10 Hz
    if (ts - lastTick >= dtMs) {
      lastTick = ts;
      if (STATE.frame < STATE.case.frames.length - 1) { STATE.frame++; renderFrame(); }
      else STATE.playing = false;
    }
  }
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);

window.addEventListener("DOMContentLoaded", () => {
  document.getElementById("btn-play").onclick = () => { STATE.playing = !STATE.playing; };
  document.getElementById("btn-step-fwd").onclick = () => {
    if (STATE.case && STATE.frame < STATE.case.frames.length - 1) { STATE.frame++; renderFrame(); }
  };
  document.getElementById("btn-step-back").onclick = () => {
    if (STATE.case && STATE.frame > 0) { STATE.frame--; renderFrame(); }
  };
  document.getElementById("scrubber").oninput = (ev) => {
    if (STATE.case) { STATE.frame = parseInt(ev.target.value, 10); renderFrame(); }
  };
  document.getElementById("speed").onchange = (ev) => { STATE.speed = parseFloat(ev.target.value); };
  document.getElementById("persp-toggle").onchange = () => renderFrame();
});
```

Note: the shell (Task 9) also has a `DOMContentLoaded` handler calling `loadIndex()`. Both handlers coexist (multiple listeners are allowed). When `selectCase` calls `renderFrame()`, it is now defined.

- [ ] **Step 3: Manual smoke tests (V2, V3, V4)**

Run:
```bash
python -m parking_proj.generate
cd viewer && python -m http.server 8000
```
Open `http://localhost:8000/index.html`. Expected:
- **V2:** selecting a case shows BEV (route + growing driven trajectory + car) and driver-view (green route slice, `+x` up / `+y` left, car at origin). Telemetry updates: heading, speed ≈8 km/h, pos, est+true lat dev, progress, seg, frame k/N. Play/step/scrub all work; the perspective checkbox overlays a converging line.
- **V3:** during continuous play there is no pan/zoom/flicker (fixed transforms; BEV static layer blitted).
- **V4:** clicking between cases shows identical data each time; no regeneration occurs (viewer only fetches JSON).

- [ ] **Step 4: Commit**

```bash
git add viewer/viewer.js
git commit -m "feat: viewer BEV, driver-view, playback, telemetry"
```

---

### Task 11: End-to-end generation + acceptance verification

**Files:**
- Create: `README.md`
- Test: `tests/test_acceptance.py`

**Interfaces:**
- Consumes: everything above.
- Produces: `README.md` (run instructions); a full-suite acceptance test that asserts every case meets §6 criteria.

- [ ] **Step 1: Write the acceptance test** — `tests/test_acceptance.py`

```python
from parking_proj.scenarios import build_scenarios
from parking_proj.generate import build_case_dict


def test_every_case_meets_acceptance():
    failures = []
    for s in build_scenarios():
        v = build_case_dict(s)["verdict"]
        ok = (v["mismatches"] <= 3 and v["backward_jumps"] == 0 and v["dropouts"] == 0)
        if not ok:
            failures.append((s.case_id, v))
    assert not failures, f"cases failing acceptance: {failures}"
```

- [ ] **Step 2: Run the acceptance test**

Run: `pytest tests/test_acceptance.py -v`
Expected: PASS. If a self-crossing case (E/F/G) fails on `mismatches`, the fix is geometric (increase stroke separation / scale) or a `W_SEARCH` reduction in `Projector` defaults — never loosening the 3-frame bar.

- [ ] **Step 3: Run the full suite**

Run: `pytest -v`
Expected: all tests across `test_geo`, `test_route`, `test_geometry`, `test_transform`, `test_projection`, `test_simulate`, `test_scenarios`, `test_generate`, `test_acceptance` PASS.

- [ ] **Step 4: Generate artifacts and eyeball the viewer**

Run:
```bash
python -m parking_proj.generate
cd viewer && python -m http.server 8000
```
Open the viewer, click through all 14 cases, confirm each PASS badge, and confirm the X-crossing (E) and two-crossing (G) driver-views stay on the correct stroke through both crossings.

- [ ] **Step 5: Create `README.md`**

```markdown
# Parking Route Projection

Projects a global (ENU) planned route into the vehicle-body frame each frame,
with a seeded two-layer simulation and a static HTML viewer.

## Setup
    pip install -e ".[dev]"

## Test
    pytest -v

## Generate test-case JSON
    python -m parking_proj.generate      # writes out/

## View
    cd viewer && python -m http.server 8000
    # open http://localhost:8000/index.html

Body frame: +x forward, +y left, +z up. Heading = CCW from ENU East.
```

- [ ] **Step 6: Commit**

```bash
git add README.md tests/test_acceptance.py
git commit -m "test: end-to-end acceptance across all 14 cases + README"
```

---

## Self-Review

**1. Spec coverage:**
- §1 conventions → Task 1 (geo), Task 4 (body transform), Global Constraints. ✓
- §2 two-halves architecture → Python core (Tasks 1–8) + static viewer (Tasks 9–10); viewer holds no matching logic (only `worldToBody` display rotation). ✓
- §3.3 algorithm (seed, forward window, heading gate, monotonic cursor, emit, telemetry) → Task 5. ✓
- §3.4 body transform → Task 4. ✓
- §3.5 two-layer sim (tracking + RTK, tiers, determinism, corner rounding, ground truth) → Task 6. ✓
- §3.6 JSON record (decisions, not full slice) → Task 8 `build_case_dict`. ✓
- §3.7 viewer (panorama numbered, BEV, driver-view, controls, anti-flicker, pinhole overlay) → Tasks 9–10. ✓
- §4 error handling (route start/end, cap, gate fallback, seed) → Task 5 (gate widen, end_flag, seed) + Task 6 (caps). ✓
- §5 tests U1–U6/S1–S6/E1–E5/V1–V4 → mapped: U1 T1; U2 T2; U3 T4; U4/U5/U6 T5; sim caps/continuity T6; S/E acceptance T8+T11; V1 T9; V2–V4 T10. ✓
- §6 acceptance → Task 8 grading + Task 11 suite. ✓
- 14-case matrix (§5.5) → Task 7. ✓

**2. Placeholder scan:** No "TBD/TODO" in steps. The two spec TBDs (exact pinhole params, exact Hefei origin) are resolved with concrete defaults (camera 1.4 m / −2° / 60° in `drawPerspective`; Hefei origin `31.8206, 117.2290` in `geo.py`).

**3. Type consistency:** `ProjectionResult` fields (`cursor_s, matched_index, matched_seg, est_lat_dev, end_flag, gate_widened`) are produced in Task 5 and consumed consistently in Task 8. `Frame` fields (Task 6) match `build_case_dict` access (Task 8). `Route` API (`points, s, tangents, length, waypoint_indices, waypoint_labels, index_at_s, point_at_s, tangent_at_s, segment_at_s, seg_of_index`) is defined in Task 2 and used identically in Tasks 3/5/6/8. JS `STATE`, `fitTransform`, `routeXY`, `routeBounds`, `worldToBody`, `renderFrame` are defined once and reused across Tasks 9–10.

**Note for the implementer:** edge tests E1–E5 from the spec are covered behaviorally by the scenario runs (route start/end via every case, cap via high-tier cases, seed via E-high). If a reviewer wants them as isolated unit tests, add them against the `Projector`/`simulate` APIs already defined — no new interfaces required.
