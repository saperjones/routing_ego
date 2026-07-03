# Real-Data Ingestion + OSM Basemap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the existing projection algorithm on real vehicle datasets (`dataset/`), prebake the results (+ an OSM basemap) to JSON, and add a Simulation/Real-data tab to the viewer that plays real cases back with a map-backed BEV — all in WGS-84.

**Architecture:** Python computes offline, the viewer replays (existing invariant). A new adapter reads a dataset dir → builds the `Route` (from `planned_route`) and a per-frame measured-pose stream (from `ego_route_llh`, GCJ-02→WGS-84→ENU, heading = `yaw_boot`+θ), runs the unchanged `Projector`, and writes `out/real/<id>.json` + downloaded OSM tiles. The viewer gains tabs and a real-data BEV renderer; the algorithm and sim path are untouched.

**Tech Stack:** Python 3.12 + numpy (existing); `urllib` for tile download (stdlib, no new dep); vanilla HTML/Canvas JS viewer; pytest + playwright (existing).

## Global Constraints

- **Datum: WGS-84 everywhere in this feature.** `ego_route_llh.json` `llh` is **GCJ-02** → convert GCJ-02→WGS-84 first. `planned_route.json` is WGS-84 `[lat, lon]`.
- **Algorithm frame:** local ENU meters, origin = **mean of the dataset's WGS-84 planned-route points**; planned route and ego poses share that origin.
- **Heading:** `yaw_enu = yaw_boot + θ`; θ = per-dataset boot→ENU rotation estimated from **positions only** via least-squares (`θ = atan2(Σ(aₓbᵧ−aᵧbₓ), Σ(aₓbₓ+aᵧbᵧ))`, `a`=boot displacements, `b`=ENU displacements, stride 10, keep `|a|>0.3 m`); validate `scale=Σ|b|/Σ|a|≈1`.
- **No pose/heading smoothing** in the algorithm (unchanged Projector).
- **Real cases have no ground truth:** no `gt_s`/`gt_seg`, no `true_lat_dev`, no PASS/FAIL, no correct-branch metric. `est_lat_dev` is still emitted.
- **BEV display** uses Web-Mercator tile pixel space; overlays via `lon/lat → global pixel` at the basemap zoom. Tiles fetched **only at prep time**; on failure → `basemap:null` and a **gray graticule fallback** (everything else still works).
- **Direction arrows:** every ~**20 m** of arc length, on **both** the planned route and the ego track.
- **Anti-flicker:** basemap + route + ego track + arrows drawn **once** to an offscreen layer; only the car marker + progress dot redraw per frame; fixed transform (no pan/zoom during playback).
- **Qualifying dataset dir:** a first-level `dataset/` subdirectory containing both `ego_route_llh.json` and `route_generation_result/planned_route.json`.
- `out/` is git-ignored (includes `out/real/`).
- Tile fetch politeness: send a `User-Agent` header; cap tiles per dataset (~25); cache to disk.

---

### Task 1: `geo.py` — GCJ-02 ⇄ WGS-84 + parameterized ENU

**Files:**
- Modify: `src/parking_proj/geo.py`
- Test: `tests/test_geo_gcj.py`

**Interfaces:**
- Consumes: nothing new.
- Produces:
  - `wgs84_to_gcj02(lat: float, lon: float) -> tuple[float, float]`
  - `gcj02_to_wgs84(lat: float, lon: float) -> tuple[float, float]`
  - `enu_about(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]` → `(e, n)` meters about an arbitrary origin

- [ ] **Step 1: Write the failing test** — `tests/test_geo_gcj.py`

```python
import math
from parking_proj import geo


def test_gcj_wgs_roundtrip_sub_meter():
    # a point near the Hefei datasets
    lat, lon = 31.8341, 117.1447
    glat, glon = geo.wgs84_to_gcj02(lat, lon)
    # GCJ offset in China is tens-to-hundreds of meters, so it must differ
    assert abs(glat - lat) + abs(glon - lon) > 1e-4
    wlat, wlon = geo.gcj02_to_wgs84(glat, glon)
    # inverse recovers the original to well under a meter (~1e-6 deg)
    assert abs(wlat - lat) < 1e-6 and abs(wlon - lon) < 1e-6


def test_enu_about_origin_is_zero_and_scales():
    lat0, lon0 = 31.8341, 117.1447
    e, n = geo.enu_about(lat0, lon0, lat0, lon0)
    assert abs(e) < 1e-9 and abs(n) < 1e-9
    # 0.001 deg north ~ 111 m; east scaled by cos(lat0)
    e2, n2 = geo.enu_about(lat0 + 0.001, lon0 + 0.001, lat0, lon0)
    assert abs(n2 - 111.19) < 1.0
    assert abs(e2 - 94.6) < 2.0   # 111.19 * cos(31.83deg)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./.venv/bin/python -m pytest tests/test_geo_gcj.py -v`
Expected: FAIL (`AttributeError: module 'parking_proj.geo' has no attribute 'wgs84_to_gcj02'`).

- [ ] **Step 3: Append to `src/parking_proj/geo.py`**

```python
# --- GCJ-02 (China datum) <-> WGS-84, and parameterized ENU -----------------
_GCJ_A = 6378245.0
_GCJ_EE = 0.00669342162296594323


def _gcj_tlat(x, y):
    r = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    r += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    r += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
    r += (160.0 * math.sin(y / 12.0 * math.pi) + 320.0 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
    return r


def _gcj_tlon(x, y):
    r = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    r += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    r += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
    r += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
    return r


def wgs84_to_gcj02(lat: float, lon: float) -> tuple[float, float]:
    dlat = _gcj_tlat(lon - 105.0, lat - 35.0)
    dlon = _gcj_tlon(lon - 105.0, lat - 35.0)
    rl = lat * _DEG
    m = math.sin(rl)
    m = 1 - _GCJ_EE * m * m
    sm = math.sqrt(m)
    dlat = (dlat * 180.0) / ((_GCJ_A * (1 - _GCJ_EE)) / (m * sm) * math.pi)
    dlon = (dlon * 180.0) / (_GCJ_A / sm * math.cos(rl) * math.pi)
    return lat + dlat, lon + dlon


def gcj02_to_wgs84(lat: float, lon: float) -> tuple[float, float]:
    """Iterative inverse of wgs84_to_gcj02 (converges to sub-mm in a few steps)."""
    wlat, wlon = lat, lon
    for _ in range(3):
        glat, glon = wgs84_to_gcj02(wlat, wlon)
        wlat += lat - glat
        wlon += lon - glon
    return wlat, wlon


def enu_about(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """Local ENU (meters) about an arbitrary origin (lat0, lon0)."""
    e = (lon - lon0) * _DEG * _EARTH_R * math.cos(lat0 * _DEG)
    n = (lat - lat0) * _DEG * _EARTH_R
    return e, n
```

Note: `_DEG` and `_EARTH_R` already exist in `geo.py` (used by `wgs84_to_enu`); reuse them.

- [ ] **Step 4: Run test to verify it passes**

Run: `./.venv/bin/python -m pytest tests/test_geo_gcj.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/geo.py tests/test_geo_gcj.py
git commit -m "feat: GCJ-02<->WGS-84 conversion and parameterized ENU in geo"
```

---

### Task 2: `osm.py` — slippy-tile math (pure)

**Files:**
- Create: `src/parking_proj/osm.py`
- Test: `tests/test_osm_math.py`

**Interfaces:**
- Consumes: nothing (math only).
- Produces:
  - `lonlat_to_global_px(lon: float, lat: float, z: int) -> tuple[float, float]` — pixel in the full world map at zoom `z` (256-px tiles)
  - `deg2tile(lon: float, lat: float, z: int) -> tuple[int, int]` — `(xtile, ytile)`
  - `choose_zoom(min_lon, min_lat, max_lon, max_lat, max_tiles: int = 25, zmax: int = 18) -> int`
  - `tile_span(min_lon, min_lat, max_lon, max_lat, z) -> tuple[int, int, int, int]` — `(x0, y0, nx, ny)` covering tiles

- [ ] **Step 1: Write the failing test** — `tests/test_osm_math.py`

```python
import math
from parking_proj import osm


def test_global_px_monotonic_and_tile_consistent():
    z = 16
    lon, lat = 117.1447, 31.8341
    px, py = osm.lonlat_to_global_px(lon, lat, z)
    # moving east increases px; moving north DECREASES py (y grows southward)
    px_e, _ = osm.lonlat_to_global_px(lon + 0.001, lat, z)
    _, py_n = osm.lonlat_to_global_px(lon, lat + 0.001, z)
    assert px_e > px
    assert py_n < py
    # tile index == floor(global_px / 256)
    xt, yt = osm.deg2tile(lon, lat, z)
    assert xt == int(px // 256) and yt == int(py // 256)


def test_choose_zoom_respects_tile_cap():
    # ~2 km box
    box = (117.135, 31.828, 117.150, 31.840)
    z = osm.choose_zoom(*box, max_tiles=25)
    x0, y0, nx, ny = osm.tile_span(*box, z)
    assert nx * ny <= 25
    # one zoom higher would exceed the cap (so choose_zoom picked the max feasible)
    x0b, y0b, nxb, nyb = osm.tile_span(*box, z + 1)
    assert nxb * nyb > 25
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./.venv/bin/python -m pytest tests/test_osm_math.py -v`
Expected: FAIL (`ModuleNotFoundError: parking_proj.osm`).

- [ ] **Step 3: Implement `src/parking_proj/osm.py`**

```python
"""Web-Mercator slippy-tile math (EPSG:3857, 256-px tiles)."""
import math

TILE = 256


def lonlat_to_global_px(lon, lat, z):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n * TILE
    rl = math.radians(lat)
    y = (1.0 - math.asinh(math.tan(rl)) / math.pi) / 2.0 * n * TILE
    return x, y


def deg2tile(lon, lat, z):
    px, py = lonlat_to_global_px(lon, lat, z)
    return int(px // TILE), int(py // TILE)


def tile_span(min_lon, min_lat, max_lon, max_lat, z):
    x0, y_top = deg2tile(min_lon, max_lat, z)   # north-west tile
    x1, y_bot = deg2tile(max_lon, min_lat, z)   # south-east tile
    return x0, y_top, (x1 - x0 + 1), (y_bot - y_top + 1)


def choose_zoom(min_lon, min_lat, max_lon, max_lat, max_tiles=25, zmax=18):
    best = 0
    for z in range(1, zmax + 1):
        _, _, nx, ny = tile_span(min_lon, min_lat, max_lon, max_lat, z)
        if nx * ny <= max_tiles:
            best = z
    return best
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./.venv/bin/python -m pytest tests/test_osm_math.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/osm.py tests/test_osm_math.py
git commit -m "feat: OSM slippy-tile math (mercator px, tile span, zoom pick)"
```

---

### Task 3: `osm.py` — tile fetch + basemap manifest (graceful failure)

**Files:**
- Modify: `src/parking_proj/osm.py`
- Test: `tests/test_osm_fetch.py`

**Interfaces:**
- Consumes: `lonlat_to_global_px`, `choose_zoom`, `tile_span` (Task 2).
- Produces:
  - `fetch_basemap(min_lon, min_lat, max_lon, max_lat, out_dir: str, max_tiles: int = 25, tile_getter=None) -> dict | None` — downloads covering tiles into `out_dir/tiles/`, writes nothing else, and **returns a manifest dict** `{z, x0, y0, nx, ny, tile, tiles:[{x,y,file}]}` (paths relative to `out_dir`), or `None` if any tile fails. `tile_getter(z, x, y) -> bytes | None` is injectable for testing; default hits the OSM raster server.

- [ ] **Step 1: Write the failing test** — `tests/test_osm_fetch.py`

```python
import os
from parking_proj import osm


def _fake_tile(z, x, y):
    # a minimal valid 1x1 PNG (bytes); content doesn't matter for the manifest
    return (b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01"
            b"\x08\x06\x00\x00\x00\x1f\x15\xc4\x89\x00\x00\x00\nIDATx\x9cc\x00"
            b"\x01\x00\x00\x05\x00\x01\r\n-\xb4\x00\x00\x00\x00IEND\xaeB`\x82")


def test_fetch_writes_tiles_and_manifest(tmp_path):
    box = (117.135, 31.828, 117.150, 31.840)
    man = osm.fetch_basemap(*box, out_dir=str(tmp_path), max_tiles=25, tile_getter=_fake_tile)
    assert man is not None
    assert man["nx"] * man["ny"] == len(man["tiles"]) <= 25
    for t in man["tiles"]:
        assert os.path.exists(os.path.join(str(tmp_path), t["file"]))
    assert man["tile"] == 256


def test_fetch_returns_none_on_failure(tmp_path):
    box = (117.135, 31.828, 117.150, 31.840)
    man = osm.fetch_basemap(*box, out_dir=str(tmp_path), tile_getter=lambda z, x, y: None)
    assert man is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./.venv/bin/python -m pytest tests/test_osm_fetch.py -v`
Expected: FAIL (`AttributeError: ... has no attribute 'fetch_basemap'`).

- [ ] **Step 3: Append to `src/parking_proj/osm.py`**

```python
import os
import urllib.request

_TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
_UA = "parking-route-projection/0.1 (offline research viewer)"


def _http_tile(z, x, y):
    url = _TILE_URL.format(z=z, x=x, y=y)
    try:
        req = urllib.request.Request(url, headers={"User-Agent": _UA})
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.read()
    except Exception:
        return None


def fetch_basemap(min_lon, min_lat, max_lon, max_lat, out_dir,
                  max_tiles=25, tile_getter=None):
    getter = tile_getter or _http_tile
    z = choose_zoom(min_lon, min_lat, max_lon, max_lat, max_tiles)
    if z < 1:
        return None
    x0, y0, nx, ny = tile_span(min_lon, min_lat, max_lon, max_lat, z)
    tiles_dir = os.path.join(out_dir, "tiles")
    os.makedirs(tiles_dir, exist_ok=True)
    tiles = []
    for x in range(x0, x0 + nx):
        for y in range(y0, y0 + ny):
            data = getter(z, x, y)
            if not data:
                return None
            fname = f"tiles/{z}_{x}_{y}.png"
            with open(os.path.join(out_dir, fname), "wb") as fh:
                fh.write(data)
            tiles.append({"x": x, "y": y, "file": fname})
    return {"z": z, "x0": x0, "y0": y0, "nx": nx, "ny": ny, "tile": TILE, "tiles": tiles}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./.venv/bin/python -m pytest tests/test_osm_fetch.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/osm.py tests/test_osm_fetch.py
git commit -m "feat: OSM tile fetch + basemap manifest with graceful failure"
```

---

### Task 4: `realdata.py` — θ estimation + dataset adapter

**Files:**
- Create: `src/parking_proj/realdata.py`
- Test: `tests/test_realdata.py`

**Interfaces:**
- Consumes: `geo.gcj02_to_wgs84`, `geo.enu_about` (Task 1); `geometry.route_from_dense` (existing); `route.Route` (existing).
- Produces:
  - `estimate_boot_to_enu_theta(pos_boot: np.ndarray, pos_enu: np.ndarray, stride: int = 10, min_disp: float = 0.3) -> tuple[float, float]` → `(theta_rad, scale)`
  - `@dataclass RealDataset`: `route` (`Route`), `route_llh` (`np.ndarray (K,2)` lat,lon), `ego_llh` (`np.ndarray (M,2)` lat,lon WGS-84), `meas_e/meas_n/meas_yaw/speed/t_us` (`np.ndarray (M,)`), `lat0/lon0/theta_rad` (float), `dataset_id` (str)
  - `is_dataset_dir(path: str) -> bool`
  - `load_dataset(path: str) -> RealDataset`

- [ ] **Step 1: Write the failing test** — `tests/test_realdata.py`

```python
import math
import numpy as np
from pathlib import Path
from parking_proj import realdata

DATASET = ("dataset/dev_CHERY_M32T_46651_ALL_MANUAL_2026-06-22-14-08-25_"
           "20260625_101425_annotation")


def test_theta_recovers_synthetic_rotation():
    rng = np.random.default_rng(0)
    enu = np.cumsum(rng.normal(0, 1, (500, 2)), axis=0)  # a wandering path
    th = math.radians(33.0)
    R = np.array([[math.cos(-th), -math.sin(-th)], [math.sin(-th), math.cos(-th)]])
    boot = enu @ R.T           # boot = ENU rotated by -th, so recovered theta = +th
    est, scale = realdata.estimate_boot_to_enu_theta(boot, enu)
    assert abs(math.degrees(est) - 33.0) < 0.5
    assert abs(scale - 1.0) < 0.02


def test_is_dataset_dir_and_load():
    d = Path(DATASET)
    if not d.exists():
        import pytest
        pytest.skip("sample dataset not present")
    assert realdata.is_dataset_dir(str(d))
    ds = realdata.load_dataset(str(d))
    assert ds.route.length > 100.0
    assert ds.meas_e.shape == ds.meas_n.shape == ds.meas_yaw.shape
    assert len(ds.meas_e) > 1000
    # theta near the measured ~33 deg for this dataset
    assert 20.0 < math.degrees(ds.theta_rad) < 45.0
    # measured poses are finite
    assert np.all(np.isfinite(ds.meas_e)) and np.all(np.isfinite(ds.meas_yaw))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./.venv/bin/python -m pytest tests/test_realdata.py -v`
Expected: FAIL (`ModuleNotFoundError: parking_proj.realdata`).

- [ ] **Step 3: Implement `src/parking_proj/realdata.py`**

```python
"""Adapter: a dataset dir -> Route + measured-pose stream for the Projector."""
import json
import math
import os
from dataclasses import dataclass
import numpy as np
from . import geo, geometry
from .route import Route


def estimate_boot_to_enu_theta(pos_boot, pos_enu, stride=10, min_disp=0.3):
    pb = np.asarray(pos_boot, float)
    pe = np.asarray(pos_enu, float)
    a = pb[stride:] - pb[:-stride]      # boot-frame displacements
    b = pe[stride:] - pe[:-stride]      # ENU displacements
    keep = np.hypot(a[:, 0], a[:, 1]) > min_disp
    a, b = a[keep], b[keep]
    if len(a) < 5:
        return 0.0, 1.0
    ssin = float(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]))
    scos = float(np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    theta = math.atan2(ssin, scos)
    scale = float(np.sum(np.hypot(b[:, 0], b[:, 1])) / np.sum(np.hypot(a[:, 0], a[:, 1])))
    return theta, scale


@dataclass
class RealDataset:
    dataset_id: str
    route: Route
    route_llh: np.ndarray
    ego_llh: np.ndarray
    meas_e: np.ndarray
    meas_n: np.ndarray
    meas_yaw: np.ndarray
    speed: np.ndarray
    t_us: np.ndarray
    lat0: float
    lon0: float
    theta_rad: float


def _ego_path(path):
    return os.path.join(path, "ego_route_llh.json")


def _planned_path(path):
    return os.path.join(path, "route_generation_result", "planned_route.json")


def is_dataset_dir(path):
    return os.path.isfile(_ego_path(path)) and os.path.isfile(_planned_path(path))


def load_dataset(path):
    ego = json.load(open(_ego_path(path)))
    pr = json.load(open(_planned_path(path)))
    pts = ego["points"]

    # planned route + waypoints are WGS-84 [lat, lon]
    planned = np.array(pr["planned_route"], float)          # (K,2) lat,lon
    waypoints = np.array(pr["waypoints"], float)            # (11,2) lat,lon
    lat0 = float(planned[:, 0].mean())
    lon0 = float(planned[:, 1].mean())

    dense_enu = np.array([geo.enu_about(la, lo, lat0, lon0) for la, lo in planned])
    wps_enu = [geo.enu_about(la, lo, lat0, lon0) for la, lo in waypoints]
    labels = list(range(1, len(waypoints) + 1))
    route = geometry.route_from_dense(dense_enu, wps_enu, labels)

    # ego llh is GCJ-02 -> WGS-84
    ego_wgs = np.array([geo.gcj02_to_wgs84(p["latitude"], p["longitude"]) for p in pts])
    ego_enu = np.array([geo.enu_about(la, lo, lat0, lon0) for la, lo in ego_wgs])
    pos_boot = np.array([[p["position_boot"]["x"], p["position_boot"]["y"]] for p in pts])
    theta, _scale = estimate_boot_to_enu_theta(pos_boot, ego_enu)

    yaw_boot = np.array([p["yaw_boot"] for p in pts], float)
    return RealDataset(
        dataset_id=os.path.basename(os.path.normpath(path)),
        route=route,
        route_llh=planned,
        ego_llh=ego_wgs,
        meas_e=ego_enu[:, 0], meas_n=ego_enu[:, 1],
        meas_yaw=yaw_boot + theta,
        speed=np.array([p["v"] for p in pts], float),
        t_us=np.array([p["timestamp_us"] for p in pts], float),
        lat0=lat0, lon0=lon0, theta_rad=theta,
    )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./.venv/bin/python -m pytest tests/test_realdata.py -v`
Expected: PASS (2 tests; the load test runs against the sample dataset).

- [ ] **Step 5: Commit**

```bash
git add src/parking_proj/realdata.py tests/test_realdata.py
git commit -m "feat: real-data adapter (boot->ENU theta, GCJ->WGS->ENU, Route+pose stream)"
```

---

### Task 5: `generate_real.py` — run algorithm + write `out/real/`

**Files:**
- Create: `src/parking_proj/generate_real.py`
- Modify: `run.sh`
- Test: `tests/test_generate_real.py`

**Interfaces:**
- Consumes: `realdata.load_dataset`/`is_dataset_dir` (Task 4); `projection.Projector` (existing); `osm.fetch_basemap` (Task 3); `geo.enu_about` (Task 1).
- Produces:
  - `build_real_case_dict(dataset_dir: str, basemap: dict | None = None) -> dict`
  - `arrow_indices(points_llh: np.ndarray, lat0: float, lon0: float, step_m: float = 20.0) -> list[int]`
  - `main(dataset_root: str = "dataset", out_dir: str = "out/real") -> None`

- [ ] **Step 1: Write the failing test** — `tests/test_generate_real.py`

```python
import math
import numpy as np
from pathlib import Path
from parking_proj.generate_real import build_real_case_dict, arrow_indices

DATASET = ("dataset/dev_CHERY_M32T_46651_ALL_MANUAL_2026-06-22-14-08-25_"
           "20260625_101425_annotation")


def test_arrow_indices_spacing():
    # a straight 100 m E-W line at lat0
    lat0, lon0 = 31.834, 117.14
    import parking_proj.geo as geo
    lls = []
    for e in np.linspace(0, 100, 500):
        # invert enu_about: lon = lon0 + e/(DEG*R*cos), lat=lat0
        lls.append([lat0, lon0 + e / (geo._DEG * geo._EARTH_R * math.cos(lat0 * geo._DEG))])
    idx = arrow_indices(np.array(lls), lat0, lon0, step_m=20.0)
    assert 4 <= len(idx) <= 6           # ~5 arrows over 100 m at 20 m
    assert idx[0] == 0


def test_real_case_dict_schema_and_monotonic():
    if not Path(DATASET).exists():
        import pytest
        pytest.skip("sample dataset not present")
    case = build_real_case_dict(DATASET, basemap=None)
    assert case["mode"] == "real"
    assert "route" in case and "route_llh" in case and "ego_track_llh" in case
    assert case["basemap"] is None
    assert "theta_deg" in case and "origin" in case
    f0 = case["frames"][0]
    for k in ("t", "speed", "meas_pose", "meas_ll", "cursor_s", "matched_seg",
              "est_lat_dev", "end_flag"):
        assert k in f0
    assert "true_lat_dev" not in f0 and "gt_seg" not in f0
    cs = [f["cursor_s"] for f in case["frames"]]
    assert all(cs[i + 1] >= cs[i] - 1e-6 for i in range(len(cs) - 1))  # monotonic
    assert all(math.isfinite(f["est_lat_dev"]) for f in case["frames"])
    assert case["route_arrow_idx"] and case["ego_arrow_idx"]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./.venv/bin/python -m pytest tests/test_generate_real.py -v`
Expected: FAIL (`ModuleNotFoundError`).

- [ ] **Step 3: Implement `src/parking_proj/generate_real.py`**

```python
"""Prebake real datasets into out/real/<id>.json (+ OSM tiles) for the viewer."""
import json
import math
import os
import numpy as np
from . import geo, osm
from .realdata import load_dataset, is_dataset_dir
from .projection import Projector

MARGIN_M = 40.0


def _r(x, nd=3):
    return round(float(x), nd)


def _rl(a, nd=3):
    return [round(float(x), nd) for x in a]


def arrow_indices(points_llh, lat0, lon0, step_m=20.0):
    enu = np.array([geo.enu_about(la, lo, lat0, lon0) for la, lo in points_llh])
    seg = np.sqrt((np.diff(enu, axis=0) ** 2).sum(1))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    idx, target = [], 0.0
    for i in range(len(s)):
        if s[i] >= target:
            idx.append(i)
            target += step_m
    return idx


def build_real_case_dict(dataset_dir, basemap=None):
    ds = load_dataset(dataset_dir)
    route = ds.route
    proj = Projector(route)
    frames = []
    for i in range(len(ds.meas_e)):
        r = proj.step(float(ds.meas_e[i]), float(ds.meas_n[i]), float(ds.meas_yaw[i]))
        frames.append({
            "t": _r(ds.t_us[i] / 1e6),
            "speed": _r(ds.speed[i]),
            "meas_pose": {"e": _r(ds.meas_e[i]), "n": _r(ds.meas_n[i]),
                          "h": _r(ds.meas_yaw[i], 5)},
            "meas_ll": {"lat": _r(ds.ego_llh[i, 0], 7), "lon": _r(ds.ego_llh[i, 1], 7)},
            "cursor_s": _r(r.cursor_s),
            "matched_seg": r.matched_seg,
            "est_lat_dev": _r(r.est_lat_dev, 4),
            "end_flag": bool(r.end_flag),
        })
    return {
        "case_id": ds.dataset_id,
        "name": ds.dataset_id,
        "mode": "real",
        "theta_deg": _r(math.degrees(ds.theta_rad), 3),
        "origin": {"lat0": ds.lat0, "lon0": ds.lon0},
        "basemap": basemap,
        "config": {"ahead": proj.ahead, "behind": proj.behind},
        "route": {
            "points_e": _rl(route.points[:, 0]), "points_n": _rl(route.points[:, 1]),
            "s": _rl(route.s), "waypoint_indices": route.waypoint_indices,
            "waypoint_labels": route.waypoint_labels,
        },
        "route_llh": [[_r(la, 7), _r(lo, 7)] for la, lo in ds.route_llh],
        "ego_track_llh": [[_r(la, 7), _r(lo, 7)] for la, lo in ds.ego_llh],
        "route_arrow_idx": arrow_indices(ds.route_llh, ds.lat0, ds.lon0),
        "ego_arrow_idx": arrow_indices(ds.ego_llh, ds.lat0, ds.lon0),
    }


def _bbox(ds_case):
    lls = np.array(ds_case["route_llh"] + ds_case["ego_track_llh"], float)
    lat0 = lls[:, 0].mean()
    dm = MARGIN_M / (geo._DEG * geo._EARTH_R)
    dmlon = MARGIN_M / (geo._DEG * geo._EARTH_R * math.cos(lat0 * geo._DEG))
    return (lls[:, 1].min() - dmlon, lls[:, 0].min() - dm,
            lls[:, 1].max() + dmlon, lls[:, 0].max() + dm)


def main(dataset_root="dataset", out_dir="out/real"):
    os.makedirs(out_dir, exist_ok=True)
    index = []
    if os.path.isdir(dataset_root):
        subdirs = sorted(d for d in os.listdir(dataset_root)
                         if is_dataset_dir(os.path.join(dataset_root, d)))
    else:
        subdirs = []
    for name in subdirs:
        path = os.path.join(dataset_root, name)
        case = build_real_case_dict(path, basemap=None)
        case_out = os.path.join(out_dir, name)
        os.makedirs(case_out, exist_ok=True)
        man = osm.fetch_basemap(*_bbox(case), out_dir=case_out)
        if man is not None:
            man = {**man, "tiles": [{**t, "file": f"{name}/{t['file']}"} for t in man["tiles"]]}
        case["basemap"] = man
        with open(os.path.join(out_dir, f"{name}.json"), "w") as fh:
            json.dump(case, fh)
        index.append({"case_id": name, "name": name, "mode": "real",
                      "has_map": man is not None})
        print(f"  {name}: {len(case['frames'])} frames, map={'yes' if man else 'no'}")
    with open(os.path.join(out_dir, "index.json"), "w") as fh:
        json.dump({"cases": index}, fh)
    print(f"Wrote {len(index)} real cases to {out_dir}/")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `./.venv/bin/python -m pytest tests/test_generate_real.py -v`
Expected: PASS (2 tests). (`build_real_case_dict` is called with `basemap=None`, so no network.)

- [ ] **Step 5: Add a `gen-real` verb to `run.sh`**

In `run.sh`, add a `gen()`-style helper and a case branch. After the existing `gen()` function, add:

```bash
gen_real() {
    ensure_venv
    echo ">> generating real-data cases -> out/real/ ..."
    PYTHONPATH=src "$PY" -m parking_proj.generate_real
}
```

And in the `case "${1:-serve}" in` block, add a branch next to `gen`:

```bash
    gen-real)      gen_real ;;
```

Also update the two usage strings to include `gen-real`:
`usage: ./run.sh [serve|gen|gen-real|test|e2e|setup]`.

- [ ] **Step 6: Run it to confirm it produces output**

Run: `PYTHONPATH=src ./.venv/bin/python -m parking_proj.generate_real`
Expected: prints `... frames, map=yes|no` per dataset and `Wrote N real cases to out/real/`. (Map may be `no` if tiles can't be fetched — that's the accepted fallback.)

- [ ] **Step 7: Commit**

```bash
git add src/parking_proj/generate_real.py tests/test_generate_real.py run.sh
git commit -m "feat: prebake real datasets to out/real/ (+ OSM tiles) and gen-real verb"
```

---

### Task 6: Viewer — Simulation/Real-data tabs + index switching

**Files:**
- Modify: `viewer/index.html`
- Modify: `viewer/viewer.css`
- Modify: `viewer/viewer.js`

**Interfaces:**
- Consumes: existing `STATE`, `loadIndex`, `loadCase`, `selectCase`, `renderFrame`, `updateTelemetry`.
- Produces: `STATE.mode` (`"real"` | `"sim"`); `selectTab(mode)`; `loadIndex(mode)` fetching `../out/real/index.json` (real) or `../out/index.json` (sim); telemetry that tolerates a missing `verdict`.

- [ ] **Step 1: Add the tab bar to `viewer/index.html`**

Just inside `<aside id="left">`, before `<h2>Test cases</h2>`, add:

```html
    <div id="tabs">
      <button id="tab-real" class="tab active">Real data</button>
      <button id="tab-sim" class="tab">Simulation</button>
    </div>
```

- [ ] **Step 2: Add tab CSS to `viewer/viewer.css`**

```css
#tabs { display: flex; gap: 4px; margin-bottom: 8px; }
#tabs .tab { flex: 1; padding: 6px 8px; border: 1px solid #ccd; background: #eef0f3;
  cursor: pointer; border-radius: 4px 4px 0 0; font-weight: 600; }
#tabs .tab.active { background: #d3e0f5; border-bottom-color: #d3e0f5; }
```

- [ ] **Step 3: Write a manual DOM check (no pytest)**

There is no unit test for this task; verification is `node --check` (Step 6) plus the e2e suite in Task 9. Record in the report that these are the checks used.

- [ ] **Step 4: Refactor `loadIndex` and add tab logic in `viewer/viewer.js`**

Change `STATE` init (line 1) to include `mode`:

```javascript
const STATE = { case: null, frame: 0, playing: false, speed: 1, mode: "real" };
```

Replace the body of `loadIndex()` so it takes the mode and fetches the right index:

```javascript
async function loadIndex() {
  const url = STATE.mode === "real" ? "../out/real/index.json" : "../out/index.json";
  const ul = document.getElementById("case-list");
  ul.innerHTML = "";
  let data;
  try { data = await (await fetch(url)).json(); }
  catch (e) { ul.innerHTML = "<li>(no cases — run ./run.sh " +
    (STATE.mode === "real" ? "gen-real" : "gen") + ")</li>"; return; }
  let lastGroup = null;
  for (const c of data.cases) {
    if (STATE.mode === "sim" && c.group !== lastGroup) {
      const h = document.createElement("li");
      h.className = "group-header";
      h.textContent = GROUP_NAMES[c.group] || c.group;
      ul.appendChild(h); lastGroup = c.group;
    }
    const li = document.createElement("li");
    const badge = c.verdict
      ? `<span class="badge ${c.verdict.passed ? "pass" : "fail"}">${c.verdict.passed ? "PASS" : "FAIL"}</span>`
      : "";
    li.innerHTML = `${c.name}${badge}`;
    li.onclick = () => selectCase(c.case_id, li);
    ul.appendChild(li);
  }
}
```

Add a `selectTab` and wire the buttons (put near the other `DOMContentLoaded` handler):

```javascript
function selectTab(mode) {
  STATE.mode = mode;
  document.getElementById("tab-real").classList.toggle("active", mode === "real");
  document.getElementById("tab-sim").classList.toggle("active", mode === "sim");
  STATE.case = null; STATE.playing = false;
  loadIndex();
}
window.addEventListener("DOMContentLoaded", () => {
  document.getElementById("tab-real").onclick = () => selectTab("real");
  document.getElementById("tab-sim").onclick = () => selectTab("sim");
});
```

The existing `DOMContentLoaded` that calls `loadIndex()` stays; it now loads the default (`real`).

- [ ] **Step 5: Guard telemetry against a missing verdict**

In `updateTelemetry()`, replace the verdict line (currently `const v = c.verdict; set("tm-verdict", ...)`) with:

```javascript
  const v = c.verdict;
  set("tm-verdict", v ? `${v.passed ? "PASS" : "FAIL"} (mis ${v.mismatches})` : "— (real data)");
```

(`true_lat_dev` is already null-guarded from earlier work.)

- [ ] **Step 6: Verify JS parses and the fallback index message works**

Run: `node --check viewer/viewer.js` → must print nothing / exit 0.
Then: `PYTHONPATH=src ./.venv/bin/python -m parking_proj.generate && PYTHONPATH=src ./.venv/bin/python -m parking_proj.generate_real` (so both indexes exist).

- [ ] **Step 7: Commit**

```bash
git add viewer/index.html viewer/viewer.css viewer/viewer.js
git commit -m "feat: viewer Simulation/Real-data tabs + index switching"
```

---

### Task 7: Viewer — real-data BEV renderer (basemap + overlays + arrows + anti-flicker)

**Files:**
- Modify: `viewer/viewer.js`

**Interfaces:**
- Consumes: `STATE`, `renderFrame`, the real case JSON (`route_llh`, `ego_track_llh`, `route_arrow_idx`, `ego_arrow_idx`, `basemap`, per-frame `meas_ll`), `osm`-style mercator math (reimplemented in JS).
- Produces: `drawBevReal()`, `mercatorPx(lon, lat, z)`, `STATE.basemapImgs`; `renderFrame` routes to `drawBevReal` when `STATE.case.mode === "real"`.

- [ ] **Step 1: Add mercator helper + real static-layer builder to `viewer/viewer.js`**

```javascript
let BEVREAL_STATIC = null, BEVREAL_T = null;

function mercatorGlobalPx(lon, lat, z) {         // matches osm.lonlat_to_global_px
  const n = 2 ** z, T = 256;
  const x = (lon + 180) / 360 * n * T;
  const y = (1 - Math.asinh(Math.tan(lat * Math.PI / 180)) / Math.PI) / 2 * n * T;
  return { x, y };
}

// Build the fixed lon/lat -> canvas transform for a real case, fitting the
// route+track bounds (and basemap if present) into the BEV canvas.
function buildRealTransform(c, cv) {
  const z = c.basemap ? c.basemap.z : 18;
  const pts = c.route_llh.concat(c.ego_track_llh);
  let minx = 1e18, miny = 1e18, maxx = -1e18, maxy = -1e18;
  const bounds = c.basemap
    ? [[c.basemap.y0 * 256, c.basemap.x0 * 256],
       [(c.basemap.y0 + c.basemap.ny) * 256, (c.basemap.x0 + c.basemap.nx) * 256]]
    : null;
  if (bounds) { miny = bounds[0][0]; minx = bounds[0][1]; maxy = bounds[1][0]; maxx = bounds[1][1]; }
  else for (const [la, lo] of pts) {
    const p = mercatorGlobalPx(lo, la, z);
    minx = Math.min(minx, p.x); maxx = Math.max(maxx, p.x);
    miny = Math.min(miny, p.y); maxy = Math.max(maxy, p.y);
  }
  const pad = 8;
  const s = Math.min((cv.width - 2 * pad) / (maxx - minx), (cv.height - 2 * pad) / (maxy - miny));
  const ox = pad + (cv.width - 2 * pad - (maxx - minx) * s) / 2;
  const oy = pad + (cv.height - 2 * pad - (maxy - miny) * s) / 2;
  return { z, toX: gx => ox + (gx - minx) * s, toY: gy => oy + (gy - miny) * s };
}

function llToCanvas(T, lon, lat) {
  const p = mercatorGlobalPx(lon, lat, T.z);
  return { x: T.toX(p.x), y: T.toY(p.y) };
}

function drawTrackReal(ctx, T, llh, color, width) {
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
  for (let i = 0; i < llh.length; i++) {
    const p = llToCanvas(T, llh[i][1], llh[i][0]);
    if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
  }
  ctx.stroke();
}

function drawArrowsReal(ctx, T, llh, idxs, color) {
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
  for (const i of idxs) {
    if (i + 1 >= llh.length) continue;
    const a = llToCanvas(T, llh[i][1], llh[i][0]);
    const b = llToCanvas(T, llh[i + 1][1], llh[i + 1][0]);
    const ang = Math.atan2(b.y - a.y, b.x - a.x);
    const L = 7;
    ctx.beginPath();
    ctx.moveTo(b.x, b.y);
    ctx.lineTo(b.x - L * Math.cos(ang - 0.4), b.y - L * Math.sin(ang - 0.4));
    ctx.moveTo(b.x, b.y);
    ctx.lineTo(b.x - L * Math.cos(ang + 0.4), b.y - L * Math.sin(ang + 0.4));
    ctx.stroke();
  }
}
```

- [ ] **Step 2: Add the static-layer compositor (basemap tiles or gray) + `drawBevReal`**

```javascript
function buildBevRealStatic(cv) {
  const c = STATE.case;
  BEVREAL_T = buildRealTransform(c, cv);
  BEVREAL_STATIC = document.createElement("canvas");
  BEVREAL_STATIC.width = cv.width; BEVREAL_STATIC.height = cv.height;
  const ctx = BEVREAL_STATIC.getContext("2d");
  const T = BEVREAL_T;
  if (c.basemap && STATE.basemapImgs) {
    for (const t of c.basemap.tiles) {
      const img = STATE.basemapImgs[t.file];
      if (!img) continue;
      const gx = t.x * 256, gy = t.y * 256;   // tile's top-left global px
      const x0 = T.toX(gx), y0 = T.toY(gy);
      const x1 = T.toX(gx + 256), y1 = T.toY(gy + 256);
      ctx.drawImage(img, x0, y0, x1 - x0, y1 - y0);
    }
  } else {
    ctx.fillStyle = "#eef0f2"; ctx.fillRect(0, 0, cv.width, cv.height);
    ctx.strokeStyle = "#dfe3e8"; ctx.lineWidth = 1;
    for (let x = 0; x < cv.width; x += 40) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, cv.height); ctx.stroke(); }
    for (let y = 0; y < cv.height; y += 40) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(cv.width, y); ctx.stroke(); }
  }
  drawTrackReal(ctx, T, c.route_llh, "#4477cc", 3);       // planned route
  drawTrackReal(ctx, T, c.ego_track_llh, "#222", 2);      // ego driven track
  drawArrowsReal(ctx, T, c.route_llh, c.route_arrow_idx, "#2b5fb0");
  drawArrowsReal(ctx, T, c.ego_track_llh, c.ego_arrow_idx, "#cc3a3a");
}

function drawBevReal() {
  const c = STATE.case, cv = document.getElementById("bev");
  const ctx = cv.getContext("2d");
  if (!BEVREAL_STATIC) buildBevRealStatic(cv);
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.drawImage(BEVREAL_STATIC, 0, 0);
  const f = c.frames[STATE.frame];
  const p = llToCanvas(BEVREAL_T, f.meas_ll.lon, f.meas_ll.lat);
  ctx.fillStyle = "#cc3a3a";
  ctx.beginPath(); ctx.arc(p.x, p.y, 5, 0, 2 * Math.PI); ctx.fill();   // car marker
}
```

- [ ] **Step 3: Route `renderFrame` to the real BEV and preload tiles in `loadCase`**

In `renderFrame()`, replace the `drawBev();` call with:

```javascript
  if (STATE.case.mode === "real") drawBevReal(); else drawBev();
```

In `selectCase`, after `BEV_STATIC = null;` add `BEVREAL_STATIC = null;`.

Extend `loadCase(caseId)` so that for real cases it preloads the tile images before returning (so the static layer builds with images ready — avoids a blank first frame):

```javascript
async function loadCase(caseId) {
  const dir = STATE.mode === "real" ? "../out/real/" : "../out/";
  STATE.case = await (await fetch(`${dir}${caseId}.json`)).json();
  STATE.frame = 0;
  STATE.basemapImgs = null;
  if (STATE.case.mode === "real" && STATE.case.basemap) {
    STATE.basemapImgs = {};
    await Promise.all(STATE.case.basemap.tiles.map(t => new Promise(res => {
      const img = new Image();
      img.onload = () => { STATE.basemapImgs[t.file] = img; res(); };
      img.onerror = () => res();
      img.src = `../out/real/${t.file}`;
    })));
  }
  const sc = document.getElementById("scrubber");
  sc.max = STATE.case.frames.length - 1; sc.value = 0;
}
```

(Note: the sim path uses `dir = "../out/"` and no basemap; behavior for sim cases is unchanged.)

- [ ] **Step 4: Verify JS parses; regenerate and eyeball**

Run: `node --check viewer/viewer.js` (exit 0).
Run: `PYTHONPATH=src ./.venv/bin/python -m parking_proj.generate_real`
Then serve from repo root (`python -m http.server 8000`) and open `/viewer/index.html`: Real-data tab is default, the dataset appears, clicking it shows the BEV with (gray-or-map) background + blue planned route + dark ego track + arrows + red car; driver-view + telemetry populate; playback is smooth.

- [ ] **Step 5: Commit**

```bash
git add viewer/viewer.js
git commit -m "feat: real-data BEV renderer (OSM basemap + route/track + arrows, anti-flicker)"
```

---

### Task 8: e2e test for the real-data tab

**Files:**
- Modify: `tests/e2e/test_viewer_e2e.py`

**Interfaces:**
- Consumes: the existing e2e fixtures (`base_url`, `viewer`), the real index/case JSON.

- [ ] **Step 1: Ensure real data is generated in the e2e server fixture**

In `tests/e2e/test_viewer_e2e.py`, in the `base_url` fixture, after the block that generates `out/index.json`, add generation of the real cases so the server serves them:

```python
    real_index = OUT / "real" / "index.json"
    if not real_index.exists():
        env = dict(os.environ, PYTHONPATH=str(SRC))
        subprocess.run([sys.executable, "-m", "parking_proj.generate_real"],
                       cwd=REPO, env=env, check=True)
```

(`OUT` is `REPO / "out"`, already defined.)

- [ ] **Step 2: Add the real-data e2e tests**

Append to `tests/e2e/test_viewer_e2e.py`:

```python
def test_real_tab_is_default_and_lists_datasets(viewer):
    page, _ = viewer
    # default tab is Real data
    assert "active" in (page.get_attribute("#tab-real", "class") or "")
    rows = page.locator("#case-list li:not(.group-header)")
    assert rows.count() >= 1                      # at least the sample dataset


def test_real_case_renders_bev_and_drives(viewer):
    page, errors = viewer
    page.locator("#case-list li:not(.group-header)").first.click()
    page.wait_for_function(_NONBLANK, arg="#bev", timeout=8000)
    page.wait_for_function(_NONBLANK, arg="#driver", timeout=8000)
    bev = page.evaluate(_GEOM, "#bev")
    assert bev["drawn"] > 1500, bev                # basemap-or-gray + route + track fill area
    # telemetry shows real-data verdict placeholder, speed present
    assert "real" in page.inner_text("#tm-verdict").lower() or "—" in page.inner_text("#tm-verdict")
    assert "km/h" in page.inner_text("#tm-speed")
    # step advances the frame
    before = page.inner_text("#tm-frame")
    page.click("#btn-step-fwd")
    assert page.inner_text("#tm-frame") != before


def test_tab_switch_to_simulation_works(viewer):
    page, _ = viewer
    page.click("#tab-sim")
    page.wait_for_selector("#case-list li.group-header")   # sim list has group headers
    assert page.locator("#case-list .badge.pass").count() >= 1
    page.click("#tab-real")                                 # back to real
```

- [ ] **Step 3: Run the e2e suite**

Run: `./.venv/bin/python -m pytest -m e2e -q`
Expected: all e2e tests pass (existing sim tests + the 3 new real-data tests). If a real case has no basemap (offline), the BEV is the gray fallback and still passes the coverage assertion (route + track + arrows fill > 1500 px).

- [ ] **Step 4: Commit**

```bash
git add tests/e2e/test_viewer_e2e.py
git commit -m "test: e2e for real-data tab (default, lists datasets, BEV renders, tab switch)"
```

---

### Task 9: Docs + full verification

**Files:**
- Modify: `README.md`
- Modify: `spec_design.md`
- Modify: `CLAUDE.md`

**Interfaces:** none (docs + final run).

- [ ] **Step 1: Update `README.md`**

Add a `gen-real` line to the Quick start block and a short "Real data" subsection:

```markdown
    ./run.sh gen-real   # prebake dataset/ real cases (+ OSM tiles) -> out/real/
```

```markdown
## Real data

The viewer's **Real data** tab (default) plays back real vehicle datasets from
`dataset/<pkg>/` (each with `ego_route_llh.json` + `route_generation_result/planned_route.json`).
`./run.sh gen-real` runs the same projection algorithm on them (ego `llh`
GCJ-02→WGS-84→ENU, heading `yaw_boot`+boot→ENU offset, planned route as the
Route) and prebakes `out/real/<id>.json` plus an OSM basemap (falls back to a
gray grid if tiles can't be fetched). The BEV shows the region map with the
planned route + ego track + direction arrows, all in WGS-84. Real cases have no
PASS/FAIL or true-lat-dev (no ground truth); `est_lat_dev` is still shown.
```

- [ ] **Step 2: Fold the real-data feature into `spec_design.md`**

Replace the `## Extensions` pointer block (added during brainstorming) with a real "shipped" note pointing to both the design and this plan, and add one line to the viewer section (§3.7) noting the tabs + real-data BEV. Concretely, change the Extensions block to:

```markdown
## Extensions (shipped)

- **Real-data ingestion + OSM basemap:** Simulation / Real-data tabs; the same
  algorithm runs on `dataset/` packages (ego GCJ-02→WGS-84, `yaw_boot`+boot→ENU
  offset, planned route as the Route); the real-data BEV shows an OSM basemap
  with route + ego track + direction arrows in WGS-84. Design:
  `docs/superpowers/specs/2026-07-03-real-data-osm-design.md`; plan:
  `docs/superpowers/plans/2026-07-03-real-data-osm.md`.
```

- [ ] **Step 3: Update `CLAUDE.md`**

Add to the Commands section:

```markdown
./run.sh gen-real    # prebake real datasets from dataset/ -> out/real/ (+ OSM tiles)
```

And add one bullet under the architecture/invariants describing real data: real
datasets are prebaked through the same Python algorithm into `out/real/<id>.json`
with `mode:"real"` (no ground truth → no verdict/true_lat_dev); the viewer's
real-data BEV renders an OSM basemap in Web-Mercator with route+track+arrows,
everything converted to WGS-84 (ego `llh` is GCJ-02).

- [ ] **Step 4: Run the FULL suite + generation end-to-end**

Run: `./.venv/bin/python -m pytest -q` → all unit + acceptance pass.
Run: `./.venv/bin/python -m pytest -m e2e -q` → all e2e pass.
Run: `PYTHONPATH=src ./.venv/bin/python -m parking_proj.generate && PYTHONPATH=src ./.venv/bin/python -m parking_proj.generate_real` → both succeed.

- [ ] **Step 5: Commit**

```bash
git add README.md spec_design.md CLAUDE.md
git commit -m "docs: document real-data tab + gen-real; sync spec_design and CLAUDE"
```

---

## Self-Review

**1. Spec coverage:**
- Tabs default Real data → Task 6. ✓
- List first-level qualifying dataset dirs → `is_dataset_dir` (Task 4) + `main` (Task 5) + tab list (Task 6). ✓
- Click → algorithm result via prebake → Tasks 4/5 (adapter+generate), Task 6/7 (load/render). ✓
- Field mapping (planned_route→Route; ego llh GCJ→WGS→ENU; yaw_boot+θ; v; timestamp) → Task 4. ✓
- θ estimation (positions-only least squares, scale check) → Task 4 (+ Global Constraints). ✓
- No ground truth (no verdict/true_lat_dev/gt) → Task 5 schema + Task 6 telemetry guard. ✓
- OSM basemap, WGS-84, minimal tiles, prep-time only, gray fallback → Tasks 2/3/5/7. ✓
- Direction arrows every ~20 m on both paths → `arrow_indices` (Task 5) + `drawArrowsReal` (Task 7). ✓
- Anti-flicker (static layer once, fixed transform, only car redraw) → Task 7. ✓
- Driver-view/perspective + telemetry unchanged, just fed real → reuse (Task 6/7); telemetry verdict guard (Task 6). ✓
- Testing (geo, osm math/fetch, adapter, generate, e2e) → Tasks 1–5, 8. ✓

**2. Placeholder scan:** No TBD/TODO/"handle edge cases" in steps; every code step has complete code. The spec's two TBDs (tile source, arrow colors) are resolved concretely here (OSM tile server; blue route `#4477cc`/`#2b5fb0`, dark/red ego `#222`/`#cc3a3a`; 20 m).

**3. Type consistency:** `estimate_boot_to_enu_theta(pos_boot, pos_enu) -> (theta, scale)` defined Task 4, used Task 4. `RealDataset` fields produced Task 4, consumed Task 5. `build_real_case_dict -> dict` (Task 5) keys (`mode, route, route_llh, ego_track_llh, route_arrow_idx, ego_arrow_idx, basemap, frames[].meas_ll/meas_pose/cursor_s`) match what the viewer reads in Tasks 6/7. `fetch_basemap(...) -> manifest|None` (Task 3) manifest keys (`z,x0,y0,nx,ny,tile,tiles[]`) match `buildRealTransform`/`buildBevRealStatic` (Task 7). `mercatorGlobalPx` (JS, Task 7) mirrors `lonlat_to_global_px` (Python, Task 2) — same formula (`asinh(tan)`). `Projector(route).step(e,n,yaw) -> ProjectionResult(cursor_s, matched_seg, est_lat_dev, end_flag)` used unchanged (Task 5).

**Note for implementer:** `geo._DEG` / `geo._EARTH_R` are module-private but reused by `realdata`/`generate_real` for ENU/arc-length; they exist in `geo.py`. If a reviewer objects to the leading underscore, promote them to public constants in `geo.py` as part of Task 1 and update references — behavior-neutral.
