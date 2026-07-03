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
