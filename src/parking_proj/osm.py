"""Web-Mercator slippy-tile math (EPSG:3857, 256-px tiles)."""
import math
import os
import urllib.request

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


# OpenStreetMap raster tiles. The main tile.openstreetmap.org server blocks
# scripted/bulk access per its usage policy (it serves an identical placeholder
# tile), so the default is the OSM community server that permits programmatic
# access. Override with PARKING_TILE_URL for a keyed provider or satellite imagery.
_DEFAULT_TILE_URL = "https://tile.openstreetmap.de/{z}/{x}/{y}.png"
_DEFAULT_UA = "parking-route-projection/0.1 (offline research viewer)"


def build_tile_url(template, z, x, y, key=""):
    """Fill a slippy-tile URL template. Supports {z}, {x}, {y}, and optional {key}."""
    return template.format(z=z, x=x, y=y, key=key)


def _http_tile(z, x, y):
    # Tile source is configurable via environment variables so a compliant
    # provider / API key can be used (default: OpenStreetMap):
    #   PARKING_TILE_URL  URL template with {z}/{x}/{y} (and optional {key})
    #   PARKING_TILE_KEY  API key substituted into {key}
    #   PARKING_TILE_UA   User-Agent header
    template = os.environ.get("PARKING_TILE_URL", _DEFAULT_TILE_URL)
    ua = os.environ.get("PARKING_TILE_UA", _DEFAULT_UA)
    key = os.environ.get("PARKING_TILE_KEY", "")
    try:
        url = build_tile_url(template, z, x, y, key)
        req = urllib.request.Request(url, headers={"User-Agent": ua})
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
    fetched = []
    for x in range(x0, x0 + nx):
        for y in range(y0, y0 + ny):
            data = getter(z, x, y)
            if not data:
                return None
            fetched.append((x, y, data))
    # A server that refuses scripted access (e.g. OSM tile-usage policy) returns
    # an identical "blocked" tile for every request; if all tiles are byte-identical,
    # treat it as no basemap so the viewer uses its gray fallback.
    if len(fetched) >= 2 and len({d for _, _, d in fetched}) == 1:
        return None
    tiles_dir = os.path.join(out_dir, "tiles")
    os.makedirs(tiles_dir, exist_ok=True)
    tiles = []
    for x, y, data in fetched:
        fname = f"tiles/{z}_{x}_{y}.png"
        with open(os.path.join(out_dir, fname), "wb") as fh:
            fh.write(data)
        tiles.append({"x": x, "y": y, "file": fname})
    return {"z": z, "x0": x0, "y0": y0, "nx": nx, "ny": ny, "tile": TILE, "tiles": tiles}
