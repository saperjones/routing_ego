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
