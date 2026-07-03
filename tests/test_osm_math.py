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
