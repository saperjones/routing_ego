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
