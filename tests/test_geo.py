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
