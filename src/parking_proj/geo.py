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
