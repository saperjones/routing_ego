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
