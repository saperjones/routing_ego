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
