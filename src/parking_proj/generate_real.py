"""Prebake real datasets into out/real/<id>.json (+ OSM tiles) for the viewer."""
import json
import math
import os
import numpy as np
from . import geo, osm
from .realdata import load_dataset, is_dataset_dir
from .projection import Projector, follow_path, FOLLOW_AHEAD, FOLLOW_DS

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
        fp, lat_shift = follow_path(route, float(ds.meas_e[i]), float(ds.meas_n[i]),
                                    float(ds.meas_yaw[i]), r.cursor_s)
        frames.append({
            "t": _r(ds.t_us[i] / 1e6),
            "speed": _r(ds.speed[i]),
            "meas_pose": {"e": _r(ds.meas_e[i]), "n": _r(ds.meas_n[i]),
                          "h": _r(ds.meas_yaw[i], 5)},
            "meas_ll": {"lat": _r(ds.ego_llh[i, 0], 7), "lon": _r(ds.ego_llh[i, 1], 7)},
            "cursor_s": _r(r.cursor_s),
            "matched_seg": r.matched_seg,
            "est_lat_dev": _r(r.est_lat_dev, 4),
            "follow_path": [[_r(x), _r(y)] for x, y in fp],
            "lat_shift": _r(lat_shift, 4),
            "end_flag": bool(r.end_flag),
        })
    return {
        "case_id": ds.dataset_id,
        "name": ds.dataset_id,
        "mode": "real",
        "theta_deg": _r(math.degrees(ds.theta_rad), 3),
        "origin": {"lat0": ds.lat0, "lon0": ds.lon0},
        "basemap": basemap,
        "config": {"ahead": proj.ahead, "behind": proj.behind,
                   "follow_ahead": FOLLOW_AHEAD, "follow_ds": FOLLOW_DS},
        "route": {
            "points_e": _rl(route.points[:, 0]), "points_n": _rl(route.points[:, 1]),
            "s": _rl(route.s), "waypoint_indices": route.waypoint_indices,
            "waypoint_labels": route.waypoint_labels,
        },
        "route_llh": [[_r(la, 7), _r(lo, 7)] for la, lo in ds.route_llh],
        "ego_track_llh": [[_r(la, 7), _r(lo, 7)] for la, lo in ds.ego_llh],
        "route_arrow_idx": arrow_indices(ds.route_llh, ds.lat0, ds.lon0),
        "ego_arrow_idx": arrow_indices(ds.ego_llh, ds.lat0, ds.lon0),
        "frames": frames,
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
