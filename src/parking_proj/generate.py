"""Run all scenarios, grade them, and write prebaked JSON for the viewer."""
import json
import os
from .simulate import simulate, SimConfig
from .projection import Projector
from .scenarios import build_scenarios
from . import grade as grading


def _round3(a):
    return [round(float(x), 3) for x in a]


def build_case_dict(scenario) -> dict:
    route = scenario.route
    frames = simulate(route, SimConfig(tier=scenario.tier, seed=scenario.seed))
    proj = Projector(route)
    results = []
    for f in frames:
        try:
            results.append(proj.step(f.meas_e, f.meas_n, f.meas_yaw))
        except Exception:
            results.append(None)

    verdict = grading.grade_case(route, frames, results)

    frame_dicts = []
    for f, r in zip(frames, results):
        frame_dicts.append({
            "t": round(f.t, 3),
            "speed": round(f.speed, 3),
            "true_pose": {"e": round(f.true_e, 3), "n": round(f.true_n, 3),
                          "h": round(f.true_yaw, 5)},
            "meas_pose": {"e": round(f.meas_e, 3), "n": round(f.meas_n, 3),
                          "h": round(f.meas_yaw, 5),
                          "pitch": round(f.pitch, 6), "roll": round(f.roll, 6)},
            "cursor_s": None if r is None else round(r.cursor_s, 3),
            "matched_seg": None if r is None else r.matched_seg,
            "est_lat_dev": None if r is None else round(r.est_lat_dev, 4),
            "true_lat_dev": round(grading.true_lat_dev(route, f), 4),
            "end_flag": None if r is None else bool(r.end_flag),
            "gt_seg": f.gt_seg,
            "gt_s": round(f.gt_s, 3),
        })

    return {
        "case_id": scenario.case_id,
        "name": scenario.name,
        "tier": scenario.tier,
        "route": {
            "points_e": _round3(route.points[:, 0]),
            "points_n": _round3(route.points[:, 1]),
            "s": _round3(route.s),
            "waypoint_indices": route.waypoint_indices,
            "waypoint_labels": route.waypoint_labels,
        },
        "config": {"ahead": proj.ahead, "behind": proj.behind},
        "frames": frame_dicts,
        "verdict": verdict,
    }


def main(out_dir="out"):
    os.makedirs(out_dir, exist_ok=True)
    index = []
    for scenario in build_scenarios():
        case = build_case_dict(scenario)
        with open(os.path.join(out_dir, f"{scenario.case_id}.json"), "w") as fh:
            json.dump(case, fh)
        index.append({
            "case_id": scenario.case_id,
            "name": scenario.name,
            "tier": scenario.tier,
            "group": scenario.case_id.split("_")[0],
            "verdict": case["verdict"],
        })
    with open(os.path.join(out_dir, "index.json"), "w") as fh:
        json.dump({"cases": index}, fh)
    print(f"Wrote {len(index)} cases to {out_dir}/")


if __name__ == "__main__":
    main()
