"""Offline post-processing interface: turn a pre-processed bag's ego pose stream
+ generated global route into a per-frame ego-frame (body-frame) routing path,
by driving the authoritative Python ``project_route``.

Two entry points sharing one code path:
  * ``run(ego_path, route_path, config) -> dict``  — in-memory core.
  * ``main(argv=None) -> int``                     — argparse CLI; all parameters
    are passed on the command line (no config file).

Run as:
  python -m parking_proj.offline_processing_routing_projection \\
    --ego-json <ego_route_llh.json> --route-json <planned_route.json> \\
    --out <out.json> [--strategy human_centered --ahead-m 40 ...]
"""
import argparse
import dataclasses
import json
import sys

from .project_route import ProjectConfig, ProjectState, project_route
from .realdata import load_dataset_files


def _round(x, nd=4):
    return round(float(x), nd)


def run(ego_path, route_path, config):
    """Load the two input files and drive ``project_route`` over every ego frame.

    Returns the output dict (see the module docstring / spec). Raises on load
    errors — the CLI/HTTP layers turn those into a status + non-zero exit / 500.
    """
    ds = load_dataset_files(ego_path, route_path)
    route = ds.route
    state = ProjectState()
    frames = []
    for i in range(len(ds.meas_e)):
        e = float(ds.meas_e[i])
        n = float(ds.meas_n[i])
        yaw = float(ds.meas_yaw[i])
        speed = float(ds.speed[i])
        out = project_route(route, e, n, yaw, config, state, speed)
        state = out.state
        lat, lon = float(ds.ego_llh[i, 0]), float(ds.ego_llh[i, 1])
        frames.append({
            "timestamp_us": int(ds.t_us[i]),
            "pose": {"e": _round(e, 3), "n": _round(n, 3), "yaw": _round(yaw, 6),
                     "lat": _round(lat, 7), "lon": _round(lon, 7)},
            "speed": _round(speed, 3),
            "path": [[_round(x, 3), _round(y, 3)] for x, y in out.path],
            "cursor_s": _round(out.cursor_s, 3),
            "lat_dev": _round(out.lat_dev, 4),
            "matched_seg": int(out.matched_seg),
            "end_flag": bool(out.end_flag),
        })
    return {
        "status": {"generated": True, "n_frames": len(frames), "message": ""},
        "meta": {
            "ego_json": ego_path,
            "route_json": route_path,
            "frame": "body: +x forward, +y left, meters",
            "config": dataclasses.asdict(config),
            "generated_by": "offline_processing_routing_projection.py",
        },
        "frames": frames,
    }


def _build_parser():
    p = argparse.ArgumentParser(
        prog="offline_processing_routing_projection",
        description="Offline ego-frame routing-path generation via project_route.")
    p.add_argument("--ego-json", required=True, help="path to ego_route_llh.json")
    p.add_argument("--route-json", required=True,
                   help="path to route_generation_result/planned_route.json")
    p.add_argument("--out", required=True, help="output JSON path")
    d = ProjectConfig()  # defaults
    p.add_argument("--strategy", default=d.strategy,
                   choices=["raw", "centered", "smoothed", "human", "human_centered"])
    p.add_argument("--behind-m", type=float, default=d.behind_m)
    p.add_argument("--ahead-m", type=float, default=d.ahead_m)
    p.add_argument("--sample-ds-m", type=float, default=d.sample_ds_m)
    p.add_argument("--search-ahead-m", type=float, default=d.search_ahead_m)
    p.add_argument("--search-back-m", type=float, default=d.search_back_m)
    p.add_argument("--heading-gate-deg", type=float, default=d.heading_gate_deg)
    p.add_argument("--min-turn-radius-m", type=float, default=d.min_turn_radius_m)
    p.add_argument("--corner-angle-deg", type=float, default=d.corner_angle_deg)
    p.add_argument("--simplify-eps-m", type=float, default=d.simplify_eps_m)
    p.add_argument("--corner-style", default=d.corner_style,
                   choices=["clothoid", "arc", "driver"])
    p.add_argument("--clothoid-transition-m", type=float, default=d.clothoid_transition_m)
    p.add_argument("--human-cut-m", type=float, default=d.human_cut_m)
    return p


def config_from_args(args):
    return ProjectConfig(
        strategy=args.strategy, behind_m=args.behind_m, ahead_m=args.ahead_m,
        sample_ds_m=args.sample_ds_m, search_ahead_m=args.search_ahead_m,
        search_back_m=args.search_back_m, heading_gate_deg=args.heading_gate_deg,
        min_turn_radius_m=args.min_turn_radius_m, corner_angle_deg=args.corner_angle_deg,
        simplify_eps_m=args.simplify_eps_m, corner_style=args.corner_style,
        clothoid_transition_m=args.clothoid_transition_m, human_cut_m=args.human_cut_m)


def config_from_dict(cfg):
    """Build a ProjectConfig from a partial dict (used by the HTTP endpoint);
    unknown keys are ignored, missing keys fall back to defaults."""
    fields = {f.name for f in dataclasses.fields(ProjectConfig)}
    kw = {k: v for k, v in (cfg or {}).items() if k in fields}
    return ProjectConfig(**kw)


def main(argv=None):
    args = _build_parser().parse_args(argv)
    config = config_from_args(args)
    try:
        result = run(args.ego_json, args.route_json, config)
    except Exception as exc:  # noqa: BLE001 — surface any load/parse failure
        status = {"generated": False, "n_frames": 0, "message": f"{type(exc).__name__}: {exc}"}
        try:
            with open(args.out, "w") as fh:
                json.dump({"status": status, "meta": {}, "frames": []}, fh)
        except Exception:  # noqa: BLE001
            pass
        print(f"offline projection FAILED: {status['message']}", file=sys.stderr)
        return 1
    with open(args.out, "w") as fh:
        json.dump(result, fh)
    print(f"generated {result['status']['n_frames']} frames -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
