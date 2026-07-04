import http.client, os, socket, subprocess, sys, time
from pathlib import Path
import pytest

pytest.importorskip("playwright.sync_api")
from playwright.sync_api import sync_playwright  # noqa: E402

pytestmark = pytest.mark.e2e
REPO = Path(__file__).resolve().parents[2]
SRC = REPO / "src"


def _free_port():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close(); return p


def _wait(port, path, timeout=15):
    end = time.time() + timeout
    while time.time() < end:
        try:
            c = http.client.HTTPConnection("127.0.0.1", port, timeout=1)
            c.request("GET", path)
            if c.getresponse().status == 200:
                return True
        except OSError:
            time.sleep(0.1)
    return False


def _fixtures():
    """(route_dict, pose, cfg_dict, expected_path, expected_matched_seg, expected_end_flag) tuples."""
    sys.path.insert(0, str(SRC))
    from parking_proj.geometry import route_from_waypoints
    from parking_proj.project_route import project_route, ProjectConfig
    from dataclasses import asdict
    cases = []
    routes = {
        "straight": route_from_waypoints([[0.0, 0.0], [100.0, 0.0]], ["1", "2"]),
        "corner": route_from_waypoints([[0.0, 0.0], [30.0, 0.0], [30.0, 30.0]], ["1", "2", "3"]),
        # shallow bend: second-leg direction atan2(8,30)≈14.9°, corner angle ≈15°.
        # Above the 10° fillet threshold but below 2*theta_sp≈17.2°, so the
        # shortened-spiral branch (2*theta_sp > delta) fires in project_route.
        "shallow": route_from_waypoints([[0.0, 0.0], [30.0, 0.0], [60.0, 8.0]], ["1", "2", "3"]),
    }
    # Standard poses plus large-yaw poses that exercise the negative-angle-diff branch.
    # yaw=2.9 and yaw=-2.9 make atan2(tangent)-yaw < -pi for some route orientations,
    # which would diverge before the true-modulo fix.
    poses = [(10.0, 2.0, 0.0), (25.0, 1.0, 0.1), (0.0, 0.0, 0.0),
             (10.0, 2.0, 2.9), (10.0, 2.0, -2.9)]
    for rk, r in routes.items():
        rd = {
            "points": r.points.tolist(),
            "s": r.s.tolist(),
            "tangents": r.tangents.tolist(),
            "length": r.length,
            "seg_of_index": r.seg_of_index.tolist(),
            "waypoint_indices": r.waypoint_indices,
        }
        for (pe, pn, yaw) in poses:
            for strat in ("raw", "centered", "smoothed"):
                styles = ("arc", "clothoid") if strat == "smoothed" else ("clothoid",)
                for style in styles:
                    cfg = ProjectConfig(strategy=strat, corner_style=style)
                    out = project_route(r, pe, pn, yaw, cfg)
                    cases.append((rd, {"e": pe, "n": pn, "h": yaw}, asdict(cfg),
                                   out.path, out.matched_seg, out.end_flag))
    return cases


def test_js_matches_python():
    port = _free_port()
    proc = subprocess.Popen([sys.executable, "-m", "http.server", str(port)],
                            cwd=REPO, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        assert _wait(port, "/tests/e2e/parity_harness.html")
        with sync_playwright() as p:
            try:
                br = p.chromium.launch()
            except Exception as exc:
                pytest.skip(f"chromium unavailable: {exc}")
            pg = br.new_page()
            pg.goto(f"http://127.0.0.1:{port}/tests/e2e/parity_harness.html")
            pg.wait_for_function("() => !!window.runCase")
            cases = _fixtures()
            for rd, pose, cfg, expected_path, expected_seg, expected_end in cases:
                got = pg.evaluate("([r,po,c]) => window.runCase(r,po,c)",
                                  [rd, pose, cfg])
                got_path = got["path"]
                assert len(got_path) == len(expected_path), \
                    (cfg["strategy"], pose, len(got_path), len(expected_path))
                for (gx, gy), (ex, ey) in zip(got_path, expected_path):
                    assert abs(gx - ex) < 1e-3 and abs(gy - ey) < 1e-3, \
                        (cfg["strategy"], pose, gx, gy, ex, ey)
                assert got["matched_seg"] == expected_seg, \
                    (cfg["strategy"], pose, "matched_seg", got["matched_seg"], expected_seg)
                assert bool(got["end_flag"]) == bool(expected_end), \
                    (cfg["strategy"], pose, "end_flag", got["end_flag"], expected_end)
            br.close()
            print(f"\nparity: {len(cases)} cases passed (path + matched_seg + end_flag)")
    finally:
        proc.terminate(); proc.wait()
