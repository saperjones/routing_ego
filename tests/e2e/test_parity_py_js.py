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
    """(route_dict, pose, cfg_dict, expected_path) tuples from the Python impl."""
    sys.path.insert(0, str(SRC))
    from parking_proj.geometry import route_from_waypoints
    from parking_proj.project_route import project_route, ProjectConfig
    from dataclasses import asdict
    cases = []
    routes = {
        "straight": route_from_waypoints([[0.0, 0.0], [100.0, 0.0]], ["1", "2"]),
        "corner": route_from_waypoints([[0.0, 0.0], [30.0, 0.0], [30.0, 30.0]], ["1", "2", "3"]),
    }
    poses = [(10.0, 2.0, 0.0), (25.0, 1.0, 0.1), (0.0, 0.0, 0.0)]
    for rk, r in routes.items():
        rd = {"points": r.points.tolist(), "s": r.s.tolist(),
              "tangents": r.tangents.tolist(), "length": r.length}
        for (pe, pn, yaw) in poses:
            for strat in ("raw", "centered", "smoothed"):
                cfg = ProjectConfig(strategy=strat)
                out = project_route(r, pe, pn, yaw, cfg)
                cases.append((rd, {"e": pe, "n": pn, "h": yaw}, asdict(cfg), out.path))
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
            for rd, pose, cfg, expected in _fixtures():
                got = pg.evaluate("([r,po,c]) => window.runCase(r,po,c).path",
                                  [rd, pose, cfg])
                assert len(got) == len(expected), (cfg["strategy"], len(got), len(expected))
                for (gx, gy), (ex, ey) in zip(got, expected):
                    assert abs(gx - ex) < 1e-3 and abs(gy - ey) < 1e-3, \
                        (cfg["strategy"], gx, gy, ex, ey)
            br.close()
    finally:
        proc.terminate(); proc.wait()
