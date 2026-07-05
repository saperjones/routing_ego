"""Tests for the offline routing-projection interface + its HTTP endpoint."""
import glob
import json
import os
import threading
import urllib.request
import urllib.error

import pytest

from parking_proj import offline_processing_routing_projection as offline
from parking_proj import viewer_server
from parking_proj.project_route import ProjectConfig

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _first_dataset():
    dirs = sorted(d for d in glob.glob(os.path.join(REPO, "dataset", "*"))
                  if os.path.isfile(os.path.join(d, "ego_route_llh.json")))
    if not dirs:
        pytest.skip("no dataset/ available")
    d = dirs[0]
    return (os.path.join(d, "ego_route_llh.json"),
            os.path.join(d, "route_generation_result", "planned_route.json"),
            os.path.basename(d))


def test_run_shape_and_timestamp_sync():
    ego, route, _ = _first_dataset()
    res = offline.run(ego, route, ProjectConfig(strategy="human_centered"))
    assert res["status"]["generated"] is True
    n = res["status"]["n_frames"]
    src = json.load(open(ego))
    assert n == src["point_count"] == len(res["frames"])
    f0 = res["frames"][0]
    assert f0["timestamp_us"] == src["points"][0]["timestamp_us"]
    assert isinstance(f0["path"], list) and len(f0["path"]) > 0
    assert all(len(p) == 2 for p in f0["path"])            # [x, y] pairs
    for key in ("cursor_s", "lat_dev", "matched_seg", "end_flag"):
        assert key in f0
    assert set(("e", "n", "yaw", "lat", "lon")) <= set(f0["pose"])


def test_strategy_changes_output():
    ego, route, _ = _first_dataset()
    raw = offline.run(ego, route, ProjectConfig(strategy="raw"))
    ctr = offline.run(ego, route, ProjectConfig(strategy="centered"))
    # a mid-run frame's path differs once a lateral offset is present
    mid = len(raw["frames"]) // 2
    assert raw["frames"][mid]["path"] != ctr["frames"][mid]["path"]


def test_determinism():
    ego, route, _ = _first_dataset()
    cfg = ProjectConfig(strategy="smoothed")
    a = offline.run(ego, route, cfg)
    b = offline.run(ego, route, cfg)
    assert json.dumps(a) == json.dumps(b)


def test_main_missing_file(tmp_path):
    out = tmp_path / "out.json"
    rc = offline.main(["--ego-json", "/nope/ego.json",
                       "--route-json", "/nope/route.json", "--out", str(out)])
    assert rc != 0
    data = json.load(open(out))
    assert data["status"]["generated"] is False and data["status"]["message"]


def test_main_writes_output(tmp_path):
    ego, route, _ = _first_dataset()
    out = tmp_path / "out.json"
    rc = offline.main(["--ego-json", ego, "--route-json", route,
                       "--out", str(out), "--strategy", "centered", "--ahead-m", "30"])
    assert rc == 0
    data = json.load(open(out))
    assert data["status"]["generated"] is True
    assert data["meta"]["config"]["strategy"] == "centered"
    assert data["meta"]["config"]["ahead_m"] == 30.0


@pytest.fixture()
def server():
    import socket
    s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    httpd = viewer_server.serve(port)
    t = threading.Thread(target=httpd.serve_forever, daemon=True); t.start()
    try:
        yield port
    finally:
        httpd.shutdown()


def _post(port, body):
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}/api/offline",
        data=json.dumps(body).encode(), headers={"Content-Type": "application/json"},
        method="POST")
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_endpoint_success(server):
    _, _, dataset_id = _first_dataset()
    code, data = _post(server, {"dataset_id": dataset_id,
                                "config": {"strategy": "human_centered"}})
    assert code == 200
    assert data["status"]["generated"] is True and data["status"]["n_frames"] > 0
    assert len(data["frames"]) == data["status"]["n_frames"]


def test_endpoint_unknown_dataset(server):
    code, data = _post(server, {"dataset_id": "does-not-exist", "config": {}})
    assert code == 500
    assert data["status"]["generated"] is False and data["status"]["message"]
