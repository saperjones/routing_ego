"""Static file server for the viewer + a POST /api/offline endpoint that runs
the offline projection on a dataset and returns the result JSON.

Serves the repo root (like ``python -m http.server``) so the viewer's ``../out``
and ``dataset/`` paths resolve. Used by ``run.sh serve`` and the e2e fixture.

  python -m parking_proj.viewer_server [PORT]   # default 8000
"""
import json
import os
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

from . import offline_processing_routing_projection as offline

# repo root = two levels up from this file (src/parking_proj/viewer_server.py)
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DATASET_DIR = os.path.join(REPO_ROOT, "dataset")


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=REPO_ROOT, **kwargs)

    def _send_json(self, code, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path.split("?")[0] != "/api/offline":
            self._send_json(404, {"status": {"generated": False, "message": "not found"}})
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            req = json.loads(self.rfile.read(length) or b"{}")
            dataset_id = req.get("dataset_id")
            if not dataset_id:
                raise ValueError("dataset_id required")
            ds_dir = os.path.join(DATASET_DIR, dataset_id)
            ego = os.path.join(ds_dir, "ego_route_llh.json")
            route = os.path.join(ds_dir, "route_generation_result", "planned_route.json")
            if not (os.path.isfile(ego) and os.path.isfile(route)):
                raise FileNotFoundError(f"dataset files not found for {dataset_id!r}")
            config = offline.config_from_dict(req.get("config"))
            result = offline.run(ego, route, config)
        except Exception as exc:  # noqa: BLE001 — report any failure to the client
            self._send_json(500, {"status": {"generated": False, "n_frames": 0,
                                              "message": f"{type(exc).__name__}: {exc}"}})
            return
        self._send_json(200, result)

    def log_message(self, *args):  # keep the console quiet
        pass


def serve(port):
    httpd = ThreadingHTTPServer(("", port), Handler)
    return httpd


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    httpd = serve(port)
    print(f"serving repo root + /api/offline at http://localhost:{port}/")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        httpd.shutdown()
