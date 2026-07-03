"""End-to-end browser test for the static viewer.

Serves the repo root over HTTP, drives the viewer in headless Chromium, and
asserts the things unit tests cannot: the canvases actually render, no JS
errors occur, playback advances frames, and the BEV static layer is rebuilt
when switching cases (regression guard for the case-switch cache bug).

Requires: playwright + chromium (`python -m playwright install chromium`).
Marked `e2e`; run with `pytest -m e2e` or skip with `pytest -m "not e2e"`.
"""
import http.client
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

pytest.importorskip("playwright.sync_api")
from playwright.sync_api import sync_playwright  # noqa: E402

pytestmark = pytest.mark.e2e

REPO = Path(__file__).resolve().parents[2]
SRC = REPO / "src"
OUT = REPO / "out"

# JS: does a canvas have any drawn (non-transparent) pixels?
_NONBLANK = """(sel) => {
  const c = document.querySelector(sel);
  if (!c) return false;
  const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
  for (let i = 3; i < d.length; i += 4) { if (d[i] > 0) return true; }
  return false;
}"""

# JS: a cheap signature of a canvas's drawn content
_SIGNATURE = """(sel) => {
  const c = document.querySelector(sel);
  const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
  let n = 0, sum = 0;
  for (let i = 0; i < d.length; i += 4) {
    if (d[i + 3] > 0) { n++; sum += d[i] + d[i + 1] + d[i + 2] + i % 997; }
  }
  return n * 1000003 + sum;
}"""

# JS: drawn-pixel count and bounding box [minx, miny, maxx, maxy] of a canvas
_GEOM = """(sel) => {
  const c = document.querySelector(sel);
  const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
  let n = 0, minx = 1e9, miny = 1e9, maxx = -1, maxy = -1;
  for (let y = 0; y < c.height; y++) for (let x = 0; x < c.width; x++) {
    if (d[(y * c.width + x) * 4 + 3] > 0) {
      n++;
      if (x < minx) minx = x; if (x > maxx) maxx = x;
      if (y < miny) miny = y; if (y > maxy) maxy = y;
    }
  }
  return {w: c.width, h: c.height, drawn: n, bbox: [minx, miny, maxx, maxy]};
}"""


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _wait_http(port, path, timeout=15):
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


@pytest.fixture(scope="module")
def base_url():
    if not (OUT / "index.json").exists():
        env = dict(os.environ, PYTHONPATH=str(SRC))
        subprocess.run([sys.executable, "-m", "parking_proj.generate"],
                       cwd=REPO, env=env, check=True)
    port = _free_port()
    proc = subprocess.Popen(
        [sys.executable, "-m", "http.server", str(port)],
        cwd=REPO, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    try:
        assert _wait_http(port, "/viewer/index.html"), "static server did not start"
        yield f"http://127.0.0.1:{port}"
    finally:
        proc.terminate()
        proc.wait()


@pytest.fixture(scope="module")
def viewer(base_url):
    with sync_playwright() as p:
        try:
            browser = p.chromium.launch()
        except Exception as exc:  # pragma: no cover - env without browser
            pytest.skip(f"chromium unavailable: {exc}")
        page = browser.new_page()
        errors = []
        page.on("pageerror", lambda exc: errors.append(f"pageerror: {exc}"))
        page.on("console", lambda m: errors.append(f"console: {m.text}")
                if m.type == "error" and "favicon" not in m.text else None)
        page.goto(f"{base_url}/viewer/index.html")
        page.wait_for_selector("#case-list li")
        yield page, errors
        browser.close()


def _select(page, case_name):
    page.locator("#case-list li", has_text=case_name).first.click()
    page.wait_for_function(_NONBLANK, arg="#driver", timeout=6000)
    page.wait_for_function(_NONBLANK, arg="#bev", timeout=6000)


def test_case_list_populates(viewer):
    page, _ = viewer
    rows = page.locator("#case-list li:not(.group-header)")
    headers = page.locator("#case-list li.group-header")
    assert rows.count() == 14
    assert headers.count() == 7  # A,B,C,D,E,F,G
    assert page.locator("#case-list .badge.pass").count() == 14
    assert page.locator("#case-list .badge.fail").count() == 0


def test_panorama_renders_on_load(viewer):
    page, _ = viewer
    _select(page, "Straight (low)")
    assert page.evaluate(_NONBLANK, "#panorama") is True


def test_canvases_render_for_selected_case(viewer):
    page, _ = viewer
    _select(page, "X-crossing (low)")
    assert page.evaluate(_NONBLANK, "#bev") is True
    assert page.evaluate(_NONBLANK, "#driver") is True


def test_render_has_meaningful_coverage(viewer):
    """Guard against 'renders a stray pixel' false-passes: the route must
    actually span the canvas, not just leave a dot."""
    page, _ = viewer
    _select(page, "X-crossing (low)")
    bev = page.evaluate(_GEOM, "#bev")
    drv = page.evaluate(_GEOM, "#driver")
    # BEV: the X-shaped route spans a large fraction of the canvas both ways
    assert bev["drawn"] > 1500, bev
    assert (bev["bbox"][2] - bev["bbox"][0]) > bev["w"] * 0.4, bev
    assert (bev["bbox"][3] - bev["bbox"][1]) > bev["h"] * 0.4, bev
    # Driver view: at the start the path runs straight ahead, so it must have a
    # tall vertical extent (the projected route going forward/up).
    assert drv["drawn"] > 300, drv
    assert (drv["bbox"][3] - drv["bbox"][1]) > drv["h"] * 0.4, drv


def test_telemetry_populates(viewer):
    page, _ = viewer
    _select(page, "Straight (low)")
    assert "km/h" in page.inner_text("#tm-speed")
    assert "/" in page.inner_text("#tm-frame")
    # heading and position are filled (not the placeholder dash)
    assert page.inner_text("#tm-heading").strip() not in ("", "–")


def test_step_advances_frame(viewer):
    page, _ = viewer
    _select(page, "Straight (low)")
    before = page.inner_text("#tm-frame")
    page.click("#btn-step-fwd")
    after = page.inner_text("#tm-frame")
    assert before != after
    assert before.split("/")[0].strip() == "0"
    assert after.split("/")[0].strip() == "1"


def test_scrubber_seeks(viewer):
    page, _ = viewer
    _select(page, "S-shape (medium)")
    page.eval_on_selector(
        "#scrubber",
        "el => { el.value = el.max; el.dispatchEvent(new Event('input')); }",
    )
    frame = page.inner_text("#tm-frame")
    lhs, rhs = [s.strip() for s in frame.split("/")]
    assert lhs == rhs  # seeked to the last frame

def test_bev_rebuilds_on_case_switch(viewer):
    """Regression guard: switching cases must rebuild the BEV static layer."""
    page, _ = viewer
    _select(page, "Straight (low)")
    sig_straight = page.evaluate(_SIGNATURE, "#bev")
    _select(page, "X-crossing (low)")
    sig_cross = page.evaluate(_SIGNATURE, "#bev")
    assert sig_straight != sig_cross, "BEV did not change between cases (stale static layer)"


def test_no_js_errors(viewer):
    page, errors = viewer
    # exercise a couple more interactions before the final error check
    _select(page, "Figure-eight (medium)")
    page.click("#btn-step-fwd")
    assert errors == [], f"JS errors during viewer use: {errors}"
