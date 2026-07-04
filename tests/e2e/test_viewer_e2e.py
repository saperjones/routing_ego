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

# JS: mean x of the route stroke (green #2e9e5b) within a canvas y-band [y0, y1)
_ROUTE_MEAN_X = """([sel, y0, y1]) => {
  const c = document.querySelector(sel);
  const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
  let sx = 0, n = 0;
  for (let y = y0; y < y1; y++) for (let x = 0; x < c.width; x++) {
    const i = (y * c.width + x) * 4;
    const r = d[i], g = d[i + 1], b = d[i + 2], a = d[i + 3];
    if (a > 0 && g > 90 && g - r > 25 && g - b > 25) { sx += x; n++; }  // greenish route
  }
  return {mean: n ? sx / n : null, n: n};
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
    real_index = OUT / "real" / "index.json"
    if not real_index.exists():
        env = dict(os.environ, PYTHONPATH=str(SRC))
        subprocess.run([sys.executable, "-m", "parking_proj.generate_real"],
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
    # all _select cases are simulation cases; ensure the sim tab is active so the
    # test is independent of prior tests / test order (the fixture is module-scoped)
    if "active" not in (page.get_attribute("#tab-sim", "class") or ""):
        page.click("#tab-sim")
        page.wait_for_selector("#case-list li.group-header")
    page.locator("#case-list li", has_text=case_name).first.click()
    # Wait for the async loadCase() fetch to complete so STATE.case reflects the
    # new selection before any subsequent scrubber/slider manipulation.
    escaped = case_name.replace("'", "\\'")
    page.wait_for_function(
        f"() => typeof STATE !== 'undefined' && STATE.case && STATE.case.name === '{escaped}'",
        timeout=8000,
    )
    page.wait_for_function(_NONBLANK, arg="#driver", timeout=6000)
    page.wait_for_function(_NONBLANK, arg="#bev", timeout=6000)


def test_case_list_populates(viewer):
    page, _ = viewer
    # on fresh page load the Real-data tab is the default (acceptance #1)
    assert "active" in (page.get_attribute("#tab-real", "class") or "")
    assert "active" not in (page.get_attribute("#tab-sim", "class") or "")
    page.click("#tab-sim")
    page.wait_for_selector("#case-list li.group-header")
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
    # wait for the seek to land on the last frame (render is driven off the event)
    page.wait_for_function(
        "() => { const t = document.getElementById('tm-frame').textContent.split('/');"
        " return t.length === 2 && t[0].trim() === t[1].trim() && t[0].trim() !== '0'; }",
        timeout=3000,
    )

def test_bev_rebuilds_on_case_switch(viewer):
    """Regression guard: switching cases must rebuild the BEV static layer."""
    page, _ = viewer
    _select(page, "Straight (low)")
    sig_straight = page.evaluate(_SIGNATURE, "#bev")
    _select(page, "X-crossing (low)")
    sig_cross = page.evaluate(_SIGNATURE, "#bev")
    assert sig_straight != sig_cross, "BEV did not change between cases (stale static layer)"


def test_perspective_view_renders(viewer):
    """The 'perspective' toggle switches the driver-view into the windshield
    3D projection: sky/ground fill make it far denser than the top-down slice,
    and it spans essentially the full width."""
    page, _ = viewer
    _select(page, "X-crossing (low)")
    top = page.evaluate(_GEOM, "#driver")
    page.check("#persp-toggle")
    page.wait_for_timeout(200)
    persp = page.evaluate(_GEOM, "#driver")
    assert persp["drawn"] > top["drawn"] * 3, (top, persp)          # filled sky+ground
    assert (persp["bbox"][2] - persp["bbox"][0]) > persp["w"] * 0.8, persp
    page.uncheck("#persp-toggle")


def test_no_js_errors(viewer):
    page, errors = viewer
    # exercise a couple more interactions before the final error check
    _select(page, "Figure-eight (medium)")
    page.click("#btn-step-fwd")
    assert errors == [], f"JS errors during viewer use: {errors}"


def test_real_tab_lists_datasets(viewer):
    page, _ = viewer
    # prior sim tests left the sim tab active; switch back to real
    page.click("#tab-real")
    page.wait_for_selector("#case-list li:not(.group-header)")
    rows = page.locator("#case-list li:not(.group-header)")
    assert rows.count() >= 1                      # at least the sample dataset


def test_real_case_renders_bev_and_drives(viewer):
    page, errors = viewer
    # ensure the real tab is active and its list is populated (order-independent)
    if "active" not in (page.get_attribute("#tab-real", "class") or ""):
        page.click("#tab-real")
    page.wait_for_selector("#case-list li:not(.group-header)")
    page.locator("#case-list li:not(.group-header)").first.click()
    # wait until the real case is actually loaded, not just the previous case's
    # telemetry (the real JSON is large — asserting too early reads a stale verdict)
    page.wait_for_function("() => typeof STATE !== 'undefined' && STATE.case"
                           " && STATE.case.mode === 'real'", timeout=8000)
    page.wait_for_function(_NONBLANK, arg="#bev", timeout=8000)
    page.wait_for_function(_NONBLANK, arg="#driver", timeout=8000)
    bev = page.evaluate(_GEOM, "#bev")
    assert bev["drawn"] > 1500, bev                # basemap-or-gray + route + track fill area
    # telemetry shows real-data verdict placeholder, speed present
    assert "real" in page.inner_text("#tm-verdict").lower() or "—" in page.inner_text("#tm-verdict")
    assert "km/h" in page.inner_text("#tm-speed")
    # step advances the frame
    before = page.inner_text("#tm-frame")
    page.click("#btn-step-fwd")
    assert page.inner_text("#tm-frame") != before


def test_algo_selector_switches_driver_view(viewer):
    page, _ = viewer
    _select(page, "X-crossing (high)")
    page.eval_on_selector(
        "#scrubber",
        "el => { el.value = Math.floor(el.max * 0.5); el.dispatchEvent(new Event('input')); }",
    )
    page.wait_for_timeout(80)
    page.select_option("#algo-select", "centered")
    page.wait_for_timeout(80)
    sig_c = page.evaluate(_SIGNATURE, "#driver")
    page.select_option("#algo-select", "raw")
    page.wait_for_timeout(80)
    sig_r = page.evaluate(_SIGNATURE, "#driver")
    assert sig_c != sig_r, "raw vs centered should differ when offset is present"
    page.select_option("#algo-select", "smoothed")


def test_smoothed_rounds_sharp_corner(viewer):
    page, _ = viewer
    _select(page, "Near-90 corner (low)")            # near-90 deg scenario (group C)
    # a frame with the corner in the look-ahead
    page.eval_on_selector("#scrubber",
        "el => { el.value = Math.floor(el.max*0.15); el.dispatchEvent(new Event('input')); }")
    def max_heading_step(strategy):
        page.select_option("#algo-select", strategy)
        page.wait_for_timeout(80)
        return page.evaluate("""() => {
          const c = STATE.case, f = STATE.frame;
          const p = computeBodyPath(c, f).filter(q => q.x >= 0);
          let m = 0;
          for (let i = 2; i < p.length; i++) {
            const a = Math.atan2(p[i-1].y-p[i-2].y, p[i-1].x-p[i-2].x);
            const b = Math.atan2(p[i].y-p[i-1].y, p[i].x-p[i-1].x);
            m = Math.max(m, Math.abs(((b-a+Math.PI)%(2*Math.PI))-Math.PI));
          }
          return m;
        }""")
    sharp = max_heading_step("centered")
    smooth = max_heading_step("smoothed")
    assert sharp > 0.5, sharp                # centered keeps the sharp corner
    assert smooth < sharp * 0.6, (sharp, smooth)   # smoothed clearly rounds it


def test_radius_slider_widens_arc(viewer):
    page, _ = viewer
    _select(page, "Near-90 corner (low)")
    page.eval_on_selector("#scrubber",
        "el => { el.value = Math.floor(el.max*0.15); el.dispatchEvent(new Event('input')); }")
    page.select_option("#algo-select", "smoothed")
    page.eval_on_selector("#p-radius", "el => { el.value = 4; el.dispatchEvent(new Event('input')); }")
    page.wait_for_timeout(80)
    sig4 = page.evaluate(_SIGNATURE, "#driver")
    page.eval_on_selector("#p-radius", "el => { el.value = 10; el.dispatchEvent(new Event('input')); }")
    page.wait_for_timeout(80)
    sig10 = page.evaluate(_SIGNATURE, "#driver")
    assert sig4 != sig10, "changing min turning radius must change the smoothed path"


def test_tab_switch_to_simulation_works(viewer):
    page, _ = viewer
    page.click("#tab-sim")
    page.wait_for_selector("#case-list li.group-header")   # sim list has group headers
    assert page.locator("#case-list .badge.pass").count() >= 1
    page.click("#tab-real")                                 # back to real
