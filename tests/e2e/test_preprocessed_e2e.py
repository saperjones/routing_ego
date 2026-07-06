"""End-to-end browser test for the 'Pre-processed' viewer tab.

Serves the repo root over HTTP, drives the viewer in headless Chromium, feeds the
committed 3-file fixture (tests/e2e/fixtures/preproc/) into the folder picker, and
asserts that the BEV + driver + panorama render, telemetry populates, the live
controls are disabled, playback advances, and switching away clears the pre case.

Requires: playwright + chromium (`python -m playwright install chromium`).
Marked `e2e`; run with `pytest -m e2e`.
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
FIX = REPO / "tests" / "e2e" / "fixtures" / "preproc"

_NONBLANK = """(sel) => {
  const c = document.querySelector(sel);
  if (!c) return false;
  const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
  for (let i = 3; i < d.length; i += 4) { if (d[i] > 0) return true; }
  return false;
}"""

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

_SIGNATURE = """(sel) => {
  const c = document.querySelector(sel);
  const d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
  let n = 0, sum = 0;
  for (let i = 0; i < d.length; i += 4) {
    if (d[i + 3] > 0) { n++; sum += d[i] + d[i + 1] + d[i + 2] + i % 997; }
  }
  return n * 1000003 + sum;
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
    # the real-data index must exist so switching back to the Real tab has a list
    if not (OUT / "index.json").exists():
        env = dict(os.environ, PYTHONPATH=str(SRC))
        subprocess.run([sys.executable, "-m", "parking_proj.generate"],
                       cwd=REPO, env=env, check=True)
    if not (OUT / "real" / "index.json").exists():
        env = dict(os.environ, PYTHONPATH=str(SRC))
        subprocess.run([sys.executable, "-m", "parking_proj.generate_real"],
                       cwd=REPO, env=env, check=True)
    port = _free_port()
    proc = subprocess.Popen(
        [sys.executable, "-m", "parking_proj.viewer_server", str(port)],
        cwd=REPO, env=dict(os.environ, PYTHONPATH=str(SRC)),
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    try:
        assert _wait_http(port, "/viewer/index.html"), "static server did not start"
        yield f"http://127.0.0.1:{port}"
    finally:
        proc.terminate()
        proc.wait()


@pytest.fixture(scope="module")
def page_ctx(base_url):
    with sync_playwright() as p:
        try:
            browser = p.chromium.launch()
        except Exception as exc:  # pragma: no cover
            pytest.skip(f"chromium unavailable: {exc}")
        page = browser.new_page()
        errors = []
        page.on("pageerror", lambda exc: errors.append(f"pageerror: {exc}"))
        page.on("console", lambda m: errors.append(f"console: {m.text}")
                if m.type == "error" and "favicon" not in m.text else None)
        page.goto(f"{base_url}/viewer/index.html")
        page.wait_for_selector("#tab-pre")
        yield page, errors
        browser.close()


def _wait_pre_rendered(page):
    page.wait_for_function(
        "() => typeof STATE !== 'undefined' && STATE.case"
        " && STATE.mode === 'pre' && STATE.case.mode === 'pre'"
        " && STATE.case.frames.length > 0",
        timeout=8000,
    )
    page.wait_for_function(_NONBLANK, arg="#bev", timeout=6000)
    page.wait_for_function(_NONBLANK, arg="#driver", timeout=6000)


def _open_pre_picker(page):
    """Exercise the real folder picker (webkitdirectory) once."""
    page.click("#tab-pre")
    page.wait_for_selector("#pre-picker", state="visible")
    page.set_input_files("#pre-folder", str(FIX))   # webkitdirectory -> pass the folder
    _wait_pre_rendered(page)


def _open_pre(page):
    """Load the fixture via the served JSON + window.loadPreCase (the same code
    path as the picker, minus the File read). Re-selecting the *same* directory
    on a webkitdirectory input hangs Playwright, so state-dependent tests use
    this JS loader; _open_pre_picker covers the real picker wiring once."""
    page.click("#tab-pre")
    page.wait_for_selector("#pre-picker", state="visible")
    page.evaluate(
        """async () => {
          const base = '/tests/e2e/fixtures/preproc';
          const proj = await (await fetch(base + '/routing_projection.json')).json();
          const route = await (await fetch(base + '/planned_route.json')).json();
          window.loadPreCase(proj, route, 'preproc');
        }""")
    _wait_pre_rendered(page)


def test_pre_tab_shows_picker(page_ctx):
    page, _ = page_ctx
    page.click("#tab-pre")
    assert "active" in (page.get_attribute("#tab-pre", "class") or "")
    assert page.locator("#pre-picker").is_visible()
    assert page.locator("#case-list").is_hidden()
    assert page.locator("#cases-head").is_hidden()


def test_pre_renders_all_views(page_ctx):
    page, _ = page_ctx
    _open_pre_picker(page)      # the real folder picker, exercised once
    # BEV: gray graticule + route + track fill a real area (not a stray pixel)
    bev = page.evaluate(_GEOM, "#bev")
    assert bev["drawn"] > 800, bev
    # Driver view: the path runs forward -> tall vertical extent
    drv = page.evaluate(_GEOM, "#driver")
    assert drv["drawn"] > 200, drv
    assert (drv["bbox"][3] - drv["bbox"][1]) > drv["h"] * 0.3, drv
    assert page.evaluate(_NONBLANK, "#panorama") is True


def test_pre_telemetry_and_scrubber(page_ctx):
    page, _ = page_ctx
    _open_pre(page)
    assert "km/h" in page.inner_text("#tm-speed")
    assert "/" in page.inner_text("#tm-frame")
    assert page.inner_text("#tm-heading").strip() not in ("", "–")
    assert "pre-processed" in page.inner_text("#tm-verdict").lower()
    assert page.inner_text("#tm-truedev").strip() == "–"       # no ground truth
    n = page.evaluate("() => STATE.case.frames.length")
    assert int(page.get_attribute("#scrubber", "max")) == n - 1


def test_pre_controls_disabled(page_ctx):
    page, _ = page_ctx
    _open_pre(page)
    for sel in ("#algo-select", "#p-ahead", "#p-behind", "#corner-style",
                "#compare-toggle", "#btn-offline"):
        assert not page.is_enabled(sel), f"{sel} should be disabled in pre mode"
    # config caption reflects the file's strategy
    assert "strategy=human_centered" in page.inner_text("#pre-config")


def test_pre_step_advances_and_repaints(page_ctx):
    page, _ = page_ctx
    _open_pre(page)
    sig0 = page.evaluate(_SIGNATURE, "#bev")
    frame0 = page.inner_text("#tm-frame")
    # seek to the last frame -> car marker moves, BEV signature changes
    page.eval_on_selector(
        "#scrubber", "el => { el.value = el.max; el.dispatchEvent(new Event('input')); }")
    page.wait_for_function(
        "() => { const t = document.getElementById('tm-frame').textContent.split('/');"
        " return t.length === 2 && t[0].trim() === t[1].trim() && t[0].trim() !== '0'; }",
        timeout=3000)
    assert page.inner_text("#tm-frame") != frame0
    assert page.evaluate(_SIGNATURE, "#bev") != sig0
    assert page.evaluate(_NONBLANK, "#driver") is True


def test_pre_perspective_view(page_ctx):
    page, _ = page_ctx
    _open_pre(page)
    top = page.evaluate(_GEOM, "#driver")
    page.check("#persp-toggle")
    page.wait_for_timeout(150)
    persp = page.evaluate(_GEOM, "#driver")
    assert persp["drawn"] > top["drawn"] * 2, (top, persp)     # sky/ground fill
    page.uncheck("#persp-toggle")


def test_switch_away_clears_pre(page_ctx):
    page, _ = page_ctx
    _open_pre(page)
    page.click("#tab-real")
    page.wait_for_selector("#case-list li:not(.group-header)")
    assert page.evaluate("() => STATE.mode") == "real"
    assert page.evaluate("() => STATE.case") is None or \
        page.evaluate("() => STATE.case && STATE.case.mode") != "pre"
    # controls re-enabled after leaving pre mode
    assert page.is_enabled("#algo-select")
    assert page.locator("#pre-picker").is_hidden()


def test_build_pre_case_pure(page_ctx):
    """Unit-test the pure builder in the page context with tiny stub inputs."""
    page, _ = page_ctx
    result = page.evaluate(
        """() => {
          const proj = { status: {generated: true, message: ""},
            meta: {config: {strategy: "human_centered", behind_m: 5, ahead_m: 40, sample_ds_m: 0.5}},
            frames: [
              {pose:{e:1,n:2,yaw:0.5,lat:31.1,lon:117.1}, speed:3, path:[[0,0],[1,0]],
               cursor_s:0.5, lat_dev:0.2, matched_seg:0, end_flag:false},
              {pose:{e:2,n:3,yaw:0.6,lat:31.2,lon:117.2}, speed:4, path:[[0,0],[2,0]],
               cursor_s:1.5, lat_dev:0.3, matched_seg:1, end_flag:true},
            ] };
          const route = { planned_route: [[31.0,117.0],[31.1,117.1],[31.2,117.2]],
                          waypoints: [[31.0,117.0],[31.2,117.2]] };
          const c = window.buildPreCase(proj, route, "myfolder");
          const cNoRoute = window.buildPreCase(proj, null, "x");
          return {
            mode: c.mode, name: c.name,
            nRoute: c.route_llh.length, nTrack: c.ego_track_llh.length,
            nFrames: c.frames.length,
            trackMatchesPose: c.ego_track_llh[0][0] === 31.1 && c.ego_track_llh[1][1] === 117.2,
            headFromYaw: c.frames[0].meas_pose.h === 0.5,
            estFromLatDev: c.frames[1].est_lat_dev === 0.3,
            trueDevNull: c.frames[0].true_lat_dev === null,
            pathCopied: JSON.stringify(c.frames[1].path) === JSON.stringify([[0,0],[2,0]]),
            endFlag: c.frames[1].end_flag === true,
            routeLenPositive: c.route_total_len_m > 0,
            noRouteLenNull: cNoRoute.route_total_len_m === null,
            basemapNull: c.basemap === null,
            verdictNull: c.verdict === null,
          };
        }""")
    assert result == {
        "mode": "pre", "name": "myfolder", "nRoute": 3, "nTrack": 2, "nFrames": 2,
        "trackMatchesPose": True, "headFromYaw": True, "estFromLatDev": True,
        "trueDevNull": True, "pathCopied": True, "endFlag": True,
        "routeLenPositive": True, "noRouteLenNull": True, "basemapNull": True,
        "verdictNull": True,
    }, result


def test_pre_missing_projection_errors(page_ctx, tmp_path):
    page, _ = page_ctx
    # a folder WITHOUT routing_projection.json -> clear error
    import shutil
    bad = tmp_path / "noproj"
    bad.mkdir()
    shutil.copy(FIX / "planned_route.json", bad / "planned_route.json")
    page.click("#tab-pre")
    page.wait_for_selector("#pre-picker", state="visible")
    page.set_input_files("#pre-folder", str(bad))
    page.wait_for_function(
        "() => document.getElementById('pre-status').textContent"
        ".indexOf('routing_projection.json not found') >= 0", timeout=4000)


def test_pre_no_js_errors(page_ctx):
    page, errors = page_ctx
    _open_pre(page)
    page.click("#btn-step-fwd")
    assert errors == [], f"JS errors during pre-processed use: {errors}"
