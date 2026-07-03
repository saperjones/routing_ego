# Parking Route Projection

Projects a global (ENU) planned route into the vehicle-body frame each frame,
with a seeded two-layer simulation and a static HTML viewer.

## Setup
    pip install -e ".[dev]"

## Test
    pytest -v                    # unit + acceptance (fast, no browser)

## End-to-end viewer test (headless browser)
    pip install -e ".[e2e]"
    python -m playwright install chromium
    pytest -m e2e -v             # drives the served viewer in headless Chromium

The e2e suite serves the repo root, opens the viewer, and asserts the canvases
render, playback advances, the scrubber seeks, no JS errors occur, and the BEV
layer rebuilds on case switch. It is excluded from the default `pytest` run.

## Generate test-case JSON
    python -m parking_proj.generate      # writes out/

## View
    python -m http.server 8000        # run from the repo ROOT (not cd viewer)
    # open http://localhost:8000/viewer/index.html

Body frame: +x forward, +y left, +z up. Heading = CCW from ENU East.
