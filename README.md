# Parking Route Projection

Projects a global (ENU) planned route into the vehicle-body frame each frame,
with a seeded two-layer simulation and a static HTML viewer.

## Setup
    pip install -e ".[dev]"

## Test
    pytest -v

## Generate test-case JSON
    python -m parking_proj.generate      # writes out/

## View
    python -m http.server 8000        # run from the repo ROOT (not cd viewer)
    # open http://localhost:8000/viewer/index.html

Body frame: +x forward, +y left, +z up. Heading = CCW from ENU East.
