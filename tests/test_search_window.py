"""The forward search window ('endurable offset') must be wide enough to catch
the vehicle after an along-track jump (real-data localization gaps), while its
default is the configurable SEARCH_AHEAD."""
import pytest
from parking_proj.geometry import route_from_waypoints
from parking_proj.projection import Projector, SEARCH_AHEAD


def straight():
    # due-east centerline, 100 m long
    return route_from_waypoints([[0.0, 0.0], [100.0, 0.0]], ["1", "2"])


def test_default_search_ahead_is_15():
    assert SEARCH_AHEAD == 15.0
    assert Projector(straight()).w_search == 15.0


def test_wide_window_catches_alongtrack_jump():
    route = straight()
    p = Projector(route)
    p.step(0.0, 0.0, 0.0)                 # initialize near s = 0
    r = p.step(10.0, 0.0, 0.0)            # vehicle is ~10 m further along in one frame
    # the cursor catches up to the vehicle (a 3.5 m window would stall near 3.5)
    assert r.cursor_s == pytest.approx(10.0, abs=0.2)
    assert abs(r.est_lat_dev) < 0.1       # anchor lands beside the vehicle, not behind


def test_search_window_is_configurable():
    route = straight()
    p = Projector(route, w_search=3.5)    # opt back into the narrow window
    p.step(0.0, 0.0, 0.0)
    r = p.step(10.0, 0.0, 0.0)
    assert r.cursor_s == pytest.approx(3.5, abs=0.2)   # capped by the narrow window
