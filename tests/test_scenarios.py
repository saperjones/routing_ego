from parking_proj.scenarios import build_scenarios


def test_fourteen_cases_with_unique_ids():
    sc = build_scenarios()
    assert len(sc) == 14
    ids = [s.case_id for s in sc]
    assert len(set(ids)) == 14


def test_tiers_present():
    sc = {s.case_id: s for s in build_scenarios()}
    assert sc["A_low"].tier == "low"
    assert sc["E_high"].tier == "high"
    assert all(s.route.length > 5.0 for s in sc.values())


def test_x_crossing_labels():
    sc = {s.case_id: s for s in build_scenarios()}
    assert sc["E_low"].route.waypoint_labels == [1, 2, 3, 4]
