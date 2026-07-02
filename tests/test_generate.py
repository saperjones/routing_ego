from parking_proj.scenarios import build_scenarios
from parking_proj.generate import build_case_dict


def test_case_dict_schema_and_pass():
    scen = {s.case_id: s for s in build_scenarios()}
    case = build_case_dict(scen["E_low"])
    assert case["case_id"] == "E_low"
    assert "route" in case and "waypoints" in case["route"]
    assert case["route"]["waypoint_labels"] == [1, 2, 3, 4]
    assert len(case["frames"]) > 0
    f0 = case["frames"][0]
    for key in ("t", "true_pose", "meas_pose", "cursor_s", "matched_seg",
                "est_lat_dev", "true_lat_dev", "end_flag", "gt_seg", "gt_s"):
        assert key in f0
    v = case["verdict"]
    assert v["backward_jumps"] == 0
    assert v["dropouts"] == 0
    assert v["mismatches"] <= 3
    assert v["passed"] is True


def test_all_cases_pass_branch_and_monotonic():
    for s in build_scenarios():
        case = build_case_dict(s)
        v = case["verdict"]
        assert v["backward_jumps"] == 0, s.case_id
        assert v["mismatches"] <= 3, (s.case_id, v["mismatches"])
