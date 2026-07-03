from parking_proj.scenarios import build_scenarios
from parking_proj.generate import build_case_dict


def test_every_case_meets_acceptance():
    failures = []
    for s in build_scenarios():
        v = build_case_dict(s)["verdict"]
        ok = (v["mismatches"] <= 3 and v["backward_jumps"] == 0 and v["dropouts"] == 0)
        if not ok:
            failures.append((s.case_id, v))
    assert not failures, f"cases failing acceptance: {failures}"
