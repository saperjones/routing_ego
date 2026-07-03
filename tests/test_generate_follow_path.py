from parking_proj.generate import build_case_dict
from parking_proj.scenarios import build_scenarios


def test_case_dict_emits_follow_path():
    case = build_case_dict(build_scenarios()[0])
    assert case["config"]["follow_ahead"] == 70.0
    assert case["config"]["follow_ds"] == 0.5
    fr = case["frames"][5]
    assert isinstance(fr["follow_path"], list) and len(fr["follow_path"]) > 1
    assert all(len(p) == 2 for p in fr["follow_path"])
    assert fr["lat_shift"] is not None
