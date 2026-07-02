"""Grade algorithm output against simulation ground truth."""
import numpy as np


def true_lat_dev(route, frame) -> float:
    mp = route.point_at_s(frame.gt_s)
    tang = route.tangent_at_s(frame.gt_s)
    normal_left = np.array([-tang[1], tang[0]])
    return float(np.dot(np.array([frame.true_e, frame.true_n]) - mp, normal_left))


def grade_case(route, frames, results) -> dict:
    total = len(frames)
    mismatches = sum(1 for f, r in zip(frames, results)
                     if r is None or r.matched_seg != f.gt_seg)
    dropouts = sum(1 for r in results if r is None)
    backward = 0
    prev = -1.0
    for r in results:
        if r is None:
            continue
        if r.cursor_s < prev - 1e-6:
            backward += 1
        prev = r.cursor_s
    # deviation gap on (near-)straight frames only
    gaps = []
    for f, r in zip(frames, results):
        if r is None:
            continue
        gaps.append(abs(r.est_lat_dev - true_lat_dev(route, f)))
    max_gap = float(max(gaps)) if gaps else 0.0
    passed = (mismatches <= 3) and (backward == 0) and (dropouts == 0)
    return {
        "total_frames": total,
        "correct_branch_frames": total - mismatches,
        "mismatches": mismatches,
        "backward_jumps": backward,
        "dropouts": dropouts,
        "max_dev_gap": round(max_gap, 4),
        "passed": bool(passed),
    }
