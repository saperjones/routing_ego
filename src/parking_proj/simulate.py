"""Two-layer simulation: imperfect tracking + RTK localization error."""
import math
from dataclasses import dataclass
import numpy as np

TIERS = {"low": (0.10, 0.01), "medium": (0.50, 0.03), "high": (1.50, 0.05)}


@dataclass
class SimConfig:
    tier: str
    seed: int
    speed_kmh: float = 8.0
    hz: float = 10.0
    track_sigma: float = 0.15
    track_cap: float = 0.4
    smooth_win: int = 25
    pos_cap: float = 2.0
    ang_cap_deg: float = 0.05


@dataclass
class Frame:
    t: float
    speed: float
    true_e: float
    true_n: float
    true_yaw: float
    meas_e: float
    meas_n: float
    meas_yaw: float
    pitch: float
    roll: float
    gt_s: float
    gt_seg: int


def _gauss_kernel(win):
    win = max(int(win), 1)
    if win % 2 == 0:
        win += 1
    x = np.arange(win) - win // 2
    sig = win / 5.0
    k = np.exp(-0.5 * (x / sig) ** 2)
    return k / k.sum()


def _smooth(a, win):
    k = _gauss_kernel(win)
    # np.convolve(mode="same") returns max(len(a), len(k)) samples, so a
    # kernel wider than the signal (very short routes) yields an output
    # longer than the input. Extract the input-length centered window from
    # the full convolution; identical to mode="same" when len(a) >= len(k).
    full = np.convolve(a, k, mode="full")
    off = (len(k) - 1) // 2
    return full[off:off + len(a)]


def _lowpass_noise(rng, n, win, target_sigma, cap):
    w = rng.standard_normal(n)
    s = _smooth(w, win)
    std = s.std()
    if std > 0:
        s = s * (target_sigma / std)
    return np.clip(s, -cap, cap)


def simulate(route, cfg: SimConfig) -> list[Frame]:
    rng = np.random.default_rng(cfg.seed)
    lat_sigma, ang_sigma_deg = TIERS[cfg.tier]
    speed_mps = cfg.speed_kmh / 3.6
    ds_frame = speed_mps / cfg.hz

    # 1) nominal ordinal walk along planned route -> gt_s per frame (exact)
    n_frames = max(int(route.length / ds_frame) + 1, 2)
    gt_s = np.minimum(np.arange(n_frames) * ds_frame, route.length)
    nom = np.array([route.point_at_s(s) for s in gt_s])          # (F,2)
    gt_seg = np.array([route.segment_at_s(s) for s in gt_s])

    # 2) corner rounding via low-pass on nominal positions (cuts corners inside)
    smooth_e = _smooth(nom[:, 0], cfg.smooth_win)
    smooth_n = _smooth(nom[:, 1], cfg.smooth_win)
    # keep endpoints anchored (convolution 'same' biases ends)
    aw = min(cfg.smooth_win, n_frames // 2)
    if aw > 0:
        smooth_e[:aw] = nom[:aw, 0]
        smooth_n[:aw] = nom[:aw, 1]
        smooth_e[-aw:] = nom[-aw:, 0]
        smooth_n[-aw:] = nom[-aw:, 1]

    # tangent of smoothed centerline -> perpendicular for tracking offset
    dse = np.gradient(smooth_e)
    dsn = np.gradient(smooth_n)
    tnorm = np.hypot(dse, dsn)
    tnorm[tnorm == 0] = 1.0
    perp_e, perp_n = -dsn / tnorm, dse / tnorm

    # 3) tracking lateral offset (smooth, capped), same construction all tiers
    off = _lowpass_noise(rng, n_frames, cfg.smooth_win * 4,
                         cfg.track_sigma, cfg.track_cap)
    true_e = smooth_e + off * perp_e
    true_n = smooth_n + off * perp_n

    # true heading = tangent of final true trajectory
    te = np.gradient(true_e)
    tn = np.gradient(true_n)
    true_yaw = np.arctan2(tn, te)

    # 4) RTK localization error (correlated bias + white), capped to 2 m
    bias_e = _lowpass_noise(rng, n_frames, cfg.smooth_win * 6, lat_sigma, cfg.pos_cap)
    bias_n = _lowpass_noise(rng, n_frames, cfg.smooth_win * 6, lat_sigma, cfg.pos_cap)
    white_e = rng.normal(0, 0.02, n_frames)
    white_n = rng.normal(0, 0.02, n_frames)
    err_e = bias_e + white_e
    err_n = bias_n + white_n
    mag = np.hypot(err_e, err_n)
    scale = np.where(mag > cfg.pos_cap, cfg.pos_cap / np.maximum(mag, 1e-9), 1.0)
    meas_e = true_e + err_e * scale
    meas_n = true_n + err_n * scale

    ang_sigma = math.radians(ang_sigma_deg)
    ang_cap = math.radians(cfg.ang_cap_deg)
    yaw_err = np.clip(_lowpass_noise(rng, n_frames, cfg.smooth_win * 6,
                                     ang_sigma, ang_cap), -ang_cap, ang_cap)
    meas_yaw = true_yaw + yaw_err
    pitch = np.clip(_lowpass_noise(rng, n_frames, cfg.smooth_win * 6,
                                   ang_sigma, ang_cap), -ang_cap, ang_cap)
    roll = np.clip(_lowpass_noise(rng, n_frames, cfg.smooth_win * 6,
                                  ang_sigma, ang_cap), -ang_cap, ang_cap)

    frames = []
    for i in range(n_frames):
        frames.append(Frame(
            t=i / cfg.hz, speed=speed_mps,
            true_e=float(true_e[i]), true_n=float(true_n[i]),
            true_yaw=float(true_yaw[i]),
            meas_e=float(meas_e[i]), meas_n=float(meas_n[i]),
            meas_yaw=float(meas_yaw[i]),
            pitch=float(pitch[i]), roll=float(roll[i]),
            gt_s=float(gt_s[i]), gt_seg=int(gt_seg[i]),
        ))
    return frames
