"""Fast, embedded-friendly polyline smoothing: RDP corner detection + circular-arc fillet.

All operations are closed-form (no iteration-to-converge); curvature on filleted
corners is bounded by 1/min_radius, so the resulting path is drivable at a
minimum turning radius of min_radius.
"""
import math
from .clothoid import clothoid_corner


def rdp(pts, eps):
    """Ramer-Douglas-Peucker polyline simplification. pts: list of (x, y)."""
    if len(pts) < 3:
        return list(pts)
    x0, y0 = pts[0]
    x1, y1 = pts[-1]
    dx, dy = x1 - x0, y1 - y0
    seg2 = dx * dx + dy * dy
    dmax, idx = -1.0, 0
    for i in range(1, len(pts) - 1):
        px, py = pts[i]
        if seg2 == 0.0:
            d = math.hypot(px - x0, py - y0)
        else:
            t = ((px - x0) * dx + (py - y0) * dy) / seg2
            t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
            d = math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
        if d > dmax:
            dmax, idx = d, i
    if dmax > eps:
        left = rdp(pts[:idx + 1], eps)
        right = rdp(pts[idx:], eps)
        return left[:-1] + right
    return [pts[0], pts[-1]]


def resample(pts, ds):
    """Uniformly resample a polyline at spacing ds (arc length)."""
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    px, py = pts[0]
    acc = 0.0
    for i in range(1, len(pts)):
        qx, qy = pts[i]
        seg = math.hypot(qx - px, qy - py)
        while seg > 0.0 and acc + seg >= ds:
            t = (ds - acc) / seg
            px, py = px + t * (qx - px), py + t * (qy - py)
            out.append((px, py))
            seg = math.hypot(qx - px, qy - py)
            acc = 0.0
        acc += seg
        px, py = qx, qy
    if out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


def _unit(dx, dy):
    n = math.hypot(dx, dy)
    return (0.0, 0.0) if n < 1e-9 else (dx / n, dy / n)


def _gaussian_smooth(pts, sigma, ds):
    """Gaussian low-pass of a polyline (endpoints clamped). sigma in metres;
    ds is the point spacing. Rounds corners AND shifts the turn earlier."""
    if sigma <= 1e-9 or len(pts) < 3:
        return list(pts)
    r = max(1, int(round(3.0 * sigma / ds)))
    w = [math.exp(-((k * ds) ** 2) / (2.0 * sigma * sigma)) for k in range(-r, r + 1)]
    wsum = sum(w)
    w = [x / wsum for x in w]
    n = len(pts)
    out = []
    for i in range(n):
        sx = sy = 0.0
        for idx, k in enumerate(range(-r, r + 1)):
            j = min(max(i + k, 0), n - 1)          # clamp at ends
            sx += pts[j][0] * w[idx]
            sy += pts[j][1] * w[idx]
        out.append((sx, sy))
    return out


def _arc_world(ax, ay, vx, vy, bx, by, min_radius, delta, cross, ds):
    d1x, d1y = _unit(vx - ax, vy - ay)
    d2x, d2y = _unit(bx - vx, by - vy)
    tan_half = math.tan(delta / 2.0)
    if tan_half < 1e-9:
        return [(vx, vy)]
    T = min(min_radius * tan_half, 0.5 * math.hypot(vx - ax, vy - ay),
            0.5 * math.hypot(bx - vx, by - vy))
    if T < 1e-6:
        return [(vx, vy)]
    r_eff = T / tan_half
    p1x, p1y = vx - T * d1x, vy - T * d1y
    nx, ny = (-d1y, d1x) if cross >= 0 else (d1y, -d1x)
    cx, cy = p1x + r_eff * nx, p1y + r_eff * ny
    a1 = math.atan2(p1y - cy, p1x - cx)
    sign = 1.0 if cross >= 0 else -1.0
    steps = max(1, int(math.ceil(r_eff * delta / ds)))
    out = [(p1x, p1y)]
    for k in range(1, steps + 1):
        a = a1 + sign * delta * (k / steps)
        out.append((cx + r_eff * math.cos(a), cy + r_eff * math.sin(a)))
    return out


def _clothoid_world(ax, ay, vx, vy, bx, by, min_radius, transition, delta, cross):
    d1x, d1y = _unit(vx - ax, vy - ay)
    clamp = 0.5 * min(math.hypot(vx - ax, vy - ay), math.hypot(bx - vx, by - vy))
    for factor in (1.0, 0.5, 0.25):
        local, T = clothoid_corner(delta, min_radius, transition * factor)
        if 0.0 < T <= clamp:
            p1x, p1y = vx - T * d1x, vy - T * d1y
            nx, ny = (-d1y, d1x) if cross >= 0 else (d1y, -d1x)   # +y (left) -> turn side
            return [(p1x + lx * d1x + ly * nx, p1y + lx * d1y + ly * ny) for lx, ly in local]
    return None                                                    # doesn't fit -> caller uses arc


def smooth_corners(pts, min_radius, corner_angle_deg, ds, eps,
                   corner_style="arc", transition=3.0):
    """Replace sharp corners with fillets. corner_style="arc" uses a circular arc
    (curvature bounded by 1/min_radius, but jumps at entry); "clothoid" uses a
    curvature-continuous clothoid of the given transition length, falling back to
    the arc when the clothoid cannot fit the adjacent legs. Output is resampled at
    ds. Only corners sharper than corner_angle_deg are filleted; eps is the RDP
    corner-detection tolerance."""
    if len(pts) < 3:
        return resample(pts, ds)
    if corner_style == "driver":
        # "driver-like": Gaussian low-pass of the whole route. Unlike a fillet
        # (which rounds AT the vertex), this makes the path start bending well
        # BEFORE the corner and be very smooth — how a human anticipates a turn.
        # `transition` is the anticipation sigma (m): larger => earlier + smoother.
        return _gaussian_smooth(resample(pts, ds), transition, ds)
    verts = rdp(pts, eps)
    if len(verts) < 3:
        return resample(verts, ds)
    thresh = math.radians(corner_angle_deg)
    out = [verts[0]]
    for i in range(1, len(verts) - 1):
        ax, ay = verts[i - 1]
        vx, vy = verts[i]
        bx, by = verts[i + 1]
        d1x, d1y = _unit(vx - ax, vy - ay)
        d2x, d2y = _unit(bx - vx, by - vy)
        dot = max(-1.0, min(1.0, d1x * d2x + d1y * d2y))
        delta = math.acos(dot)
        if delta < thresh:
            out.append((vx, vy))
            continue
        cross = d1x * d2y - d1y * d2x
        corner = None
        if corner_style == "clothoid":
            corner = _clothoid_world(ax, ay, vx, vy, bx, by, min_radius, transition, delta, cross)
        if corner is None:
            corner = _arc_world(ax, ay, vx, vy, bx, by, min_radius, delta, cross, ds)
        out.extend(corner)
    out.append(verts[-1])
    return resample(out, ds)
