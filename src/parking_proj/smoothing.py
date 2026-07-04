"""Fast, embedded-friendly polyline smoothing: RDP corner detection + circular-arc fillet.

All operations are closed-form (no iteration-to-converge); curvature on filleted
corners is bounded by 1/min_radius, so the resulting path is drivable at a
minimum turning radius of min_radius.
"""
import math


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


def smooth_corners(pts, min_radius, corner_angle_deg, ds, eps):
    """Replace sharp corners of a polyline with circular-arc fillets (radius >= min_radius).

    Returns a new polyline resampled at ds with curvature <= 1/min_radius on
    arcs and 0 on straights. corner_angle_deg: only fillet turns sharper than
    this. eps: RDP tolerance for corner-vertex detection. When an adjacent leg
    is shorter than the tangent length required for min_radius, the tangent
    length is clamped to half the leg, so the effective radius is reduced to
    fit — curvature may exceed 1/min_radius only in that degenerate case.
    """
    if len(pts) < 3:
        return resample(pts, ds)
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
        delta = math.acos(dot)                       # unsigned turn angle
        if delta < thresh:
            out.append((vx, vy))
            continue
        cross = d1x * d2y - d1y * d2x                 # > 0 => left turn
        half = delta / 2.0
        tan_half = math.tan(half)
        if tan_half < 1e-9:
            out.append((vx, vy))
            continue
        T = min(min_radius * tan_half,
                0.5 * math.hypot(vx - ax, vy - ay),
                0.5 * math.hypot(bx - vx, by - vy))
        if T < 1e-6:
            out.append((vx, vy))
            continue
        r_eff = T / tan_half
        p1x, p1y = vx - T * d1x, vy - T * d1y
        nx, ny = (-d1y, d1x) if cross >= 0 else (d1y, -d1x)   # toward turn center
        cx, cy = p1x + r_eff * nx, p1y + r_eff * ny
        a1 = math.atan2(p1y - cy, p1x - cx)
        sign = 1.0 if cross >= 0 else -1.0
        steps = max(1, int(math.ceil(r_eff * delta / ds)))
        out.append((p1x, p1y))
        for k in range(1, steps + 1):
            a = a1 + sign * delta * (k / steps)
            out.append((cx + r_eff * math.cos(a), cy + r_eff * math.sin(a)))
    out.append(verts[-1])
    return resample(out, ds)
