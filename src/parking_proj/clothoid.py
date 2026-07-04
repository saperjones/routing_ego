"""Clothoid (Euler spiral) corner: curvature ramps linearly, so the curve is
curvature-continuous (no jump at entry) — a smoother, more human-like turn than
a circular arc, while peak curvature is still 1/radius (drivable).

Computed by integrating the linear-curvature profile at a fixed internal step
(equivalent to Fresnel integrals but simpler and bit-identical across the
Python and JavaScript ports — no lookup table, no convergence loop).
"""
import math

INTERNAL_DS = 0.1


def clothoid_corner(delta, radius, transition, internal_ds=INTERNAL_DS):
    """Canonical symmetric clothoid corner turning LEFT by `delta` (radians, > 0).

    Curvature profile over arc length: 0 -> 1/radius (spiral in) over
    `transition`, an optional constant-1/radius arc, then 1/radius -> 0 (spiral
    out). If the two spirals alone already turn >= delta, they are shortened so
    they meet with no middle arc (peak curvature still 1/radius).

    Returns (pts, T): `pts` is the fine polyline from (0,0) heading +x to heading
    +delta; `T` is the tangent length (distance from the start point to where the
    incoming (+x axis) and outgoing tangent lines intersect).
    """
    if (delta <= 1e-9 or transition <= 1e-9 or radius <= 1e-9
            or abs(math.sin(delta)) < 1e-9):
        return [(0.0, 0.0)], 0.0
    theta_sp = transition / (2.0 * radius)          # heading each full spiral turns
    if 2.0 * theta_sp <= delta:
        lt = transition
        arc_len = radius * (delta - 2.0 * theta_sp)
    else:
        lt = radius * delta                         # each spiral turns delta/2, peak kappa = 1/R
        arc_len = 0.0
    total = 2.0 * lt + arc_len
    inv_r = 1.0 / radius

    def kappa(s):
        if s < lt:
            return (s / lt) * inv_r                  # ramp up
        if s <= lt + arc_len:
            return inv_r                              # constant arc
        return ((total - s) / lt) * inv_r            # ramp down

    n = max(2, int(math.ceil(total / internal_ds)))
    h = total / n
    x = y = theta = 0.0
    pts = [(0.0, 0.0)]
    s = 0.0
    for _ in range(n):
        k0 = kappa(s)
        k1 = kappa(s + h)
        theta_mid = theta + 0.5 * k0 * h             # midpoint heading for position
        x += math.cos(theta_mid) * h
        y += math.sin(theta_mid) * h
        theta += 0.5 * (k0 + k1) * h                 # trapezoid heading update
        s += h
        pts.append((x, y))
    xe, ye = pts[-1]
    T = xe - ye / math.tan(delta)
    return pts, T
