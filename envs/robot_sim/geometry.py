"""Analytic planar geometry. No Shapely dependency (it is the independent test oracle)."""
from __future__ import annotations
import math
import numpy as np

EPS = 1e-10


def cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def point_edge_distances(p, edges):
    """Distance from one point to each closed line segment."""
    p = np.asarray(p, dtype=float)
    a, v = edges[:, 0], edges[:, 1] - edges[:, 0]
    t = np.clip(np.sum((p - a) * v, axis=1) / np.sum(v * v, axis=1), 0, 1)
    return np.linalg.norm(p - (a + t[:, None] * v), axis=1)


def inside_polygon(p, polygon) -> bool:
    """Even-odd rule; callers separately handle boundary distance."""
    x, y = p
    a = np.asarray(polygon)
    b = np.roll(a, -1, axis=0)
    crosses = (a[:, 1] > y) != (b[:, 1] > y)
    ii = np.flatnonzero(crosses)
    if not len(ii):
        return False
    xx = a[ii, 0] + (y - a[ii, 1]) * (b[ii, 0] - a[ii, 0]) / (b[ii, 1] - a[ii, 1])
    return bool(np.count_nonzero(x < xx) % 2)


def ray_distance(p, angle: float, edges, max_range: float) -> float:
    """Nearest intersection on a closed segment; infinity denotes no hit in range."""
    p = np.asarray(p, dtype=float)
    u = np.array([math.cos(angle), math.sin(angle)])
    a, v = edges[:, 0], edges[:, 1] - edges[:, 0]
    w = a - p
    den = cross(u, v)
    regular = np.abs(den) > EPS
    t = np.full(len(edges), np.inf)
    s = np.zeros(len(edges))
    t[regular] = cross(w[regular], v[regular]) / den[regular]
    s[regular] = cross(w[regular], u) / den[regular]
    good = regular & (t >= -EPS) & (s >= -EPS) & (s <= 1 + EPS)
    t[~good] = np.inf
    collinear = (~regular) & (np.abs(cross(w, u)) <= EPS)
    if np.any(collinear):
        projections = (edges[collinear] - p) @ u
        lo, hi = projections.min(axis=1), projections.max(axis=1)
        t[collinear] = np.where(hi >= -EPS, np.maximum(lo, 0), np.inf)
    distance = max(0.0, float(t.min()))
    return distance if distance <= max_range + EPS else math.inf


def cone_distance(p, angle: float, width: float, edges, max_range: float) -> float:
    """Exact nearest reflector in a planar sector, not a sparse fan approximation.

    A segment's constrained nearest point is either its perpendicular projection,
    an endpoint, or an intersection with a sector boundary. Acoustic strength,
    specularity, multipath and material absorption are deliberately not modeled.
    """
    if width <= EPS:
        return ray_distance(p, angle, edges, max_range)
    p = np.asarray(p, dtype=float)
    a, v = edges[:, 0], edges[:, 1] - edges[:, 0]
    f = np.clip(np.sum((p - a) * v, axis=1) / np.sum(v * v, axis=1), 0, 1)
    candidates = np.concatenate((edges.reshape(-1, 2), a + f[:, None] * v)) - p
    bearings = np.arctan2(candidates[:, 1], candidates[:, 0])
    delta = (bearings - angle + math.pi) % (2 * math.pi) - math.pi
    in_sector = np.abs(delta) <= width / 2 + EPS
    norms = np.linalg.norm(candidates, axis=1)
    in_sector |= norms <= EPS
    best = float(np.min(norms[in_sector])) if np.any(in_sector) else math.inf
    best = min(best,
               ray_distance(p, angle - width / 2, edges, max_range),
               ray_distance(p, angle + width / 2, edges, max_range))
    return best if best <= max_range + EPS else math.inf


def swept_distance(p, q, edges) -> float:
    """Minimum distance between path segment p--q and all obstacle segments."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    u = q - p
    if float(u @ u) < EPS * EPS:
        return float(point_edge_distances(p, edges).min())
    a, v = edges[:, 0], edges[:, 1] - edges[:, 0]
    w = a - p
    den = cross(u, v)
    regular = np.abs(den) > EPS
    t, s = np.zeros(len(edges)), np.zeros(len(edges))
    t[regular] = cross(w[regular], v[regular]) / den[regular]
    s[regular] = cross(w[regular], u) / den[regular]
    if np.any(regular & (t >= -EPS) & (t <= 1 + EPS) & (s >= -EPS) & (s <= 1 + EPS)):
        return 0.0
    endpoints = edges.reshape(-1, 2)
    f = np.clip(((endpoints - p) @ u) / (u @ u), 0, 1)
    d = np.linalg.norm(endpoints - (p + f[:, None] * u), axis=1).min()
    return float(min(d, point_edge_distances(p, edges).min(), point_edge_distances(q, edges).min()))


def integrate(x: float, y: float, yaw: float, v: float, omega: float, dt: float):
    """Exact constant-twist SE(2) integration, with a continuous omega=0 limit."""
    half = omega * dt / 2
    sinc = 1.0 if abs(half) < 1e-12 else math.sin(half) / half
    ds = v * dt * sinc
    return (x + ds * math.cos(yaw + half), y + ds * math.sin(yaw + half),
            wrap(yaw + 2 * half))
