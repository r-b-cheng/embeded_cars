"""Versioned JSON maps, polygon geometry and lossless occupancy import."""
from __future__ import annotations
import copy
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from .geometry import EPS, inside_polygon, point_edge_distances, swept_distance, ray_distance, cone_distance


def finite_values(values, size, label):
    a = np.asarray(values, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f"{label}: expected {size} finite numbers")
    return a


class WorldMap:
    """Read-only geometry. Bounds are physical closed walls, not open space."""
    def __init__(self, specification: dict):
        self._spec = copy.deepcopy(specification)
        s = self._spec
        if type(s.get("version")) is not int or s.get("version") != 1 or s.get("units", "m") != "m":
            raise ValueError("map requires version=1 and units='m'")
        allowed = {"version", "units", "name", "bounds", "polygons", "walls", "occupancy", "metadata"}
        if set(s) - allowed:
            raise ValueError(f"unknown map keys: {sorted(set(s) - allowed)}")
        self.bounds = tuple(finite_values(s["bounds"], 4, "bounds"))
        x0, y0, x1, y1 = self.bounds
        if x1 <= x0 or y1 <= y0:
            raise ValueError("bounds must be [xmin,ymin,xmax,ymax]")
        self.name = str(s.get("name", "untitled"))
        polygons = list(s.get("polygons", []))
        if "occupancy" in s:
            polygons += self._grid_rectangles(s["occupancy"])
        self.polygons = []
        boundary = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
        edges = [np.stack((boundary, np.roll(boundary, -1, axis=0)), axis=1)]
        for raw in polygons:
            p = np.asarray(raw, dtype=float)
            if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.isfinite(p).all():
                raise ValueError("polygon requires >=3 finite [x,y] vertices")
            if len({tuple(v) for v in p}) != len(p):
                raise ValueError("polygon must not repeat vertices (do not repeat closing vertex)")
            if np.any(p[:, 0] < x0 - EPS) or np.any(p[:, 0] > x1 + EPS) or np.any(p[:, 1] < y0 - EPS) or np.any(p[:, 1] > y1 + EPS):
                raise ValueError("polygon outside map bounds")
            e = np.stack((p, np.roll(p, -1, axis=0)), axis=1)
            area = np.sum(p[:, 0] * np.roll(p[:, 1], -1) - p[:, 1] * np.roll(p[:, 0], -1)) / 2
            if abs(area) <= EPS:
                raise ValueError("zero-area polygon")
            for i in range(len(e)):
                for j in range(i + 1, len(e)):
                    if j == i + 1 or (i == 0 and j == len(e) - 1):
                        continue
                    if swept_distance(e[i, 0], e[i, 1], e[j:j+1]) <= EPS:
                        raise ValueError("self-intersecting polygon")
            p.setflags(write=False)
            self.polygons.append(p)
            edges.append(e)
        self.walls = []
        for raw in s.get("walls", []):
            e = np.asarray(raw, dtype=float)
            if e.shape != (2, 2) or not np.isfinite(e).all() or np.linalg.norm(e[1] - e[0]) <= EPS:
                raise ValueError("wall requires two distinct finite points")
            if np.any(e[:, 0] < x0 - EPS) or np.any(e[:, 0] > x1 + EPS) or np.any(e[:, 1] < y0 - EPS) or np.any(e[:, 1] > y1 + EPS):
                raise ValueError("wall outside bounds")
            e.setflags(write=False)
            self.walls.append(e)
            edges.append(e[None, ...])
        self.edges = np.concatenate(edges)
        self.edges.setflags(write=False)
        self.polygons = tuple(self.polygons)
        self.walls = tuple(self.walls)

    def _grid_rectangles(self, grid):
        allowed = {"resolution_m", "origin_m", "row_order", "cells", "occupied_threshold", "unknown_policy"}
        if set(grid) - allowed:
            raise ValueError("unknown occupancy keys")
        r = float(grid["resolution_m"])
        if not math.isfinite(r) or r <= 0:
            raise ValueError("grid resolution must be positive")
        origin = finite_values(grid.get("origin_m", self.bounds[:2]), 2, "origin_m")
        a = np.asarray(grid["cells"], float)
        if a.ndim != 2 or min(a.shape) == 0 or not np.isfinite(a).all():
            raise ValueError("cells must be a nonempty finite rectangular matrix")
        if np.any((a != -1) & ((a < 0) | (a > 100))) or np.any(a != np.floor(a)):
            raise ValueError("cells: -1 unknown; integers 0..100 occupancy percentage")
        if grid.get("row_order", "bottom_to_top") != "bottom_to_top":
            raise ValueError("row_order must be bottom_to_top; flip image rows on import")
        threshold = grid.get("occupied_threshold", 65)
        if not 1 <= threshold <= 100:
            raise ValueError("occupied_threshold must be in [1,100]")
        policy = grid.get("unknown_policy", "reject")
        if policy not in ("reject", "occupied", "free"):
            raise ValueError("unknown_policy must be reject/occupied/free")
        if policy == "reject" and np.any(a == -1):
            raise ValueError("unknown map cells require an explicit occupied/free simulation policy")
        h, w = a.shape
        expected = [origin[0], origin[1], origin[0] + w*r, origin[1] + h*r]
        if not np.allclose(expected, self.bounds, atol=1e-9, rtol=0):
            raise ValueError("occupancy extent must match map bounds")
        occupied = (a >= threshold) | ((a == -1) & (policy == "occupied"))
        rectangles = []
        # Merge horizontal runs; cell boundaries stay exact (no pixel center shortcuts).
        for row in range(h):
            col = 0
            while col < w:
                if not occupied[row, col]:
                    col += 1
                    continue
                start = col
                while col < w and occupied[row, col]:
                    col += 1
                x0, x1 = origin[0] + start*r, origin[0] + col*r
                y0, y1 = origin[1] + row*r, origin[1] + (row+1)*r
                rectangles.append([[x0,y0],[x1,y0],[x1,y1],[x0,y1]])
        return rectangles

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            return cls(json.load(f))

    @classmethod
    def from_occupancy(cls, cells, resolution_m, origin_m=(0, 0), *, unknown_policy="reject", name="imported"):
        a = np.asarray(cells)
        if a.ndim != 2:
            raise ValueError("occupancy must be 2D")
        h, w = a.shape
        x, y = origin_m
        return cls({"version":1, "units":"m", "name":name,
                    "bounds":[x,y,x+w*resolution_m,y+h*resolution_m],
                    "occupancy":{"resolution_m":resolution_m,"origin_m":list(origin_m),
                                 "row_order":"bottom_to_top","unknown_policy":unknown_policy,
                                 "occupied_threshold":65,"cells":a.tolist()}})

    def to_dict(self):
        return copy.deepcopy(self._spec)

    def save(self, path):
        Path(path).write_text(json.dumps(self._spec, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")

    @property
    def sha256(self):
        return hashlib.sha256(json.dumps(self._spec, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()

    def clearance(self, point):
        p = finite_values(point, 2, "point")
        x0,y0,x1,y1 = self.bounds
        if not (x0 < p[0] < x1 and y0 < p[1] < y1) or any(inside_polygon(p, poly) for poly in self.polygons):
            return 0.0
        return float(point_edge_distances(p, self.edges).min())

    def is_free(self, point, radius_m=0.0):
        return self.clearance(point) > radius_m + EPS

    def swept_free(self, p, q, radius_m):
        return self.is_free(p, radius_m) and self.is_free(q, radius_m) and swept_distance(p, q, self.edges) > radius_m + EPS

    def range(self, point, angle_rad, max_range_m, beam_width_rad=0.0):
        p = finite_values(point, 2, "range origin")
        if not all(math.isfinite(v) for v in [angle_rad,max_range_m,beam_width_rad]) or max_range_m <= 0 or not 0 <= beam_width_rad < math.pi:
            raise ValueError("invalid range query")
        if not self.is_free(p):
            return 0.0
        return cone_distance(p, angle_rad, beam_width_rad, self.edges, max_range_m)
