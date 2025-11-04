"""Convex hull and bounding rectangle utilities."""

from __future__ import annotations

from typing import Dict

import numpy as np


def convex_hull_xy(points_xy: np.ndarray) -> np.ndarray:
    """Compute the 2D convex hull using the Andrew monotone chain algorithm."""
    pts = np.asarray(points_xy, dtype=float)
    if pts.shape[0] <= 1:
        return pts

    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(tuple(p))

    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(tuple(p))

    hull = np.array(lower[:-1] + upper[:-1], dtype=float)
    return hull


def _edge_directions(hull_xy: np.ndarray) -> np.ndarray:
    edges = np.roll(hull_xy, -1, axis=0) - hull_xy
    with np.errstate(invalid="ignore"):
        norms = np.linalg.norm(edges, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    return edges / norms


def min_area_rect(hull_xy: np.ndarray) -> Dict[str, object]:
    """Compute a minimum-area rectangle around a convex hull via rotating calipers."""
    if hull_xy.shape[0] == 0:
        raise ValueError("Hull must contain at least one point.")
    if hull_xy.shape[0] == 1:
        pt = hull_xy[0]
        return {
            "center": pt,
            "width": 0.0,
            "height": 0.0,
            "angle": 0.0,
            "u_long": np.array([1.0, 0.0]),
        }

    hull = np.asarray(hull_xy, dtype=float)
    edges = _edge_directions(hull)

    best_area = np.inf
    best_rect = None
    for direction in edges:
        if np.linalg.norm(direction) < 1e-8:
            continue
        u = direction / np.linalg.norm(direction)
        v = np.array([-u[1], u[0]])

        proj_u = hull @ u
        proj_v = hull @ v
        min_u, max_u = proj_u.min(), proj_u.max()
        min_v, max_v = proj_v.min(), proj_v.max()
        width = max_u - min_u
        height = max_v - min_v
        area = width * height
        if area < best_area:
            center = (u * (min_u + width / 2.0)) + (v * (min_v + height / 2.0))
            angle = np.arctan2(u[1], u[0])
            best_area = area
            best_rect = {
                "center": center,
                "width": width,
                "height": height,
                "angle": angle,
                "u_long": u,
            }

    if best_rect is None:
        raise RuntimeError("Failed to compute minimum area rectangle.")

    return best_rect
