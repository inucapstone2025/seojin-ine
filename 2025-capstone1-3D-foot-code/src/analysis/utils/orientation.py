"""Orientation helpers for establishing the canonical foot frame."""

from __future__ import annotations

import numpy as np

from .hull import convex_hull_xy, min_area_rect


def alpha_shape_or_hull(points_xy: np.ndarray, alpha=None) -> np.ndarray:
    """Return an outline for the footprint (convex hull placeholder)."""
    del alpha
    return convex_hull_xy(points_xy)


def resolve_length_axis_with_heel(hull_xy: np.ndarray, points_xy: np.ndarray) -> np.ndarray:
    """Resolve the +Y axis (heel to forefoot)."""
    rect = min_area_rect(hull_xy)
    u_long = rect["u_long"]
    projections = points_xy @ u_long
    heel_idx = np.argmin(projections)
    toe_idx = np.argmax(projections)
    heel_point = points_xy[heel_idx]
    toe_point = points_xy[toe_idx]
    direction = toe_point - heel_point
    if np.dot(direction, u_long) < 0:
        u_long = -u_long
    return u_long


def resolve_medial_axis_for_left(uY: np.ndarray, points_xy: np.ndarray) -> np.ndarray:
    """Resolve the +X axis (medial, toward the hallux) for the left foot."""
    uY = uY / np.linalg.norm(uY)
    uX = np.array([-uY[1], uY[0]])
    uX /= np.linalg.norm(uX)

    projections_y = points_xy @ uY
    threshold = np.percentile(projections_y, 75)
    forefoot_mask = projections_y >= threshold
    if forefoot_mask.sum() == 0:
        forefoot_mask = projections_y >= projections_y.mean()
    medial_mean = (points_xy[forefoot_mask] @ uX).mean()
    if medial_mean < 0:
        uX = -uX
    return uX
