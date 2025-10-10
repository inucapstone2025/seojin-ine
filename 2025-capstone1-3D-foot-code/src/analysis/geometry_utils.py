"""Geometry helpers for foot measurements.

This module centralises the logic to estimate the intrinsic foot axes from a
point cloud and to switch between the world coordinate frame and the derived
foot frame. All helpers operate on numpy arrays for easy reuse.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class FootAxes:
    """Collects the orthonormal basis aligned with the foot."""

    origin: np.ndarray
    foot_axis: np.ndarray
    width_axis: np.ndarray
    vertical_axis: np.ndarray
    toe_point: np.ndarray
    rotation: np.ndarray

    def as_tuple(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.foot_axis, self.width_axis, self.vertical_axis


def _ensure_unit(vector: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm == 0:
        raise ValueError("Cannot normalise zero-length vector")
    return vector / norm


def compute_foot_axes(points: np.ndarray) -> FootAxes:
    """Estimate heel/toe and principal axes from a point set."""

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be of shape (N, 3)")

    centroid = points.mean(axis=0)
    centered = points - centroid

    cov = np.cov(centered, rowvar=False)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    principal_axes = eigvecs[:, order]

    foot_axis = _ensure_unit(principal_axes[:, 0])
    width_axis = _ensure_unit(principal_axes[:, 1])
    vertical_axis = _ensure_unit(np.cross(foot_axis, width_axis))
    width_axis = _ensure_unit(np.cross(vertical_axis, foot_axis))

    projections = centered @ foot_axis
    heel_idx = np.argmin(projections)
    toe_idx = np.argmax(projections)

    heel_point = points[heel_idx]
    toe_point = points[toe_idx]

    heel_to_toe = toe_point - heel_point
    if np.dot(heel_to_toe, foot_axis) < 0:
        foot_axis = -foot_axis
        vertical_axis = -vertical_axis
        projections = centered @ foot_axis
        heel_idx = np.argmin(projections)
        toe_idx = np.argmax(projections)
        heel_point = points[heel_idx]
        toe_point = points[toe_idx]

    rotation = np.column_stack((foot_axis, width_axis, vertical_axis))

    return FootAxes(
        origin=heel_point,
        foot_axis=foot_axis,
        width_axis=width_axis,
        vertical_axis=vertical_axis,
        toe_point=toe_point,
        rotation=rotation,
    )


def align_points_to_axes(points: np.ndarray, axes: FootAxes) -> np.ndarray:
    """Express *points* in the foot coordinate frame."""

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be of shape (N, 3)")

    shifted = points - axes.origin
    return shifted @ axes.rotation


def restore_points_from_axes(aligned_points: np.ndarray, axes: FootAxes) -> np.ndarray:
    """Map points from the foot frame back to the original frame."""

    if aligned_points.ndim != 2 or aligned_points.shape[1] != 3:
        raise ValueError("aligned_points must be of shape (N, 3)")

    return aligned_points @ axes.rotation.T + axes.origin


def project_point_to_axis(point: np.ndarray, axis_origin: np.ndarray, axis: np.ndarray) -> float:
    """Return the signed distance of *point* from *axis_origin* along *axis*."""

    axis = _ensure_unit(axis)
    return float(np.dot(point - axis_origin, axis))


def distance_to_plane(point: np.ndarray, plane_point: np.ndarray, plane_normal: np.ndarray) -> float:
    """Signed distance of point from plane defined by point+normal."""

    plane_normal = _ensure_unit(plane_normal)
    return float(np.dot(point - plane_point, plane_normal))
