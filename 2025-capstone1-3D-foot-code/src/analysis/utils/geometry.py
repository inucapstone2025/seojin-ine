"""Geometric helpers for plane fitting and frame construction."""

from __future__ import annotations

from typing import Tuple

import numpy as np


def _safe_normal(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v)
    if norm == 0:
        raise ValueError("Zero-length vector cannot be normalized.")
    return v / norm


def fit_plane_ransac(
    points: np.ndarray,
    max_iter: int = 2000,
    dist_thresh: float = 0.008,
) -> Tuple[np.ndarray, float, np.ndarray]:
    """Estimate a plane using RANSAC."""
    if points.shape[0] < 3:
        raise ValueError("At least three points are required to fit a plane.")

    rng = np.random.default_rng()
    best_inliers = None
    best_normal = None
    best_d = None
    num_points = points.shape[0]

    for _ in range(max_iter):
        sample_idx = rng.choice(num_points, size=3, replace=False)
        p0, p1, p2 = points[sample_idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-8:
            continue
        normal /= norm
        d = -np.dot(normal, p0)
        distances = np.abs(points @ normal + d)
        inliers = distances < dist_thresh
        if best_inliers is None or inliers.sum() > best_inliers.sum():
            best_inliers = inliers
            best_normal = normal
            best_d = d

    if best_inliers is None:
        raise RuntimeError("Failed to fit plane to the provided point cloud.")

    inlier_points = points[best_inliers]
    centroid = inlier_points.mean(axis=0)
    cov = inlier_points - centroid
    _, _, vh = np.linalg.svd(cov, full_matrices=False)
    normal = vh[-1]
    normal = _safe_normal(normal)
    d = -np.dot(normal, centroid)

    if np.dot(normal, best_normal) < 0:
        normal = -normal
        d = -d

    distances = np.abs(points @ normal + d)
    inliers = distances < dist_thresh
    return normal, d, inliers


def project_points_to_plane_frame(
    points: np.ndarray,
    normal: np.ndarray,
    offset: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Project points to the plane frame aligned with the fitted plane."""
    normal = _safe_normal(normal)
    origin = -offset * normal
    tmp = np.array([1.0, 0.0, 0.0])
    if np.allclose(np.abs(np.dot(tmp, normal)), 1.0, atol=1e-3):
        tmp = np.array([0.0, 1.0, 0.0])
    u = _safe_normal(np.cross(normal, tmp))
    v = np.cross(normal, u)
    R = np.vstack([u, v, normal])
    shifted = points - origin
    plane_coords = (R @ shifted.T).T
    plane_coords[:, 2] = 0.0

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = -R @ origin
    return plane_coords, T


def world_to_foot_transform(
    uX: np.ndarray,
    uY: np.ndarray,
    origin_world: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return rotation matrices mapping between world and foot frames."""
    del origin_world
    uX = _safe_normal(uX)
    uY = _safe_normal(uY)
    uZ = np.cross(uX, uY)
    uZ = _safe_normal(uZ)
    if np.dot(np.cross(uX, uY), uZ) < 0:
        uY = -uY
        uZ = np.cross(uX, uY)
        uZ = _safe_normal(uZ)
    R_foot_to_world = np.column_stack([uX, uY, uZ])
    R_world_to_foot = R_foot_to_world.T
    return R_world_to_foot, R_foot_to_world
