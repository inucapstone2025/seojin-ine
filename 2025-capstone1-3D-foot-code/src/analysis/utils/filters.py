"""Point cloud filtering utilities with Open3D/ckDTree fallbacks."""

from __future__ import annotations

import numpy as np

try:
    import open3d as o3d

    _HAS_O3D = True
except ImportError:  # pragma: no cover - optional dependency
    o3d = None
    _HAS_O3D = False

try:
    from scipy.spatial import cKDTree

    _HAS_CKDTREE = True
except ImportError:  # pragma: no cover - optional dependency
    cKDTree = None
    _HAS_CKDTREE = False


def _pcd_from_numpy(points3d: np.ndarray):
    """Create an Open3D PointCloud from an Nx3 numpy array."""
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points3d)
    return cloud


def _query_knn(tree: "cKDTree", pts: np.ndarray, k: int) -> np.ndarray:
    """Query distances to the k nearest neighbours (excluding self)."""
    if k <= 1:
        raise ValueError("k must be greater than 1 for kNN queries.")
    try:
        distances, _ = tree.query(pts, k=k, workers=-1)  # SciPy >=1.8
    except TypeError:  # pragma: no cover - fallback for older SciPy
        distances, _ = tree.query(pts, k=k, n_jobs=-1)
    if distances.ndim == 1:
        distances = distances[:, np.newaxis]
    return distances[:, 1:]  # discard self distance


def statistical_outlier_removal(
    points3d: np.ndarray,
    nb_neighbors: int = 12,
    std_ratio: float = 3.0,
) -> np.ndarray:
    """Remove statistical outliers using mean distance to nearest neighbours."""
    if points3d.shape[0] == 0:
        return points3d

    if _HAS_O3D:
        pcd = _pcd_from_numpy(points3d)
        cleaned, _ = pcd.remove_statistical_outlier(nb_neighbors=max(nb_neighbors, 1), std_ratio=std_ratio)
        return np.asarray(cleaned.points)

    nb_neighbors = min(nb_neighbors, max(points3d.shape[0] - 1, 0))
    if nb_neighbors <= 0:
        return points3d

    if _HAS_CKDTREE:
        tree = cKDTree(points3d)
        neighbour_dists = _query_knn(tree, points3d, k=nb_neighbors + 1)
    else:  # Fallback: dense O(N^2)
        diffs = points3d[:, None, :] - points3d[None, :, :]
        dists = np.linalg.norm(diffs, axis=-1)
        np.fill_diagonal(dists, np.inf)
        idx = np.argpartition(dists, nb_neighbors, axis=1)[:, :nb_neighbors]
        neighbour_dists = np.take_along_axis(dists, idx, axis=1)

    mean_dists = neighbour_dists.mean(axis=1)
    mean = mean_dists.mean()
    std = mean_dists.std()
    threshold = mean + std_ratio * std
    mask = mean_dists <= threshold
    return points3d[mask]


def radius_outlier_removal(
    points3d: np.ndarray,
    radius: float = 0.01,
    min_neighbors: int = 8,
) -> np.ndarray:
    """Remove points that do not have enough neighbours within a given radius."""
    if points3d.shape[0] == 0:
        return points3d

    if _HAS_O3D:
        pcd = _pcd_from_numpy(points3d)
        cleaned, _ = pcd.remove_radius_outlier(nb_points=max(min_neighbors, 1), radius=radius)
        return np.asarray(cleaned.points)

    if _HAS_CKDTREE:
        tree = cKDTree(points3d)
        neighbour_lists = tree.query_ball_point(points3d, r=radius, workers=-1)
        neighbours = np.fromiter((len(lst) - 1 for lst in neighbour_lists), dtype=int, count=len(neighbour_lists))
    else:  # Fallback: dense O(N^2)
        diffs = points3d[:, None, :] - points3d[None, :, :]
        dists = np.linalg.norm(diffs, axis=-1)
        neighbours = (dists <= radius).sum(axis=1) - 1

    mask = neighbours >= min_neighbors
    if not mask.any():
        return points3d
    return points3d[mask]
