"""Utilities for extracting a right-foot 2D footprint and axes.

This module follows the staged pipeline requested for computing a footprint
from a 3D point cloud:

1. Ground-plane alignment via RANSAC.
2. Bottom-layer slicing.
3. Orthogonal projection onto the ground plane.
4. Alpha-shape polygon reconstruction (visualised).
5. Minimum-area rotated rectangle.
6. Length/width axes definition and visualisation.

All geometric calculations are performed in metres; reporting converts to
millimetres. Results are tailored for a right foot where the medial side is at
lower width-axis coordinates. The module emits Matplotlib windows for the
requested stages when ``visualize`` is enabled.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from alphashape import alphashape
from shapely.geometry import MultiPoint, MultiPolygon, Polygon
from shapely.ops import unary_union
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN


MM_PER_M = 1000.0


@dataclass(frozen=True)
class FootprintResult:
    """Output bundle for the right-foot footprint extraction."""

    aligned_point_cloud: o3d.geometry.PointCloud
    sliced_points: np.ndarray
    projected_points: np.ndarray
    polygon: Polygon
    rectangle: Polygon
    rectangle_corners: np.ndarray
    length_axis: np.ndarray
    width_axis: np.ndarray
    axis_origin: np.ndarray
    length_mm: float
    width_mm: float
    alpha_used: float
    transformation: np.ndarray


def _rotation_matrix_from_vectors(vec1: np.ndarray, vec2: np.ndarray) -> np.ndarray:
    """Return the rotation matrix that aligns ``vec1`` with ``vec2``."""

    a = vec1 / np.linalg.norm(vec1)
    b = vec2 / np.linalg.norm(vec2)
    cross = np.cross(a, b)
    dot = np.dot(a, b)
    if np.isclose(dot + 1.0, 0.0, atol=1e-8):
        # 180 degree rotation – choose arbitrary orthogonal axis.
        ortho = np.array([1.0, 0.0, 0.0])
        if np.allclose(a, ortho, atol=1e-6):
            ortho = np.array([0.0, 1.0, 0.0])
        axis = np.cross(a, ortho)
        axis /= np.linalg.norm(axis)
        angle = np.pi
    else:
        axis = cross
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-10:
            return np.eye(3)
        axis /= axis_norm
        angle = np.arccos(np.clip(dot, -1.0, 1.0))

    K = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    R = np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)
    return R


def _align_to_ground(
    point_cloud: o3d.geometry.PointCloud,
    distance_threshold_mm: float,
    ransac_n: int,
    num_iterations: int,
) -> Tuple[o3d.geometry.PointCloud, np.ndarray]:
    """Align the point cloud so the detected ground plane is ``Z=0``."""

    threshold_m = distance_threshold_mm / MM_PER_M
    plane_model, inliers = point_cloud.segment_plane(
        distance_threshold=threshold_m,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
    )

    normal = np.asarray(plane_model[:3], dtype=float)
    if normal[2] < 0:
        normal = -normal
        plane_model = [-plane_model[0], -plane_model[1], -plane_model[2], -plane_model[3]]

    R = _rotation_matrix_from_vectors(normal, np.array([0.0, 0.0, 1.0]))
    points = np.asarray(point_cloud.points)
    rotated = (R @ points.T).T

    inlier_rotated = rotated[inliers]
    plane_height = np.mean(inlier_rotated[:, 2]) if inlier_rotated.size else 0.0
    rotated[:, 2] -= plane_height

    aligned_pcd = o3d.geometry.PointCloud()
    aligned_pcd.points = o3d.utility.Vector3dVector(rotated)
    if point_cloud.has_colors():
        aligned_pcd.colors = point_cloud.colors

    transformation = np.eye(4)
    transformation[:3, :3] = R
    translation = np.array([0.0, 0.0, -plane_height])
    transformation[:3, 3] = translation
    return aligned_pcd, transformation


def _slice_bottom_layer(points: np.ndarray, bottom_ratio: float) -> np.ndarray:
    """Return the slice of points occupying the bottom ``bottom_ratio`` height."""

    if points.size == 0:
        raise ValueError("Point cloud is empty after alignment")

    z_min = float(np.min(points[:, 2]))
    z_max = float(np.max(points[:, 2]))
    slice_limit = z_min + bottom_ratio * (z_max - z_min)
    mask = points[:, 2] <= slice_limit + 1e-6
    sliced = points[mask]
    if sliced.shape[0] < 50:
        raise ValueError("Too few points remain after bottom-layer slicing")
    return sliced


def _select_foot_cluster(
    proj_xy: np.ndarray,
    foot_side: str,
    cluster_eps: float,
    cluster_min_samples: int,
) -> np.ndarray:
    if foot_side.lower() not in {"left", "right", "both"}:
        raise ValueError("foot_side must be 'left', 'right', or 'both'")

    if foot_side.lower() == "both":
        return proj_xy

    if proj_xy.shape[0] < cluster_min_samples:
        return proj_xy

    clustering = DBSCAN(eps=cluster_eps, min_samples=cluster_min_samples)
    labels = clustering.fit_predict(proj_xy)
    unique = [lab for lab in np.unique(labels) if lab >= 0]

    if not unique:
        return proj_xy

    clusters = []
    for lab in unique:
        cluster_points = proj_xy[labels == lab]
        if cluster_points.shape[0] < cluster_min_samples:
            continue
        centroid = cluster_points.mean(axis=0)
        area = MultiPoint(cluster_points).convex_hull.area
        clusters.append((cluster_points, centroid, area))

    if not clusters:
        return proj_xy

    if foot_side.lower() == "right":
        selected = max(clusters, key=lambda entry: entry[1][0])
    else:
        selected = min(clusters, key=lambda entry: entry[1][0])

    return selected[0]


def _estimate_alpha(proj_xy: np.ndarray, k: int) -> float:
    tree = cKDTree(proj_xy)
    k = min(k, len(proj_xy) - 1)
    distances, _ = tree.query(proj_xy, k=k + 1)
    mean_dist = np.mean(distances[:, 1:])
    return 1.8 * mean_dist


def _build_alpha_shape_polygon(
    proj_xy: np.ndarray,
    alpha: Optional[float],
    k_neighbors: int,
) -> Tuple[Polygon, float]:
    if alpha is None:
        alpha = _estimate_alpha(proj_xy, k_neighbors)

    geometry = alphashape(proj_xy, alpha)
    if isinstance(geometry, MultiPolygon):
        polygon = max(geometry.geoms, key=lambda g: g.area)
    elif isinstance(geometry, Polygon):
        polygon = geometry
    else:
        union = unary_union(geometry)
        if isinstance(union, Polygon):
            polygon = union
        elif isinstance(union, MultiPolygon):
            polygon = max(union.geoms, key=lambda g: g.area)
        else:
            polygon = MultiPoint(proj_xy).convex_hull

    if polygon.area <= 0.0:
        polygon = MultiPoint(proj_xy).convex_hull

    return polygon, alpha


def _compute_min_area_rectangle(polygon: Polygon) -> Tuple[Polygon, np.ndarray]:
    hull = polygon.convex_hull
    rect = hull.minimum_rotated_rectangle
    coords = np.array(rect.exterior.coords)[:-1]
    if coords.shape[0] != 4:
        # Deduplicate in case of numeric degeneracy.
        unique = np.unique(np.round(coords, decimals=8), axis=0)
        if unique.shape[0] == 4:
            coords = unique
        else:
            raise ValueError("Failed to obtain four unique rectangle corners")
    return rect, coords


def _define_axes(
    rect_corners: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
    edge_vectors = np.roll(rect_corners, -1, axis=0) - rect_corners
    edge_lengths = np.linalg.norm(edge_vectors, axis=1)
    long_edge_idx = int(np.argmax(edge_lengths))
    short_edge_idx = (long_edge_idx + 1) % 4

    e_len = edge_vectors[long_edge_idx] / edge_lengths[long_edge_idx]
    e_len /= np.linalg.norm(e_len)
    e_wid = np.array([-e_len[1], e_len[0]])

    length_m = np.abs(np.dot(edge_vectors[long_edge_idx], e_len))
    width_m = np.abs(np.dot(edge_vectors[short_edge_idx], e_wid))

    length_mm = length_m * MM_PER_M
    width_mm = width_m * MM_PER_M

    axis_origin = rect_corners.mean(axis=0)
    axis_origin_3d = np.array([axis_origin[0], axis_origin[1], 0.0])
    length_axis3d = np.array([e_len[0], e_len[1], 0.0])
    width_axis3d = np.array([e_wid[0], e_wid[1], 0.0])

    return length_axis3d, width_axis3d, axis_origin_3d, length_mm, width_mm


def _visualize_stage4(proj_xy: np.ndarray, polygon: Polygon) -> None:
    fig, ax = plt.subplots()
    ax.set_title("Stage 4: Alpha-shape Footprint")
    ax.scatter(proj_xy[:, 0], proj_xy[:, 1], s=6, c="k", label="Projected points")
    if polygon and not polygon.is_empty:
        x, y = polygon.exterior.xy
        ax.plot(x, y, c="b", linewidth=2, label="Alpha boundary")
        for interior in polygon.interiors:
            xi, yi = interior.xy
            ax.plot(xi, yi, c="b", linestyle="--", linewidth=1)
    ax.set_aspect("equal", adjustable="box")
    ax.legend()
    ax.grid(True, linestyle="--", alpha=0.3)
    plt.show()


def _visualize_stage6(
    proj_xy: np.ndarray,
    polygon: Polygon,
    rect_corners: np.ndarray,
    length_axis: np.ndarray,
    width_axis: np.ndarray,
    axis_origin: np.ndarray,
    length_mm: float,
    width_mm: float,
) -> None:
    fig, ax = plt.subplots()
    ax.set_title("Stage 6: Min-area Rectangle & Axes")
    ax.scatter(proj_xy[:, 0], proj_xy[:, 1], s=6, c="k")
    if polygon and not polygon.is_empty:
        x, y = polygon.exterior.xy
        ax.plot(x, y, c="b", linewidth=2)

    rect_loop = np.vstack([rect_corners, rect_corners[0]])
    ax.plot(rect_loop[:, 0], rect_loop[:, 1], c="r", linewidth=2)

    length_m = length_mm / MM_PER_M
    width_m = width_mm / MM_PER_M

    ax.arrow(
        axis_origin[0],
        axis_origin[1],
        length_axis[0] * length_m,
        length_axis[1] * length_m,
        head_width=0.01,
        head_length=0.02,
        fc="g",
        ec="g",
        linewidth=2,
        length_includes_head=True,
    )

    ax.arrow(
        axis_origin[0],
        axis_origin[1],
        width_axis[0] * width_m,
        width_axis[1] * width_m,
        head_width=0.01,
        head_length=0.02,
        fc="c",
        ec="c",
        linewidth=2,
        length_includes_head=True,
    )

    mid_length = axis_origin[:2] + 0.5 * length_axis[:2] * length_m
    mid_width = axis_origin[:2] + 0.5 * width_axis[:2] * width_m
    ax.text(
        mid_length[0],
        mid_length[1],
        f"L = {length_mm:.1f} mm",
        color="g",
        fontsize=10,
        ha="center",
    )
    ax.text(
        mid_width[0],
        mid_width[1],
        f"W = {width_mm:.1f} mm",
        color="c",
        fontsize=10,
        ha="center",
    )

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.3)
    plt.show()


def extract_right_footprint(
    point_cloud: o3d.geometry.PointCloud,
    *,
    distance_threshold_mm: float = 3.0,
    ransac_n: int = 3,
    num_iterations: int = 2000,
    voxel_size_m: Optional[float] = 0.0025,
    bottom_ratio: float = 0.75,
    alpha: Optional[float] = None,
    alpha_k_neighbors: int = 8,
    foot_side: str = "right",
    cluster_eps: float = 0.04,
    cluster_min_samples: int = 40,
    visualize: bool = True,
) -> FootprintResult:
    """Extract a 2D footprint and axes for a right foot.

    Parameters
    ----------
    point_cloud:
        The input foot point cloud (metres).
    distance_threshold_mm:
        RANSAC plane distance threshold in millimetres.
    ransac_n, num_iterations:
        Plane-fitting parameters forwarded to Open3D.
    voxel_size_m:
        Optional down-sampling voxel size before slicing (metres).
    bottom_ratio:
        Fraction of the foot height retained for footprint analysis.
    alpha:
        Optional alpha parameter for the alpha-shape. If ``None`` the value is
        estimated from local point spacing.
    alpha_k_neighbors:
        Number of neighbours used during alpha estimation.
    visualize:
        When ``True`` the Stage 4 and Stage 6 Matplotlib windows are shown.
    """

    aligned_pcd, transformation = _align_to_ground(
        point_cloud,
        distance_threshold_mm=distance_threshold_mm,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
    )

    working_pcd = aligned_pcd
    if voxel_size_m:
        working_pcd = aligned_pcd.voxel_down_sample(voxel_size_m)
    working_points = np.asarray(working_pcd.points)

    sliced_points = _slice_bottom_layer(working_points, bottom_ratio=bottom_ratio)
    proj_xy = sliced_points[:, :2]
    proj_xy = _select_foot_cluster(
        proj_xy,
        foot_side=foot_side,
        cluster_eps=cluster_eps,
        cluster_min_samples=cluster_min_samples,
    )
    if proj_xy.shape[0] < 4:
        proj_xy = sliced_points[:, :2]

    polygon, alpha_used = _build_alpha_shape_polygon(
        proj_xy, alpha=alpha, k_neighbors=alpha_k_neighbors
    )

    if visualize:
        _visualize_stage4(proj_xy, polygon)

    rectangle, rect_corners = _compute_min_area_rectangle(polygon)
    length_axis, width_axis, axis_origin, length_mm, width_mm = _define_axes(
        rect_corners
    )

    if visualize:
        _visualize_stage6(
            proj_xy,
            polygon,
            rect_corners,
            length_axis,
            width_axis,
            axis_origin,
            length_mm,
            width_mm,
        )

    return FootprintResult(
        aligned_point_cloud=aligned_pcd,
        sliced_points=sliced_points,
        projected_points=proj_xy,
        polygon=polygon,
        rectangle=rectangle,
        rectangle_corners=rect_corners,
        length_axis=length_axis,
        width_axis=width_axis,
        axis_origin=axis_origin,
        length_mm=length_mm,
        width_mm=width_mm,
        alpha_used=alpha_used,
        transformation=transformation,
    )
