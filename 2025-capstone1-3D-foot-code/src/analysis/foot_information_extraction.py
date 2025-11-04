"""Foot information extraction utilities."""

from __future__ import annotations

from typing import Tuple

import numpy as np

from .common_types import FootFrame, Footprint2D
from .utils.filters import radius_outlier_removal, statistical_outlier_removal
from .utils.geometry import fit_plane_ransac, project_points_to_plane_frame, world_to_foot_transform
from .utils.hull import convex_hull_xy
from .utils.orientation import alpha_shape_or_hull, resolve_length_axis_with_heel, resolve_medial_axis_for_left


def _to_world(R: np.ndarray, t: np.ndarray, coords: np.ndarray) -> np.ndarray:
    return (R.T @ (coords.T - t.reshape(3, 1))).T


def _local_width(y_vals: np.ndarray, x_vals: np.ndarray, center: float, window: float) -> float:
    mask = np.abs(y_vals - center) <= window
    if mask.sum() == 0:
        return float(x_vals.max() - x_vals.min())
    xs = x_vals[mask]
    return float(xs.max() - xs.min()) if xs.size > 0 else 0.0


def extract_foot_information(points3d: np.ndarray, side: str = "L") -> Tuple[FootFrame, Footprint2D]:
    """Extract the foot frame and planar footprint from a raw point cloud."""
    if side.upper() != "L":
        raise ValueError("Only left foot is supported by convention.")
    if points3d.size == 0:
        raise ValueError("Input point cloud is empty.")

    # print("[pipeline]  - starting statistical_outlier_removal")
    filtered = statistical_outlier_removal(points3d)
    # print(f"[pipeline]  - statistical_outlier_removal done -> {filtered.shape[0]} pts")
    # print("[pipeline]  - starting radius_outlier_removal")
    filtered = radius_outlier_removal(filtered)
    # print(f"[pipeline]  - radius_outlier_removal done -> {filtered.shape[0]} pts")
    if filtered.shape[0] < 3:
        raise ValueError("Insufficient points after filtering.")

    # print("[pipeline]  - fitting plane with RANSAC")
    normal, offset, _ = fit_plane_ransac(filtered)
    # print(f"[pipeline]  - plane fit complete normal={normal}, offset={offset}")
    distances = filtered @ normal + offset
    abs_dist = np.abs(distances)
    threshold = np.percentile(abs_dist, 30)
    slice_mask = abs_dist <= max(threshold, 1e-4)
    plantar_slice = filtered[slice_mask]
    if plantar_slice.shape[0] < 3:
        plantar_slice = filtered

    # print(f"[pipeline]  - projecting {plantar_slice.shape[0]} points to plane")
    plane_coords, T_world_to_plane = project_points_to_plane_frame(plantar_slice, normal, offset)
    R_plane = T_world_to_plane[:3, :3]
    t_plane = T_world_to_plane[:3, 3]
    footprint_xy_plane = plane_coords[:, :2]

    # print("[pipeline]  - extracting outline via alpha_shape_or_hull")
    outline = alpha_shape_or_hull(footprint_xy_plane)
    if outline.shape[0] < 3:
        outline = convex_hull_xy(footprint_xy_plane)
    # print(f"[pipeline]  - outline has {outline.shape[0]} vertices")

    uY = resolve_length_axis_with_heel(outline, footprint_xy_plane)
    uX = resolve_medial_axis_for_left(uY, footprint_xy_plane)
    # print(f"[pipeline]  - resolved axes uX={uX}, uY={uY}")

    x_coords = footprint_xy_plane @ uX
    y_coords = footprint_xy_plane @ uY

    # Heuristic sanity check: heel region should be narrower than forefoot.
    span = y_coords.max() - y_coords.min()
    footprint_xy = np.column_stack([x_coords, y_coords])
    if span > 1e-6:
        window = span * 0.15
        width_min = _local_width(y_coords, x_coords, y_coords.min(), window)
        width_max = _local_width(y_coords, x_coords, y_coords.max(), window)
        # print(f"[pipeline]  - width@minY ≈ {width_min:.2f}, width@maxY ≈ {width_max:.2f}")
        if width_min > width_max:
            # print("[pipeline]  - detected inverted length axis; flipping orientation")
            uY = -uY
            uX = resolve_medial_axis_for_left(uY, footprint_xy_plane)
            x_coords = footprint_xy_plane @ uX
            y_coords = footprint_xy_plane @ uY
            footprint_xy = np.column_stack([x_coords, y_coords])

    # Ensure medial axis (+X) aligns with hallux (positive for forefoot mean).
    forefoot_mask = y_coords >= (y_coords.max() - span * 0.2) if span > 1e-6 else np.ones_like(y_coords, dtype=bool)
    if forefoot_mask.sum() > 0:
        medial_mean = x_coords[forefoot_mask].mean()
        # print(f"[pipeline]  - forefoot medial mean {medial_mean:.2f}")
        if medial_mean > 0:
            # print("[pipeline]  - flipping medial axis to place hallux on negative X")
            uX = -uX
            x_coords = -x_coords
            footprint_xy[:, 0] = x_coords

    heel_idx = np.argmin(y_coords)
    heel_xy = footprint_xy[heel_idx]
    footprint_xy = footprint_xy - heel_xy
    x_coords = footprint_xy[:, 0]
    y_coords = footprint_xy[:, 1]
    heel_xy = footprint_xy[heel_idx]
    heel_plane = np.array(
        [
            footprint_xy_plane[heel_idx, 0],
            footprint_xy_plane[heel_idx, 1],
            0.0,
        ]
    )
    heel_world = _to_world(R_plane, t_plane, heel_plane.reshape(1, 3))[0]

    uX_world = R_plane.T @ np.array([uX[0], uX[1], 0.0])
    uY_world = R_plane.T @ np.array([uY[0], uY[1], 0.0])

    R_world_to_foot, R_foot_to_world = world_to_foot_transform(uX_world, uY_world, heel_world)

    length_est = y_coords.max() - y_coords.min()
    length_mm = float(length_est * 1000.0)
    # print(f"[pipeline]  - length estimate {length_mm:.2f} mm")

    frame = FootFrame(
        side=side.upper(),
        floor_normal=normal,
        floor_offset=offset,
        R_world_to_foot=R_world_to_foot,
        R_foot_to_world=R_foot_to_world,
        origin_world=heel_world,
        unit="mm",
    )

    footprint_xy_mm = footprint_xy * 1000.0
    heel_xy_mm = heel_xy * 1000.0

    footprint = Footprint2D(
        points_xy=footprint_xy_mm,
        heel_x=float(heel_xy_mm[0]),
        heel_y=float(heel_xy_mm[1]),
        length_est_mm=length_mm,
    )

    # print("[pipeline] extract_foot_information returning")
    return frame, footprint
