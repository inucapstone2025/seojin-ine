"""Landmark extraction for the left foot."""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from .common_types import FootFrame, FootLandmarks, Footprint2D, Landmark2D

GAMMA_MEDIAL = 0.40
GAMMA_LATERAL = 0.20
TOE_BAND_SEQUENCE_MM = (6.0, 10.0, 15.0)
TOE_PERCENTILE = 98.0
TOE_MIN_RATIO = 0.02  # ensure we keep enough points


def _toe_candidates(points_xy: np.ndarray, heel_xy: np.ndarray) -> Tuple[np.ndarray, Optional[float]]:
    """Return toe candidates using an interpolated forefoot envelope."""
    del heel_xy  # heel retained for signature compatibility
    if points_xy.size == 0:
        return points_xy, None

    x = points_xy[:, 0]
    y = points_xy[:, 1]
    num_bins = max(6, min(18, int(np.ceil(np.sqrt(points_xy.shape[0] / 40))) + 6))
    edges = np.linspace(x.min(), x.max(), num_bins + 1)
    centers = 0.5 * (edges[:-1] + edges[1:])

    env_x = []
    env_y = []
    per_bin_top: list[np.ndarray] = []
    for i in range(num_bins):
        mask = (x >= edges[i]) & (x < edges[i + 1] if i < num_bins - 1 else x <= edges[i + 1])
        bin_pts = points_xy[mask]
        if bin_pts.shape[0] == 0:
            per_bin_top.append(np.empty((0, 2)))
            continue
        per_bin_top.append(bin_pts[np.argsort(bin_pts[:, 1])[-min(2, bin_pts.shape[0]):]])
        perc = np.percentile(bin_pts[:, 1], TOE_PERCENTILE)
        env_x.append(centers[i])
        env_y.append(perc)

    if len(env_x) >= 2:
        env_x = np.array(env_x)
        env_y = np.array(env_y)
        order = np.argsort(env_x)
        env_x = env_x[order]
        env_y = env_y[order]
        interp_y = np.interp(x, env_x, env_y)
        for band_mm in TOE_BAND_SEQUENCE_MM:
            mask = np.abs(y - interp_y) <= band_mm
            count = mask.sum()
            ratio = count / float(points_xy.shape[0])
            # print(
            #     f"[pipeline]  - envelope band {band_mm} mm selected {count} points (ratio {ratio:.3f})"
            # )
            if count >= 3 and ratio >= TOE_MIN_RATIO:
                return points_xy[mask], band_mm

    # Fallback: aggregate per-bin top points
    fallback_pts = np.vstack([pts for pts in per_bin_top if pts.size > 0]) if per_bin_top else np.empty((0, 2))
    if fallback_pts.shape[0] >= 3:
        # print("[pipeline]  - fallback to per-bin top points")
        return fallback_pts, None

    # Ultimate fallback: global top-3 by Y
    top_idx = np.argsort(y)[-3:]
    # print("[pipeline]  - fallback to top-3 Y points for toes")
    return points_xy[top_idx], None


def extract_landmarks(
    foot2d: Footprint2D,
    points3d: np.ndarray,
    frame: FootFrame,
) -> FootLandmarks:
    """Extract anatomical landmarks for the left foot."""
    # print("[pipeline]  - landmark extraction started")
    points_xy = foot2d.points_xy
    x_coords = points_xy[:, 0]
    y_coords = points_xy[:, 1]
    heel_idx = np.argmin(y_coords)
    heel_xy = np.array([x_coords[heel_idx], y_coords[heel_idx]])
    # print(
    #     f"[pipeline]  - heel at {heel_xy}, x-range [{x_coords.min():.2f}, {x_coords.max():.2f}]"
    # )

    candidates, _ = _toe_candidates(points_xy, heel_xy)
    cand_x = candidates[:, 0]
    cand_y = candidates[:, 1]

    heel_x = heel_xy[0]
    idx_toe2 = np.argmin(np.abs(cand_x - heel_x))
    toe2_xy = np.array([cand_x[idx_toe2], cand_y[idx_toe2]])

    width = cand_x.max() - cand_x.min()
    target_toe1_x = toe2_xy[0] + GAMMA_MEDIAL * width
    target_toe3_x = toe2_xy[0] - GAMMA_LATERAL * width
    # print(
    #     f"[pipeline]  - toe width {width:.2f}, targets toe1={target_toe1_x:.2f}, toe3={target_toe3_x:.2f}"
    # )

    mask = np.ones_like(cand_x, dtype=bool)
    mask[idx_toe2] = False
    remaining = candidates[mask]

    if remaining.shape[0] < 2:
        # print("[pipeline]  - insufficient toe candidates after Toe2, using fallback ordering")
        order = np.argsort(cand_x)
        toe1_xy = candidates[order[0]]
        toe3_xy = candidates[order[-1]]
    else:
        idx_toe1 = np.argmin(np.abs(remaining[:, 0] - target_toe1_x))
        idx_toe3 = np.argmin(np.abs(remaining[:, 0] - target_toe3_x))
        if idx_toe1 == idx_toe3:
            # print("[pipeline]  - toe snapping collision; falling back to min/max X")
            order = np.argsort(remaining[:, 0])
            toe1_xy = remaining[order[0]]
            toe3_xy = remaining[order[-1]]
        else:
            toe1_xy = remaining[idx_toe1]
            toe3_xy = remaining[idx_toe3]
    # if toe1_xy[0] > toe3_xy[0]:
    #     print("[pipeline]  - enforcing Toe1 medial ordering via swap")
    #     toe1_xy, toe3_xy = toe3_xy, toe1_xy

    length_y = y_coords.max() - y_coords.min()
    target_instep_y = y_coords.min() + 0.55 * length_y
    instep_idx = np.argmin(np.abs(y_coords - target_instep_y))
    instep_xy = np.array([x_coords[instep_idx], y_coords[instep_idx]])

    R_w2f = frame.R_world_to_foot
    points_local = (R_w2f @ (points3d - frame.origin_world).T).T
    points_local_mm = points_local * 1000.0
    xy_local = points_local_mm[:, :2]
    z_local = points_local_mm[:, 2]

    target_cross = np.array([heel_xy[0], target_instep_y])
    distances = np.linalg.norm(xy_local - target_cross, axis=1)
    mask = distances <= 10.0
    instep_height = 0.0
    instep_point_mm = instep_xy
    if mask.any():
        band = mask & (distances >= 5.0)
        if not band.any():
            band = mask
        idx = np.argmax(z_local[band])
        if z_local[band][idx] > 0:
            instep_height = float(z_local[band][idx])
            instep_point_mm = xy_local[band][idx]
    # print(f"[pipeline]  - instep target y {target_instep_y:.2f}, height {instep_height:.2f}")

    instep_point = np.asarray(instep_point_mm, dtype=float)
    instep_point_tuple = (float(instep_point[0]), float(instep_point[1]))
    instep_cross = (target_cross[0], target_instep_y)

    landmarks = FootLandmarks(
        heel_tip=Landmark2D(tuple(heel_xy), 1.0),
        toe1_tip=Landmark2D(tuple(toe1_xy), 0.9),
        toe2_tip=Landmark2D(tuple(toe2_xy), 0.9),
        toe3_tip=Landmark2D(tuple(toe3_xy), 0.9),
        mtp1_head=Landmark2D((float("nan"), float("nan")), 0.0),
        instep_point_xy=instep_point_tuple,
        instep_height_mm=instep_height,
        instep_cross_xy=instep_cross,
        toe_band_points=candidates,
    )
    # print("[pipeline]  - landmark extraction completed")
    return landmarks
