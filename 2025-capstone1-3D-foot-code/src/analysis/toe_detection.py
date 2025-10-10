"""Automatic toe landmark detection for foot point clouds."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from sklearn.cluster import DBSCAN

from .geometry_utils import (
    FootAxes,
    align_points_to_axes,
    restore_points_from_axes,
)


@dataclass(frozen=True)
class ToeDetectionParams:
    forefoot_slice_depth: float = 0.025
    dbscan_eps: float = 0.011
    dbscan_min_samples: int = 12
    min_cluster_size: int = 80


@dataclass(frozen=True)
class ToeDetectionResult:
    big_toe_tip: np.ndarray
    second_toe_tip: np.ndarray
    big_toe_tip_aligned: np.ndarray
    second_toe_tip_aligned: np.ndarray
    forefoot_points_aligned: np.ndarray
    labels: np.ndarray


def _slice_forefoot_points(aligned_points: np.ndarray, depth: float) -> np.ndarray:
    if aligned_points.size == 0:
        return aligned_points

    max_x = np.max(aligned_points[:, 0])
    return aligned_points[aligned_points[:, 0] >= max_x - depth]


def _cluster_forefoot(points: np.ndarray, params: ToeDetectionParams) -> np.ndarray:
    features = points[:, 1:3]
    db = DBSCAN(eps=params.dbscan_eps, min_samples=params.dbscan_min_samples)
    labels = db.fit_predict(features)
    return labels


def _filter_clusters(points: np.ndarray, labels: np.ndarray, min_size: int) -> np.ndarray:
    filtered = labels.copy()
    unique = [lab for lab in np.unique(labels) if lab != -1]
    for lab in unique:
        if np.sum(labels == lab) < min_size:
            filtered[labels == lab] = -1
    return filtered


def detect_toe_tips(
    points: np.ndarray,
    axes: FootAxes,
    params: Optional[ToeDetectionParams] = None,
) -> Optional[ToeDetectionResult]:
    if params is None:
        params = ToeDetectionParams()

    aligned = align_points_to_axes(points, axes)
    slice_points = _slice_forefoot_points(aligned, params.forefoot_slice_depth)
    if slice_points.shape[0] < params.min_cluster_size:
        return None

    labels = _cluster_forefoot(slice_points, params)
    labels = _filter_clusters(slice_points, labels, params.min_cluster_size)

    candidates = []
    for lab in np.unique(labels):
        if lab == -1:
            continue
        cluster_points = slice_points[labels == lab]
        if cluster_points.shape[0] == 0:
            continue
        tip_aligned = cluster_points[np.argmax(cluster_points[:, 0])]
        candidates.append((cluster_points[:, 1].mean(), tip_aligned))

    if len(candidates) < 2:
        return None

    candidates.sort(key=lambda x: x[0])
    big_toe_tip_aligned = candidates[0][1]
    second_toe_tip_aligned = candidates[1][1]

    big_world = restore_points_from_axes(big_toe_tip_aligned[None, :], axes)[0]
    second_world = restore_points_from_axes(second_toe_tip_aligned[None, :], axes)[0]

    return ToeDetectionResult(
        big_toe_tip=big_world,
        second_toe_tip=second_world,
        big_toe_tip_aligned=big_toe_tip_aligned,
        second_toe_tip_aligned=second_toe_tip_aligned,
        forefoot_points_aligned=slice_points,
        labels=labels,
    )
