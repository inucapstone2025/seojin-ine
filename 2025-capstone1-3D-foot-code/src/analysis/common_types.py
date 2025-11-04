"""Common dataclasses shared across the foot analysis pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np


Array3 = np.ndarray
Matrix3 = np.ndarray


@dataclass
class FootFrame:
    """Pose of the left foot frame with respect to the world frame."""

    side: str
    floor_normal: Array3
    floor_offset: float
    R_world_to_foot: Matrix3
    R_foot_to_world: Matrix3
    origin_world: Array3
    unit: str = "mm"


@dataclass
class Footprint2D:
    """2D footprint projected onto the plantar plane."""

    points_xy: np.ndarray
    heel_x: float
    heel_y: float
    length_est_mm: float


@dataclass
class Landmark2D:
    """Single 2D landmark with an optional confidence score."""

    xy: Tuple[float, float]
    confidence: float = 1.0


@dataclass
class FootLandmarks:
    """Collection of derived 2D landmarks for the left foot."""

    heel_tip: Landmark2D
    toe1_tip: Landmark2D
    toe2_tip: Landmark2D
    toe3_tip: Landmark2D
    mtp1_head: Landmark2D
    instep_point_xy: Tuple[float, float]
    instep_height_mm: float
    instep_cross_xy: Tuple[float, float]
    toe_band_points: Optional[np.ndarray] = None


@dataclass
class FootMetrics:
    """Key scalar metrics derived from the landmarks."""

    delta21: float
    delta31: float
    instep_height: float
    truncated_length: float
    heel_tip_xy: Tuple[float, float]
    quality: Dict[str, float] = field(default_factory=dict)
