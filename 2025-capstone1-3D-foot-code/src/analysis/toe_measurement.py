"""Foot toe measurement helpers."""

from __future__ import annotations

from dataclasses import dataclass

from .geometry_utils import FootAxes, project_point_to_axis
from .toe_detection import ToeDetectionResult


@dataclass(frozen=True)
class ToeMeasurements:
    heel_to_big_toe_mm: float
    heel_to_second_toe_mm: float
    big_toe_from_heel_plane_mm: float
    second_toe_from_heel_plane_mm: float
    foot_length_mm: float

    @property
    def second_minus_big_mm(self) -> float:
        return self.heel_to_second_toe_mm - self.heel_to_big_toe_mm


def compute_toe_measurements(
    axes: FootAxes,
    detection: ToeDetectionResult,
) -> ToeMeasurements:
    heel_projection = project_point_to_axis(axes.origin, axes.origin, axes.foot_axis)
    big_projection = project_point_to_axis(detection.big_toe_tip, axes.origin, axes.foot_axis)
    second_projection = project_point_to_axis(detection.second_toe_tip, axes.origin, axes.foot_axis)
    toe_projection = project_point_to_axis(axes.toe_point, axes.origin, axes.foot_axis)

    scale = 1000.0
    heel_to_big = (big_projection - heel_projection) * scale
    heel_to_second = (second_projection - heel_projection) * scale
    heel_to_toe = (toe_projection - heel_projection) * scale

    return ToeMeasurements(
        heel_to_big_toe_mm=heel_to_big,
        heel_to_second_toe_mm=heel_to_second,
        big_toe_from_heel_plane_mm=heel_to_big,
        second_toe_from_heel_plane_mm=heel_to_second,
        foot_length_mm=heel_to_toe,
    )
