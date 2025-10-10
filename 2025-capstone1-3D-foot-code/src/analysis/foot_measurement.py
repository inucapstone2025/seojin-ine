from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np
import open3d as o3d

from .geometry_utils import FootAxes, align_points_to_axes, compute_foot_axes
from .toe_detection import ToeDetectionParams, ToeDetectionResult, detect_toe_tips
from .toe_measurement import ToeMeasurements, compute_toe_measurements


@dataclass
class FootMeasurementResult:
    file_name: str
    axes: FootAxes
    aligned_pcd: o3d.geometry.PointCloud
    aabb: o3d.geometry.AxisAlignedBoundingBox
    extent_mm: np.ndarray
    toe_detection: Optional[ToeDetectionResult]
    toe_measurements: Optional[ToeMeasurements]


def measure_foot(ply_path: str, toe_params: Optional[ToeDetectionParams] = None) -> FootMeasurementResult:
    path = Path(ply_path)
    pcd = o3d.io.read_point_cloud(str(path))
    points = np.asarray(pcd.points)

    axes = compute_foot_axes(points)
    aligned_points = align_points_to_axes(points, axes)

    aligned_pcd = o3d.geometry.PointCloud()
    aligned_pcd.points = o3d.utility.Vector3dVector(aligned_points)

    aabb = aligned_pcd.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)
    extent_mm = aabb.get_extent() * 1000.0

    detection = detect_toe_tips(points, axes, toe_params)
    measurements = compute_toe_measurements(axes, detection) if detection else None

    return FootMeasurementResult(
        file_name=path.name,
        axes=axes,
        aligned_pcd=aligned_pcd,
        aabb=aabb,
        extent_mm=extent_mm,
        toe_detection=detection,
        toe_measurements=measurements,
    )


def measure_both_feet(
    mesh_dir: str,
    gap: float = 0.2,
    toe_params: Optional[ToeDetectionParams] = None,
) -> List[FootMeasurementResult]:
    mesh_path = Path(mesh_dir)
    foot_files = sorted(p for p in mesh_path.glob("*_poisson.ply"))

    visuals: List[o3d.geometry.Geometry] = []
    results: List[FootMeasurementResult] = []

    for idx, file_path in enumerate(foot_files):
        result = measure_foot(str(file_path), toe_params)

        result.aligned_pcd.paint_uniform_color([0.7, 0.7, 0.7])
        translation = np.array([(idx * 2 - 1) * gap / 2, 0, 0])
        result.aligned_pcd.translate(translation)
        result.aabb.translate(translation)

        visuals.append(result.aligned_pcd)
        visuals.append(result.aabb)
        results.append(result)

    if visuals:
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        visuals.append(frame)
        o3d.visualization.draw_geometries(visuals)

    return results
