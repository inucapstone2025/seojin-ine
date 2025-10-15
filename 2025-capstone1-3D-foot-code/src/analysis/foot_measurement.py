from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np
import open3d as o3d

from .geometry_utils import (
    FootAxes,
    align_points_to_axes,
    compute_foot_axes,
    restore_points_from_axes,
)
from .toe_detection import ToeDetectionParams, ToeDetectionResult, detect_toe_tips
from .toe_measurement import ToeMeasurements, compute_toe_measurements


def _create_sphere(center: np.ndarray, color: List[float], radius: float = 0.004) -> o3d.geometry.TriangleMesh:
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    mesh.paint_uniform_color(color)
    mesh.translate(center, relative=False)
    return mesh


def _create_line(start: np.ndarray, end: np.ndarray, color: List[float]) -> o3d.geometry.LineSet:
    line = o3d.geometry.LineSet()
    line.points = o3d.utility.Vector3dVector(np.vstack([start, end]))
    line.lines = o3d.utility.Vector2iVector([[0, 1]])
    line.colors = o3d.utility.Vector3dVector([color])
    return line


def _clone_geometry(geometry: o3d.geometry.Geometry) -> o3d.geometry.Geometry:
    if hasattr(geometry, "clone"):
        return geometry.clone()
    return copy.deepcopy(geometry)


def _build_debug_geometries(
    aligned_points: np.ndarray,
    axes: FootAxes,
    detection: Optional[ToeDetectionResult],
) -> tuple[List[o3d.geometry.Geometry], List[o3d.geometry.Geometry]]:
    aligned_geoms: List[o3d.geometry.Geometry] = []
    world_geoms: List[o3d.geometry.Geometry] = []
    if aligned_points.size == 0:
        return aligned_geoms, world_geoms

    heel_aligned = np.zeros(3)
    heel_world = axes.origin
    max_x = float(np.max(aligned_points[:, 0]))
    width_half = float(np.max(np.abs(aligned_points[:, 1])))
    vertical_extent = float(np.max(np.abs(aligned_points[:, 2])))

    aligned_geoms.append(_create_sphere(heel_aligned, [1.0, 0.6, 0.0]))
    world_geoms.append(_create_sphere(heel_world, [1.0, 0.6, 0.0]))

    if max_x > 0:
        toe_axis_point_aligned = np.array([max_x, 0.0, 0.0])
        toe_axis_point_world = restore_points_from_axes(
            toe_axis_point_aligned[None, :], axes
        )[0]
        aligned_geoms.append(
            _create_line(heel_aligned, toe_axis_point_aligned, [1.0, 0.0, 0.0])
        )
        aligned_geoms.append(
            _create_sphere(toe_axis_point_aligned, [1.0, 0.3, 0.3], radius=0.003)
        )
        world_geoms.append(
            _create_line(heel_world, toe_axis_point_world, [1.0, 0.0, 0.0])
        )
        world_geoms.append(
            _create_sphere(toe_axis_point_world, [1.0, 0.3, 0.3], radius=0.003)
        )

    if vertical_extent > 0:
        vertical_tip_aligned = np.array([0.0, 0.0, vertical_extent])
        vertical_tip_world = restore_points_from_axes(
            vertical_tip_aligned[None, :], axes
        )[0]
        aligned_geoms.append(
            _create_line(heel_aligned, vertical_tip_aligned, [0.0, 0.0, 1.0])
        )
        aligned_geoms.append(
            _create_sphere(vertical_tip_aligned, [0.2, 0.4, 1.0], radius=0.003)
        )
        world_geoms.append(
            _create_line(heel_world, vertical_tip_world, [0.0, 0.0, 1.0])
        )
        world_geoms.append(
            _create_sphere(vertical_tip_world, [0.2, 0.4, 1.0], radius=0.003)
        )

    if width_half > 0:
        width_tip_pos_aligned = np.array([0.0, width_half, 0.0])
        width_tip_neg_aligned = np.array([0.0, -width_half, 0.0])
        width_tip_pos_world = restore_points_from_axes(
            width_tip_pos_aligned[None, :], axes
        )[0]
        width_tip_neg_world = restore_points_from_axes(
            width_tip_neg_aligned[None, :], axes
        )[0]
        aligned_geoms.append(
            _create_line(heel_aligned, width_tip_pos_aligned, [0.0, 1.0, 0.0])
        )
        aligned_geoms.append(
            _create_line(heel_aligned, width_tip_neg_aligned, [0.0, 1.0, 0.0])
        )
        aligned_geoms.append(
            _create_sphere(width_tip_pos_aligned, [0.0, 0.8, 0.2], radius=0.003)
        )
        aligned_geoms.append(
            _create_sphere(width_tip_neg_aligned, [0.0, 0.8, 0.2], radius=0.003)
        )
        world_geoms.append(
            _create_line(heel_world, width_tip_pos_world, [0.0, 1.0, 0.0])
        )
        world_geoms.append(
            _create_line(heel_world, width_tip_neg_world, [0.0, 1.0, 0.0])
        )
        world_geoms.append(
            _create_sphere(width_tip_pos_world, [0.0, 0.8, 0.2], radius=0.003)
        )
        world_geoms.append(
            _create_sphere(width_tip_neg_world, [0.0, 0.8, 0.2], radius=0.003)
        )

    if detection:
        aligned_geoms.append(
            _create_sphere(detection.big_toe_tip_aligned, [1.0, 0.0, 0.0])
        )
        aligned_geoms.append(
            _create_sphere(detection.second_toe_tip_aligned, [0.0, 0.3, 1.0])
        )
        aligned_geoms.append(
            _create_line(heel_aligned, detection.big_toe_tip_aligned, [1.0, 0.4, 0.4])
        )
        aligned_geoms.append(
            _create_line(heel_aligned, detection.second_toe_tip_aligned, [0.4, 0.6, 1.0])
        )

        world_geoms.append(
            _create_sphere(detection.big_toe_tip, [1.0, 0.0, 0.0])
        )
        world_geoms.append(
            _create_sphere(detection.second_toe_tip, [0.0, 0.3, 1.0])
        )
        world_geoms.append(
            _create_line(heel_world, detection.big_toe_tip, [1.0, 0.4, 0.4])
        )
        world_geoms.append(
            _create_line(heel_world, detection.second_toe_tip, [0.4, 0.6, 1.0])
        )

    return aligned_geoms, world_geoms


@dataclass
class FootMeasurementResult:
    file_name: str
    axes: FootAxes
    original_pcd: o3d.geometry.PointCloud
    aligned_pcd: o3d.geometry.PointCloud
    aabb: o3d.geometry.AxisAlignedBoundingBox
    extent_mm: np.ndarray
    toe_detection: Optional[ToeDetectionResult]
    toe_measurements: Optional[ToeMeasurements]
    aligned_debug_geometries: List[o3d.geometry.Geometry] = field(default_factory=list)
    world_debug_geometries: List[o3d.geometry.Geometry] = field(default_factory=list)


@dataclass
class DebugOptions:
    single_foot_index: Optional[int] = None
    show_aligned_cloud: bool = True
    show_bounding_box: bool = True
    show_world_view: bool = True
    show_world_reference: bool = True

    @classmethod
    def from_dict(cls, config: Optional[dict]) -> "DebugOptions":
        if config is None:
            return cls()
        return cls(
            single_foot_index=config.get("single_foot_index"),
            show_aligned_cloud=config.get("show_aligned_cloud", True),
            show_bounding_box=config.get("show_bounding_box", True),
            show_world_view=config.get("show_world_view", True),
            show_world_reference=config.get("show_world_reference", True),
        )


def measure_foot(
    ply_path: str,
    toe_params: Optional[ToeDetectionParams] = None,
    toe_detection_enabled: bool = True,
) -> FootMeasurementResult:
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

    foot_length = float(np.max(aligned_points[:, 0])) if aligned_points.size else 0.0
    width_span = (
        float(np.max(aligned_points[:, 1]) - np.min(aligned_points[:, 1]))
        if aligned_points.size
        else 0.0
    )
    print(
        f"[디버그] {path.name}: heel={axes.origin}, "
        f"foot_axis={np.array2string(axes.foot_axis, precision=3)}, "
        f"width_axis={np.array2string(axes.width_axis, precision=3)}"
    )
    print(f"          length_aligned={foot_length:.4f} m, width_span_aligned={width_span:.4f} m")

    detection = None
    if toe_detection_enabled:
        params = toe_params or ToeDetectionParams()
        detection = detect_toe_tips(points, axes, params)
    measurements = compute_toe_measurements(axes, detection) if detection else None
    # aligned_debug, world_debug = _build_debug_geometries(aligned_points, axes, detection)
    aligned_debug, world_debug = [], []

    return FootMeasurementResult(
        file_name=path.name,
        axes=axes,
        original_pcd=pcd,
        aligned_pcd=aligned_pcd,
        aabb=aabb,
        extent_mm=extent_mm,
        toe_detection=detection,
        toe_measurements=measurements,
        aligned_debug_geometries=aligned_debug,
        world_debug_geometries=world_debug,
    )


def measure_both_feet(
    mesh_dir: str,
    gap: float = 0.2,
    toe_params: Optional[ToeDetectionParams] = None,
    toe_detection_enabled: bool = True,
    debug_config: Optional[dict] = None,
    reference_cloud: Optional[o3d.geometry.PointCloud] = None,
) -> List[FootMeasurementResult]:
    mesh_path = Path(mesh_dir)
    foot_files = sorted(p for p in mesh_path.glob("*_poisson.ply"))

    debug_options = DebugOptions.from_dict(debug_config)
    selected_files: List[Path] = list(foot_files)
    if debug_options.single_foot_index is not None and foot_files:
        idx = debug_options.single_foot_index
        if idx < 0:
            idx += len(foot_files)
        if 0 <= idx < len(foot_files):
            selected_files = [foot_files[idx]]
        else:
            print(f"[디버그] single_foot_index={debug_options.single_foot_index} 범위를 벗어남 (총 {len(foot_files)}개)")

    visuals: List[o3d.geometry.Geometry] = []
    world_visuals: List[o3d.geometry.Geometry] = []
    results: List[FootMeasurementResult] = []

    if (
        debug_options.show_world_view
        and debug_options.show_world_reference
        and reference_cloud is not None
    ):
        world_visuals.append(_clone_geometry(reference_cloud))

    num_selected = len(selected_files)

    for display_idx, file_path in enumerate(selected_files):
        result = measure_foot(
            str(file_path),
            toe_params=toe_params,
            toe_detection_enabled=toe_detection_enabled,
        )

        if debug_options.show_world_view and reference_cloud is None and debug_options.show_world_reference:
            world_visuals.append(_clone_geometry(result.original_pcd))

        translation = np.zeros(3)
        if num_selected > 1:
            translation = np.array([(display_idx * 2 - 1) * gap / 2, 0, 0])

        if debug_options.show_aligned_cloud:
            aligned_display = _clone_geometry(result.aligned_pcd)
            aligned_display.paint_uniform_color([0.7, 0.7, 0.7])
            if np.any(translation):
                aligned_display.translate(translation)
            visuals.append(aligned_display)

        if debug_options.show_bounding_box:
            aabb_display = _clone_geometry(result.aabb)
            if np.any(translation):
                aabb_display.translate(translation)
            visuals.append(aabb_display)

        for geom in result.aligned_debug_geometries:
            geom_copy = _clone_geometry(geom)
            if np.any(translation):
                geom_copy.translate(translation)
            visuals.append(geom_copy)

        if debug_options.show_world_view:
            for geom in result.world_debug_geometries:
                world_visuals.append(_clone_geometry(geom))

        results.append(result)

    # if visuals:
    #     frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    #     visuals.append(frame)
    #     o3d.visualization.draw_geometries(
    #         visuals,
    #         window_name="Foot Debug (Aligned Frame)",
    #     )

    # if debug_options.show_world_view and world_visuals:
    #     frame_world = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    #     world_visuals.append(frame_world)
    #     o3d.visualization.draw_geometries(
    #         world_visuals,
    #         window_name="Foot Debug (Merged Cloud)",
    #     )

    return results
