from typing import List, Dict, Any
import numpy as np
import open3d as o3d

# ------------------------------
# 발 모양 분석 (E/G/R) *************** 수정 필요
# ------------------------------
def classify_shape(pcd: o3d.geometry.PointCloud, aabb: o3d.geometry.AxisAlignedBoundingBox) -> str:
    """
    발 모양 분석
    AABB 안 점군을 활용하여 발 길이 대비 너비 비율로 판정
    """
    points = np.asarray(pcd.points)
    indices = aabb.get_point_indices_within_bounding_box(pcd.points)
    points_in_box = points[indices]

    # 발 길이 (Z축) / 발 너비 (X축)
    length = np.max(points_in_box[:,2]) - np.min(points_in_box[:,2])
    width  = np.max(points_in_box[:,0]) - np.min(points_in_box[:,0])
    ratio = length / width if width > 0 else 0

    if ratio < 2.5:
        return "E"
    elif ratio < 3.0:
        return "G"
    else:
        return "R"

# ------------------------------
# 발 볼 너비 분석 (W/N)
# ------------------------------
def classify_width(pcd: o3d.geometry.PointCloud, aabb: o3d.geometry.AxisAlignedBoundingBox) -> str:
    """
    발 볼 너비 분석 (length 대비 width 비율 기준)
    W(넓은 발볼)/N(좁은 발볼) 중 하나 반환
    
    계산법:
        ratio = 발 길이(mm) / 발 너비(mm)
        ratio < 2.4 : 넓음 (W)
        ratio >= 2.4 : 좁음 또는 보통 (N)
    """
    points = np.asarray(pcd.points)
    indices = aabb.get_point_indices_within_bounding_box(pcd.points)
    points_in_box = points[indices]

    length = np.max(points_in_box[:,2]) - np.min(points_in_box[:,2])
    width  = np.max(points_in_box[:,0]) - np.min(points_in_box[:,0])
    ratio = length / width if width > 0 else 0

    print(f"🔍 발 길이/너비 비율: {ratio:.2f}")
    return "W" if ratio <= 2.6 else "N"


# ------------------------------
# 발 등 높이 분석 (I/S) *************** 수정 필요
# ------------------------------
def classify_instep(pcd: o3d.geometry.PointCloud, aabb: o3d.geometry.AxisAlignedBoundingBox) -> str:
    """
    발 등 높이 분석
    계산법: 발 뒤꿈치로부터 발 길이 55% 위치에서 Y축 최대값 측정
    """
    points = np.asarray(pcd.points)
    indices = aabb.get_point_indices_within_bounding_box(pcd.points)
    points_in_box = points[indices]

    z_min, z_max = np.min(points_in_box[:,2]), np.max(points_in_box[:,2])
    target_z = z_min + 0.55 * (z_max - z_min)

    tol = 0.002  # ±2mm 범위
    slice_points = points_in_box[np.abs(points_in_box[:,2] - target_z) < tol]

    if len(slice_points) == 0:
        return "S"  # 기본값

    instep_height = np.max(slice_points[:,1]) * 1000  # m → mm
    return "I" if instep_height < 65 else "S"


# ------------------------------
# 아치 높이 분석 (H/L) *************** 수정 필요
# ------------------------------
def classify_arch(pcd: o3d.geometry.PointCloud, aabb: o3d.geometry.AxisAlignedBoundingBox) -> str:
    """
    아치 높이 분석
    계산법: 발 중앙 (Z축 50% 위치) Y축 최대값 대비 평균 높이 비율로 H/L 판정
    """
    points = np.asarray(pcd.points)
    indices = aabb.get_point_indices_within_bounding_box(pcd.points)
    points_in_box = points[indices]

    z_min, z_max = np.min(points_in_box[:,2]), np.max(points_in_box[:,2])
    target_z = z_min + 0.5 * (z_max - z_min)

    tol = 0.002
    slice_points = points_in_box[np.abs(points_in_box[:,2] - target_z) < tol]

    if len(slice_points) == 0:
        return "L"

    arch_height = np.max(slice_points[:,1]) * 1000  # m → mm
    length = z_max - z_min
    arch_ratio = arch_height / length if length > 0 else 0

    return "H" if arch_ratio > 0.25 else "L"

# ------------------------------
# 종합 BTI 분석
# ------------------------------
def analyze_foot_bti(left_pcd: o3d.geometry.PointCloud, left_aabb: o3d.geometry.AxisAlignedBoundingBox) -> Dict[str, str]:
    """
    왼발 PCD와 AABB를 받아서 BTI 유형 결과 반환
    """
    result = {
        "shape": classify_shape(left_pcd, left_aabb),
        "width": classify_width(left_pcd, left_aabb),
        "instep": classify_instep(left_pcd, left_aabb),
        "arch": classify_arch(left_pcd, left_aabb)
    }
    return result