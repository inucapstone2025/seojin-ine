from typing import List, Dict, Any
import numpy as np
import open3d as o3d

from .runner import process

# ------------------------------
# 발 모양 분석 (E/G/R) *************** 수정 필요
# ------------------------------
def classify_shape(pcd: o3d.geometry.PointCloud, 
                   aabb: o3d.geometry.AxisAlignedBoundingBox,
                   metrics: Any) -> str:
    """
    발 모양 분석
    AABB 안 점군을 활용하여 발 길이 대비 너비 비율로 판정
    """
    delta21 = metrics.delta21
    delta31 = metrics.delta31
    # print(f"🔍 delta21: {delta21:.2f}, delta31: {delta31:.2f}")

    if delta21 >= 3.0 :
        return "G"  # 그리스형
    elif delta21 <= -3.0:
        return "E"  # 이집트형
    elif abs(delta21) < 3.0 and abs(delta31) < 3.0:
        return "R"  # 로마형
    else:
        return "R"  # 기본값 로마형

# ------------------------------
# 발 볼 너비 분석 (W/N)
# ------------------------------
def classify_width(pcd: o3d.geometry.PointCloud, 
                   aabb: o3d.geometry.AxisAlignedBoundingBox,
                   measurements: List[Dict[str, float]]) -> str:
    """
    발 볼 너비 분석 (length 대비 width 비율 기준)
        W(넓은 발볼)/N(좁은 발볼) 중 하나 반환
    
    계산법:
        ratio = 발 길이(mm) / 발 너비(mm)
        ratio < 2.4 : 넓음 (W)
        ratio >= 2.4 : 좁음 또는 보통 (N)
    """

    ratio = measurements[0]["length_mm"] / measurements[0]["width_mm"]
    # print(f"🔍 발 길이/너비 비율: {ratio:.2f}")

    if ratio < 2.4:
        return "W"
    else:
        return "N"

# ------------------------------
# 발 등 높이 분석 (I/S)
# ------------------------------
def classify_instep(pcd: o3d.geometry.PointCloud, 
                    aabb: o3d.geometry.AxisAlignedBoundingBox,
                    measurements: List[Dict[str, float]]) -> str:
    """
    발 등 높이 분석 (발 길이와 발등 높이의 비율 기준)
        I(높은 발등)/S(낮은 발등) 중 하나 반환
    
    계산법:
        ratio = (발등 높이(mm) / 발 길이(mm))*100
        ratio < 25 : 낮은 발등 (S)
        ratio >= 25 : 높은 발등 (I)
    """

    ratio = (measurements[0]["height_mm"] / measurements[0]["length_mm"])*100
    # print(f"🔍 발등 높이/길이 비율: {ratio:.2f}")
    if ratio < 25:
        return "S"
    else:
        return "I"

# ------------------------------
# 아치 높이 분석 (H/L) 
# ------------------------------
def classify_arch(pcd: o3d.geometry.PointCloud, 
                  aabb: o3d.geometry.AxisAlignedBoundingBox,
                  measurements: List[Dict[str, float]]) -> str:
    """
    아치 높이 분석 (발등 높이/잘린 발 길이(뒤꿈치~중족지관절) 비율 기준)
        H(낮은 아치)/L(높은 아치) 중 하나 반환
    
    계산법: 
        ratio = 발등 높이(mm) / (뒤꿈치에서 중족지관절까지 거리(mm))
        ratio < 0.33 : 낮은 아치 (L)
        ratio >= 0.33 : 높은 아치 (H)
    """

    ratio = measurements[0]["height_mm"] / measurements[0]["heel_to_MTP_joint_mm"]
    # print(f"🔍 아치 높이/발 길이(뒤꿈치~중족지관절) 비율: {ratio:.2f}")
    if ratio < 0.33:
        return "L"
    else:
        return "H"

# ------------------------------
# 종합 BTI 분석
# ------------------------------
def analyze_foot_bti(left_pcd: o3d.geometry.PointCloud, 
                     left_aabb: o3d.geometry.AxisAlignedBoundingBox, 
                     measurements: List[Dict[str, float]]) -> Dict[str, str]:
    """
    왼발 PCD와 AABB를 받아서 BTI 유형 결과 반환
    """
    metrics, landmarks, frame, footprint = process(np.asarray(left_pcd.points), side="L",visualize_steps=False)
    result = {
        "shape": classify_shape(left_pcd, left_aabb, metrics),
        "width": classify_width(left_pcd, left_aabb, measurements),
        "instep": classify_instep(left_pcd, left_aabb, measurements),
        "arch": classify_arch(left_pcd, left_aabb, measurements),
    }

    # print(f"\n[발 측정 결과: {'왼발'}]")
    # print(f"  발 길이   : {measurements[1]['length_mm']:.2f} mm")
    # print(f"  발 너비   : {measurements[1]['width_mm']:.2f} mm")
    # print(f"  발등 높이 : {measurements[1]['height_mm']:.2f} mm")

    # print(f"\n[발 측정 결과: {'오른발'}]")
    print(f"\n[발 측정 결과]")
    print(f"  발 길이   : {measurements[0]['length_mm']:.2f} mm")
    print(f"  발 너비   : {measurements[0]['width_mm']:.2f} mm")
    print(f"  발등 높이 : {measurements[0]['height_mm']:.2f} mm")

    return result, footprint