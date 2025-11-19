import open3d as o3d
import numpy as np
import os

def rotation_matrix_from_vectors(vec1, vec2):
    a = vec1 / np.linalg.norm(vec1)
    b = vec2 / np.linalg.norm(vec2)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    if s == 0:
        return np.eye(3)
    kmat = np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])
    return np.eye(3) + kmat + kmat @ kmat * ((1 - c) / (s ** 2))

def yaw_only_rotation_matrix(vec):
    """
    입력 벡터(vec)의 XZ 평면 투영 방향을 Z축(0,0,1)에 맞추는 yaw 회전 행렬
    => Y축은 고정, pitch/roll 제거
    """
    # XZ 평면에 투영
    vec_xz = np.array([vec[0], 0, vec[2]])
    vec_xz /= np.linalg.norm(vec_xz)

    # 목표 방향 (Z축)
    target = np.array([0, 0, 1])

    # yaw 각도 계산
    yaw = np.arctan2(vec_xz[0], vec_xz[2])  # atan2(x,z)

    # Y축 회전 행렬 (yaw 회전)
    R_yaw = np.array([
        [ np.cos(-yaw), 0, np.sin(-yaw)],
        [ 0,            1, 0],
        [-np.sin(-yaw), 0, np.cos(-yaw)]
    ])
    return R_yaw


def measure_foot(ply_path, idx):
    """단일 발 PLY 파일 처리 및 치수 계산 + 길이/너비/높이 선 생성"""
    pcd = o3d.io.read_point_cloud(ply_path)
    points = np.asarray(pcd.points)

    # === (1) Y축 고정: XZ 평면에서만 PCA 정렬 ===
    points_xz = points[:, [0, 2]]  # XZ 평면 사용 (Y는 높이)
    C = np.cov(points_xz.T)
    eigenvalues, eigenvectors = np.linalg.eig(C)
    main_axis_2d = eigenvectors[:, np.argmax(eigenvalues)]
    main_axis = np.array([main_axis_2d[0], 0, main_axis_2d[1]])

    # === (2) 발끝/뒤꿈치 결정 ===
    projections = points @ main_axis
    toe_point = points[np.argmax(projections)]
    heel_point = points[np.argmin(projections)]

    # === (3) 제일 긴 축을 Z축으로 align ===
    foot_vector = toe_point - heel_point
    foot_dir = foot_vector / np.linalg.norm(foot_vector)

    if foot_dir[2] < 0:
        foot_dir = -foot_dir
        toe_point, heel_point = heel_point, toe_point

    # R_align = rotation_matrix_from_vectors(foot_dir, np.array([0, 0, 1]))
    R_align = yaw_only_rotation_matrix(foot_dir)
    rotated_points = points @ R_align.T

    rotated_pcd = o3d.geometry.PointCloud()
    rotated_pcd.points = o3d.utility.Vector3dVector(rotated_points)

    # === (4) AABB 계산 ===
    aabb = rotated_pcd.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)
    extent = aabb.get_extent() * 1000  # mm 단위

    # === (5) 발 높이 계산 (55% 지점) ===
    heel_z = np.min(rotated_points[:, 2])
    toe_z = np.max(rotated_points[:, 2])
    foot_length_z = toe_z - heel_z # 발 전체 길이 (발끝 - 뒤꿈치)
    target_z = heel_z + foot_length_z * 0.55 # 뒤꿈치 기준 55% 위치 계산
    band = 0.002
    mask = (rotated_points[:, 2] > target_z - band) & (rotated_points[:, 2] < target_z + band)
    mid_section = rotated_points[mask]

    if len(mid_section) > 0:
        height_mm = (np.max(mid_section[:, 1]) - np.min(mid_section[:, 1])) * 1000
        height_min = np.min(mid_section[:, 1])
        height_max = np.max(mid_section[:, 1])
    else:
        height_mm = extent[1] * 1000
        height_min, height_max = 0, extent[1]

    # ====== 발 뒤꿈치에서 72% 지점까지의 실제 길이 ====== 
    z_MTP_joint = heel_z + foot_length_z * 0.75
    heel_to_MTP_joint = z_MTP_joint - heel_z
    heel_to_MTP_joint_mm = heel_to_MTP_joint * 1000
    # print(f"🔍 뒤꿈치에서 중족지관절까지 거리: {heel_to_MTP_joint_mm:.2f} mm")

    if idx == 1:
        foot = "왼발"
        length_mm = extent[2]
    else:
        foot = "오른발"
        length_mm = extent[2]
    width_mm = extent[0]

    # print(f"\n[발 측정 결과: {os.path.basename(ply_path)}]")
    # print(f"\n[발 측정 결과: {foot}]")
    # print(f"  발 길이   : {length_mm:.2f} mm")
    # print(f"  발 너비   : {width_mm:.2f} mm")
    # print(f"  발등 높이 : {height_mm:.2f} mm")

    # === (6) 길이/너비/높이 선(LineSet) 생성 ===
    lines = []
    colors = []
    points_for_lines = []

    # 발 길이 (빨강)
    heel_point_vis = np.array([0, 0, heel_z])
    toe_point_vis = np.array([0, 0, toe_z])
    points_for_lines += [heel_point_vis, toe_point_vis]
    lines.append([0, 1])
    colors.append([1, 0, 0])  # 빨강

    # 발 너비 (파랑)
    mid_y = np.mean(rotated_points[:, 1])
    mid_z = heel_z + foot_length_z / 2
    x_min = np.min(rotated_points[:, 0])
    x_max = np.max(rotated_points[:, 0])
    points_for_lines += [[x_min, mid_y, mid_z], [x_max, mid_y, mid_z]]
    lines.append([2, 3])
    colors.append([0, 0, 1])  # 파랑

    # 발 높이 (초록)
    mid_x = np.mean(rotated_points[:, 0])
    points_for_lines += [[mid_x, height_min, target_z], [mid_x, height_max, target_z]]
    lines.append([4, 5])
    colors.append([0, 1, 0])  # 초록

    # 발 길이 72% 지점 직선 (노랑)
    mid_y = np.mean(rotated_points[:, 1])
    heel_point_72 = np.array([0, mid_y, heel_z])
    toe_point_72 = np.array([0, mid_y, z_MTP_joint])
    points_for_lines += [heel_point_72, toe_point_72]
    lines.append([len(points_for_lines)-2, len(points_for_lines)-1])
    colors.append([1, 0.5, 0])  # 노랑

    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points_for_lines),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return rotated_pcd, aabb, extent, line_set, length_mm, width_mm, height_mm, heel_to_MTP_joint_mm


def measure_both_feet(mesh_dir, gap=0.2):
    """
    양쪽 발 모두 처리 및 시각화 (길이/너비/높이 선 포함)
    gap: 두 발 간격 (m)
    """
    foot_files = [f for f in os.listdir(mesh_dir) if f.endswith("_poisson.ply")]
    foot_files.sort()

    visuals = []
    results = []
    measurements = []

    for idx, file in enumerate(foot_files):
        ply_path = os.path.join(mesh_dir, file)

        rotated_pcd, aabb, extent, line_set, length_mm, width_mm, height_mm, heel_to_MTP_joint_mm = measure_foot(ply_path, idx)
        rotated_pcd.paint_uniform_color([0.7, 0.7, 0.7])

        # === 발 위치 보정 ===
        translation = np.array([(idx * 2 - 1) * gap / 2, 0, 0])
        rotated_pcd.translate(translation)
        aabb.translate(translation)
        line_set.translate(translation)

        visuals += [rotated_pcd, aabb, line_set]
        results.append((file, extent))
        measurements.append({
            "file": file,
            "length_mm": length_mm,
            "width_mm": width_mm,
            "height_mm": height_mm,
            "heel_to_MTP_joint_mm": heel_to_MTP_joint_mm
        })

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    visuals.append(frame)

    o3d.visualization.draw_geometries(visuals)

    return results, visuals, measurements  