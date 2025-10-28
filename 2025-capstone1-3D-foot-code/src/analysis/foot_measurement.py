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

def measure_foot(ply_path):
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

    R_align = rotation_matrix_from_vectors(foot_dir, np.array([0, 0, 1]))
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
    foot_length_z = toe_z - heel_z
    target_z = heel_z + foot_length_z * 0.55
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

    length_mm = extent[2]
    width_mm = extent[0]

    print(f"\n[발 측정 결과: {os.path.basename(ply_path)}]")
    print(f"  발 길이   : {length_mm:.2f} mm")
    print(f"  발 너비   : {width_mm:.2f} mm")
    print(f"  발등 높이 : {height_mm:.2f} mm")

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

    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points_for_lines),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return rotated_pcd, aabb, extent, line_set, length_mm, width_mm, height_mm


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

        rotated_pcd, aabb, extent, line_set, length_mm, width_mm, height_mm = measure_foot(ply_path)
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
            "height_mm": height_mm
        })

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    visuals.append(frame)

    o3d.visualization.draw_geometries(visuals)

    return results, visuals, measurements  

# ------------------------------
# def measure_foot(ply_path):
#     """단일 발 PLY 파일 처리 및 치수 계산"""
#     pcd = o3d.io.read_point_cloud(ply_path)
#     points = np.asarray(pcd.points)

#     # === (1) Y축 고정: XZ 평면에서만 PCA 정렬 ===
#     points_xz = points[:, [0, 2]]  # XZ 평면 사용 (Y는 높이)
#     C = np.cov(points_xz.T)
#     eigenvalues, eigenvectors = np.linalg.eig(C)
#     main_axis_2d = eigenvectors[:, np.argmax(eigenvalues)]
#     main_axis = np.array([main_axis_2d[0], 0, main_axis_2d[1]])

#     # === (2) 발끝/뒤꿈치 결정 ===
#     projections = points @ main_axis
#     toe_point = points[np.argmax(projections)]
#     heel_point = points[np.argmin(projections)]

#     # === (3) 제일 긴 축을 Z축으로 align ===
#     foot_vector = toe_point - heel_point
#     foot_dir = foot_vector / np.linalg.norm(foot_vector)

#     # 만약 toe가 heel보다 아래쪽으로 가면 방향 뒤집기 (항상 toe가 +Z로 가도록)
#     if foot_dir[2] < 0:  
#         foot_dir = -foot_dir
#         toe_point, heel_point = heel_point, toe_point

#     R_align = rotation_matrix_from_vectors(foot_dir, np.array([0, 0, 1]))

#     # 점군 회전
#     rotated_points = points @ R_align.T

#     # === (추가) 발목 부분 제거 ===
#     # y_values = rotated_points[:, 1]  # Y축이 높이라면
#     # threshold = np.percentile(y_values, 80)  # 상위 10% 잘라냄 (발목 제거)
#     # rotated_points = rotated_points[y_values < threshold]

#     rotated_pcd = o3d.geometry.PointCloud()
#     rotated_pcd.points = o3d.utility.Vector3dVector(rotated_points)

#     # AABB 계산
#     aabb = rotated_pcd.get_axis_aligned_bounding_box()
#     aabb.color = (1, 0, 0)
#     extent = aabb.get_extent() * 1000  # mm 단위

#     # 발 길이 -> extent[2]
#     # 발 너비 -> extent[0]
#     # 발 높이 -> 발 뒤꿈치로부터 발 길이 55% 위치에서 Y축 최대값 측정 (추가 필요)
#     # === (3.5) 발 높이 계산 ===
#     # 발 길이 방향 (Z축) 기준으로 뒤꿈치~발끝 거리 구함
#     heel_z = np.min(rotated_points[:, 2])
#     toe_z = np.max(rotated_points[:, 2])
#     foot_length_z = toe_z - heel_z

#     # 발 뒤꿈치로부터 55% 위치 계산
#     target_z = heel_z + foot_length_z * 0.55

#     # ±2mm (0.002m) 범위 내 포인트 선택
#     band = 0.002
#     mask = (rotated_points[:, 2] > target_z - band) & (rotated_points[:, 2] < target_z + band)
#     mid_section = rotated_points[mask]

#     if len(mid_section) > 0:
#         height_mm = (np.max(mid_section[:, 1]) - np.min(mid_section[:, 1])) * 1000  # mm 단위 변환
#     else:
#         height_mm = extent[1] * 1000  # fallback (AABB 높이 사용)

#     print(f"\n[발 측정 결과: {os.path.basename(ply_path)}]")
#     print(f"  길이 (Z, 발 길이): {extent[2]:.2f} mm")
#     print(f"  너비 (X): {extent[0]:.2f} mm")
#     print(f"  높이 (Y, 55% 위치): {height_mm:.2f} mm")         

#     return rotated_pcd, aabb, extent


# def measure_both_feet(mesh_dir, gap=0.2):
#     """
#     양쪽 발 모두 처리 및 시각화
#     gap: 두 발 시각화 시 좌우 간격 (m 단위, 기본 0.2m = 20cm)
#     """
#     foot_files = [f for f in os.listdir(mesh_dir) if f.endswith("_poisson.ply")]
#     foot_files.sort()  # cluster1, cluster2 순서 보장

#     visuals = []
#     results = []

#     for idx, file in enumerate(foot_files):
#         ply_path = os.path.join(mesh_dir, file)

#         # === 원본 PLY 파일 로드 & 시각화 ===
#         # original_pcd = o3d.io.read_point_cloud(ply_path)
#         # frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)  # 좌표축 추가
#         # o3d.visualization.draw_geometries([original_pcd, frame])

#         # === 발 측정 & 시각화 ===
#         rotated_pcd, aabb, extent = measure_foot(ply_path)
#         rotated_pcd.paint_uniform_color([0.7, 0.7, 0.7])

#         # === 발 시각화 위치 보정 (X축 기준 이동) ===
#         translation = np.array([(idx * 2 - 1) * gap / 2, 0, 0])  # 첫 발은 -gap/2, 두 번째 발은 +gap/2
#         rotated_pcd.translate(translation)
#         aabb.translate(translation)

#         # # === (추가) 개별 발 시각화 ===
#         # print(f"\n🦶 개별 발 시각화: {file}")
#         # frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
#         # o3d.visualization.draw_geometries([rotated_pcd, aabb, frame])

#         visuals.append(rotated_pcd)
#         visuals.append(aabb)
#         results.append((file, extent))

#     # 좌표축 추가
#     frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
#     visuals.append(frame)

#     # print("\n✅ 양 발 측정 완료 (시각화 시 X축 분리)")
#     o3d.visualization.draw_geometries(visuals)

#     # visuals = [왼발_pcd, 왼발_aabb, 오른발_pcd, 오른발_aabb, frame]
#     return results, visuals

