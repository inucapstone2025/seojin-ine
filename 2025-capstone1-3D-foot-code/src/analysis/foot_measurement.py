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

# 여기부분 수정 필요!!
# def measure_foot(ply_path):
#     """단일 발 PLY 파일 처리 및 치수 계산"""
#     pcd = o3d.io.read_point_cloud(ply_path)
#     points = np.asarray(pcd.points)

#     # OBB 주축 → 발끝/뒤꿈치 탐색
#     obb = pcd.get_oriented_bounding_box()
#     main_axis = obb.R[:, 0]
#     projections = points @ main_axis
#     toe_point = points[np.argmax(projections)]
#     heel_point = points[np.argmin(projections)]

#     # 발끝-뒤꿈치 벡터 → X축 정렬
#     foot_vector = toe_point - heel_point
#     foot_dir = foot_vector / np.linalg.norm(foot_vector)
#     R_align = rotation_matrix_from_vectors(foot_dir, np.array([0, 0, 1]))

#     # 점군 회전
#     rotated_points = points @ R_align.T
#     rotated_pcd = o3d.geometry.PointCloud()
#     rotated_pcd.points = o3d.utility.Vector3dVector(rotated_points)

#     # AABB 계산
#     aabb = rotated_pcd.get_axis_aligned_bounding_box()
#     aabb.color = (1, 0, 0)
#     extent = aabb.get_extent() * 1000  # mm 단위

#     print(f"\n[발 측정 결과: {os.path.basename(ply_path)}]")
#     print(f"  가로 (X, 발 길이): {extent[0]:.2f} mm")
#     print(f"  세로 (Y, 발 폭): {extent[1]:.2f} mm")
#     print(f"  높이 (Z, 발 두께): {extent[2]:.2f} mm")

#     return rotated_pcd, aabb, extent
def measure_foot(ply_path):
    """단일 발 PLY 파일 처리 및 치수 계산"""
    pcd = o3d.io.read_point_cloud(ply_path)
    points = np.asarray(pcd.points)

    # OBB 계산
    obb = pcd.get_oriented_bounding_box()

    # === (1) OBB에서 제일 긴 축 찾기 ===
    extents = obb.extent            # [x_len, y_len, z_len] (길이)
    axes = obb.R                    # 각 축 방향 (3x3 matrix)
    main_axis = axes[:, np.argmax(extents)]  # 제일 긴 축 방향 벡터

    # === (2) 발끝/뒤꿈치 결정 ===
    projections = points @ main_axis
    toe_point = points[np.argmax(projections)]
    heel_point = points[np.argmin(projections)]

    # === (3) 제일 긴 축을 Z축으로 align ===
    foot_vector = toe_point - heel_point
    foot_dir = foot_vector / np.linalg.norm(foot_vector)

    # 만약 toe가 heel보다 아래쪽으로 가면 방향 뒤집기 (항상 toe가 +Z로 가도록)
    if foot_dir[2] < 0:  
        foot_dir = -foot_dir
        toe_point, heel_point = heel_point, toe_point

    R_align = rotation_matrix_from_vectors(foot_dir, np.array([0, 0, 1]))

    # 점군 회전
    rotated_points = points @ R_align.T
    rotated_pcd = o3d.geometry.PointCloud()
    rotated_pcd.points = o3d.utility.Vector3dVector(rotated_points)

    # AABB 계산
    aabb = rotated_pcd.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)
    extent = aabb.get_extent() * 1000  # mm 단위

    print(f"\n[발 측정 결과: {os.path.basename(ply_path)}]")
    print(f"  길이 (Z, 발 길이): {extent[2]:.2f} mm")
    print(f"  폭   (X): {extent[0]:.2f} mm")
    print(f"  두께 (Y): {extent[1]:.2f} mm")

    return rotated_pcd, aabb, extent


def measure_both_feet(mesh_dir, gap=0.2):
    """
    양쪽 발 모두 처리 및 시각화
    gap: 두 발 시각화 시 좌우 간격 (m 단위, 기본 0.2m = 20cm)
    """
    foot_files = [f for f in os.listdir(mesh_dir) if f.endswith("_poisson.ply")]
    foot_files.sort()  # cluster1, cluster2 순서 보장

    visuals = []
    results = []

    for idx, file in enumerate(foot_files):
        ply_path = os.path.join(mesh_dir, file)

        # === 원본 PLY 파일 로드 & 시각화 ===
        original_pcd = o3d.io.read_point_cloud(ply_path)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)  # 좌표축 추가
        # print(f"\n📂 원본 시각화: {file}")
        o3d.visualization.draw_geometries([original_pcd, frame])

        # === 발 측정 & 시각화 ===
        rotated_pcd, aabb, extent = measure_foot(ply_path)
        rotated_pcd.paint_uniform_color([0.7, 0.7, 0.7])

        # === 발 시각화 위치 보정 (X축 기준 이동) ===
        translation = np.array([(idx * 2 - 1) * gap / 2, 0, 0])  # 첫 발은 -gap/2, 두 번째 발은 +gap/2
        rotated_pcd.translate(translation)
        aabb.translate(translation)

        visuals.append(rotated_pcd)
        visuals.append(aabb)
        results.append((file, extent))

    # 좌표축 추가
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    visuals.append(frame)

    # print("\n✅ 양 발 측정 완료 (시각화 시 X축 분리)")
    o3d.visualization.draw_geometries(visuals)

    return results


