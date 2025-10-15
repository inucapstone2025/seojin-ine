import open3d as o3d
import numpy as np
import os

def poisson_mesh_from_pcd(pcd, top_clusters=None, eps=0.01, mesh_dir=None):
    """
    DBSCAN 후 필터링된 포인트 클라우드(pcd)에서 포아송 표면 재구성 방식으로 메쉬를 생성하는 함수입니다.
    top_clusters가 주어지면 해당 클러스터별로 메쉬를 만들고, 없으면 전체 포인트 클라우드에 대해 메쉬 생성합니다.

    Args:
        pcd (open3d.geometry.PointCloud): 입력 포인트 클라우드 (DBSCAN 필터링 후)
        top_clusters (list or None): 메쉬 생성 대상 클러스터 라벨 리스트 (dbscan_largest_clusters 결과)
        eps (float): DBSCAN 클러스터링에서 사용할 거리 임계값
        mesh_dir (str or None): 메쉬 저장 디렉터리 경로, None이면 저장하지 않음

    Returns:
        list of open3d.geometry.TriangleMesh: 생성된 메쉬 리스트
    """

    # 전체 포인트 클라우드로 메쉬 생성하는 경우
    if top_clusters is None:
        print("top_clusters 정보 없음, 전체 포인트클라우드로 메쉬 생성")

        # 노멀 추정 및 방향 일관성 맞춤
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        pcd.orient_normals_consistent_tangent_plane(100)

        # 포아송 표면 재구성 (depth=9: 재구성 해상도 조절)
        mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)

        # 포인트 클라우드 경계 내로 메쉬 자르기
        bbox = pcd.get_axis_aligned_bounding_box()
        mesh = mesh.crop(bbox)

        # 라플라시안 스무딩 및 노멀 재계산
        mesh = mesh.filter_smooth_laplacian(number_of_iterations=5)
        mesh.compute_vertex_normals()

        # 메쉬 저장
        if mesh_dir is not None:
            save_path = os.path.join(mesh_dir, "foot_mesh.ply")
            o3d.io.write_triangle_mesh(save_path, mesh)
            print(f"[mesh] 메쉬 저장 완료: {save_path}")

        # 시각화
        # o3d.visualization.draw_geometries([mesh], window_name="Poisson Mesh")
        return [mesh]

    # 클러스터별 메쉬 생성
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=20))
    filtered_points = np.asarray(pcd.points)
    filtered_colors = np.asarray(pcd.colors)

    meshes = []

    for i, cluster_id in enumerate(top_clusters):
        cluster_mask = (labels == cluster_id)
        cluster_points = filtered_points[cluster_mask]
        cluster_colors = filtered_colors[cluster_mask]

        cluster_pcd = o3d.geometry.PointCloud()
        cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)
        cluster_pcd.colors = o3d.utility.Vector3dVector(cluster_colors)

        cluster_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        cluster_pcd.orient_normals_consistent_tangent_plane(100)

        mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(cluster_pcd, depth=9)

        bbox = cluster_pcd.get_axis_aligned_bounding_box()
        mesh = mesh.crop(bbox)

        mesh = mesh.filter_smooth_laplacian(number_of_iterations=5)
        mesh.compute_vertex_normals()

        # 클러스터 메쉬는 회색으로 균일하게 색칠
        mesh.paint_uniform_color([0.8, 0.8, 0.8])

        # 메쉬 저장
        if mesh_dir is not None:
            save_path = os.path.join(mesh_dir, f"mesh_cluster{i+1}_poisson.ply")
            o3d.io.write_triangle_mesh(save_path, mesh)
            print(f"[mesh] 클러스터 {i+1} 메쉬 저장 완료: {save_path}")

        # # 🔹 개별 클러스터 메쉬 시각화 (추가)
        # o3d.visualization.draw_geometries(
        #     [mesh],
        #     window_name=f"Cluster {i+1} Mesh",
        #     width=800,
        #     height=600,
        #     mesh_show_back_face=True,
        # )

        meshes.append(mesh)

    # # 모든 클러스터 메쉬 시각화
    # o3d.visualization.draw_geometries(
    #     meshes,
    #     window_name=f"Poisson Meshes (Top {len(top_clusters)} Clusters)"
    # )

    return meshes
