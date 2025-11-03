import open3d as o3d
import numpy as np
import os

def poisson_mesh_from_clusters(cluster_pcd_list, mesh_dir=None):
    meshes = []

    for i, data in enumerate(cluster_pcd_list):
        cluster_pcd = data['pcd']
        side_label = data['side']

        # 노멀 계산
        cluster_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        cluster_pcd.orient_normals_consistent_tangent_plane(100)

        # 포아송 메쉬
        mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(cluster_pcd, depth=9)

        # 경계 내로 자르기
        bbox = cluster_pcd.get_axis_aligned_bounding_box()
        mesh = mesh.crop(bbox)

        # 스무딩 & 노멀 재계산
        mesh = mesh.filter_smooth_laplacian(number_of_iterations=5)
        mesh.compute_vertex_normals()
        mesh.paint_uniform_color([0.8, 0.8, 0.8])

        # 저장
        if mesh_dir is not None:
            save_path = os.path.join(mesh_dir, f"mesh_{side_label}_poisson.ply")
            o3d.io.write_triangle_mesh(save_path, mesh)
            # print(f"[mesh] {side_label} 메쉬 저장 완료: {save_path}")

        # 시각화
        # o3d.visualization.draw_geometries([mesh], window_name=f"{side_label.capitalize()} Mesh")
        meshes.append(mesh)

    return meshes
