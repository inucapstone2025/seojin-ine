import open3d as o3d
import numpy as np
import os

import open3d as o3d
import numpy as np

def remove_ground_ransac(pcd, distance_threshold=0.005, num_iterations=1000, visualize=False):
    points = np.asarray(pcd.points)
    y_min = np.min(points[:, 1])
    
    # 바닥 후보 인덱스
    ground_candidate_indices = np.where(points[:, 1] < y_min + 0.02)[0]
    ground_candidates = pcd.select_by_index(ground_candidate_indices)

    # RANSAC 평면 추정
    plane_model, inliers = ground_candidates.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=num_iterations
    )

    # inliers를 원본 인덱스로 변환
    original_inlier_indices = ground_candidate_indices[inliers]

    # 바닥/비바닥 분리
    mask = np.ones(len(points), dtype=bool)
    mask[original_inlier_indices] = False
    no_ground = pcd.select_by_index(np.where(mask)[0])
    ground = pcd.select_by_index(original_inlier_indices)

    if visualize:
        ground.paint_uniform_color([1, 0, 0])
        no_ground.paint_uniform_color([0.7, 0.7, 0.7])
        o3d.visualization.draw_geometries([ground, no_ground])

    return no_ground, ground



def remove_background_color_from_file(pcd_path, green_threshold=30):
    """
    포인트 클라우드 파일에서 녹색 배경을 제거하는 함수

    Parameters:
        pcd_path (str): PLY 또는 PCD 파일 경로
        green_threshold (int): G(RGB) 채널의 값이 이 값보다 크면 배경으로 간주하여 제거

    Returns:
        result (o3d.geometry.PointCloud): 배경이 제거된 포인트 클라우드 객체
    """

    # 포인트 클라우드 파일 불러오기
    pcd = o3d.io.read_point_cloud(pcd_path)
    # o3d.visualization.draw_geometries([pcd], window_name="Before Background Removal")

    # 컬러 정보 추출 (0~1) → 0~255로 정규화
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    mask = np.ones(len(points), dtype=bool)
    rgb_255 = (colors * 255).astype(np.uint8)

    # # G값이 threshold보다 큰 포인트는 배경으로 간주하여 제거
    # mask = ~(rgb_255[:, 1] > green_threshold)
    # R,G,B 모두 black_threshold 이하인 포인트만 선택
    black_threshold=30
    mask = np.all(rgb_255 <= black_threshold, axis=1)

    # Y값 범위 필터링
    y_min = -0.06 # -0.045
    y_max = 0.05
    if y_min is not None:
        mask = mask & (points[:, 1] >= y_min)
    if y_max is not None:
        mask = mask & (points[:, 1] <= y_max)

    # 필터링 적용
    filtered_points = points[mask]
    filtered_colors = colors[mask]

    # 새로운 포인트 클라우드 객체로 생성
    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(filtered_points)
    result.colors = o3d.utility.Vector3dVector(filtered_colors)

    # print(f"[noise_removal] 필터링 전: {len(colors)}, 필터링 후: {len(filtered_colors)}")

    # ======================
    # RANSAC 기반 바닥 제거 추가
    # ======================
    # print("[noise_removal] RANSAC을 통한 바닥면 제거 중...")
    # result_no_ground, ground = remove_ground_ransac(result, 
    #                                                 distance_threshold=0.005, 
    #                                                 num_iterations=1000,
    #                                                 visualize=True)
    # result = result_no_ground
    # 시각화 추가
    # o3d.visualization.draw_geometries([result], window_name="Filtered Point Cloud")
    return result

def dbscan_largest_clusters(pcd, aligned_dir=None, eps=0.01, min_points=20, top_k=2):
    """
    DBSCAN 클러스터링을 통해 가장 큰 상위 K개의 클러스터만 필터링하고,
    클러스터의 X축 중앙값을 기준으로 순서(top1/top2)를 결정하여 분리 저장 및 반환하는 함수.

    Parameters:
        pcd (o3d.geometry.PointCloud): 입력 포인트 클라우드
        aligned_dir (str): 필터링된 결과를 저장할 경로 (None이면 저장하지 않음)
        eps (float): DBSCAN 거리 임계값
        min_points (int): 클러스터로 인정받기 위한 최소 포인트 수
        top_k (int): 가장 큰 클러스터 개수 (기본값: 2)

    Returns:
        cluster_pcd_list (list of dict): 각 클러스터별 PointCloud와 side 정보 [{'pcd': cluster_pcd, 'side': 'left'}, ...]
    """
    
    if top_k != 2:
        print(f"[dbscan] 경고: top1/top2 좌우 판단을 위해 top_k는 2여야 합니다. 현재: {top_k}")
        
    # DBSCAN 클러스터링 수행 (label: -1은 노이즈)
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))

    if labels.size == 0 or np.max(labels) < 0:
        print("[dbscan] 클러스터가 없음")
        o3d.visualization.draw_geometries([pcd], window_name="Filtered Point Cloud")
        return pcd, None

    # 노이즈 제외한 클러스터만 고려
    valid = labels != -1
    unique, counts = np.unique(labels[valid], return_counts=True)

    if len(unique) == 0:
        print("[dbscan] 유효한 클러스터가 없음")
        o3d.visualization.draw_geometries([pcd], window_name="Filtered Point Cloud")
        return pcd, None

    # 1. 가장 큰 top_k 클러스터 추출 (크기 순서로 ID 확보)
    top_clusters_ids_size_order = unique[np.argsort(counts)[-top_k:]]
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    clusters_data = []

    for cluster_id in top_clusters_ids_size_order:
        mask = labels == cluster_id
        cluster_points = points[mask]
        cluster_colors = colors[mask]
        median_x = np.median(cluster_points[:, 0])

        cluster_pcd = o3d.geometry.PointCloud()
        cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)
        cluster_pcd.colors = o3d.utility.Vector3dVector(cluster_colors)
        
        clusters_data.append({
            'original_id': cluster_id,
            'pcd': cluster_pcd,
            'median_x': median_x
        })

    # 2. X축 중앙값 기준 정렬 (왼쪽 -> 오른쪽)
    sorted_clusters = sorted(clusters_data, key=lambda x: x['median_x'])

    # 3. 클러스터별 리스트 생성
    cluster_pcd_list = []

    for i, data in enumerate(sorted_clusters):
        cluster_pcd = data['pcd']

        # 왼쪽/오른쪽 라벨
        side_label = 'left' if i == 0 else 'right'
        
        cluster_pcd_list.append({
            'pcd': cluster_pcd,
            'side': side_label,
            'original_id': data['original_id']
        })

        # 저장
        if aligned_dir is not None:
            top_label = f"top{i+1}"
            save_path = os.path.join(aligned_dir, f"{top_label}_{side_label}.ply")
            o3d.io.write_point_cloud(save_path, cluster_pcd)
            # print(f"[dbscan] 필터링 결과 저장 완료: {save_path}")
        
        # print(f"[dbscan] {top_label} ({side_label}, Original ID {data['original_id']}): Points {len(cluster_pcd.points)}, Median X {data['median_x']:.4f}")

    return cluster_pcd_list