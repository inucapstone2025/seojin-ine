import open3d as o3d
import numpy as np
import os

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

    # 컬러 정보 추출 (0~1) → 0~255로 정규화
    colors = np.asarray(pcd.colors)
    rgb_255 = (colors * 255).astype(np.uint8)

    # G값이 threshold보다 큰 포인트는 배경으로 간주하여 제거
    mask = ~(rgb_255[:, 1] > green_threshold)

    # 배경이 아닌 포인트만 선택
    filtered_points = np.asarray(pcd.points)[mask]
    filtered_colors = colors[mask]

    # 새로운 포인트 클라우드 객체로 생성
    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(filtered_points)
    result.colors = o3d.utility.Vector3dVector(filtered_colors)

    print(f"[noise_removal] 필터링 전: {len(colors)}, 필터링 후: {len(filtered_colors)}")
    return result

def dbscan_largest_clusters(pcd, aligned_dir=None, eps=0.01, min_points=20, top_k=2):
    """
    DBSCAN 클러스터링을 통해 가장 큰 상위 K개의 클러스터만 필터링하는 함수

    Parameters:
        pcd (o3d.geometry.PointCloud): 입력 포인트 클라우드
        aligned_dir (str): 필터링된 결과를 저장할 경로 (None이면 저장하지 않음)
        eps (float): DBSCAN 거리 임계값
        min_points (int): 클러스터로 인정받기 위한 최소 포인트 수
        top_k (int): 가장 큰 클러스터 개수 (기본값: 2)

    Returns:
        top_pcd (o3d.geometry.PointCloud): 상위 K개 클러스터만 포함된 포인트 클라우드
        top_clusters (np.ndarray or None): 추출된 클러스터 ID 목록 (없으면 None)
    """

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

    # 가장 큰 top_k 클러스터 추출
    top_clusters = unique[np.argsort(counts)[-top_k:]]

    # top_k 클러스터에 해당하는 포인트만 필터링
    mask = np.isin(labels, top_clusters)
    filtered_points = np.asarray(pcd.points)[mask]
    filtered_colors = np.asarray(pcd.colors)[mask]

    # 새로운 포인트 클라우드 객체로 생성
    top_pcd = o3d.geometry.PointCloud()
    top_pcd.points = o3d.utility.Vector3dVector(filtered_points)
    top_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

    print(f"[dbscan] 최종 출력 포인트 수 (상위 {top_k}개 클러스터): {len(filtered_points)}")
    o3d.visualization.draw_geometries([top_pcd], window_name=f"Top {top_k} Clusters Only")

    # 결과 저장
    if aligned_dir is not None:
        save_path = os.path.join(aligned_dir, "final_filtered_aligned.ply")
        o3d.io.write_point_cloud(save_path, top_pcd)
        print(f"[dbscan] 필터링 결과 저장 완료: {save_path}")

    return top_pcd, top_clusters
