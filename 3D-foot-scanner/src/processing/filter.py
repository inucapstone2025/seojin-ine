import open3d as o3d
import numpy as np

def filter_by_distance_and_ymin(file_path, center_point, radius, y_min, y_max):
    """
    주어진 포인트 클라우드 파일을 불러와서,
    1) 중심점(center_point)으로부터 특정 반경(radius) 이내에 있는 점들만 필터링하고,
    2) y축 좌표(높이)가 y_min 이상 y_max 이하인 점들만 남겨서 필터링합니다.
    
    이 두 조건을 모두 만족하는 점들만 선택하여 새로운 PointCloud 객체를 반환합니다.
    컬러 정보가 있으면 컬러도 필터링한 점들에 맞게 유지합니다.

    Args:
        file_path (str): 필터링할 포인트 클라우드 파일 경로 (.ply 등)
        center_point (list or np.array): 기준이 되는 3차원 중심 좌표 (x, y, z)
        radius (float): 중심점으로부터의 최대 거리 (미터 단위)
        y_min (float): y축 좌표의 최소값 (이하 점들은 제거)
        y_max (float): y축 좌표의 최대값 (이상 점들은 제거)

    Returns:
        filtered_pcd (open3d.geometry.PointCloud): 필터링된 포인트 클라우드 객체
    """

    # 포인트 클라우드 파일 읽기
    pcd = o3d.io.read_point_cloud(file_path)
    points = np.asarray(pcd.points)

    # 각 점과 중심점 간 거리 계산
    distances = np.linalg.norm(points - center_point, axis=1)
    mask_distance = distances < radius  # 반경 이내 점만 True

    # y축 좌표가 y_min 이상인지, y_max 이하인지 체크
    mask_ymin = points[:, 1] >= y_min
    mask_ymax = points[:, 1] <= y_max
    mask_y = np.logical_and(mask_ymin, mask_ymax)  # y 범위 내 점만 True

    # 거리와 y축 범위 조건 모두 만족하는 점만 선택
    mask = np.logical_and(mask_distance, mask_y)

    # 필터링된 좌표만 추출
    filtered_points = points[mask]

    # 새로운 PointCloud 객체에 필터링된 좌표 할당
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

    # 컬러 정보가 있으면 필터링된 점에 맞춰서 컬러도 복사
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        filtered_colors = colors[mask]
        filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

    return filtered_pcd
