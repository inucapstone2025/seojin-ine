import open3d as o3d
import numpy as np

def transform_and_save_point_cloud(file_path, degree, radius, output_path):
    """
    주어진 각도와 반지름에 따라 Point Cloud를 회전+이동 변환하여 저장하는 함수.

    Parameters:
    - file_path: 입력 포인트 클라우드 파일 경로 (.pcd)
    - degree: 회전 각도 (도 단위)
    - radius: 카메라 원 반지름 (m)
    - output_path: 저장 경로 (기본값은 'cloud_{degree}_aligned.pcd')
    """
    # 포인트 클라우드 로드
    pcd = o3d.io.read_point_cloud(file_path)

    # 각도 → 라디안
    theta = np.radians(degree)
    
    # 회전 행렬 (yaw 회전)
    R = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    
    # 카메라 위치 계산 (반지름 적용)
    tx = radius * np.sin(theta)
    tz = radius * np.cos(theta)

    # 4x4 변환 행렬 구성
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [-tx, 0.0, -tz] # 이동 벡터

    # 변환 적용
    pcd.transform(T)

    o3d.io.write_point_cloud(output_path, pcd)
