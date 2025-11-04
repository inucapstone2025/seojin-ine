import open3d as o3d
import numpy as np
import os

# # 코드 추가 - limit_icp_to_pitch_translation 함수
# def limit_icp_to_pitch_translation(T):
#     """
#     ICP 결과 4x4 행렬에서 Pitch(Y축 회전)와 평행이동만 남기고 Roll, Yaw 제거
#     """
#     # 회전 부분 추출
#     R = T[:3, :3]

#     # Pitch 계산 (Y축 회전)
#     pitch = np.arcsin(-R[2, 0])  # R[2,0] = -sin(pitch)
    
#     # Pitch 회전 행렬 재구성
#     R_pitch = np.array([
#         [ np.cos(pitch), 0, np.sin(pitch)],
#         [ 0,            1, 0           ],
#         [-np.sin(pitch), 0, np.cos(pitch)]
#     ])

#     # 평행이동 그대로 사용
#     t = T[:3, 3]

#     # 새로운 4x4 행렬 생성
#     T_new = np.eye(4)
#     T_new[:3, :3] = R_pitch
#     T_new[:3, 3] = t

#     return T_new

# def limit_icp_to_pitch_translation(T, lock_y_translation=True):
#     # T: 4x4 변환 행렬
#     R = T[:3, :3]
#     t = T[:3, 3].copy()  # ✅ copy() 추가

#     if lock_y_translation:
#         t[1] = 0  # 높이 방향 이동 제거

#     T_limited = np.eye(4)
#     T_limited[:3, :3] = R
#     T_limited[:3, 3] = t
#     return T_limited

def pairwise_registration_xz_translation_only(source, target, voxel_size):
    """
    ICP를 수행하되, 결과 변환을 x,z 평행이동만 허용하도록 제한한 버전.
    즉, 회전 및 y축 이동은 제거함.
    """
    # 다운샘플링
    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size)

    # 노멀 계산
    source_down.estimate_normals()
    target_down.estimate_normals()

    # 초기 변환
    trans_init = np.eye(4)

    # 일반 ICP 수행
    reg = o3d.pipelines.registration.registration_icp(
        source_down, target_down, voxel_size * 2.0,
        trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )

    # 변환 행렬 제한 (x,z 이동만 남기기)
    T = reg.transformation
    t = T[:3, 3]

    # x,z만 남기고 나머지 제거
    tx, tz = t[0], t[2]

    T_limited = np.eye(4)
    T_limited[0, 3] = tx
    T_limited[2, 3] = tz

    return T_limited


def pairwise_registration(source, target, voxel_size):
    """
    두 포인트 클라우드(source)를 기준 포인트 클라우드(target)에 정합시키는 함수입니다.
    
    1) 각 클라우드를 voxel_size 크기로 다운샘플링합니다.
    2) 다운샘플된 클라우드에 대해 노멀(법선)을 추정합니다.
    3) ICP(Iterative Closest Point) 알고리즘의 point-to-plane 방법을 사용해
       source 클라우드를 target 클라우드에 맞게 정합하는 변환 행렬을 계산합니다.
    
    Args:
        source (open3d.geometry.PointCloud): 변환할 포인트 클라우드
        target (open3d.geometry.PointCloud): 기준 포인트 클라우드
        voxel_size (float): 다운샘플링 voxel 크기 (단위: m)
    
    Returns:
        np.ndarray: source를 target에 맞추기 위한 4x4 변환 행렬
    """
    # 다운샘플링
    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size)

    # 노멀 계산
    source_down.estimate_normals()
    target_down.estimate_normals()
    # source_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
    # target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

    # 초기 변환 행렬 (단위행렬)
    trans_init = np.eye(4)

    # ICP point-to-plane 정합 수행
    reg = o3d.pipelines.registration.registration_icp(
        source_down, target_down, voxel_size * 2.0, # max_correspondence_distance는 ICP가 포인트 쌍을 찾을 때 최대 거리를 의미
        # 원래 * 2.0 이었음
        trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )
    return reg.transformation


def multi_registration(pcd_files, voxel_size, save_path=None):
    """
    여러 포인트 클라우드 파일들을 첫 번째 클라우드를 기준으로 차례로 정합하고 병합하는 함수입니다.
    병합 후에는 YZ축을 반전시켜 좌표계를 보정합니다.
    save_path가 지정되면 결과를 파일로 저장합니다.

    Args:
        pcd_files (list of str): 정합할 포인트 클라우드 파일 경로 리스트
        voxel_size (float): 다운샘플링 voxel 크기 (단위: m)
        save_path (str or None): 결과를 저장할 파일 경로, None이면 저장하지 않음

    Returns:
        open3d.geometry.PointCloud: 병합 및 좌표계 보정된 최종 포인트 클라우드
    """
    # 모든 파일에서 포인트 클라우드 로드
    pcds = [o3d.io.read_point_cloud(f) for f in pcd_files]
    # print(f"[merge] 총 {len(pcds)}개 포인트 클라우드 로드 완료")

    target = pcds[0]
    for i in range(1, len(pcds)):
        # print(f"[merge] 정합 중: {i}/{len(pcds)-1}")
        # 현재 클라우드를 target에 맞게 정합할 변환 계산
        T = pairwise_registration(pcds[i], target, voxel_size)

        # 교체
        # T = pairwise_registration_xz_translation_only(pcds[i], target, voxel_size)
        # T = limit_icp_to_pitch_translation(T)  # Roll/Yaw 제거, Pitch + 평행이동만 남김 ===========> *** 코드 추가 ****
        # T = limit_icp_to_pitch_translation(T, lock_y_translation=True)
        
        # 변환 적용
        pcds[i].transform(T)
        # target에 병합
        target += pcds[i]
        # 병합 후 다운샘플링으로 크기 감소
        target = target.voxel_down_sample(voxel_size)

    # YZ축 반전 처리 (좌표계 보정)
    points = np.asarray(target.points)
    points *= np.array([1, -1, -1])
    target.points = o3d.utility.Vector3dVector(points)

    # 결과 저장
    if save_path is not None:
        o3d.io.write_point_cloud(save_path, target)
        # print(f"[merge] 병합 및 변환 결과 저장 완료: {save_path}")

    return target
