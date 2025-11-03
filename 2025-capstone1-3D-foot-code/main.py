#!/usr/bin/env python3

from src.utils.config_loader import load_config
from src.capture.realsense_capture import py_Realsense
from src.processing.filter import filter_by_distance_and_ymin
from src.processing.transform import transform_and_save_point_cloud
from src.processing.merge import multi_registration
from src.processing.noise_removal import remove_background_color_from_file, dbscan_largest_clusters
from src.processing.mesh_reconstruction import poisson_mesh_from_clusters
from src.analysis.foot_measurement import measure_both_feet
from src.analysis.foot_bti import analyze_foot_bti
from src.utils.file_utils import create_dirs
from src.display.fbti_info import get_qr_image, fbti_qr_map
from web_ui.server import launch_web_ui

import socket
import open3d as o3d
import numpy as np
import os
from PIL import Image
import cv2

def main():
    # 설정 불러오기
    config = load_config()
    paths = config["paths"]

    # 필요한 폴더들 생성
    create_dirs(paths.values())

    if config["mode"]["use_raspberry_pi"]:
        print("[MODE] Raspberry Pi 원격 촬영 모드 → 웹 UI 사용")
        session_dir = launch_web_ui(config)
        if not session_dir:
            print("Pi 촬영 실패")
            return
        paths["raw_dir"] = os.path.join(session_dir, "raw")
        paths["filtered_dir"] = os.path.join(session_dir, "filtered")
        paths["aligned_dir"] = os.path.join(session_dir, "aligned")
        paths["mesh_dir"] = os.path.join(session_dir, "mesh")
    else:
        print("[MODE] 로컬 RealSense 촬영 모드")

        # 1. 카메라 촬영 (enabled 시에만)
        if config["camera"]["enabled"]:
            cam = py_Realsense()
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((config["camera"]["ip"], config["camera"]["port"]))

            while True:
                get_data = client.recv(1024).decode().strip()
                
                if not get_data:
                    continue
                if get_data == "end":
                    break

                deg = int(get_data) # 수신된 각도 사용
                print(f"[DEBUG] Received from ESP32: '{deg}'") # ===== 디버그 코드 출력 ===== 

                vertices, colors, _ = cam.capture()
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(np.array([v.tolist() for v in vertices]))
                pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

                # raw ply만 저장 (필터링 파일은 여기서 만들지 않음)
                save_path = os.path.join(paths["raw_dir"], f"cloud_{deg}_raw.ply")
                o3d.io.write_point_cloud(save_path, pcd)

                client.send(b"go\n")

            print("[1] 카메라 촬영 완료")
        else:
            print("[1] 카메라 촬영 비활성화 (이미 저장된 raw 파일 사용)")

    # 2. 필터링
    print("[2] 필터링 시작")
    for deg in config["camera"]["degrees"]:
        in_path = os.path.join(paths["raw_dir"], f"cloud_{deg}_raw.ply")
        out_path = os.path.join(paths["filtered_dir"], f"cloud_{deg}_filtered.ply")

        pcd = filter_by_distance_and_ymin(in_path, **config["filter"])
        o3d.io.write_point_cloud(out_path, pcd)

        # 필터링 후 정합을 위해 정합용 폴더에 저장
        aligned_path = os.path.join(paths["aligned_dir"], f"cloud_{deg}_aligned.ply")
        transform_and_save_point_cloud(out_path, deg, config["align"]["radius"], aligned_path)

    print("[3] 필터링 및 정합 완료")

    # 3. 병합
    aligned_files = [os.path.join(paths["aligned_dir"], f"cloud_{d}_aligned.ply") for d in config["camera"]["degrees"]]
    final_save_path = os.path.join(paths["aligned_dir"], "final_aligned.ply")
    merged = multi_registration(aligned_files, config["merge"]["voxel_size"], save_path=final_save_path)

    # 4. 노이즈 제거 및 클러스터링
    filtered = remove_background_color_from_file(final_save_path)
    # dbscan_largest_clusters에서 cluster별 PointCloud 리스트 반환
    cluster_pcd_list = dbscan_largest_clusters(filtered, aligned_dir=paths["aligned_dir"])

    # 5. 메쉬 재구성 (왼/오 순서 유지)
    poisson_mesh_from_clusters(cluster_pcd_list, mesh_dir=paths["mesh_dir"])

    print("[4] 메쉬 재구성 완료")
    print("[완료] 모든 단계가 정상적으로 수행되었습니다.")

    # 양 발 메쉬 측정
    mesh_dir = paths["mesh_dir"]
    if os.path.exists(mesh_dir):
        # print("\n[양 발 치수 측정 시작]")
        results, visuals, measurements = measure_both_feet(mesh_dir)
    else:
        print("❌ 메쉬 폴더를 찾을 수 없습니다:", mesh_dir)

    left_pcd = visuals[0]
    left_aabb = visuals[1]

    bti_results = analyze_foot_bti(left_pcd, left_aabb)
    print("===== 발BTI 분석 결과 =====")
    for key, value in bti_results.items():
        print(f"{key}: {value}")

    # BTI 조합 문자열 만들기 (예: EWIL)
    bti_combination = bti_results["shape"] + bti_results["width"] + bti_results["instep"] + bti_results["arch"]

    # QR 이미지 열기
    # try:
    #     qr_img = get_qr_image(bti_combination)
    #     qr_img.show()  # 기본 이미지 뷰어로 표시
    # except FileNotFoundError as e:
    #     print(e)

    # OpenCV로 QR 이미지 표시
    qr_path = fbti_qr_map.get(bti_combination)
    if qr_path and os.path.exists(qr_path):
        img = cv2.imread(qr_path)
        cv2.imshow(f"FBTI QR - {bti_combination}", img)
        print(f"[INFO] {bti_combination} QR 이미지 표시 중... 키 입력 후 창 닫기")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print(f"❌ {bti_combination}에 대한 QR 이미지가 없습니다.")

if __name__ == "__main__":
    main()

# #!/usr/bin/env python3

# from src.utils.config_loader import load_config
# from src.capture.realsense_capture import py_Realsense
# from src.processing.filter import filter_by_distance_and_ymin
# from src.processing.transform import transform_and_save_point_cloud
# from src.processing.merge import multi_registration
# from src.processing.noise_removal import remove_background_color_from_file, dbscan_largest_clusters
# from src.processing.mesh_reconstruction import poisson_mesh_from_clusters
# from src.analysis.foot_measurement import measure_both_feet
# from src.analysis.foot_bti import analyze_foot_bti
# from src.utils.file_utils import create_dirs
# from src.display.fbti_info import get_qr_image, fbti_qr_map
# from web_ui.server import launch_web_ui

# import socket
# import open3d as o3d
# import numpy as np
# import os
# from PIL import Image
# import cv2
# import time

# def process_session(paths, config):
#     # 2. 필터링
#     # print("[2] 필터링 시작")
#     for deg in config["camera"]["degrees"]:
#         in_path = os.path.join(paths["raw_dir"], f"cloud_{deg}_raw.ply")
#         out_path = os.path.join(paths["filtered_dir"], f"cloud_{deg}_filtered.ply")

#         pcd = filter_by_distance_and_ymin(in_path, **config["filter"])
#         o3d.io.write_point_cloud(out_path, pcd)

#         # 필터링 후 정합을 위해 정합용 폴더에 저장
#         aligned_path = os.path.join(paths["aligned_dir"], f"cloud_{deg}_aligned.ply")
#         transform_and_save_point_cloud(out_path, deg, config["align"]["radius"], aligned_path)

#     # print("[3] 필터링 및 정합 완료")

#     # 3. 병합
#     aligned_files = [os.path.join(paths["aligned_dir"], f"cloud_{d}_aligned.ply") for d in config["camera"]["degrees"]]
#     final_save_path = os.path.join(paths["aligned_dir"], "final_aligned.ply")
#     merged = multi_registration(aligned_files, config["merge"]["voxel_size"], save_path=final_save_path)

#     # 4. 노이즈 제거 및 클러스터링
#     filtered = remove_background_color_from_file(final_save_path)
#     # dbscan_largest_clusters에서 cluster별 PointCloud 리스트 반환
#     cluster_pcd_list = dbscan_largest_clusters(filtered, aligned_dir=paths["aligned_dir"])

#     # 5. 메쉬 재구성 (왼/오 순서 유지)
#     poisson_mesh_from_clusters(cluster_pcd_list, mesh_dir=paths["mesh_dir"])

#     # print("[4] 메쉬 재구성 완료")
#     # print("[완료] 모든 단계가 정상적으로 수행되었습니다.")

#     # 양 발 메쉬 측정
#     mesh_dir = paths["mesh_dir"]
#     if os.path.exists(mesh_dir):
#         # print("\n[양 발 치수 측정 시작]")
#         results, visuals, measurements = measure_both_feet(mesh_dir)
#     else:
#         print("❌ 메쉬 폴더를 찾을 수 없습니다:", mesh_dir)

#     left_pcd = visuals[0]
#     left_aabb = visuals[1]

#     bti_results = analyze_foot_bti(left_pcd, left_aabb)
#     print("\n===== 발BTI 분석 결과 =====")
#     for key, value in bti_results.items():
#         print(f"{key}: {value}")

#     # BTI 조합 문자열 만들기 (예: EWIL)
#     bti_combination = bti_results["shape"] + bti_results["width"] + bti_results["instep"] + bti_results["arch"]

#     # QR 이미지 열기
#     # try:
#     #     qr_img = get_qr_image(bti_combination)
#     #     qr_img.show()  # 기본 이미지 뷰어로 표시
#     # except FileNotFoundError as e:
#     #     print(e)

#     # OpenCV로 QR 이미지 표시
#     qr_path = fbti_qr_map.get(bti_combination)
#     if qr_path and os.path.exists(qr_path):
#         img = cv2.imread(qr_path)
#         cv2.imshow(f"FBTI QR - {bti_combination}", img)
#         print(f"[INFO] {bti_combination} QR 이미지 표시 중... 키 입력 후 창 닫기")
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()
#     else:
#         print(f"❌ {bti_combination}에 대한 QR 이미지가 없습니다.")

# def main():
#     # 설정 불러오기
#     config = load_config()
#     paths = config["paths"]

#     # 필요한 폴더들 생성
#     create_dirs(paths.values())

#     if config["mode"]["use_raspberry_pi"]:
#         print("[MODE] Raspberry Pi 원격 촬영 모드 → 웹 UI 사용")

#         while True: 
#             session_dir = launch_web_ui(config)
#             if not session_dir:
#                 print("Pi 촬영 실패")
#                 return
            
#             print(f"\n[SESSION] 새로운 사용자 세션 시작: {session_dir}\n")
            
#             # 세션별 폴더 갱신
#             paths["raw_dir"] = os.path.join(session_dir, "raw")
#             paths["filtered_dir"] = os.path.join(session_dir, "filtered")
#             paths["aligned_dir"] = os.path.join(session_dir, "aligned")
#             paths["mesh_dir"] = os.path.join(session_dir, "mesh")

#             # 한 세션의 전체 처리 수행
#             process_session(paths, config)

#             print("\n✅ 한 사용자의 측정 및 분석 완료!")
#             print("===============================================")
#             print("다음 사용자가 촬영을 시작하려면 웹 UI를 다시 열어주세요.")
#             print("===============================================\n")

#             # 잠깐 대기 (다음 사용자 준비 시간)
#             time.sleep(3)

#     else:
#         # print("[MODE] 로컬 RealSense 촬영 모드")

#         # 1. 카메라 촬영 (enabled 시에만)
#         if config["camera"]["enabled"]:
#             cam = py_Realsense()
#             client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             client.connect((config["camera"]["ip"], config["camera"]["port"]))

#             while True:
#                 get_data = client.recv(1024).decode().strip()
                
#                 if not get_data:
#                     continue
#                 if get_data == "end":
#                     break

#                 deg = int(get_data) # 수신된 각도 사용
#                 print(f"[DEBUG] Received from ESP32: '{deg}'") # ===== 디버그 코드 출력 ===== 

#                 vertices, colors, _ = cam.capture()
#                 pcd = o3d.geometry.PointCloud()
#                 pcd.points = o3d.utility.Vector3dVector(np.array([v.tolist() for v in vertices]))
#                 pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

#                 # raw ply만 저장 (필터링 파일은 여기서 만들지 않음)
#                 save_path = os.path.join(paths["raw_dir"], f"cloud_{deg}_raw.ply")
#                 o3d.io.write_point_cloud(save_path, pcd)

#                 client.send(b"go\n")

#             print("[1] 카메라 촬영 완료")
#             process_session(paths, config)
#         else:
#             # print("[1] 카메라 촬영 비활성화 (이미 저장된 raw 파일 사용)")
#             process_session(paths, config)

# if __name__ == "__main__":
#     main()
