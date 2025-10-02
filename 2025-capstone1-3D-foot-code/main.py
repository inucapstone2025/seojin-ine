#!/usr/bin/env python3

from src.utils.config_loader import load_config
from src.capture.realsense_capture import py_Realsense
from src.processing.filter import filter_by_distance_and_ymin
from src.processing.transform import transform_and_save_point_cloud
from src.processing.merge import multi_registration
from src.processing.noise_removal import remove_background_color_from_file, dbscan_largest_clusters
from src.processing.mesh_reconstruction import poisson_mesh_from_pcd
from src.analysis.foot_measurement import measure_both_feet
from src.utils.file_utils import create_dirs

import socket
import open3d as o3d
import numpy as np
import os

def main():
    # 설정 불러오기
    config = load_config()
    paths = config["paths"]

    # 필요한 폴더들 생성
    create_dirs(paths.values())

    # 1. 카메라 촬영 (enabled 시에만)
    if config["camera"]["enabled"]:
        cam = py_Realsense()
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect((config["camera"]["ip"], config["camera"]["port"]))

        #for deg in config["camera"]["degrees"]:
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
    filtered_pcd, top_clusters = dbscan_largest_clusters(filtered, aligned_dir=paths["aligned_dir"])

    # 5. 메쉬 재구성
    mesh_list = poisson_mesh_from_pcd(filtered_pcd, top_clusters, mesh_dir=paths["mesh_dir"])

    print("[4] 메쉬 재구성 완료")
    print("[완료] 모든 단계가 정상적으로 수행되었습니다.")

    # 양 발 메쉬 측정
    mesh_dir = paths["mesh_dir"]
    if os.path.exists(mesh_dir):
        print("\n[양 발 치수 측정 시작]")
        results = measure_both_feet(mesh_dir)

        # 결과 출력 요약
        for file, extent in results:
            print(f"\n[{file}]")
            print(f"  발 길이 (X): {extent[0]:.2f} mm")
            print(f"  발 폭 (Y): {extent[1]:.2f} mm")
            print(f"  발 두께 (Z): {extent[2]:.2f} mm")
    else:
        print("❌ 메쉬 폴더를 찾을 수 없습니다:", mesh_dir)

if __name__ == "__main__":
    main()
