#!/usr/bin/env python3

from src.utils.config_loader import load_config
from src.processing.filter import filter_by_distance_and_ymin
from src.processing.transform import transform_and_save_point_cloud
from src.processing.merge import multi_registration
from src.processing.noise_removal import remove_background_color_from_file, dbscan_largest_clusters
from src.processing.mesh_reconstruction import poisson_mesh_from_pcd
from pathlib import Path

from src.analysis.foot_measurement import measure_both_feet
from src.analysis.reporting import (
    format_measurement_summary,
    save_measurements_csv,
)
from src.analysis.toe_detection import ToeDetectionParams
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
        try:
            from src.capture.realsense_capture import py_Realsense
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "RealSense 촬영을 사용하려면 'pyrealsense2' 패키지가 필요합니다. "
                "패키지를 설치하거나 config.yaml에서 camera.enabled 값을 false로 설정하세요."
            ) from exc

        cam = py_Realsense()
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect((config["camera"]["ip"], config["camera"]["port"]))

        while True:
            get_data = client.recv(1024).decode().strip()

            if not get_data:
                continue
            if get_data == "end":
                break

            deg = int(get_data)
            print(f"[DEBUG] Received from ESP32: '{deg}'")

            vertices, colors, _ = cam.capture()
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array([v.tolist() for v in vertices]))
            pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

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

    analysis_cfg = config.get("analysis", {})
    toe_cfg = dict(analysis_cfg.get("toe_detection", {}))
    toe_detection_enabled = toe_cfg.pop("enabled", True)
    toe_params = ToeDetectionParams(**toe_cfg) if toe_detection_enabled else None
    debug_cfg = dict(analysis_cfg.get("debug", {}))

    # 양 발 메쉬 측정
    mesh_dir = paths["mesh_dir"]
    if os.path.exists(mesh_dir):
        print("\n[양 발 치수 측정 시작]")
        results = measure_both_feet(
            mesh_dir,
            toe_params=toe_params,
            toe_detection_enabled=toe_detection_enabled,
            debug_config=debug_cfg,
            reference_cloud=filtered_pcd,
        )

        measurement_entries = []

        for result in results:
            print(f"\n[{result.file_name}]")
            print(
                f"  AABB (mm) - X:{result.extent_mm[0]:.1f}, "
                f"Y:{result.extent_mm[1]:.1f}, Z:{result.extent_mm[2]:.1f}"
            )
            if result.toe_measurements:
                print("  " + format_measurement_summary(result.file_name, result.toe_measurements))
                measurement_entries.append((result.file_name, result.toe_measurements))
            else:
                print("  ⚠️  발가락 자동 탐지 실패")

        if measurement_entries:
            analysis_dir = Path(paths["analysis_dir"])
            csv_path = analysis_dir / "toe_measurements.csv"
            save_measurements_csv(csv_path, measurement_entries)
            print(f"\n[저장] 발가락 치수 CSV → {csv_path}")
    else:
        print("❌ 메쉬 폴더를 찾을 수 없습니다:", mesh_dir)

if __name__ == "__main__":
    main()
