# #!/usr/bin/env python3
# import os
# import open3d as o3d
# from src.utils.config_loader import load_config

# def visualize_pointclouds(paths, degrees=None, mode="raw"):
#     """
#     각 PLY 파일을 하나씩 시각화
#     paths: dict, 'raw_dir', 'filtered_dir', 'aligned_dir' 포함
#     degrees: list, 시각화할 각도 리스트 (None이면 폴더 내 모든 _raw/_filtered/_aligned.ply 파일)
#     mode: "raw", "filtered", "aligned", "final_aligned"
#     """
#     if mode == "raw":
#         folder = paths["raw_dir"]
#         suffix = "_raw.ply"
#         files = sorted([f for f in os.listdir(folder) if f.endswith(suffix)])
#     elif mode == "filtered":
#         folder = paths["filtered_dir"]
#         suffix = "_filtered.ply"
#         files = sorted([f for f in os.listdir(folder) if f.endswith(suffix)])
#     elif mode == "aligned":
#         folder = paths["aligned_dir"]
#         suffix = "_aligned.ply"
#         files = sorted([f for f in os.listdir(folder) if f.endswith(suffix)])
#     elif mode == "final_aligned":
#         folder = paths["aligned_dir"]
#         final_file = os.path.join(folder, "final_aligned.ply")
#         if not os.path.exists(final_file):
#             print(f"❌ final_aligned.ply 파일이 없습니다: {final_file}")
#             return
#         files = [final_file]  # 단일 파일 리스트
#     else:
#         raise ValueError("mode는 'raw', 'filtered', 'aligned', 'final_aligned'만 가능합니다.")

#     # degrees 지정 시 파일 선택
#     if degrees is not None and mode != "final_aligned":
#         files = [os.path.join(folder, f"cloud_{deg}{suffix}") for deg in degrees]

#     for file_path in files:
#         if not os.path.exists(file_path):
#             print(f"❌ 파일이 없습니다: {file_path}")
#             continue
        
#         print(f"[INFO] 시각화 중: {file_path}")
#         pcd = o3d.io.read_point_cloud(file_path)
#         # pcd.paint_uniform_color([0.7, 0.7, 0.7])
#         o3d.visualization.draw_geometries([pcd], window_name=os.path.basename(file_path))

# if __name__ == "__main__":
#     config = load_config()
#     paths = config["paths"]
#     degrees = config["camera"].get("degrees", None)

#     for mode in ["raw", "filtered", "aligned", "final_aligned"]:
#         print(f"=== {mode.replace('_',' ').capitalize()} Point Clouds ===")
#         visualize_pointclouds(paths, degrees, mode=mode)

#!/usr/bin/env python3
import os
import open3d as o3d
from src.utils.config_loader import load_config

def visualize_merged_aligned(paths, degrees=None):
    """
    모든 aligned PLY 파일을 합쳐서 한 번에 시각화
    paths: dict, 'aligned_dir' 포함
    degrees: list, 시각화할 각도 리스트 (None이면 폴더 내 모든 _aligned.ply 파일)
    """
    aligned_dir = paths["aligned_dir"]
    
    # 파일 리스트 구성
    if degrees is not None:
        files = [f"cloud_{deg}_aligned.ply" for deg in degrees]
    else:
        files = sorted([f for f in os.listdir(aligned_dir) if f.endswith("_aligned.ply")])
    
    merged_pcd = o3d.geometry.PointCloud()

    for file in files:
        file_path = os.path.join(aligned_dir, file)
        if not os.path.exists(file_path):
            print(f"❌ 파일이 없습니다: {file_path}")
            continue
        
        print(f"[INFO] 불러오는 중: {file_path}")
        pcd = o3d.io.read_point_cloud(file_path)
        merged_pcd += pcd  # 포인트 클라우드 합치기
    
    # 색상 지정 (회색)
    # merged_pcd.paint_uniform_color([0.7, 0.7, 0.7])
    
    print("[INFO] 모든 aligned PLY 파일 합쳐서 시각화")
    o3d.visualization.draw_geometries([merged_pcd], window_name="Merged Aligned PointCloud")

if __name__ == "__main__":
    config = load_config()
    paths = config["paths"]
    degrees = config["camera"].get("degrees", None)
    
    visualize_merged_aligned(paths, degrees)
