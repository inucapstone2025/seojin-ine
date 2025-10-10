import os
import yaml

def load_config(path=None):
    if path is None:
        # 현재 작업 디렉터리 기준으로 config/config.yaml 위치 지정
        path = os.path.join('config', 'config.yaml')

    with open(path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    base_dir = config["base_dir"]
    user_base_dir = os.path.join(base_dir, config["date_folder"], config["user_folder"])
    config["paths"] = {
        "raw_dir": os.path.join(user_base_dir, "raw"),
        "filtered_dir": os.path.join(user_base_dir, "filtered"),
        "aligned_dir": os.path.join(user_base_dir, "aligned"),
        "mesh_dir": os.path.join(user_base_dir, "mesh"),
        "analysis_dir": os.path.join(user_base_dir, "analysis"),
    }

    return config
