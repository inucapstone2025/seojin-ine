import os

def create_dirs(dir_list):
    for d in dir_list:
        os.makedirs(d, exist_ok=True)
