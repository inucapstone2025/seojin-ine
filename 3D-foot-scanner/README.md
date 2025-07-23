# 3D Foot Scanner 프로젝트

## 프로젝트 개요
이 프로젝트는 Intel RealSense D435 카메라를 이용해 발 모양을 3D 스캔하고,  
Open3D를 활용해 포인트 클라우드 처리, 정합, 노이즈 제거, 메시 생성 등을 수행하는 시스템입니다.

---

## 폴더 구조 및 파일 설명

- 3D_foot_scanner/
  - arduino/
    - esp32-button-motor-tracker-wifi.ino  
      (ESP32 기반 모터 제어 및 버튼 입력, Wi-Fi 통신 아두이노 코드)
  - config/
    - config.yaml  
      (주요 파라미터 및 경로 설정 파일)
  - data/  
    (스캔된 원본 및 처리된 데이터 저장 폴더)
  - src/
    - capture/
      - realsense_capture.py  
        (RealSense 카메라 캡처 관련 코드)
    - processing/
      - filter.py  
        (포인트 클라우드 거리 및 y축 필터링)
      - merge.py  
        (여러 포인트 클라우드 정합 및 병합)
      - mesh_reconstruction.py  
        (Poisson 메시 생성)
      - noise_removal.py  
        (배경색 제거 및 DBSCAN 기반 노이즈 제거)
      - transform.py  
        (회전 및 변환 처리 코드)
    - utils/
      - config_loader.py  
        (YAML 설정 파일 로드 및 경로 설정)
      - file_utils.py  
        (폴더 생성, 파일 경로 관리 등 유틸리티 함수)
  - main.py  
    (프로젝트 메인 실행 파일)
  - environment.yml  
    (Anaconda 환경 설정 파일)
  - requirements.txt  
    (Python 패키지 의존성 리스트)
  - README.md  
    (프로젝트 개요 및 설명 문서) 

---

## 개발 환경

- OS: Windows 11  
- Python: 3.8.20 (Anaconda 환경 'realsense' 사용)  
- 패키지 관리자: conda & pip 혼용  
- 주요 패키지 버전:  
  - numpy 1.24.4  
  - open3d 0.19.0  
  - pyrealsense2 2.55.1.6486  
  - scikit-learn 1.3.2  
  - scipy 1.10.1  
  - opencv-python 4.12.0.88  
- 통신: 내장 socket 모듈 사용  
- 비고:  
  - 포인트 클라우드 처리는 주로 Open3D 라이브러리를 활용  
  - Intel RealSense D435 카메라와 pyrealsense2 라이브러리 연동  
  - 시각화 및 모델링은 Open3D에서 수행  
  - 전체 개발 환경은 Anaconda로 관리  

---

## 사용 방법

1. 환경 세팅  
2. config/config.yaml 파일 내 경로 및 파라미터 확인 및 수정
3. main.py 실행하여 전체 스캔 및 처리 파이프라인 수행  
- python main.py
