## 🗓️날짜 및 시간

2025년 3월 17일 월요일 16:30-18:00

## 🗽장소

온라인 zoom

## 🙇🏻‍♂️참여자

송샘, 이서진, 백승욱, 박보성

## 🏆목표

구현을 위한 기술 조사해오기

## 🥰 자료조사

---

## <이서진>

### 1. NeRF

: 이미지를 3D 장면으로 변환

- 논문
    
    NeRF 최초 논문
    
    [NeRF.pdf](attachment:727ecbbd-e560-4eff-b8fd-274c3c3d66d0:NeRF.pdf)
    
- git
    
    pythorch로 구현되어 있는 것 중에서 가장많은 star를 받은 오픈소스코드
    
    https://github.com/yenchenlin/nerf-pytorch
    
- 참고 자료

https://xoft.tistory.com/15

### 2. Instant NGP (Instant Neural Graphics Primitives)

: NeRF의 학습 속도를 극적으로 단축한 버전

기존 NeRF는 학습 속도가 느려서 실시간 사용이 어려웠는데, “멀티해시 인코딩”을 이용해 수 초~수 분 내에 3D 신(scene)을 학습하고 실시간 렌더링이 가능해짐

→ nvidia gpu 필수..

- 논문
    
    [Instant NGP.pdf](attachment:8530f2ea-251d-4662-bc6f-24a69c3f5707:Instant_NGP.pdf)
    
- git

https://github.com/NVlabs/instant-ngp

- 참고 자료

https://minhong.tistory.com/9

https://ikaros79.tistory.com/entry/Instant-NGP-01-Windows%EC%97%90%EC%84%9C-%EC%84%A4%EC%B9%98%ED%95%98%EA%B8%B0

https://xoft.tistory.com/13

### 3. PixelNeRF, MVSNeRF, MetaNERF

: 최초 NeRF에서는 Scene마다 적은 수의 학습 데이터를 사용 할 경우 성능이 좋지 않다.

이를 개선한 논문으로 적은 수 (1장 or 3장)의 이미지로도 학습이 가능한 것

- 참고 자료

https://xoft.tistory.com/18

---

### 4. COLMAP

: SFM과 MVS를 범용적으로 사용할 수 있게 만든 라이브러리

SFM은 이미지들을 입력으로 받아 Camera Parameter와 Point Cloud를 생성하고,

MVS는 ‘SFM 결과값’을 입력으로 받아 3D 모델을 Reconstruction(재건)함.

COLMAP을 사용하면 SFM을 자동으로 실행하고, 추가적으로 MVS까지 사용해서 더 촘촘한 3D 데이터를 생성할 수 있음

(COLMAP 내부 알고리즘 과정)

1. SFM 수행 → 카메라 위치와 3D 점군 생성
2. MVS 수행 → 점군을 더 촘촘하게 보정
3. 메쉬(Mesh) 변환 → 포인트 클라우드를 3D 모델로 변환 

- 논문
    
    X
    
- git

https://github.com/colmap/colmap

https://colmap.github.io/index.html

- 참고 자료

https://xoft.tistory.com/88

https://velog.io/@hsbc/SfMStructure-from-Motion-of-COLMAP

https://jseobyun.tistory.com/383

### 5. SFM(Structure From Motion)

동일한 객체를 다른 시점에서 중첩되도록 찍은 multi-view 이미지들로부터 3D structure 와 camera pose를 복원하는 프로세스

여러 장의 2D 이미지를 사용하여 카메라의 위치와 3D 포인트 클라우드를 추정하는 기술

(방법)

1. **Feature Detection** (특징점 검출) → 이미지에서 특징점을 찾음.
2. **Feature Matching** (특징점 매칭) → 여러 이미지에서 같은 특징점을 연결.
3. **Camera Pose Estimation** (카메라 위치 추정) → 카메라의 위치와 각도를 계산.
4. **3D Point Cloud Reconstruction** (3D 점군 복원) → 매칭된 특징점을 기반으로 3D 구조 생성.

- 논문
    
    [SFM & Colmap.pdf](attachment:d29f145c-53e3-47ec-987e-186cfe164dc7:SFM__Colmap.pdf)
    
- git

- 참고 자료

https://mvje.tistory.com/92

https://xoft.tistory.com/88

https://woochan-autobiography.tistory.com/950

### 6. MVS

: MVS(Multi-View Stereo)는 여러 장의 2D 이미지를 이용해 더 **정확하고 촘촘한 3D 모델을 생성하는 기술**

### **✅ SFM과의 차이점**

- **SFM (Structure from Motion)**: 여러 장의 사진을 분석해 **카메라의 위치와 일부 3D 점들(포인트 클라우드)**을 추정하는 기법.
- **MVS (Multi-View Stereo)**: SFM으로 얻은 카메라 위치를 활용해 **더 많은 3D 점을 계산하여 밀집된(denser) 3D 모델을 생성**하는 기법.

즉, **SFM이 거친 뼈대(희박한 점군, sparse point cloud)를 만든다면, MVS는 이를 기반으로 더 촘촘한 점군(밀집된 점군, dense point cloud)을 만들어 완전한 3D 모델을 생성하는 과정**

- 참고 자료

---

[쓰레기통 (1)](https://www.notion.so/1-1ba2e6f3c46f80f88639f1847899e619?pvs=21)

---

## <송샘>

## **1️⃣ 하드웨어 설계 및 데이터 수집**

📌 **사용할 주요 센서 및 역할**

| 센서/장치 | 역할 |
| --- | --- |
| **ESP32** | 센서 데이터 수집 및 네트워크 전송 |
| **Depth Camera (Luxonis OAK-D)** | 발의 3D 형상 및 보행 분석, AI 가속 기능 내장 |
| **MPU9250 (9축 센서)** | 발의 기울기, 회전 감지, 보행 중 자세 분석 |
| **압력 센서 (FSR/Flex Sensor)** | 보행 중 발바닥 압력 측정 (아치 높이, 착지 패턴 분석) |
| **2D LiDAR** | 보행 경로 및 움직임 분석 (3D Lidar 대체) |
| **Hailo-26TOPS** | AI 가속기로 실시간 데이터 분석 |

### **🚀 단계 1: 하드웨어 연결 및 데이터 수집**

✅ **ESP32에 센서 연결**

- **MPU9250 → I2C로 연결 (자이로 데이터 수집)**
- **압력 센서 → ADC로 연결 (압력 데이터 수집)**
- **LiDAR → UART 또는 I2C 연결 (보행 패턴 측정)**
- **Depth Camera → USB로 Jetson Nano/Xavier NX에 연결**

✅ **센서 데이터 동기화**

- 보행 분석을 위해 **센서 데이터의 시간 동기화** 필수
- ESP32에서 모든 센서 데이터를 수집 후 **BLE/Wi-Fi를 통해 중앙 연산 장치 (Jetson Xavier NX or Hailo-26TOPS)로 전송**

✅ **Depth Camera 데이터 전처리**

- GPU 없이 처리하려면 **Luxonis OAK-D** 같은 AI 가속 기능 내장 카메라 사용
- OAK-D에서 **발 형상 데이터만 추출**하여 ESP32로 전송

---

## **2️⃣ 데이터 처리 및 AI 분석 (GPU 없이 대체하기)**

### **🚀 단계 2: AI 가속 칩을 활용한 데이터 분석**

✅ **Hailo-26TOPS에서 AI 연산**

- 보행 분석 모델을 **TensorFlow Lite 또는 ONNX로 변환하여 Hailo에서 실행**
- Depth Camera 및 LiDAR 데이터를 사용하여 보행 패턴 분석

✅ **Jetson Xavier NX 또는 Edge TPU 활용 (선택사항)**

- TensorRT 최적화를 통해 보행 분석 모델 가속
- **Unity Digital Twin에서 사용될 3D 데이터 최소화**하여 연산량 절감

✅ **데이터 정리 및 Feature Extraction**

- **MPU9250:** 각속도, 가속도 데이터를 **FFT 변환** → 보행 주기 분석
- **압력 센서:** **발 접지 패턴을 시각화**하여 보행 특성 추출
- **Depth Camera:** AI 모델을 이용해 **발 아치, 볼, 길이 자동 인식**

---

## **3️⃣ Unity Digital Twin 및 UI 개발**

### **🚀 단계 3: 3D 모델링 및 실시간 시각화**

✅ **Unity Digital Twin 연동**

- 센서 데이터를 기반으로 **보행 3D 모델 생성**
- Unity에서 **Step Detection Algorithm (보행 단계 인식 알고리즘) 구현**
- **Unity XR/AR 지원 가능** (HoloLens 등과 연동 가능)

✅ **Unity Application 개발**

- 보행 분석 결과를 UI로 시각화
- AI 분석 결과 (발 모양, 보행 패턴)를 사용자에게 제공

✅ **연산 최적화 (GPU 없이 실행 가능하도록 설정)**

- Unity에서 **LOD (Level of Detail) 적용** → 3D 데이터 처리량 감소
- 3D 모델을 **Mesh Simplification** 기법으로 최적화

---

## **4️⃣ 최적화 및 테스트**

### **🚀 단계 4: 실시간 데이터 처리 및 튜닝**

✅ **보행 분석 속도 개선**

- AI 모델을 TensorFlow Lite로 변환하여 **최소한의 연산만 수행**
- Depth Camera & LiDAR 데이터는 **AI 모델이 아닌 전통적인 신호 처리 기법 (Kalman Filter 등)으로 1차 분석 후 AI 적용**

✅ **정확도 튜닝**

- 실제 보행 데이터를 수집하여 모델 학습
- FFT 및 Kalman Filter 조합으로 MPU9250 데이터 보정

✅ **최종 테스트**

- 여러 사람을 대상으로 보행 분석을 테스트하여 정확도 평가

---

## **✅ 최종 정리: GPU 없이 보행 분석 기기 구현 순서**

🚀 **1단계: 하드웨어 연결 및 데이터 수집**

- ESP32에 **MPU9250, 압력 센서, LiDAR, Depth Camera 연결**
- 센서 데이터를 **ESP32에서 동기화하여 Jetson Xavier NX or Hailo-26TOPS로 전송**

🚀 **2단계: AI 가속 칩을 활용한 실시간 분석**

- **Hailo-26TOPS에서 AI 모델 실행**하여 보행 패턴 분석
- **FFT & Kalman Filter로 신호 전처리**하여 보행 데이터를 최적화

🚀 **3단계: Unity Digital Twin 및 UI 개발**

- Unity에서 **보행 3D 모델링 및 실시간 시각화**
- 보행 분석 데이터를 Unity Application에서 사용자에게 제공

🚀 **4단계: 실시간 최적화 및 테스트**

- AI 모델 최적화 (TensorFlow Lite 변환)
- 센서 데이터 정확도 튜닝 및 테스트 진행

---

## <박보성>

## **NeRF (Neural Radiance Fields) 관련 프로젝트**

**NeRF**는 다수의 2D 이미지를 학습하여 3D 공간을 재구성하는 딥러닝 기반 기술

- **공식 NeRF 구현:**
    
    [bmild/nerf](https://github.com/bmild/nerf)
    
- **NVIDIA의 Instant NeRF:**
    
    [NVlabs/instant-ngp](https://github.com/NVlabs/instant-ngp)
    
- **Nerfstudio:**
    
    [nerfstudio-project/nerfstudio](https://github.com/nerfstudio-project/nerfstudio)
    

NeRF를 활용하려면 다양한 각도에서 촬영된 다수의 이미지가 필요. 촬영 시 일정한 간격과 조명을 유지하면 더 나은 결과를 얻을 수 있음.

---

## **SDF (Signed Distance Function) 기반 3D 생성**

**SDF**는 3D 공간에서 각 점이 표면으로부터의 거리를 나타내는 함수로, 명확한 표면 표현이 가능

- **DeepSDF:**[facebookresearch/DeepSDF](https://github.com/facebookresearch/DeepSDF)

SDF 기반 모델은 세밀한 3D 표면 재현에 유리하지만, 학습 데이터의 다양성과 품질이 결과에 큰 영향을 미침

---

## **Structure-from-Motion (SfM) 및 Multi-View Stereo (MVS) 기반 메쉬 재구성**

여러 각도에서 촬영한 2D 이미지를 활용하여 3D 모델을 재구성하는 전통적인 방법

- **COLMAP:**[colmap/colmap](https://github.com/colmap/colmap)

SfM/MVS 기법은 고해상도 이미지와 충분한 겹침(overlap)을 가진 촬영이 중요합니다. 촬영 시 이러한 점을 고려하면 더 정확한 3D 모델을 얻을 수 있음

---

## **Diffusion 모델 기반 3D 생성**

단일 이미지에서 3D 구조를 추론하는 최신 딥러닝 기술

- **Magic123:**[guochengqian/Magic123](https://github.com/guochengqian/Magic123)

Diffusion 모델은 단일 이미지에서도 3D 모델을 생성할 수 있지만, 생성된 모델의 세부 조정이 어려울 수 있으므로 후처리 과정이 필요할 수 있음

---

## **기타**

- **2D to 3D Modeling using AI:**[Developer-Kim/2D-to-3D-Modeling-using-AI](https://github.com/Developer-Kim/2D-to-3D-Modeling-using-AI)

---

## <백승욱>

---

종류 

---

- Meshroom
    - 사진 측량 기술을 사용하여 여러 장의 2D 이미지를 기반으로 3D 모델을 생성하는 오픈 소스
    - 사진 측량 기술을 활용하여 정렬되지 않은 사진으로부터 3D 추론이 핵심 기능
    - 사진 촬영 시 3D 장면이 2D 평면에 투영되는 과정을 역으로 추적하여 3D 공간 정보 복원하는 것
        
        https://alicevision.org/ 참고
        
    - 작업 매커니즘
        1. 객체를 다양한 시점에서 촬영한 여러 장의 중첩된 이미지를 획득
        2. 이미지에서 고유한 특징점 추출, 추출된 점들을 여러 이미지 간에 매칭 
        3. 매칭 정보를 바탕으로 SFM 과정을 통해 카메라의 자세를 추정하고, 희소한 3D 포인트 클라우드 생성 
        4. MVS 과정을 통해 더 많은 상세한 대응점을 찾아 밀집된 포인트 클라우드 생성
        5. 밀집 포인트 클라우드로부터 3D 매쉬 생성
        6. 원본 이미지의 색상 정보를 3D 매쉬에  투영하여 텍스처링된 모델 완성
        
        https://sketchfab.com/blogs/community/tutorial-meshroom-for-beginners/ 
        
        https://peterfalkingham.com/2021/03/02/meshroom-2021-1-0-whats-new-and-what-parameters-to-tweak/ 참고
        
    
    - CUDA 지원 그래픽카드 사용 권장, 만약 쿠다가 없는 라데온이나 인텔 그래픽 카드를 사용할 경우 Draft Meshing 기능만 사용해 3D를 재구성하나 결과물이 안좋을 가능성 농후
    - MacOS 지원은 안된다고 봐야
    - GUI 제공, Colmap 기반, 자동화 쉬움
    - 시험 삼아 써봤는데, 오래 걸림…
- Instant NGP
    - NeRF를 발전시킨 모델
    - 2D 이미지와 해당 카메라 자세로부터 NeRF 및 SDF, 신경 이미지, 신경 볼룸과 같은 신경 그래픽을 거의 즉각적으로 학습시키는 프레임워크
    - CUDA 코어가 필수이므로 반드시 엔비디아 그래픽 카드 사용
    - 촘촘히 찍을수록 좋은 결과를 얻을 수 있음
    - 각 사진에 중첩된 부분이 많도록 찍고, 다양한 각도의 사진이 있는 것이 좋음
    - 유사한 조명 조건에서 이미지를 캡처해야함
    - 이외에도 조건이 많음, 위 InstantNerf 소개 블로그 링크 참고
    - 즉, 이미지를 정교하게 많이 찍어야…..
        - 논문 참고
            
            [mueller2022instant.pdf](attachment:6c8833ee-a6f0-46bf-9a25-5b00d2a82dbb:mueller2022instant.pdf)
            
        - 읽기 쉽지 않으니 논문 리뷰 블로그 참고
            
            [[논문 리뷰] Instant NGP(SIGGRAPH 2022) : 인코딩 방법 개선으로 속도 향상](https://xoft.tistory.com/13)
            
            [[Instant-NGP][이론] Instant Neural Graphics Primitives 개념 설명 및 논문 리뷰](https://jaeyeol816.github.io/neural_representation/ingp-about-instant-ngp/#1-instant-ngp-%EC%86%8C%EA%B0%9C)
            
- Stable Fast 3D
    - 단일 이미지로부터 몇초 이내에 3D 이미지를 생성하는 AI 모델
    - 이미지 한 장 처리하는 데 필요한 VRAM은 7gb 정도 사용하는 것 같음
        - 3060, 3060ti, 3070ti, 4060 등
    - 그러나 단일 이미지 밖에 안된다.
        - 논문 참고
            
            [SF3D: Stable Fast 3D Mesh Reconstruction with UV-unwrapping and Illumination Disentanglement](https://hsejun07.tistory.com/399)
            
    - 잠재적인 문제
        - 입력 이미지 크기 제약 512X512 가 최대인 것으로 보임
        - 예측된 재질 파라미터의 잠재적 부정확성
        - 학습 데이터와 크게 다른 객체에 대한 일반화 문제
        - 단일 이미지 입력이기 때문에 매우 복잡하거나 상세한 객체 처리 제한
- Regard3D
    - 이미지가 많을수록 좋음
        - 설명서를 보니 40-60장 정도를 찍어야 좋은 품질이 나오는 듯 함
    - 이미지는 각각 약간 다른 시점에서 촬영해야…
    - 고해상도 이미지들은 더 상세한 모델 생성
    - 결과 모델에 구멍이 생기지 않도록 가능한 모든 각도에서 촬영해야…
    - 직사광선과 음영을 피해야 하고
    - 물체에 동일한 조명을 제공해야 하고 반사가 생기지 않아야 함
        
        [Photogrammetry/Regard3D - Wikiversity](https://en.wikiversity.org/wiki/Photogrammetry/Regard3D)
        

- visualSFM
    - SFM을 사용하여 3D 재구성
    - 빠른 특징 감지 및 매칭을 위한 SIFT on GPU, 재구성을 개선하기 위한 멀티코어 번들 조정, 그리고 대규모 데이터 세트의 효율적인 처리를 위한 선형 시간 증분 구조 기반 모션
    - 작업 매커니즘
        1. 이미지 업로드 
        2. 이미지 간의 특징점을 매칭하는 누락된 매칭 계산
        3. 카메라 자세를 추정하고 희소3D 포인트 클라우드를 생성
        4. 밀집 포인트 클라우드를 생성하는 밀집 재구성 
        5. 결과
    - 쿠다코어가 있어야 빠른 속도로 처리 가능 즉, rtx 그래픽 카드 사용
- openMVG + openMVS
    - openMVG
        - 특징 감지, 매칭 및 희소 3D 재구성
    - openMVS
        - sfm도구 (openMVG)의 출력을 사용하여 밀집 포인트 클라우드 재구성, 매쉬 생성, 매쉬  개선 및 텍스처링을 위한 알고리즘 제공
    - 이 두가지를 사용시에 장점
        - 3D 재구성을 위한 완전한 오픈소스 파이프라인 제공
    - 작업 매커니즘
        - openMVG를 사용하여 이미지 나열, 특징점 계산, 매칭 계산, sfm 재구성 수행, 장면을 MVS 형식으로 변환 후 openMVS를 사용하여 포인트 클라우드 밀집화, 매쉬 재구성
        
        chd wjdfl 
        
        | 특징 | Meshroom | InstantNGP | StableFast 3D | Regard3D | VisualSFM | openMVG + openMVS |
        | --- | --- | --- | --- | --- | --- | --- |
        | 핵심 기술 | 사진 측량 | 신경 방사 필드 (NeRF) | 피드포워드 3D 에셋 생성 | 구조 기반 모션 | 구조 기반 모션 | SfM (openMVG) + MVS (openMVS) |
        | 입력 | 다중 이미지/비디오 | 카메라 자세 (사진 측량에서) | 단일 이미지 | 다중 이미지 | 다중 이미지 | 다중 이미지 (openMVG), SfM 출력 |
        | 출력 | 3D 메쉬 (OBJ, PLY, Alembic, SfM) | 신경 방사 필드 (렌더링 가능한 뷰) | 3D 메쉬 (GLB) | 3D 포인트 클라우드 (PLY), 메쉬 (MeshLab 통해) | 3D 포인트 클라우드 (NVM), 메쉬 (PMVS/CMVS 통해) | 희소 포인트 클라우드 (openMVG), 밀집 클라우드, 메쉬, 텍스처 (openMVS) |
        | 속도 | 보통, 하드웨어/파라미터에 따라 다름 | 매우 빠름 (학습 및 렌더링) | 매우 빠름 (1초 미만 생성) | 보통에서 느림 (주로 CPU) | 빠름 (희소), 보통 (밀집) | 보통에서 느림 (밀집 재구성) |
        | 정확도 | 높음 | 높음 (렌더링) | 높음 (단일 이미지에서) | 좋음 | 좋음 | 높음 |
        | 사용 편의성 | GUI (노드 시스템) | GUI (렌더링), 명령줄 (학습) | API, 명령줄, Hugging Face 데모 | GUI | GUI | 명령줄 인터페이스 |
        | 시스템 요구 사항 | NVIDIA GPU (권장) | NVIDIA GPU (필수, 텐서 코어 선호) | 7GB VRAM 이상 GPU (권장) | OpenGL 지원 GPU, 64비트 OS | NVIDIA GPU (Windows에서 권장) | C++ 컴파일러, CMake, 잠재적 GPU |
        | 다중 플랫폼 | Windows, Linux (macOS 제한적) | Windows, Linux | 플랫폼 독립적 (PyTorch) | Windows, macOS, Linux | Windows, Linux, macOS | 크로스 플랫폼 (소스 빌드 필요) |
        | 라이선스 | MPLv2 | MIT | Stability AI Community License | MIT | 개인/비영리/학술적 용도 무료 | MPL2 (openMVG), AGPL-3.0 (openMVS) |

### 비고

---

[구조광 스캐너를 활용한 방법 (1)](https://www.notion.so/1-1ba2e6f3c46f8075b3fbf6e5e7b4ff09?pvs=21)

[오로지 센서로만 한다면? (1)](https://www.notion.so/1-1ba2e6f3c46f809c8863ff9398a6ec8a?pvs=21)

레이저 방식도 고려를 해야겠으나, 비용이 많이 든다는 이야기가 있어 추가 조사 필요
