# week 10-1 회의

## 🗓️날짜 및 시간

2025년 5월 6일 화요일 20:00-21:00

## 🗽장소

 온라인 zoom

## 🙇🏻‍♂️참여자

송샘, 이서진, 백승욱, 박보성

## 🏆목표

코드 실험 진행 상황 공유 및 하드웨어 설계 회의

---

## 📌 배경 제거(승욱) & 정합(서진) 실험 과정

### 0. 측정 방법

![촬영 방법.PNG](attachment:18e5f824-0dcf-48fe-b86b-70b2c2f1f37c:촬영_방법.png)

아직 하드웨어를 제작하지 않아서 임시로 

반지름 50cm 원이라고 생각하고 

수동으로 거리 측정해서 0도, 90도, 180도, 270도 총 4개의 각도에서 촬영

< 코드 실행 순서 >

**깊이 데이터 3D point cloud로 변환 코드 (다각도에서 측정)** → 저번 주차에 함

**3D point cloud로 변환된 결과에서 필요없는 배경을 제거하는 코드** → 이번 주차

**배경 제거한 결과를 정합하는 코드 (다각도에서 측정한 결과 글로벌 좌표계로 변환)** → 이번 주차

### 0. 배경 제거 없이 정합한 결과

![image (41).png](attachment:daeb59c8-e75d-4064-8185-dfa5c3aeb376:image_(41).png)

![image (42).png](attachment:4b0be8b2-4ec1-4222-a720-288bc7de7c5d:image_(42).png)

### 1. 3D point cloud로 변환된 결과에서 필요없는 배경을 제거하는 코드

![image.png](attachment:bbb8c776-8515-4cb6-98ec-85fe31b8314d:image.png)

![image.png](attachment:b917ced6-81df-4093-800e-91c5b2d6e7ea:image.png)

![image.png](attachment:04c006e2-d300-4d8d-bdbe-6230189c27dc:image.png)

![image.png](attachment:c5d91f26-9d05-4da9-b3f6-b9cbd5ffd565:image.png)

### 2. 배경 제거한 결과를 정합하는 코드 (다각도에서 측정한 결과를 글로벌 좌표계로 변환)

![image.png](attachment:cbb130fc-008b-4de7-ba8f-e682b6ab5fda:image.png)

![image.png](attachment:70d0c0d5-564b-4a40-bd16-a25237f7c636:image.png)

- 정합 코드

[pointcloud_registration_test1.ipynb](attachment:9e0b991b-970c-441a-90e0-4d2ad19b6ba4:pointcloud_registration_test1.ipynb)

- 정합 코드 순서
    1. 회전 행렬로 카메라 방향 바꾸기
    2. 이동 행렬로 다각도 카메라 좌표계를 글로벌 좌표계로 옮기기
    3. 여러 각도의 포인트 클라우드 병합

- 결과 분석
1. 수동으로 거리 측정한 후 촬영해서 정합할 때 오차가 발생
    
    → 추후 하드웨어 제작 후 정확한 거리에서 촬영한 결과로 다시 실험해볼 계획
    
    → 정합 오차 보정하는 알고리즘 적용해볼 계획
    
2. point cloud가 잡히지 않은 부분이 있음
    
    → 촬영 횟수를 더 늘릴 계획 (현재는 90도 간격으로 총 4개의 각도에서 촬영했지만, 추후 30도 간격으로 총 12개의 각도에서 촬영할 계획)
    
3. 현재 test data 하나로 실험한 결과여서 더 많은 test data로 배경 제거, 정합 코드가 잘 돌아가는지 실험하며, 정확도 계속해서 올릴 계획
4. 하드웨어를 제작하여 정확한 거리에서 촬영해볼 계획

## 📌 하드웨어 구조

![image.png](attachment:70a3e257-9cd6-4c83-80e5-fe4886347c85:image.png)

“올고르 알루미늄 회전판, 1개, 외경 400mm x 두께 8.5mm” 구매

회전판과 모터 활용해서 회전

## 📌 추후 목표

회전판 활용해서 회전 구조물 구축 

구축한 회전 구조물 활용해서 코드 실험 계속해서 진행
