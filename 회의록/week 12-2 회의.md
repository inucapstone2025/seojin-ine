## 🗓️날짜 및 시간

2025년 5월 20일 화요일 14:00-17:30

## 🗽장소

 7호관

## 🙇🏻‍♂️참여자

송샘, 이서진, 백승욱, 박보성

## 🏆목표

### ESP32-realsense wifi 통신
<img width="1173" alt="image" src="https://github.com/user-attachments/assets/74b5c668-8109-45ab-a95c-df1d4ecb9ac9" />
- 노트북과 카메라 연결해 3D Point Cloud 처리 -> 파이썬 사용
- 노트북과 esp32 연결해 모터 제어
- 카메라와 esp32 보드 간의 무선 통신이 필요함
  - 모터의 동작이 멈췄을 때(회전이 멈췄을 때) 카메라가 촬영을 진행해야 함.
  - 카메라 촬영이 다 끝나면 다시 모터가 회전해야 함
- ESP32 코드
- <img width="483" alt="image" src="https://github.com/user-attachments/assets/e7a93f80-f7f6-441f-a945-b1f5afd31912" />
  - wifi 이름, 비밀번호를 통해 wifi 연결
  - wifi에 연결되면 IP 주소를 받고, 해당 IP 주소를 통해 통신 가능
<img width="1210" alt="image" src="https://github.com/user-attachments/assets/b9dc4de2-532c-49ac-a26b-f942e74cc002" />
<img width="1406" alt="image" src="https://github.com/user-attachments/assets/ad2a8111-1787-4aa1-9a47-f3bd02f11120" />
- 저장된 파일로 배경 제거, 정합 코드 실행
  <img width="865" alt="image" src="https://github.com/user-attachments/assets/cca7b241-429a-40eb-aebd-e04d8afe3103" />
- 정합 오차가 매우 큼

이를 줄이기 위한 추후 목표 설정 완료

### 📌 추후 목표

하드웨어 수정 계획
- 배경 제거 오차 줄이기
- 정합 오차 줄이기
- 회전 구조물에 노트북 거치대 설치
- 발판 설치
