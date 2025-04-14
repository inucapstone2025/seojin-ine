# week 6-1 회의

## 🗓️날짜 및 시간

2025년 4월 10일 목요일 16:00-18:30

## 🗽장소

캡스톤 수업 교실

## 🙇🏻‍♂️참여자

송샘, 이서진, 백승욱, 박보성

## 🏆목표

- 하드웨어 구조 설계 및 COLMAP 정합 실험 결과 공유
- 필요한 부품 목록 정리 및 구매 계획 수립

## 📌 회의 내용 정리

**발 측정 시스템 하드웨어 구조 설계 공유**

- 발을 고정할 수 있는 플랫폼 크기 및 형태 논의

![Image](https://github.com/user-attachments/assets/c39d4acc-2ed8-4c9a-99e7-8f2abbe8677a)

**COLMAP을 활용한 3D Point Cloud 정합 실험 공유**

- 다각도에서 촬영한 발 사진을 이용해 COLMAP으로 3D 모델 생성 실험 진행
![Image](https://github.com/user-attachments/assets/bf19a674-db86-426a-b838-46cbe627b927)

**하드웨어 구성 논의**

- 시스템 구축에 필요한 부품을 선정

| 번호 | 상품코드 | 상품명(옵션명) | 제조사 | 링크 | 수량 | 단가 | 합계금액 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 37801 | 바퀴 66파이 | SMG | http://www.devicemart.co.kr/goods/view?no=37801 | 4 | 800 | 3,200 |
| 2 | 37853 | 기어박스장착모터 (NP01D-288) | OEM | http://www.devicemart.co.kr/goods/view?no=37853 | 6 | 1,800 | 10,800 |
| 3 | 1327604 | TB6612FNG 듀얼 모터 드라이버 모듈 [SZH-MDBL-003] | SMG | http://www.devicemart.co.kr/goods/view?no=1327604 | 2 | 4,200 | 8,400 |
| 4 | 14117578 | KC인증 18650 리튬배터리 3.7V 2550mAh [ZM18650-2600-KC01] | ZM | http://www.devicemart.co.kr/goods/view?no=14117578 | 2 | 4,000 | 8,000 |
| 5 | 14497608 | 18650 배터리용 홀더 직렬 2구 스위치+전선 타입 [LIBH-BCS-2] | SMG | http://www.devicemart.co.kr/goods/view?no=14497608 | 1 | 600 | 600 |
| 6 | 1153807 | 3색 점퍼용 단선- 0.6 파이 [1미터단위] | 동우전선 | http://www.devicemart.co.kr/goods/view?no=1153807 | 2 | 400 | 800 |
| 7 | 12553062 | 라즈베리파이4 (Raspberry Pi 4 Model B) 8GB + 가이드북 + 방열판 | Raspberry Pi | http://www.devicemart.co.kr/goods/view?no=12553062 | 1 | 112,700 | 112,700 |
|  |  | [옵션]필수옵션:가이드북은 구매 수량당 1권만 제공(최대 10권까지)됩니다. |  |  |  |  |  |
| 8 | 12230698 | 라즈베리파이4 듀얼 쿨러 케이스 [RBP-002] | SMG | http://www.devicemart.co.kr/goods/view?no=12230698 | 1 | 12,000 | 12,000 |
| 9 | 14825251 | Ultra microSDXC 256GB [SDSQUAC-256G-GN6MN] | SanDisk | http://www.devicemart.co.kr/goods/view?no=14825251 | 1 | 25,300 | 25,300 |
| 10 | 15502416 | [공식 정품] 라즈베리파이 27W USB-C 아답터 검은색 (Raspberry Pi 27W USB-C Power Supply Black) | Raspberry Pi | http://www.devicemart.co.kr/goods/view?no=15502416 | 1 | 16,400 | 16,400 |
| 11 | 1310704 | 아두이노 버튼 스위치 모듈 (흰색) [ELB060677] | YwRobot | http://www.devicemart.co.kr/goods/view?no=1310704 | 3 | 1,500 | 4,500 |


- 시스템 flow chart

  ![Image](https://github.com/user-attachments/assets/eb946e9d-b546-4f6e-aef1-ecfbed510ff6)

## 📌 **다음 회의까지 할 일**

- **필요한 부품들 주문**
- **Depth Camera (Intel RealSense D435) 사용을 위한 개발 환경 세팅법 조사**
