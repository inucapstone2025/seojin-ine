## 🗓️날짜 및 시간

2025년 5월 13일 화요일 14:30-17:00

## 🗽장소

 7호관 캡스톤실

## 🙇🏻‍♂️참여자

송샘, 이서진, 백승욱, 박보성

## 🏆목표

회전 구조물 정밀도 향상, sd카드에 라즈베리파이 os 설치, (esp32)모터&버튼 제어 실험

---

### 📌 회전 구조물 정밀도 향상

1. 회전 구조물의 높이 조절 - 볼트와 너트 사용
2. 아크릴 재단하기 위한 도안 제작
    
    ![image.png](attachment:2d608cdd-5f2f-4e69-acd0-667773c9e411:image.png)
    

### 📌 sd카드에 라즈베리파이 os 설치

완료. 

### 📌 esp32로 모터&버튼 제어

라즈베리파이와 무선통신하기 위해 wifi 모듈이 내장된 esp32 사용

버튼을 누르면 모터가 움직이도록 코드 작성

![image.png](attachment:cdcaf8e3-18b2-4210-9227-836b6be51487:image.png)

```arduino
// TB6612FNG right side & ESP32-WROOM-32D DEVKIT_C V4 left side pin order
const int PIN_PWMA = 32;  
const int PIN_AIN1 = 33;
const int PIN_AIN2 = 25;
const int PIN_STBY = 26;

// 버튼
const int button = 27;

int total_time = 12; // 걸리는 시간(sec)
int rotation = 30; // 30도씩 회전
int rotation_num = 360/rotation; // 30도씩 회전, 총 12번 

int state = 0;

// PWM 설정
const int CH_PWMA = 0;
const int pwmFrequency = 10000;  // Hz
const int bitResolution = 8;     // PWM 0~255

const int PIN_LED = 2;

void setup() {
  Serial.begin(9600);
  pinMode(PIN_LED, OUTPUT);

  // TB6612FNG 설정
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);

  pinMode(button, INPUT); // 내부 풀업 사용하지 않음

  // PWM 채널 연결
  ledcSetup(CH_PWMA, pwmFrequency, bitResolution);
  ledcAttachPin(PIN_PWMA, CH_PWMA);
}

void loop() {

  state = 0;
  state = digitalRead(button);
  if(state == HIGH){
    Serial.println("버튼 눌림!");
    int i = 0;
    while(i<rotation_num){
      move();
      delay(3000);

      // ▶ 모터 정지 (STANDBY)
      stop();
      delay(1000);
      i++;
    }
  }

}

void move(){
  // ▶ 모터 A: 정방향 회전 (CW)
  digitalWrite(PIN_STBY, HIGH);
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, LOW);
  ledcWrite(CH_PWMA, 255);
}

void stop(){
  // ▶ 모터 정지 (STANDBY)
  digitalWrite(PIN_STBY, LOW);
}

```
