#include <WiFi.h>
#include <WiFiClient.h>

// wifi 
const char* ssid = "wifi-ssid"; // ** 설정 필요 **
const char* password = "wifi-password"; // ** 설정 필요 **
WiFiServer tcpServer(5000);
WiFiClient tcpClient;

// TB6612FNG right side & ESP32-WROOM-32D DEVKIT_C V4 left side pin order
// 모터
const int PIN_PWMA = 32;  
const int PIN_AIN1 = 33;
const int PIN_AIN2 = 25;
const int PIN_STBY = 26;

// 버튼
const int button = 27;

// TCRT5000 tracker sensor
const int PIN_TRACKER = 34;  // TCRT5000 OUT 연결된 아날로그 핀
int tracker_state;
int prev_tracker_state;

//int total_time = 13; // 걸리는 시간(sec)
int rotation = 45; // 45도씩 회전
int rotation_num = 360/rotation; // 45도씩 회전, 총 ８번 

int button_state = 0;

// PWM 설정
const int CH_PWMA = 0;
const int pwmFrequency = 10000;  // Hz
const int bitResolution = 8;     // PWM 0~255

//const int PIN_LED = 2;

void setup(){
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  Serial.println();

  WiFi.begin(ssid, password); // Wi-Fi 연결 시도
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  tcpServer.begin(); // TCP 서버 시작
  tcpServer.setNoDelay(true);

  // TB6612FNG 설정
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);

  pinMode(button, INPUT); // 내부 풀업 사용하지 않음

  pinMode(PIN_TRACKER, INPUT); // tracker sensor input 모드

  // PWM 채널 연결
  ledcSetup(CH_PWMA, pwmFrequency, bitResolution);
  ledcAttachPin(PIN_PWMA, CH_PWMA);

}

void loop(){
  // 클라이언트 접속 확인
  tcpServer_hasClient();

  // client와 연결되면
  while(tcpClient.connected()) {
    button_state = 0;
    button_state = digitalRead(button);

    if(button_state == HIGH){ // 버튼 누르면
      delay(3000); // 버튼 누르고 3초 뒤에 시작
      int i = 0;
      tracker_state = LOW;
      while(1){ // rotation_num 횟수만큼 반복
        prev_tracker_state = tracker_state;
        tracker_state = digitalRead(PIN_TRACKER);
        if(tracker_state==HIGH && prev_tracker_state==LOW){ // 모터 정지 조건
          stop(); // 모터 멈추기
          
          int degree = i*rotation;
          tcpClient.println(degree); // 현재 degree 값 전송

          while (true) {
            if (tcpClient.available()) {
              String received = tcpClient.readStringUntil('\n');
              received.trim();  // 공백 제거
              if (received == "go") {
                break;
              }
            }
            delay(2000);
          }
          i++;

          // 원래 위치로 이동 후 정지
          if(i==rotation_num){ 
            while(1){
              prev_tracker_state = tracker_state;
              tracker_state = digitalRead(PIN_TRACKER);
              if(tracker_state==HIGH && prev_tracker_state==LOW){
                break;
              }
              move(); // 모터 회전
              delay(1);
            }
            break;
          }

        }
        move(); // 모터 회전
        delay(1);
      }
      tcpClient.println("end"); // 끝났다고 신호 전송
      stop();
      delay(3000);
    }
  }
  
}

void tcpServer_hasClient(){
    // 클라이언트 접속 확인
  if (tcpServer.hasClient()) {
    if (!tcpClient || !tcpClient.connected()) {
      tcpClient = tcpServer.available(); // 새 클라이언트 수락
      Serial.println("accept new Connection ...");
    } else {
      WiFiClient temp = tcpServer.available();
      temp.stop(); // 이미 클라이언트가 있으면 새 연결 거부
      Serial.println("reject new Connection ...");
    }
  }
  if (!tcpClient || !tcpClient.connected()) {
    delay(100);
    return;
  }
}

void move(){
  // ▶ 모터 A: 정방향 회전 (CW)
  /*
  digitalWrite(PIN_STBY, HIGH);
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, LOW);
  ledcWrite(CH_PWMA, 180);
  */

  // Motor A : CCW
  digitalWrite(PIN_STBY, HIGH); 	// STBY
  digitalWrite(PIN_AIN1, LOW); 		// AIN1
  digitalWrite(PIN_AIN2, HIGH);   	// AIN2
  ledcWrite(CH_PWMA, 170); 
}

void stop(){
  // ▶ 모터 정지 (STANDBY)
  digitalWrite(PIN_STBY, LOW);
}