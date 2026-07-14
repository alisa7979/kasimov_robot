// ==========================================
// 회전/후진/전진 시간 캘리브레이션 테스트 (1회 실행형)
// 사용법:
//   1) TEST_MODE와 TEST_DURATION_MS를 설정하고 업로드
//   2) 로봇을 경기장 표면에 놓고 (테이프로 시작 방향 표시) 손을 뗌
//   3) 리셋 버튼을 누르면 2초 후 동작을 딱 1회 실행하고 정지
//   4) 결과를 보고 TEST_DURATION_MS 조정 → 재업로드
//      (같은 값을 반복 시험할 때는 리셋 버튼만 누르면 됨)
// 조정 공식: 새시간 = 현재시간 × (목표각도 / 실제로 돈 각도)
//   예) 800ms로 220도 돌았다면 → 800 × 180/220 ≈ 655ms
// ==========================================

// [시험 설정 — 이 두 줄만 바꿔가며 시험하세요]
const int TEST_MODE = 2;                    // 1=오른쪽180도, 2=오른쪽45도, 3=왼쪽45도, 4=후진(TIME_BACK), 5=전진(TIME_FORWARD)
const unsigned long TEST_DURATION_MS = 470; // 시험할 동작 시간 (ms)

const int MAX_SPEED = 150; // main_ver2의 탈출 기동과 동일하게 항상 최대 속도

// L298N 모터 드라이버 핀 (main_ver2와 동일 — 변경 금지)
const int motorLeftEna = 3;
const int motorLeftIn1 = 2;
const int motorLeftIn2 = 4;
const int motorRightIn3 = 6;
const int motorRightIn4 = 7;
const int motorRightEnb = 5;

void setup() {
  pinMode(motorLeftEna, OUTPUT);
  pinMode(motorLeftIn1, OUTPUT);
  pinMode(motorLeftIn2, OUTPUT);
  pinMode(motorRightIn3, OUTPUT);
  pinMode(motorRightIn4, OUTPUT);
  pinMode(motorRightEnb, OUTPUT);

  delay(2000); // 로봇에서 손을 뗄 시간

  // 선택한 동작을 1회 실행
  if (TEST_MODE == 1 || TEST_MODE == 2) {
    turnRight(MAX_SPEED);
  } else if (TEST_MODE == 3) {
    turnLeft(MAX_SPEED);
  } else if (TEST_MODE == 4) {
    moveBackward(MAX_SPEED);
  } else {
    moveForward(MAX_SPEED);
  }

  delay(TEST_DURATION_MS);
  stopMotors();
}

void loop() {
  // 아무것도 하지 않음 — 반복 시험은 리셋 버튼으로
}

// ==========================================
// 모터 함수 (main_ver2와 동일)
// ==========================================
void moveForward(int speed) {
  digitalWrite(motorLeftIn1, HIGH);
  digitalWrite(motorLeftIn2, LOW);
  analogWrite(motorLeftEna, speed);
  digitalWrite(motorRightIn3, HIGH);
  digitalWrite(motorRightIn4, LOW);
  analogWrite(motorRightEnb, speed);
}

void moveBackward(int speed) {
  digitalWrite(motorLeftIn1, LOW);
  digitalWrite(motorLeftIn2, HIGH);
  analogWrite(motorLeftEna, speed);
  digitalWrite(motorRightIn3, LOW);
  digitalWrite(motorRightIn4, HIGH);
  analogWrite(motorRightEnb, speed);
}

void turnLeft(int speed) {
  digitalWrite(motorLeftIn1, LOW);
  digitalWrite(motorLeftIn2, HIGH);
  analogWrite(motorLeftEna, speed);
  digitalWrite(motorRightIn3, HIGH);
  digitalWrite(motorRightIn4, LOW);
  analogWrite(motorRightEnb, speed);
}

void turnRight(int speed) {
  digitalWrite(motorLeftIn1, HIGH);
  digitalWrite(motorLeftIn2, LOW);
  analogWrite(motorLeftEna, speed);
  digitalWrite(motorRightIn3, LOW);
  digitalWrite(motorRightIn4, HIGH);
  analogWrite(motorRightEnb, speed);
}

void stopMotors() {
  digitalWrite(motorLeftIn1, LOW);
  digitalWrite(motorLeftIn2, LOW);
  digitalWrite(motorRightIn3, LOW);
  digitalWrite(motorRightIn4, LOW);
  analogWrite(motorLeftEna, 0);
  analogWrite(motorRightEnb, 0);
}
