// ==========================================
// 1. 상수 정의 (UPPER_SNAKE_CASE)
// ==========================================

// --- L298N 모터 드라이버 핀 설정 ---
const int MOTOR_LEFT_ENA = 3;       // 왼쪽 모터 속도 (PWM 필수)
const int MOTOR_LEFT_IN1 = 2;       // 왼쪽 모터 방향 1
const int MOTOR_LEFT_IN2 = 4;       // 왼쪽 모터 방향 2

const int MOTOR_RIGHT_IN3 = 7;      // 오른쪽 모터 방향 1
const int MOTOR_RIGHT_IN4 = 8;      // 오른쪽 모터 방향 2
const int MOTOR_RIGHT_ENB = 5;      // 오른쪽 모터 속도 (PWM 필수)

// --- 라인 센서 (아날로그 핀 사용) ---
const int LINE_FRONT_LEFT  = A0;     // 앞왼쪽 센서
const int LINE_FRONT_RIGHT = A1;     // 앞오른쪽 센서
const int LINE_REAR_LEFT   = A2;     // 뒤왼쪽 센서
const int LINE_REAR_RIGHT  = A3;     // 뒤오른쪽 센서

// --- 전면 초음파 센서 (디지털 핀 사용) ---
const int SONIC_LEFT_TRIG  = 9;
const int SONIC_LEFT_ECHO  = 10;
const int SONIC_RIGHT_TRIG = 11;
const int SONIC_RIGHT_ECHO = 12;

// --- [사용자 튜닝 매개변수] ---
// [라인 센서 임계값] 아날로그 값(0~1023). 테스트 후 바닥과 테이프의 경계값으로 수정하세요.
const int LINE_THRESHOLD = 500;     

// [올인 모드 기준 거리] 이 거리(cm) 이내로 들어오면 라인 센서를 무시하고 돌진합니다.
const int ALL_IN_DISTANCE = 3;      

// [모터 속도 설정] PWM (0~255). DC모터 최소 구동 속도에 맞추어 수정하세요.
const int MIN_SPEED = 100;          // 최저 속도 (거리가 멀 때)
const int MAX_SPEED = 255;          // 최고 속도 (거리가 가깝거나 회피/돌진 시)
const int SEARCH_SPEED = 140;       // 탐색 회전 속도

// [초음파 센서 탐지 한계 거리] (cm)
const int MAX_DETECT_DISTANCE = 50; 

// [라인 탈출 회피 동작 시간 설정 (ms)] - 회전 각도(45도, 180도)에 맞춰 딜레이 시간을 튜닝하세요!
const unsigned long DELAY_RETREAT_180  = 1000; // 앞 두개 감지 시 1초 후진
const unsigned long DELAY_TURN_180     = 600;  // 180도 회전 시간 (테스트 후 수정)
const unsigned long DELAY_ADVANCE_REAR = 2000; // 뒤 두개 감지 시 2초 직진
const unsigned long DELAY_TURN_45      = 150;  // 45도 회전 시간 (테스트 후 수정)

// 필터링 버퍼 크기 (이동 평균 필터)
const int FILTER_SIZE = 3;

// ==========================================
// 2. 전역 변수 정의 (camelCase)
// ==========================================
int sonicLeftBuffer[FILTER_SIZE] = {0};
int sonicRightBuffer[FILTER_SIZE] = {0};
int sonicLeftIndex = 0;
int sonicRightIndex = 0;

// Non-blocking 타이머 변수
unsigned long escapeStartTime = 0;
unsigned long escapeDuration = 0;
bool isEscaping = false;

// ==========================================
// 3. Setup (초기화)
// ==========================================
void setup() {
  Serial.begin(115200);

  // 모터 핀 모드
  pinMode(MOTOR_LEFT_ENA, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);
  pinMode(MOTOR_RIGHT_ENB, OUTPUT);

  // 초음파 핀 모드
  pinMode(SONIC_LEFT_TRIG, OUTPUT);
  pinMode(SONIC_LEFT_ECHO, INPUT);
  pinMode(SONIC_RIGHT_TRIG, OUTPUT);
  pinMode(SONIC_RIGHT_ECHO, INPUT);

  // 경기 시작 직후 준비 동작
  stopMotors();
  delay(1000); 
}

// ==========================================
// 4. Loop (메인 제어 루프)
// ==========================================
void loop() {
  // 1. 센서 값 갱신 (초음파 필터링 적용)
  int distLeft = getFilteredDistance(SONIC_LEFT_TRIG, SONIC_LEFT_ECHO, sonicLeftBuffer, sonicLeftIndex);
  int distRight = getFilteredDistance(SONIC_RIGHT_TRIG, SONIC_RIGHT_ECHO, sonicRightBuffer, sonicRightIndex);

  // 2. 상대가 올인 기준 거리(ALL_IN_DISTANCE = 3cm) 이내에 있는지 확인
  bool isAllInMode = (distLeft <= ALL_IN_DISTANCE || distRight <= ALL_IN_DISTANCE);

  // 3. 라인 회피 동작 진행 중인지 확인 (Non-blocking 처리)
  if (isEscaping) {
    if (millis() - escapeStartTime < escapeDuration) {
      return; // 회피 시간이 끝나지 않았으면 해당 회피 동작 지속
    } else {
      isEscaping = false; // 회피 완료
    }
  }

  // 4. [우선순위 1] 라인 센서 체크 (단, 올인 모드일 때는 라인 무시하고 직진)
  if (!isAllInMode) {
    if (checkAndHandleLineSensors()) {
      return; // 라인 회피 동작이 시작되었으면 루프 종료
    }
  } else {
    // 올인 모드 : 라인이 감지되어도 무시하고 최고 속도로 돌진
    moveForward(MAX_SPEED, MAX_SPEED);
    return;
  }

  // 5. [우선순위 2] 초음파 추적 (상대 거리 비례 속도 제어)
  if (distLeft <= MAX_DETECT_DISTANCE || distRight <= MAX_DETECT_DISTANCE) {
    handleTargetTracking(distLeft, distRight);
    return;
  }

  // 6. [우선순위 3] 탐색 모드 (상대를 잃어버렸을 때 : 제자리 회전)
  spinInPlace(SEARCH_SPEED);
}

// ==========================================
// 5. 주요 로직 처리 함수
// ==========================================

// --- 라인 센서 처리 함수 ---
bool checkAndHandleLineSensors() {
  // 아날로그 값 읽기 (값 기준 센서에 맞게 조정)
  bool isFrontLeft  = analogRead(LINE_FRONT_LEFT) > LINE_THRESHOLD;
  bool isFrontRight = analogRead(LINE_FRONT_RIGHT) > LINE_THRESHOLD;
  bool isRearLeft   = analogRead(LINE_REAR_LEFT) > LINE_THRESHOLD;
  bool isRearRight  = analogRead(LINE_REAR_RIGHT) > LINE_THRESHOLD;

  // 조건 1: 앞쪽 센서 2개 모두 감지 -> 1초 후진 후 180도 회전
  if (isFrontLeft && isFrontRight) {
    moveBackward(MAX_SPEED, MAX_SPEED);
    delay(DELAY_RETREAT_180); // 후진은 정확성을 위해 짧은 delay 허용
    turnRight(MAX_SPEED);
    startEscapeTimer(DELAY_TURN_180);
    return true;
  }
  // 조건 2: 뒤쪽 센서 2개 모두 감지 -> 2초 직진 (최대 속도)
  else if (isRearLeft && isRearRight) {
    moveForward(MAX_SPEED, MAX_SPEED);
    startEscapeTimer(DELAY_ADVANCE_REAR);
    return true;
  }
  // 조건 3: 앞 왼쪽만 감지 -> 오른쪽 45도 회전
  else if (isFrontLeft) {
    turnRight(MAX_SPEED);
    startEscapeTimer(DELAY_TURN_45);
    return true;
  }
  // 조건 4: 앞 오른쪽만 감지 -> 왼쪽 45도 회전
  else if (isFrontRight) {
    turnLeft(MAX_SPEED);
    startEscapeTimer(DELAY_TURN_45);
    return true;
  }
  // 조건 5: 뒤 왼쪽만 감지 -> 왼쪽 45도 회전
  else if (isRearLeft) {
    turnLeft(MAX_SPEED);
    startEscapeTimer(DELAY_TURN_45);
    return true;
  }
  // 조건 6: 뒤 오른쪽만 감지 -> 오른쪽 45도 회전
  else if (isRearRight) {
    turnRight(MAX_SPEED);
    startEscapeTimer(DELAY_TURN_45);
    return true;
  }

  return false; // 라인 감지 안됨
}

// --- 타깃 추적 및 비례 속도 제어 함수 ---
void handleTargetTracking(int distLeft, int distRight) {
  // 유효 감지 거리 계산
  int targetDistance = min(distLeft, distRight);

  // 거리(3cm ~ 50cm)에 반비례하게 PWM 속도(255 ~ 100) 매핑
  // 가까울수록(3cm) -> MAX_SPEED(255), 멀수록(50cm) -> MIN_SPEED(100)
  int calculatedSpeed = map(targetDistance, ALL_IN_DISTANCE, MAX_DETECT_DISTANCE, MAX_SPEED, MIN_SPEED);
  calculatedSpeed = constrain(calculatedSpeed, MIN_SPEED, MAX_SPEED); // 범위 제한

  if (distLeft <= MAX_DETECT_DISTANCE && distRight <= MAX_DETECT_DISTANCE) {
    // 정면 근처 : 직진
    moveForward(calculatedSpeed, calculatedSpeed);
  } else if (distLeft <= MAX_DETECT_DISTANCE) {
    // 좌측 편향 : 왼쪽 조향 접근 (오른쪽 모터를 더 빠르게)
    moveForward(calculatedSpeed / 2, calculatedSpeed);
  } else if (distRight <= MAX_DETECT_DISTANCE) {
    // 우측 편향 : 오른쪽 조향 접근 (왼쪽 모터를 더 빠르게)
    moveForward(calculatedSpeed, calculatedSpeed / 2);
  }
}

// Non-blocking 회피 타이머 시작
void startEscapeTimer(unsigned long duration) {
  escapeStartTime = millis();
  escapeDuration = duration;
  isEscaping = true;
}

// ==========================================
// 6. 초음파 센서 이동 평균 필터 (Moving Average Filter)
// ==========================================
int getFilteredDistance(int trigPin, int echoPin, int buffer[], int &index) {
  // 트리거 펄스 전송
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // pulseIn 타임아웃 15ms 제한 (거리 약 2.5m, 응답 지연 최소화)
  long duration = pulseIn(echoPin, HIGH, 15000); 
  int rawDistance = (duration == 0) ? MAX_DETECT_DISTANCE + 10 : duration * 0.034 / 2;

  // 버퍼 업데이트
  buffer[index] = rawDistance;
  index = (index + 1) % FILTER_SIZE;

  // 평균값 계산
  int sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += buffer[i];
  }
  return sum / FILTER_SIZE;
}

// ==========================================
// 7. L298N 모터 구동 하위 함수들
// ==========================================
void moveForward(int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_LEFT_ENA, leftSpeed);

  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_RIGHT_ENB, rightSpeed);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  analogWrite(MOTOR_LEFT_ENA, leftSpeed);

  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, HIGH);
  analogWrite(MOTOR_RIGHT_ENB, rightSpeed);
}

void turnLeft(int speed) {
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  analogWrite(MOTOR_LEFT_ENA, speed);

  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_RIGHT_ENB, speed);
}

void turnRight(int speed) {
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_LEFT_ENA, speed);

  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, HIGH);
  analogWrite(MOTOR_RIGHT_ENB, speed);
}

void spinInPlace(int speed) {
  turnRight(speed); // 우측 제자리 회전으로 상대 탐색
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_LEFT_ENA, 0);
  analogWrite(MOTOR_RIGHT_ENB, 0);
}