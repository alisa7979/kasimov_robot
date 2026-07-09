// ==========================================
// 상수 정의 (UPPER_SNAKE_CASE)
// ==========================================

// L298N 모터 드라이버 핀
const int MOTOR_LEFT_ENA = 3;       
const int MOTOR_LEFT_IN1 = 2;       
const int MOTOR_LEFT_IN2 = 4;       
const int MOTOR_RIGHT_IN3 = 7;      
const int MOTOR_RIGHT_IN4 = 8;      
const int MOTOR_RIGHT_ENB = 5;

// 초음파 센서 핀
const int SONIC_LEFT_TRIG = 6;
const int SONIC_LEFT_ECHO = 9;
const int SONIC_RIGHT_TRIG = 10;
const int SONIC_RIGHT_ECHO = 11;

// 추적 제어 파라미터
const int RUSH_DISTANCE = 3;        // 최대 속도(255)로 돌진할 초근접 거리 (cm)
const int MAX_DETECT_DIST = 40;     // 상대를 감지하고 추적을 시작할 최대 거리 (cm)
const int MAX_SPEED = 255;          // 모터 최대 속도 (PWM)
const int MIN_SPEED = 100;          // 모터 최소 구동 속도 (PWM)
const int SEARCH_SPEED = 130;       // 상대를 찾기 위해 회전할 때의 속도

// 초음파 필터 크기
const int FILTER_SIZE = 3;

// ==========================================
// 전역 변수 정의 (camelCase)
// ==========================================
long leftHistory[FILTER_SIZE] = {999, 999, 999};
long rightHistory[FILTER_SIZE] = {999, 999, 999};
int filterIndex = 0;

// ==========================================
// 초기 설정 (Setup)
// ==========================================
void setup() {
  Serial.begin(9600);
  
  pinMode(MOTOR_LEFT_ENA, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);
  pinMode(MOTOR_RIGHT_ENB, OUTPUT);
  
  pinMode(SONIC_LEFT_TRIG, OUTPUT);
  pinMode(SONIC_LEFT_ECHO, INPUT);
  pinMode(SONIC_RIGHT_TRIG, OUTPUT);
  pinMode(SONIC_RIGHT_ECHO, INPUT);

  Serial.println("=== 초음파 추적 알고리즘 테스트 시작 ===");
}

// ==========================================
// 메인 루프 (Loop)
// ==========================================
void loop() {
  // 1. 양쪽 초음파 센서 거리 측정 (이동 평균 필터 적용)
  long currentLeftDist = getFilteredDistance(SONIC_LEFT_TRIG, SONIC_LEFT_ECHO, leftHistory);
  long currentRightDist = getFilteredDistance(SONIC_RIGHT_TRIG, SONIC_RIGHT_ECHO, rightHistory);

  // 필터 배열의 인덱스를 순환 시킴 (0 -> 1 -> 2 -> 0)
  filterIndex = (filterIndex + 1) % FILTER_SIZE;

  // 2. 모니터링을 위한 시리얼 출력
  Serial.print("Left: "); Serial.print(currentLeftDist);
  Serial.print(" cm | Right: "); Serial.print(currentRightDist);
  Serial.print(" cm -> ");

  // 3. 추적 알고리즘 판단 구역
  if (currentLeftDist <= MAX_DETECT_DIST || currentRightDist <= MAX_DETECT_DIST) {
    // 두 센서 값 중 더 가까운 거리를 기준으로 속도 계산
    long targetDist = min(currentLeftDist, currentRightDist);
    if (targetDist < RUSH_DISTANCE) targetDist = RUSH_DISTANCE;

    // 거리 반비례 속도 매핑 (3cm일 때 255, 40cm일 때 100)
    int calculatedSpeed = map(targetDist, RUSH_DISTANCE, MAX_DETECT_DIST, MAX_SPEED, MIN_SPEED);
    calculatedSpeed = constrain(calculatedSpeed, MIN_SPEED, MAX_SPEED);

    // 방향 제어
    if (currentLeftDist <= MAX_DETECT_DIST && currentRightDist <= MAX_DETECT_DIST) {
      Serial.print("[추적] 정면 발견! 속도: "); Serial.println(calculatedSpeed);
      moveForward(calculatedSpeed);
    } 
    else if (currentLeftDist <= MAX_DETECT_DIST) {
      Serial.print("[추적] 좌측 치우침! 좌조향 속도: "); Serial.println(calculatedSpeed);
      moveSteerLeft(calculatedSpeed);
    } 
    else {
      Serial.print("[추적] 우측 치우침! 우조향 속도: "); Serial.println(calculatedSpeed);
      moveSteerRight(calculatedSpeed);
    }
  } 
  else {
    // 4. 감지 범위(40cm) 내에 아무것도 없을 때 -> 탐색 회전
    Serial.print("[탐색] 상대 상실 -> 제자리 회전 속도: "); Serial.println(SEARCH_SPEED);
    spinInPlace();
  }

  // 시리얼 모니터가 너무 가득 차지 않도록 하면서 실시간성을 유지하는 미세 딜레이
  delay(30); 
}

// ==========================================
// 초음파 거리 측정 및 필터 함수
// ==========================================
long getFilteredDistance(int trigPin, int echoPin, long* history) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // 실시간성 확보를 위해 타임아웃을 15ms(약 2.5m 거리)로 제한
  long duration = pulseIn(echoPin, HIGH, 15000); 
  long distance = (duration == 0) ? 999 : duration * 0.034 / 2;

  if (distance <= 0) distance = 999;

  // 현재 인덱스 위치에 신규 거리 저장
  history[filterIndex] = distance;

  // 최근 3개 데이터의 평균값 계산
  long sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += history[i];
  }
  return sum / FILTER_SIZE;
}

// ==========================================
// 모터 제어 함수 하부 인터페이스
// ==========================================

void moveForward(int speed) {
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
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

void moveSteerLeft(int speed) {
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_LEFT_ENA, speed * 0.4); // 안쪽 바퀴 감속비 (0.4 배)
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_RIGHT_ENB, speed);
}

void moveSteerRight(int speed) {
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_LEFT_ENA, speed);
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_RIGHT_ENB, speed * 0.4); // 안쪽 바퀴 감속비 (0.4 배)
}

void spinInPlace() {
  turnRight(SEARCH_SPEED);
}