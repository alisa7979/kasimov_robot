// ==========================================
// 튜닝 파라미터 (사용자가 수정하는 공간)
// ==========================================

// [라인 센서 아날로그 문턱값 — 2026-07-09 실측 기반, 센서별 개별 설정]
// 실측값(2차, 최종): 빨강(경계) FL=910 FR=977 RL=971 RR=947 / 흰 바닥 FL=555 FR=813 RL=815 RR=700
// 센서별 편차가 커서 공용 문턱값 불가 → 각 센서의 (빨강+흰색)/2 중간값 사용
const int LINE_THRESH_FL = 732;
const int LINE_THRESH_FR = 895;
const int LINE_THRESH_RL = 893;
const int LINE_THRESH_RR = 823;

// [라인 센서 극성] 경계 테이프 위에서 값이 바닥보다 크면 true — 실측으로 확인됨 (빨강이 항상 더 큼)
const bool LINE_READS_HIGHER = true;

// [공격 및 추적 설정]
const int RUSH_DISTANCE = 8;       // 라인을 무시하고 돌진할 거리 (cm) — 3은 초음파 최소거리 탓에 사실상 도달 불가
const unsigned long CONTACT_LATCH_MS = 1000; // 근접 감지 후 이 시간 동안은 접촉(밀기) 상태로 간주
const int MAX_DETECT_DIST = 40;     // 상대를 추적하기 시작할 최대 거리 (cm)
const int MAX_SPEED = 255;          // 모터 최대 속도 (PWM 0~255)
const int MIN_SPEED = 150;          // 모터 최소 구동 속도 (PWM 0~255)
const int SEARCH_SPEED = 140;       // 탐색 회전 시 속도

// [조향 P제어] 좌우 거리차(cm)에 비례해 부드럽게 조향 — 급좌/급우 3분기 지그재그 방지
const int STEER_GAIN = 3;           // 조향 민감도 (지그재그하면 낮추고, 반응이 둔하면 올리세요)
const int STEER_DEADBAND_CM = 5;    // 좌우 차이가 이 값 이하면 직진 (센서 지터 무시)
const int STEER_DIFF_CLAMP = 15;    // 거리차 반영 상한 (한쪽만 감지 시 과도 조향 방지)

// [시작 동작] 전원 인가 → 대기(손 뗄 시간) → 짧은 돌진 → 탐색
const unsigned long START_DELAY_MS = 3000; // 대회 규정의 시작 대기 시간 확인 후 맞추세요 (보통 3~5초)
const int START_LUNGE_SPEED = 200;         // 시작 돌진 속도 — MIN_SPEED(150) 미만이면 한쪽만 돌아 비틀림

// [탈출 기동 속도] — 속도를 바꾸면 아래 시간 세트도 반드시 함께 교체!
// 255 = 빠른 탈출(경계 취약 시간 합계 700ms), 150 = 느리지만 회전 일관성 유리(합계 1330ms)
const int ESCAPE_SPEED = 150;

// [시간 설정 (밀리초 단위) — 2026-07-10 turn_test 실측, 두 세트 모두 측정 완료]
// ESCAPE_SPEED = 255 세트: BACK 150 / TURN180 550 / FORWARD 150 / TURN45 185
// const unsigned long TIME_BACK_MS     = 150;  // 전방 라인 감지 시 후진
// const unsigned long TIME_TURN_180_MS = 550;  // 180도 회전
// const unsigned long TIME_FORWARD_MS  = 150;  // 뒤 라인 탈출 직진
// const unsigned long TIME_TURN_45_MS  = 185;  // 45도 회전
// ESCAPE_SPEED = 150 세트: BACK 400 / TURN180 930 / FORWARD 400 / TURN45 320  ← 현재 사용 중
const unsigned long TIME_BACK_MS     = 400;  // 전방 라인 감지 시 후진
const unsigned long TIME_TURN_180_MS = 930;  // 180도 회전
const unsigned long TIME_FORWARD_MS  = 400;  // 뒤 라인 탈출 직진
const unsigned long TIME_TURN_45_MS  = 320;  // 45도 회전


// ==========================================
// 하드웨어 핀 네이밍 (camelCase)
// ==========================================

// L298N 모터 드라이버 핀
const int motorLeftEna = 3;  
const int motorLeftIn1 = 2;       
const int motorLeftIn2 = 4;       
const int motorRightIn3 = 6;      
const int motorRightIn4 = 7;      
const int motorRightEnb = 5;      

// 아날로그 라인 센서 4개
const int lineFrontLeft = A4;
const int lineFrontRight = A5;
const int lineRearLeft = A2;
const int lineRearRight = A3;

// 초음파 센서 디지털 핀
const int sonicLeftTrig = 8;
const int sonicLeftEcho = 9;
const int sonicRightTrig = 10;
const int sonicRightEcho = 11;

// ==========================================
// 시스템 상태 제어 및 필터 변수
// ==========================================

// 로봇의 현재 행동 상태 정의
enum RobotState {
  STATE_NORMAL,
  STATE_ESCAPE_FRONT_BACK,
  STATE_ESCAPE_FRONT_TURN,
  STATE_ESCAPE_REAR,
  STATE_ESCAPE_SINGLE_TURN
};

RobotState currentState = STATE_NORMAL;
unsigned long stateStartTime = 0;
unsigned long stateDuration = 0;
RobotState nextStateAfterEscape = STATE_NORMAL; // 복합 탈출용(후진 후 회전)

// 초음파 필터용 배열 (크기 3의 이동 평균 필터)
const int FILTER_SIZE = 3;
long leftHistory[FILTER_SIZE] = {999, 999, 999};
long rightHistory[FILTER_SIZE] = {999, 999, 999};
int filterIndex = 0;
long lastLeftDist = 999;
long lastRightDist = 999;
bool pingLeftTurn = true;
unsigned long contactLatchUntil = 0; // 접촉 상태 래치 만료 시각
int lastSeenDir = 1;                 // 마지막으로 상대를 본 방향: 1 = 오른쪽, -1 = 왼쪽

// ==========================================
//초기화 (Setup)
// ==========================================
void setup() {
  Serial.begin(115200); // 디버그 출력이 제어 루프를 막지 않도록 고속 통신 사용
  
  pinMode(motorLeftEna, OUTPUT);
  pinMode(motorLeftIn1, OUTPUT);
  pinMode(motorLeftIn2, OUTPUT);
  pinMode(motorRightIn3, OUTPUT);
  pinMode(motorRightIn4, OUTPUT);
  pinMode(motorRightEnb, OUTPUT);
  
  pinMode(sonicLeftTrig, OUTPUT);
  pinMode(sonicLeftEcho, INPUT);
  pinMode(sonicRightTrig, OUTPUT);
  pinMode(sonicRightEcho, INPUT);

  // 1) 시작 대기 — 로봇을 링에 놓고 손을 뗄 시간 (이 동안 완전 정지)
  delay(START_DELAY_MS);

  // 2) 시작 돌진 — SEARCH_SPEED(140)는 최소 구동 PWM(150) 미만이라 한쪽만 돌아 비틀림 → 200 사용
  moveForward(START_LUNGE_SPEED);
  delay(600);
  stopMotors();
}

// ==========================================
// 메인 루프 (Loop)
// ==========================================
void loop() {
  // 1. 센서 데이터 업데이트 (초음파 필터링 포함)
  // 두 센서를 번갈아 측정 (상호 간섭 방지 + 루프당 pulseIn 대기 절반)
  if (pingLeftTurn) {
    lastLeftDist = getFilteredDistance(sonicLeftTrig, sonicLeftEcho, leftHistory);
  } else {
    lastRightDist = getFilteredDistance(sonicRightTrig, sonicRightEcho, rightHistory);
  }
  pingLeftTurn = !pingLeftTurn;
  long currentLeftDist = lastLeftDist;
  long currentRightDist = lastRightDist;
  
  bool isLeftEnemyClose = (currentLeftDist > 0 && currentLeftDist <= RUSH_DISTANCE);
  bool isRightEnemyClose = (currentRightDist > 0 && currentRightDist <= RUSH_DISTANCE);
  if (isLeftEnemyClose || isRightEnemyClose) {
    contactLatchUntil = millis() + CONTACT_LATCH_MS; // 접촉(밀기) 상태 래치 갱신
  }
  bool isEnemyInRushZone = (millis() < contactLatchUntil); // 접촉 순간 센서가 999로 튀어도 밀기 유지

  // 2. 비동기 예외 및 탈출 상태 머신 처리 (delay 방지)
   if (currentState != STATE_NORMAL) {
    // 전진 탈출 중에도 앞 라인 센서는 계속 감시 — 반대편 경계 이탈 방지
    if (currentState == STATE_ESCAPE_REAR) {
      if (isLine(lineFrontLeft, LINE_THRESH_FL) || isLine(lineFrontRight, LINE_THRESH_FR)) {
        currentState = STATE_ESCAPE_FRONT_BACK; // 전진 중단 → 후진+180도 탈출로 전환
        stateStartTime = millis();
        stateDuration = TIME_BACK_MS;
        moveBackward(ESCAPE_SPEED);
        return;
      }
    }
    if (millis() - stateStartTime < stateDuration) {
      return; // 정해진 탈출 기동 시간이 끝나기 전에는 루프를 돌며 대기(명령 유지)
    } else {
      // 1단계 탈출(후진)이 끝난 후 2단계 탈출(180도 회전)로 넘겨야 하는 경우 처리
      if (currentState == STATE_ESCAPE_FRONT_BACK) {
        currentState = STATE_ESCAPE_FRONT_TURN;
        stateStartTime = millis();
        stateDuration = TIME_TURN_180_MS;
        turnRight(ESCAPE_SPEED); 
        return;
      }
      // 탈출 완료 -> 정상 상태로 복귀
      currentState = STATE_NORMAL;
    }
  }

  // 3. 조건별 우선순위 판단 (상대가 코앞에 있으면 라인 센서 판단 스킵)
  if (!isEnemyInRushZone && checkLineSensors()) {
    return; // 라인 탈출 기동이 시작되었으므로 루프 처음으로
  }

  // 4. 공격 및 추적 로직 (거리 반비례 속도 제어)
  if (currentLeftDist <= MAX_DETECT_DIST || currentRightDist <= MAX_DETECT_DIST) {
    // 두 센서 중 더 가까운 쪽의 거리를 기준으로 속도 결정
    long targetDist = min(currentLeftDist, currentRightDist);
    if (targetDist < RUSH_DISTANCE) targetDist = RUSH_DISTANCE;
    
    // 거리에 반비례하도록 PWM 속도 매핑 (멀면 MIN_SPEED, 가까우면 MAX_SPEED)
    int calculatedSpeed = map(targetDist, RUSH_DISTANCE, MAX_DETECT_DIST, MAX_SPEED, MIN_SPEED);
    calculatedSpeed = constrain(calculatedSpeed, MIN_SPEED, MAX_SPEED);

    // 좌우 거리차에 비례한 부드러운 조향 (P제어) — 급조향 왔다갔다(지그재그) 방지
    long diff = currentLeftDist - currentRightDist; // 음수 = 상대가 왼쪽에 치우침
    if (diff > STEER_DIFF_CLAMP)  diff = STEER_DIFF_CLAMP;
    if (diff < -STEER_DIFF_CLAMP) diff = -STEER_DIFF_CLAMP;

    if (diff < -STEER_DEADBAND_CM)     lastSeenDir = -1; // 상대를 왼쪽에서 봄 — 놓치면 왼쪽부터 탐색
    else if (diff > STEER_DEADBAND_CM) lastSeenDir = 1;  // 상대를 오른쪽에서 봄 — 놓치면 오른쪽부터 탐색

    int steer = (diff >= -STEER_DEADBAND_CM && diff <= STEER_DEADBAND_CM) ? 0 : (int)diff * STEER_GAIN;
    int leftSpeed  = constrain(calculatedSpeed + steer, 0, MAX_SPEED); // 상대가 왼쪽이면 왼쪽 바퀴 감속
    int rightSpeed = constrain(calculatedSpeed - steer, 0, MAX_SPEED);
    moveForwardLR(leftSpeed, rightSpeed);
    return;
  }

  // 5. 탐색 로직 (상대를 완전히 잃어버렸을 경우)
  spinInPlace();
}

// ==========================================
// 조건 검사 및 상태 전환 함수 (camelCase)
// ==========================================

// 라인(경계) 감지 판단 — 센서별 문턱값, 극성은 LINE_READS_HIGHER 상수 하나로 제어
bool isLine(int pin, int threshold) {
  int v = analogRead(pin);
  return LINE_READS_HIGHER ? (v > threshold) : (v < threshold);
}

bool checkLineSensors() {
  // 아날로그 센서 읽기 (붉은색 테이프 위에서 값이 threshold보다 커지거나 작아짐을 판단)
  bool fl = isLine(lineFrontLeft, LINE_THRESH_FL);
  bool fr = isLine(lineFrontRight, LINE_THRESH_FR);
  bool rl = isLine(lineRearLeft, LINE_THRESH_RL);
  bool rr = isLine(lineRearRight, LINE_THRESH_RR);

  if (fl && fr) {
    // 앞의 두 개 센서 둘 다 감지: 1초 후진 후 180도 회전 (1단계: 후진 세팅)
    currentState = STATE_ESCAPE_FRONT_BACK;
    stateStartTime = millis();
    stateDuration = TIME_BACK_MS;
    moveBackward(ESCAPE_SPEED);
    return true;
  }
  else if (rl && rr) {
    // 뒤의 두 개 센서 둘 다 감지: 2초 직진
    currentState = STATE_ESCAPE_REAR;
    stateStartTime = millis();
    stateDuration = TIME_FORWARD_MS;
    moveForward(ESCAPE_SPEED);
    return true;
  }
  else if (fl) {
    // 앞 왼쪽만 감지: 오른쪽으로 45도 회전
    currentState = STATE_ESCAPE_SINGLE_TURN;
    stateStartTime = millis();
    stateDuration = TIME_TURN_45_MS;
    turnRight(ESCAPE_SPEED);
    return true;
  }
  else if (fr) {
    // 앞 오른쪽만 감지: 왼쪽으로 45도 회전
    currentState = STATE_ESCAPE_SINGLE_TURN;
    stateStartTime = millis();
    stateDuration = TIME_TURN_45_MS;
    turnLeft(ESCAPE_SPEED);
    return true;
  }
  else if (rl) {
    // 뒤 왼쪽만 감지: 왼쪽으로 45도 회전
    currentState = STATE_ESCAPE_SINGLE_TURN;
    stateStartTime = millis();
    stateDuration = TIME_TURN_45_MS;
    turnLeft(ESCAPE_SPEED);
    return true;
  }
  else if (rr) {
    // 뒤 오른쪽만 감지: 오른쪽으로 45도 회전
    currentState = STATE_ESCAPE_SINGLE_TURN;
    stateStartTime = millis();
    stateDuration = TIME_TURN_45_MS;
    turnRight(ESCAPE_SPEED);
    return true;
  }
  return false;
}

// ==========================================
// 필터 및 모터 하부 드라이버 함수
// ==========================================

// 이동 평균 필터가 적용된 거리 측정 함수 (딜레이 없음)
long getFilteredDistance(int trigPin, int echoPin, long* history) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // 타임아웃을 15ms로 제한하여 에코를 기다리느라 대기하는 시간 최소화 (약 2.5m 환산)
  long duration = pulseIn(echoPin, HIGH, 15000); 
  long distance = (duration == 0) ? 999 : duration * 0.034 / 2;

  // 값이 너무 튀는 0 이하의 값 예외처리
  if (distance <= 0) distance = 999;

  // 필터 배열 원형 업데이트
  history[filterIndex] = distance;
  filterIndex = (filterIndex + 1) % FILTER_SIZE;

   // 중앙값(median) 계산 — 999 드롭아웃 1회가 결과를 오염시키지 않음
  return median3(history[0], history[1], history[2]);
}

long median3(long a, long b, long c) {
  if (a > b) { long t = a; a = b; b = t; }
  if (b > c) { long t = b; b = c; c = t; }
  if (a > b) { long t = a; a = b; b = t; }
  return b;
}

void moveForward(int speed) {
  digitalWrite(motorLeftIn1, HIGH);
  digitalWrite(motorLeftIn2, LOW);
  analogWrite(motorLeftEna, speed);
  digitalWrite(motorRightIn3, HIGH);
  digitalWrite(motorRightIn4, LOW);
  analogWrite(motorRightEnb, speed);
}

// 좌우 바퀴 속도를 따로 지정하는 전진 (P제어 조향용)
void moveForwardLR(int leftSpeed, int rightSpeed) {
  digitalWrite(motorLeftIn1, HIGH);
  digitalWrite(motorLeftIn2, LOW);
  analogWrite(motorLeftEna, leftSpeed);
  digitalWrite(motorRightIn3, HIGH);
  digitalWrite(motorRightIn4, LOW);
  analogWrite(motorRightEnb, rightSpeed);
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

void moveSteerLeft(int speed) {
  digitalWrite(motorLeftIn1, HIGH);
  digitalWrite(motorLeftIn2, LOW);
  analogWrite(motorLeftEna, speed * 0.4); // 조향을 위해 안쪽 바퀴 감속비 조정 가능
  digitalWrite(motorRightIn3, HIGH);
  digitalWrite(motorRightIn4, LOW);
  analogWrite(motorRightEnb, speed);
}

void moveSteerRight(int speed) {
  digitalWrite(motorLeftIn1, HIGH);
  digitalWrite(motorLeftIn2, LOW);
  analogWrite(motorLeftEna, speed);
  digitalWrite(motorRightIn3, HIGH);
  digitalWrite(motorRightIn4, LOW);
  analogWrite(motorRightEnb, speed * 0.4);
}

void spinInPlace() {
  // 마지막으로 상대를 본 쪽으로 회전 — 재탐지 시간 단축
  if (lastSeenDir < 0) turnLeft(SEARCH_SPEED);
  else                 turnRight(SEARCH_SPEED);
}

void stopMotors() {
  digitalWrite(motorLeftIn1, LOW);
  digitalWrite(motorLeftIn2, LOW);
  digitalWrite(motorRightIn3, LOW);
  digitalWrite(motorRightIn4, LOW);
  analogWrite(motorLeftEna, 0);
  analogWrite(motorRightEnb, 0);
}