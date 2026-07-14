// ==========================================
// 튜닝 파라미터 (사용자가 수정하는 공간)
// ==========================================

// [라인 센서 아날로그 문턱값]
// 테스트 후 테이프 위에서의 값과 바닥 값의 중간치로 설정하세요.
const int LINE_THRESHOLD = 500; 

// [공격 및 추적 설정]
const int RUSH_DISTANCE = 3;        // 라인을 무시하고 돌진할 거리 (cm)
const int MAX_DETECT_DIST = 40;     // 상대를 추적하기 시작할 최대 거리 (cm)
const int MAX_SPEED = 255;          // 모터 최대 속도 (PWM 0~255)
const int MIN_SPEED = 100;          // 모터 최소 구동 속도 (PWM 0~255)
const int SEARCH_SPEED = 140;       // 탐색 회전 시 속도

// [시간 설정 (밀리초 단위, 1000 = 1초)]
// 테스트를 통해 180도와 45도 회전에 딱 맞는 시간을 입력하세요.
const unsigned long TIME_BACK_1초 = 1000;
const unsigned long TIME_TURN_180도 = 800;
const unsigned long TIME_FORWARD_2초 = 2000;
const unsigned long TIME_TURN_45도 = 250;

// ==========================================
// 하드웨어 핀 네이밍 (camelCase)
// ==========================================

// L298N 모터 드라이버 핀
const int motorLeftEna = 3;  
const int motorLeftIn1 = 2;       
const int motorLeftIn2 = 4;       
const int motorRightIn3 = 7;      
const int motorRightIn4 = 8;      
const int motorRightEnb = 5;      

// 아날로그 라인 센서 4개
const int lineFrontLeft = A0;
const int lineFrontRight = A1;
const int lineRearLeft = A2;
const int lineRearRight = A3;

// 초음파 센서 디지털 핀
const int sonicLeftTrig = 6;
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

// ==========================================
//초기화 (Setup)
// ==========================================
void setup() {
  Serial.begin(9600);
  
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

  // 경기 시작 후 대기 없이 즉시 기동 (필요시 delay 추가)
  moveForward(SEARCH_SPEED);
  delay(600); 
  stopMotors();
}

// ==========================================
// 메인 루프 (Loop)
// ==========================================
void loop() {
  // 1. 센서 데이터 업데이트 (초음파 필터링 포함)
  long currentLeftDist = getFilteredDistance(sonicLeftTrig, sonicLeftEcho, leftHistory);
  long currentRightDist = getFilteredDistance(sonicRightTrig, sonicRightEcho, rightHistory);
  
  bool isLeftEnemyClose = (currentLeftDist > 0 && currentLeftDist <= RUSH_DISTANCE);
  bool isRightEnemyClose = (currentRightDist > 0 && currentRightDist <= RUSH_DISTANCE);
  bool isEnemyInRushZone = (isLeftEnemyClose || isRightEnemyClose);

  // 2. 비동기 예외 및 탈출 상태 머신 처리 (delay 방지)
  if (currentState != STATE_NORMAL) {
    if (millis() - stateStartTime < stateDuration) {
      return; // 정해진 탈출 기동 시간이 끝나기 전에는 루프를 돌며 대기(명령 유지)
    } else {
      // 1단계 탈출(후진)이 끝난 후 2단계 탈출(180도 회전)로 넘겨야 하는 경우 처리
      if (currentState == STATE_ESCAPE_FRONT_BACK) {
        currentState = STATE_ESCAPE_FRONT_TURN;
        stateStartTime = millis();
        stateDuration = TIME_TURN_180도;
        turnRight(MAX_SPEED); 
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

    if (currentLeftDist <= MAX_DETECT_DIST && currentRightDist <= MAX_DETECT_DIST) {
      moveForward(calculatedSpeed); // 정면 추적 돌진
    } else if (currentLeftDist <= MAX_DETECT_DIST) {
      moveSteerLeft(calculatedSpeed); // 좌측 조향 추적
    } else {
      moveSteerRight(calculatedSpeed); // 우측 조향 추적
    }
    return;
  }

  // 5. 탐색 로직 (상대를 완전히 잃어버렸을 경우)
  spinInPlace();
}

// ==========================================
// 조건 검사 및 상태 전환 함수 (camelCase)
// ==========================================

bool checkLineSensors() {
  // 아날로그 센서 읽기 (붉은색 테이프 위에서 값이 threshold보다 커지거나 작아짐을 판단)
  bool fl = (analogRead(lineFrontLeft) > LINE_THRESHOLD);
  bool fr = (analogRead(lineFrontRight) > LINE_THRESHOLD);
  bool rl = (analogRead(lineRearLeft) > LINE_THRESHOLD);
  bool rr = (analogRead(lineRearRight) > LINE_THRESHOLD);

  if (fl && fr) {
    // 앞의 두 개 센서 둘 다 감지: 1초 후진 후 180도 회전 (1단계: 후진 세팅)
    currentState = STATE_ESCAPE_FRONT_BACK;
    stateStartTime = millis();
    stateDuration = TIME_BACK_1초;
    moveBackward(MAX_SPEED);
    return true;
  }
  else if (rl && rr) {
    // 뒤의 두 개 센서 둘 다 감지: 2초 직진
    currentState = STATE_ESCAPE_REAR;
    stateStartTime = millis();
    stateDuration = TIME_FORWARD_2초;
    moveForward(MAX_SPEED);
    return true;
  }
  else if (fl) {
    // 앞 왼쪽만 감지: 오른쪽으로 45도 회전
    currentState = STATE_ESCAPE_SINGLE_TURN;
    stateStartTime = millis();
    stateDuration = TIME_TURN_45도;
    turnRight(MAX_SPEED);
    return true;
  }
  else if (fr) {
    // 앞 오른쪽만 감지: 왼쪽으로 45도 회전
    currentState = STATE_ESCAPE_SINGLE_TURN;
    stateStartTime = millis();
    stateDuration = TIME_TURN_45도;
    turnLeft(MAX_SPEED);
    return true;
  }
  else if (rl) {
    // 뒤 왼쪽만 감지: 왼쪽으로 45도 회전
    currentState = STATE_ESCAPE_SINGLE_TURN;
    stateStartTime = millis();
    stateDuration = TIME_TURN_45도;
    turnLeft(MAX_SPEED);
    return true;
  }
  else if (rr) {
    // 뒤 오른쪽만 감지: 오른쪽으로 45도 회전
    currentState = STATE_ESCAPE_SINGLE_TURN;
    stateStartTime = millis();
    stateDuration = TIME_TURN_45도;
    turnRight(MAX_SPEED);
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

  // 평균값 계산
  long sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += history[i];
  }
  return sum / FILTER_SIZE;
}

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
  turnRight(SEARCH_SPEED); // 상대를 찾을 때까지 한 방향 제자리 스핀
}

void stopMotors() {
  digitalWrite(motorLeftIn1, LOW);
  digitalWrite(motorLeftIn2, LOW);
  digitalWrite(motorRightIn3, LOW);
  digitalWrite(motorRightIn4, LOW);
  analogWrite(motorLeftEna, 0);
  analogWrite(motorRightEnb, 0);
}