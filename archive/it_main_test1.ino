#include <math.h>

// ============================================================
// Battle Robot Integrated Main Code - Debug Full Version
//
// 역할:
// - 라인트레이서 4개, 측면 적외선 2개, 정면 초음파 2개를 통합
// - 센서 감지 우선순위에 따라 모터 동작 결정
// - 시리얼 모니터로 각 센서 raw값 / 감지 여부 / 최종 행동 확인 가능
//
// 동작 우선순위:
// 1. 라인트레이서 감지
//    -> 경기장 밖으로 나가는 것이 가장 위험하므로 최우선 탈출
//
// 2. 측면 적외선 감지
//    -> 상대가 측면에 있다고 보고 그 방향으로 회전
//    -> 상대를 정면 초음파 센서 범위 안에 넣는 목적
//
// 3. 정면 초음파 감지
//    -> 상대가 정면에 있다고 보고 추적 / 전진
//
// 4. 아무것도 감지 안 됨
//    -> 제자리 회전하면서 상대 탐색
//
// 실제 배선 기준:
// Motor Driver
//   In1   -> D2   left motor direction 1
//   In2   -> D3   left motor direction 2
//   In3   -> D4   right motor direction 1
//   In4   -> D5   right motor direction 2
//   PWM_L -> D6
//   PWM_R -> D13
//
// 현재 PWM 속도제어는 사용하지 않고 enable HIGH / LOW만 사용
//
// Side IR
//   Left IR  -> D7
//   Right IR -> D8
//
// Ultrasonic
//   Left echo  -> D9
//   Left trig  -> D10
//   Right echo -> D11
//   Right trig -> D12
//
// Line Tracer
//   Front Left  -> A2
//   Front Right -> A3
//   Back Left   -> A4
//   Back Right  -> A5
// ============================================================


// ============================================================
// 전체 설정
// ============================================================

// true면 시리얼 모니터에 센서값 출력
const bool DEBUG_SERIAL = true;

// 모터 동작 허용 여부
// 센서값 디버깅만 할 때는 false 추천
// 실제 주행 테스트할 때 true로 변경
const bool MOTOR_ENABLE = false;

// 센서값 출력 주기
// 너무 짧으면 출력이 너무 많아져서 보기 힘들고 동작도 느려질 수 있음
const unsigned long DEBUG_PRINT_INTERVAL_MS = 500;

// 시작하자마자 중앙 쪽으로 살짝 전진할지 여부
// 디버깅할 때는 false 추천
const bool USE_START_FORWARD = false;

// 시작 전진 시간
const unsigned long START_FORWARD_MS = 600;

// 메인 루프 기본 딜레이
const unsigned long MAIN_LOOP_DELAY_MS = 20;


// ============================================================
// 모터 핀 설정
// ============================================================

const int M_L_IN1 = 2;
const int M_L_IN2 = 3;
const int M_R_IN1 = 4;
const int M_R_IN2 = 5;

const int M_L_EN = 6;
const int M_R_EN = 13;

// 모터 방향이 반대로 돌면 true로 변경
const bool LEFT_MOTOR_REVERSED  = false;
const bool RIGHT_MOTOR_REVERSED = false;


// ============================================================
// 라인트레이서 핀 설정 - 4모서리 방식
//
// A2 : 앞왼쪽
// A3 : 앞오른쪽
// A4 : 뒤왼쪽
// A5 : 뒤오른쪽
// ============================================================

const int LINE_FRONT_LEFT_PIN  = A2;
const int LINE_FRONT_RIGHT_PIN = A3;
const int LINE_BACK_LEFT_PIN   = A4;
const int LINE_BACK_RIGHT_PIN  = A5;

// 아날로그 라인트레이서 기준 임계값
// 시리얼 모니터에서 raw값 보고 조절
//
// 예시 1:
// 흰색에서 100 근처, 라인에서 800 근처
// -> LINE_DETECTED_WHEN_HIGH = true
// -> LINE_THRESHOLD = 500 정도
//
// 예시 2:
// 흰색에서 900 근처, 라인에서 100 근처
// -> LINE_DETECTED_WHEN_HIGH = false
// -> LINE_THRESHOLD = 500 정도
const int LINE_THRESHOLD = 500;

// 라인을 밟았을 때 analogRead 값이 커지면 true
// 라인을 밟았을 때 analogRead 값이 작아지면 false
const bool LINE_DETECTED_WHEN_HIGH = true;

// 같은 상태가 이 시간 이상 유지되어야 실제 감지로 인정
// 센서 튐 방지용
const unsigned long LINE_STABLE_TIME_MS = 50;

// 라인트레이서 배열 index
const int LINE_FRONT_LEFT  = 0;
const int LINE_FRONT_RIGHT = 1;
const int LINE_BACK_LEFT   = 2;
const int LINE_BACK_RIGHT  = 3;
const int LINE_SENSOR_COUNT = 4;

const int LINE_PINS[LINE_SENSOR_COUNT] =
{
    LINE_FRONT_LEFT_PIN,
    LINE_FRONT_RIGHT_PIN,
    LINE_BACK_LEFT_PIN,
    LINE_BACK_RIGHT_PIN
};

// 라인트레이서 안정화용 변수
bool lastRawLineDetected[LINE_SENSOR_COUNT] = {false, false, false, false};
bool stableLineDetected[LINE_SENSOR_COUNT]  = {false, false, false, false};
unsigned long lastLineChangeTime[LINE_SENSOR_COUNT] = {0, 0, 0, 0};


// ============================================================
// 라인트레이서 탈출 행동 코드
//
// enum 대신 int 상수로 정의
// Arduino IDE에서 enum / struct 자동 원형 생성 문제를 줄이기 위함
// ============================================================

const int ACTION_NO_LINE = 0;

const int ACTION_ESCAPE_BACKWARD_ROTATE = 1;  // 앞쪽 라인 -> 후진 후 회전
const int ACTION_ESCAPE_FORWARD_ROTATE  = 2;  // 뒤쪽 라인 -> 전진 후 회전

const int ACTION_ESCAPE_TURN_RIGHT      = 3;  // 왼쪽 라인 -> 오른쪽 회전
const int ACTION_ESCAPE_TURN_LEFT       = 4;  // 오른쪽 라인 -> 왼쪽 회전

const int ACTION_ESCAPE_BACKWARD_RIGHT  = 5;  // 앞왼쪽 라인 -> 후진 후 오른쪽
const int ACTION_ESCAPE_BACKWARD_LEFT   = 6;  // 앞오른쪽 라인 -> 후진 후 왼쪽
const int ACTION_ESCAPE_FORWARD_RIGHT   = 7;  // 뒤왼쪽 라인 -> 전진 후 오른쪽
const int ACTION_ESCAPE_FORWARD_LEFT    = 8;  // 뒤오른쪽 라인 -> 전진 후 왼쪽

const int ACTION_ESCAPE_EMERGENCY       = 9;  // 애매한 경우


// ============================================================
// 라인 탈출 모터 동작 시간
// 실제 로봇에서 테스트하면서 조절 필요
// ============================================================

const unsigned long LINE_BACKWARD_MS = 400;
const unsigned long LINE_FORWARD_MS  = 400;
const unsigned long LINE_TURN_MS     = 300;
const unsigned long LINE_180_TURN_MS = 700;
const unsigned long LINE_STOP_MS     = 80;


// ============================================================
// 측면 적외선 센서 설정
//
// 왼쪽 측면 IR   -> D7
// 오른쪽 측면 IR -> D8
// ============================================================

const int LEFT_IR_PIN  = 7;
const int RIGHT_IR_PIN = 8;

// HW-201 감지값이 HIGH인지 LOW인지 확인 필요
// 아무것도 없는데 det=1이면 LOW로 바꿔보기
const int IR_DETECTED_VALUE = HIGH;

// 적외선 센서 튐 방지 시간
const unsigned long IR_STABLE_TIME_MS = 50;

const int IR_SENSOR_LEFT = 0;
const int IR_SENSOR_RIGHT = 1;
const int IR_SENSOR_COUNT = 2;

int lastRawIrState[IR_SENSOR_COUNT] = {LOW, LOW};
bool stableIrState[IR_SENSOR_COUNT] = {false, false};
unsigned long lastIrChangeTime[IR_SENSOR_COUNT] = {0, 0};

// 적외선 판단 결과
const int IR_TARGET_NONE  = 0;
const int IR_TARGET_LEFT  = 1;
const int IR_TARGET_RIGHT = 2;
const int IR_TARGET_BOTH  = 3;


// ============================================================
// 정면 초음파 센서 설정
//
// 왼쪽 초음파:
//   trig -> D10
//   echo -> D9
//
// 오른쪽 초음파:
//   trig -> D12
//   echo -> D11
// ============================================================

const int LEFT_TRIG  = 10;
const int LEFT_ECHO  = 9;

const int RIGHT_TRIG = 12;
const int RIGHT_ECHO = 11;

// 상대 로봇으로 인정할 최대 거리
const int US_MAX_RANGE_CM = 60;

// 좌우 거리 차이가 이 값 이하이면 중앙으로 판단
const int US_CENTER_THRESHOLD_CM = 3;

// pulseIn timeout
// echo가 안 들어올 때 코드가 너무 오래 멈추는 것 방지
const unsigned long US_TIMEOUT_US = 10000UL;

// 좌우 초음파 간섭 방지 딜레이
const int US_SENSOR_GAP_MS = 20;

// 좌우 초음파 센서 사이 거리
const float US_BASELINE_CM = 5.5;

// 거리 측정 실패값
const int INVALID_DISTANCE = -1;


// ============================================================
// 정면 초음파 판단 결과 저장 구조체
//
// detected:
//   정면 상대 감지 여부
//
// direction:
//   -1 : 상대가 왼쪽
//    0 : 상대가 중앙
//    1 : 상대가 오른쪽
//   99 : 감지 없음
//
// distance:
//   속도 / 공격 판단에 쓸 최종 거리
//
// steeringError:
//   rightDistance - leftDistance
//   양수면 왼쪽이 더 가까운 상황
//   음수면 오른쪽이 더 가까운 상황
// ============================================================

struct FrontTargetData
{
    bool detected;
    int direction;
    int distance;
    int steeringError;
    int leftDistance;
    int rightDistance;
};

FrontTargetData frontTarget;


// ============================================================
// 디버그 출력용 전역 변수
// ============================================================

unsigned long lastDebugPrintTime = 0;


// ============================================================
// 모터 제어 함수
//
// 현재는 PWM 속도제어를 사용하지 않음.
// enable 핀을 HIGH / LOW로만 사용.
//
// dir:
//   1  -> 전진
//  -1  -> 후진
//   0  -> 정지
// ============================================================

void setOneMotor(int in1, int in2, int enPin, int dir, bool reversed)
{
    if (reversed)
    {
        dir = -dir;
    }

    if (dir == 0)
    {
        digitalWrite(enPin, LOW);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        return;
    }

    if (dir > 0)
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }

    digitalWrite(enPin, HIGH);
}

void tankDrive(int leftDir, int rightDir)
{
    if (!MOTOR_ENABLE)
    {
        setOneMotor(M_L_IN1, M_L_IN2, M_L_EN, 0, LEFT_MOTOR_REVERSED);
        setOneMotor(M_R_IN1, M_R_IN2, M_R_EN, 0, RIGHT_MOTOR_REVERSED);
        return;
    }

    setOneMotor(M_L_IN1, M_L_IN2, M_L_EN, leftDir, LEFT_MOTOR_REVERSED);
    setOneMotor(M_R_IN1, M_R_IN2, M_R_EN, rightDir, RIGHT_MOTOR_REVERSED);
}

void motorsInit()
{
    pinMode(M_L_IN1, OUTPUT);
    pinMode(M_L_IN2, OUTPUT);
    pinMode(M_R_IN1, OUTPUT);
    pinMode(M_R_IN2, OUTPUT);
    pinMode(M_L_EN, OUTPUT);
    pinMode(M_R_EN, OUTPUT);

    tankDrive(0, 0);
}

void motorsStop()
{
    tankDrive(0, 0);
}

void driveForward()
{
    tankDrive(1, 1);
}

void driveBackward()
{
    tankDrive(-1, -1);
}

void turnLeft()
{
    tankDrive(-1, 1);
}

void turnRight()
{
    tankDrive(1, -1);
}

// PWM을 안 쓰는 곡선 주행 비슷한 동작
// 한쪽 바퀴만 굴려서 방향을 살짝 틀게 함
void forwardLeft()
{
    tankDrive(0, 1);
}

void forwardRight()
{
    tankDrive(1, 0);
}

void backwardLeft()
{
    tankDrive(0, -1);
}

void backwardRight()
{
    tankDrive(-1, 0);
}


// ============================================================
// 라인트레이서 초기화
// ============================================================

void initializeLineSensors()
{
    unsigned long now = millis();

    for (int i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        pinMode(LINE_PINS[i], INPUT);

        // 처음 시작은 경기장 안쪽이라고 가정
        lastRawLineDetected[i] = false;
        stableLineDetected[i] = false;
        lastLineChangeTime[i] = now;
    }
}


// ============================================================
// 라인트레이서 raw값 읽기
// ============================================================

int readLineRawByIndex(int index)
{
    return analogRead(LINE_PINS[index]);
}


// ============================================================
// 라인트레이서 raw값을 라인 감지 여부로 변환
// ============================================================

bool convertLineRawToDetected(int rawValue)
{
    if (LINE_DETECTED_WHEN_HIGH)
    {
        return rawValue >= LINE_THRESHOLD;
    }
    else
    {
        return rawValue <= LINE_THRESHOLD;
    }
}


// ============================================================
// 라인트레이서 안정화 포함 감지 함수
// ============================================================

bool isLineDetectedByIndex(int index)
{
    int rawValue = readLineRawByIndex(index);
    bool currentDetected = convertLineRawToDetected(rawValue);
    unsigned long now = millis();

    // 감지값이 바뀐 순간부터 시간 다시 측정
    if (currentDetected != lastRawLineDetected[index])
    {
        lastRawLineDetected[index] = currentDetected;
        lastLineChangeTime[index] = now;
    }

    // 같은 값이 일정 시간 유지되면 안정 상태로 인정
    if (now - lastLineChangeTime[index] >= LINE_STABLE_TIME_MS)
    {
        stableLineDetected[index] = currentDetected;
    }

    return stableLineDetected[index];
}


// ============================================================
// 라인트레이서 4모서리 기준 탈출 판단
//
// fl : front left  앞왼쪽
// fr : front right 앞오른쪽
// bl : back left   뒤왼쪽
// br : back right  뒤오른쪽
//
// 판단 흐름:
// - 아무것도 감지 안 됨 -> ACTION_NO_LINE
// - 앞쪽 감지 -> 뒤로 탈출
// - 뒤쪽 감지 -> 앞으로 탈출
// - 왼쪽 감지 -> 오른쪽으로 탈출
// - 오른쪽 감지 -> 왼쪽으로 탈출
// - 애매하면 emergency
// ============================================================

int decideLineEscapeAction()
{
    bool fl = isLineDetectedByIndex(LINE_FRONT_LEFT);
    bool fr = isLineDetectedByIndex(LINE_FRONT_RIGHT);
    bool bl = isLineDetectedByIndex(LINE_BACK_LEFT);
    bool br = isLineDetectedByIndex(LINE_BACK_RIGHT);

    // 아무 라인도 감지되지 않음
    if (!fl && !fr && !bl && !br)
    {
        return ACTION_NO_LINE;
    }

    // 왼쪽 전체가 위험한 경우
    // 앞왼쪽 + 뒤왼쪽 감지
    if (fl && bl && !fr && !br)
    {
        return ACTION_ESCAPE_TURN_RIGHT;
    }

    // 오른쪽 전체가 위험한 경우
    // 앞오른쪽 + 뒤오른쪽 감지
    if (!fl && !bl && fr && br)
    {
        return ACTION_ESCAPE_TURN_LEFT;
    }

    // 앞쪽만 위험한 경우
    if ((fl || fr) && !bl && !br)
    {
        // 앞 양쪽 다 감지
        if (fl && fr)
        {
            return ACTION_ESCAPE_BACKWARD_ROTATE;
        }

        // 앞왼쪽만 감지
        if (fl)
        {
            return ACTION_ESCAPE_BACKWARD_RIGHT;
        }

        // 앞오른쪽만 감지
        if (fr)
        {
            return ACTION_ESCAPE_BACKWARD_LEFT;
        }
    }

    // 뒤쪽만 위험한 경우
    if ((bl || br) && !fl && !fr)
    {
        // 뒤 양쪽 다 감지
        if (bl && br)
        {
            return ACTION_ESCAPE_FORWARD_ROTATE;
        }

        // 뒤왼쪽만 감지
        if (bl)
        {
            return ACTION_ESCAPE_FORWARD_RIGHT;
        }

        // 뒤오른쪽만 감지
        if (br)
        {
            return ACTION_ESCAPE_FORWARD_LEFT;
        }
    }

    // 대각선 감지, 3개 이상 감지, 앞뒤 동시 감지 등
    return ACTION_ESCAPE_EMERGENCY;
}


// ============================================================
// 라인 탈출 실제 모터 동작
// ============================================================

void performLineEscape(int action)
{
    motorsStop();
    delay(LINE_STOP_MS);

    switch (action)
    {
        case ACTION_ESCAPE_BACKWARD_ROTATE:
            driveBackward();
            delay(LINE_BACKWARD_MS);

            turnRight();
            delay(LINE_180_TURN_MS);
            break;

        case ACTION_ESCAPE_FORWARD_ROTATE:
            driveForward();
            delay(LINE_FORWARD_MS);

            turnRight();
            delay(LINE_TURN_MS);
            break;

        case ACTION_ESCAPE_TURN_RIGHT:
            turnRight();
            delay(LINE_TURN_MS);
            break;

        case ACTION_ESCAPE_TURN_LEFT:
            turnLeft();
            delay(LINE_TURN_MS);
            break;

        case ACTION_ESCAPE_BACKWARD_RIGHT:
            driveBackward();
            delay(LINE_BACKWARD_MS);

            turnRight();
            delay(LINE_TURN_MS);
            break;

        case ACTION_ESCAPE_BACKWARD_LEFT:
            driveBackward();
            delay(LINE_BACKWARD_MS);

            turnLeft();
            delay(LINE_TURN_MS);
            break;

        case ACTION_ESCAPE_FORWARD_RIGHT:
            driveForward();
            delay(LINE_FORWARD_MS);

            turnRight();
            delay(LINE_TURN_MS);
            break;

        case ACTION_ESCAPE_FORWARD_LEFT:
            driveForward();
            delay(LINE_FORWARD_MS);

            turnLeft();
            delay(LINE_TURN_MS);
            break;

        case ACTION_ESCAPE_EMERGENCY:
            driveBackward();
            delay(LINE_BACKWARD_MS);

            turnRight();
            delay(LINE_TURN_MS);
            break;

        case ACTION_NO_LINE:
        default:
            break;
    }

    motorsStop();
    delay(LINE_STOP_MS);
}


// ============================================================
// 적외선 센서 초기화
// ============================================================

void initializeIrSensors()
{
    pinMode(LEFT_IR_PIN, INPUT);
    pinMode(RIGHT_IR_PIN, INPUT);

    unsigned long now = millis();

    for (int i = 0; i < IR_SENSOR_COUNT; i++)
    {
        lastRawIrState[i] = LOW;
        stableIrState[i] = false;
        lastIrChangeTime[i] = now;
    }
}


// ============================================================
// 적외선 센서 index 변환
// ============================================================

int getIrSensorIndex(int pin)
{
    if (pin == LEFT_IR_PIN)
    {
        return IR_SENSOR_LEFT;
    }

    if (pin == RIGHT_IR_PIN)
    {
        return IR_SENSOR_RIGHT;
    }

    return IR_SENSOR_LEFT;
}


// ============================================================
// 적외선 raw값 읽기
// ============================================================

int readIrRaw(int pin)
{
    return digitalRead(pin);
}


// ============================================================
// 적외선 안정화 포함 감지 함수
// ============================================================

bool isIrDetected(int pin)
{
    int sensorIndex = getIrSensorIndex(pin);
    int value = readIrRaw(pin);
    unsigned long now = millis();

    if (value != lastRawIrState[sensorIndex])
    {
        lastRawIrState[sensorIndex] = value;
        lastIrChangeTime[sensorIndex] = now;
    }

    if (now - lastIrChangeTime[sensorIndex] >= IR_STABLE_TIME_MS)
    {
        stableIrState[sensorIndex] = (value == IR_DETECTED_VALUE);
    }

    return stableIrState[sensorIndex];
}


// ============================================================
// 좌/우 측면 적외선으로 상대 위치 판단
// ============================================================

int detectIrTarget()
{
    bool leftDetected  = isIrDetected(LEFT_IR_PIN);
    bool rightDetected = isIrDetected(RIGHT_IR_PIN);

    if (!leftDetected && !rightDetected)
    {
        return IR_TARGET_NONE;
    }

    if (leftDetected && !rightDetected)
    {
        return IR_TARGET_LEFT;
    }

    if (!leftDetected && rightDetected)
    {
        return IR_TARGET_RIGHT;
    }

    return IR_TARGET_BOTH;
}


// ============================================================
// 측면 적외선 감지 시 모터 동작
// ============================================================

void followIrTarget(int target)
{
    if (target == IR_TARGET_LEFT)
    {
        turnLeft();
    }
    else if (target == IR_TARGET_RIGHT)
    {
        turnRight();
    }
    else if (target == IR_TARGET_BOTH)
    {
        driveForward();
    }
}


// ============================================================
// 초음파 센서 초기화
// ============================================================

void initializeUltrasonicSensors()
{
    pinMode(LEFT_TRIG, OUTPUT);
    pinMode(LEFT_ECHO, INPUT);

    pinMode(RIGHT_TRIG, OUTPUT);
    pinMode(RIGHT_ECHO, INPUT);

    digitalWrite(LEFT_TRIG, LOW);
    digitalWrite(RIGHT_TRIG, LOW);
}


// ============================================================
// 초음파 거리 측정
//
// 정상 거리: cm
// 실패: INVALID_DISTANCE
// ============================================================

int measureDistance(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, US_TIMEOUT_US);

    if (duration == 0)
    {
        return INVALID_DISTANCE;
    }

    int distance = (int)(duration * 0.0343 / 2.0);

    if (distance <= 0 || distance > US_MAX_RANGE_CM)
    {
        return INVALID_DISTANCE;
    }

    return distance;
}


// ============================================================
// 삼각형 계산으로 정면 직선거리 구하기
//
// 계산 성공: 정면 직선거리 반환
// 계산 실패: INVALID_DISTANCE 반환
// ============================================================

int calculateStraightDistance(int leftDistance, int rightDistance)
{
    float L = leftDistance;
    float R = rightDistance;
    float b = US_BASELINE_CM;

    float diff = L - R;

    if (diff < 0)
    {
        diff = -diff;
    }

    if (diff > b)
    {
        return INVALID_DISTANCE;
    }

    float x = (L * L - R * R) / (2.0 * b);
    float horizontalFromLeft = x + (b / 2.0);
    float ySquared = L * L - horizontalFromLeft * horizontalFromLeft;

    if (ySquared < 0)
    {
        return INVALID_DISTANCE;
    }

    return (int)(sqrt(ySquared) + 0.5);
}


// ============================================================
// 초음파 결과 초기화
// ============================================================

void resetFrontTarget()
{
    frontTarget.detected = false;
    frontTarget.direction = 99;
    frontTarget.distance = INVALID_DISTANCE;
    frontTarget.steeringError = 0;
    frontTarget.leftDistance = INVALID_DISTANCE;
    frontTarget.rightDistance = INVALID_DISTANCE;
}


// ============================================================
// 정면 상대 감지
// ============================================================

void detectFrontTarget()
{
    resetFrontTarget();

    int leftDistance = measureDistance(LEFT_TRIG, LEFT_ECHO);

    delay(US_SENSOR_GAP_MS);

    int rightDistance = measureDistance(RIGHT_TRIG, RIGHT_ECHO);

    frontTarget.leftDistance = leftDistance;
    frontTarget.rightDistance = rightDistance;

    bool leftValid = (leftDistance > 0);
    bool rightValid = (rightDistance > 0);

    if (!leftValid && !rightValid)
    {
        return;
    }

    frontTarget.detected = true;

    if (leftValid && !rightValid)
    {
        frontTarget.direction = -1;
        frontTarget.distance = leftDistance;
        frontTarget.steeringError = 0;
        return;
    }

    if (!leftValid && rightValid)
    {
        frontTarget.direction = 1;
        frontTarget.distance = rightDistance;
        frontTarget.steeringError = 0;
        return;
    }

    int error = rightDistance - leftDistance;
    frontTarget.steeringError = error;

    if (abs(error) <= US_CENTER_THRESHOLD_CM)
    {
        frontTarget.direction = 0;
        frontTarget.steeringError = 0;
    }
    else if (error > 0)
    {
        frontTarget.direction = -1;
    }
    else
    {
        frontTarget.direction = 1;
    }

    int straightDistance = calculateStraightDistance(leftDistance, rightDistance);

    if (straightDistance > 0)
    {
        frontTarget.distance = straightDistance;
    }
    else
    {
        if (leftDistance < rightDistance)
        {
            frontTarget.distance = leftDistance;
        }
        else
        {
            frontTarget.distance = rightDistance;
        }
    }
}


// ============================================================
// 정면 초음파 감지 시 모터 동작
// ============================================================

void followFrontTarget()
{
    if (!frontTarget.detected)
    {
        return;
    }

    if (frontTarget.direction == 0)
    {
        driveForward();
    }
    else if (frontTarget.direction == -1)
    {
        forwardLeft();
    }
    else if (frontTarget.direction == 1)
    {
        forwardRight();
    }
}


// ============================================================
// 아무것도 감지 안 됐을 때 탐색 동작
// ============================================================

void searchTarget()
{
    turnRight();
}


// ============================================================
// 디버깅 출력 보조 함수들
// 문자열을 최대한 F()로 출력해서 SRAM 사용량을 줄임
// ============================================================

void printLineActionText(int action)
{
    switch (action)
    {
        case ACTION_NO_LINE:
            Serial.println(F("NO LINE"));
            break;

        case ACTION_ESCAPE_BACKWARD_ROTATE:
            Serial.println(F("FRONT LINE -> BACKWARD + TURN"));
            break;

        case ACTION_ESCAPE_FORWARD_ROTATE:
            Serial.println(F("REAR LINE -> FORWARD + TURN"));
            break;

        case ACTION_ESCAPE_TURN_RIGHT:
            Serial.println(F("LEFT SIDE LINE -> TURN RIGHT"));
            break;

        case ACTION_ESCAPE_TURN_LEFT:
            Serial.println(F("RIGHT SIDE LINE -> TURN LEFT"));
            break;

        case ACTION_ESCAPE_BACKWARD_RIGHT:
            Serial.println(F("FRONT LEFT LINE -> BACKWARD + RIGHT"));
            break;

        case ACTION_ESCAPE_BACKWARD_LEFT:
            Serial.println(F("FRONT RIGHT LINE -> BACKWARD + LEFT"));
            break;

        case ACTION_ESCAPE_FORWARD_RIGHT:
            Serial.println(F("BACK LEFT LINE -> FORWARD + RIGHT"));
            break;

        case ACTION_ESCAPE_FORWARD_LEFT:
            Serial.println(F("BACK RIGHT LINE -> FORWARD + LEFT"));
            break;

        case ACTION_ESCAPE_EMERGENCY:
            Serial.println(F("LINE EMERGENCY"));
            break;

        default:
            Serial.println(F("UNKNOWN LINE ACTION"));
            break;
    }
}

void printIrTargetText(int target)
{
    switch (target)
    {
        case IR_TARGET_NONE:
            Serial.println(F("NO IR"));
            break;

        case IR_TARGET_LEFT:
            Serial.println(F("LEFT IR"));
            break;

        case IR_TARGET_RIGHT:
            Serial.println(F("RIGHT IR"));
            break;

        case IR_TARGET_BOTH:
            Serial.println(F("BOTH IR"));
            break;

        default:
            Serial.println(F("UNKNOWN IR"));
            break;
    }
}

void printFrontDirectionText(int direction)
{
    if (direction == -1)
    {
        Serial.println(F("LEFT"));
    }
    else if (direction == 0)
    {
        Serial.println(F("CENTER"));
    }
    else if (direction == 1)
    {
        Serial.println(F("RIGHT"));
    }
    else
    {
        Serial.println(F("NONE"));
    }
}


// ============================================================
// 센서값 실시간 디버깅 출력
//
// 한 줄에 너무 길게 쓰지 않고,
// LINE / IR / ULTRASONIC / SELECTED ACTION을 나눠서 출력
// ============================================================

void printLiveDebug(int lineAction, int irTarget)
{
    int flRaw = readLineRawByIndex(LINE_FRONT_LEFT);
    int frRaw = readLineRawByIndex(LINE_FRONT_RIGHT);
    int blRaw = readLineRawByIndex(LINE_BACK_LEFT);
    int brRaw = readLineRawByIndex(LINE_BACK_RIGHT);

    int leftIrRaw = readIrRaw(LEFT_IR_PIN);
    int rightIrRaw = readIrRaw(RIGHT_IR_PIN);

    Serial.println();
    Serial.println(F("========== DEBUG =========="));

    // ----------------------------
    // Line tracer
    // ----------------------------
    Serial.println(F("[LINE TRACER]"));

    Serial.print(F("FL raw = "));
    Serial.print(flRaw);
    Serial.print(F(" / det = "));
    Serial.println(stableLineDetected[LINE_FRONT_LEFT]);

    Serial.print(F("FR raw = "));
    Serial.print(frRaw);
    Serial.print(F(" / det = "));
    Serial.println(stableLineDetected[LINE_FRONT_RIGHT]);

    Serial.print(F("BL raw = "));
    Serial.print(blRaw);
    Serial.print(F(" / det = "));
    Serial.println(stableLineDetected[LINE_BACK_LEFT]);

    Serial.print(F("BR raw = "));
    Serial.print(brRaw);
    Serial.print(F(" / det = "));
    Serial.println(stableLineDetected[LINE_BACK_RIGHT]);

    Serial.print(F("LINE ACTION = "));
    printLineActionText(lineAction);

    // ----------------------------
    // Side IR
    // ----------------------------
    Serial.println(F("[SIDE IR]"));

    Serial.print(F("LEFT  raw = "));
    Serial.print(leftIrRaw);
    Serial.print(F(" / det = "));
    Serial.println(stableIrState[IR_SENSOR_LEFT]);

    Serial.print(F("RIGHT raw = "));
    Serial.print(rightIrRaw);
    Serial.print(F(" / det = "));
    Serial.println(stableIrState[IR_SENSOR_RIGHT]);

    Serial.print(F("IR TARGET = "));
    printIrTargetText(irTarget);

    // ----------------------------
    // Ultrasonic
    // ----------------------------
    Serial.println(F("[ULTRASONIC]"));

    Serial.print(F("LEFT distance  = "));
    Serial.print(frontTarget.leftDistance);
    Serial.println(F(" cm"));

    Serial.print(F("RIGHT distance = "));
    Serial.print(frontTarget.rightDistance);
    Serial.println(F(" cm"));

    Serial.print(F("US detected = "));
    Serial.println(frontTarget.detected);

    Serial.print(F("US direction = "));
    printFrontDirectionText(frontTarget.direction);

    Serial.print(F("US distance = "));
    Serial.print(frontTarget.distance);
    Serial.println(F(" cm"));

    Serial.print(F("US steeringError = "));
    Serial.println(frontTarget.steeringError);

    // ----------------------------
    // Selected behavior
    // ----------------------------
    Serial.println(F("[SELECTED ACTION]"));

    if (lineAction != ACTION_NO_LINE)
    {
        Serial.print(F("MAIN -> LINE ESCAPE: "));
        printLineActionText(lineAction);
    }
    else if (irTarget != IR_TARGET_NONE)
    {
        Serial.print(F("MAIN -> SIDE IR FOLLOW: "));
        printIrTargetText(irTarget);
    }
    else if (frontTarget.detected)
    {
        Serial.print(F("MAIN -> FRONT US FOLLOW: "));
        printFrontDirectionText(frontTarget.direction);
    }
    else
    {
        Serial.println(F("MAIN -> SEARCH ROTATE"));
    }

    Serial.println(F("==========================="));
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    motorsInit();
    initializeLineSensors();
    initializeIrSensors();
    initializeUltrasonicSensors();

    resetFrontTarget();

    if (DEBUG_SERIAL)
    {
        Serial.println(F("=== Battle Robot Integrated Main Start ==="));
        Serial.println(F("Serial Monitor Baud Rate: 115200"));
        Serial.println(F("Live debug print enabled"));

        if (MOTOR_ENABLE)
        {
            Serial.println(F("Motor: ENABLED"));
        }
        else
        {
            Serial.println(F("Motor: DISABLED for sensor debug"));
        }
    }

    delay(1000);

    // 그림의 "중간까지 직진" 부분
    if (USE_START_FORWARD)
    {
        if (DEBUG_SERIAL)
        {
            Serial.println(F("START: drive forward to center"));
        }

        driveForward();
        delay(START_FORWARD_MS);
        motorsStop();
        delay(200);
    }
}


// ============================================================
// MAIN LOOP
//
// 현재 디버깅 버전:
// - 매 loop마다 라인트레이서, IR, 초음파를 모두 읽음
// - 500ms마다 보기 좋게 시리얼 출력
// - 최종 행동 우선순위는 아래와 같음
//
// 1. 라인트레이서
// 2. 측면 IR
// 3. 정면 초음파
// 4. 탐색 회전
//
// 주의:
// - 초음파 pulseIn 때문에 loop가 약간 느릴 수 있음
// - 실제 경기용으로 갈 때는 DEBUG_SERIAL을 false로 바꾸면 됨
// ============================================================

void loop()
{
    // 1. 라인트레이서 판단
    int lineAction = decideLineEscapeAction();

    // 2. 적외선 판단
    int irTarget = detectIrTarget();

    // 3. 초음파 판단
    detectFrontTarget();

    // 4. 센서값 실시간 출력
    if (DEBUG_SERIAL)
    {
        unsigned long now = millis();

        if (now - lastDebugPrintTime >= DEBUG_PRINT_INTERVAL_MS)
        {
            lastDebugPrintTime = now;
            printLiveDebug(lineAction, irTarget);
        }
    }

    // ========================================================
    // 1순위: 라인트레이서
    // ========================================================
    if (lineAction != ACTION_NO_LINE)
    {
        performLineEscape(lineAction);
        delay(MAIN_LOOP_DELAY_MS);
        return;
    }

    // ========================================================
    // 2순위: 측면 적외선
    // ========================================================
    if (irTarget != IR_TARGET_NONE)
    {
        followIrTarget(irTarget);
        delay(MAIN_LOOP_DELAY_MS);
        return;
    }

    // ========================================================
    // 3순위: 정면 초음파
    // ========================================================
    if (frontTarget.detected)
    {
        followFrontTarget();
        delay(MAIN_LOOP_DELAY_MS);
        return;
    }

    // ========================================================
    // 4순위: 아무것도 감지 안 됨
    // ========================================================
    searchTarget();

    delay(MAIN_LOOP_DELAY_MS);
}
