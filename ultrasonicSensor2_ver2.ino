// ============================================================
// Front Ultrasonic Sensor Control Code - Simple Version
//
// 역할:
// - 정면 좌/우 초음파 센서 거리 측정
// - 상대 감지 여부 detected 판단
// - 방향 direction 판단
// - 속도 제어용 distance 계산
// - 좌우 회전 제어용 steeringError 계산
//
// direction:
// -1 : 왼쪽
//  0 : 중앙
//  1 : 오른쪽
// 99 : 감지 없음
//
// steeringError:
// 둘 다 감지됐을 때만 rightDistance - leftDistance 사용
// ============================================================


// ============================================================
// 핀 설정
// ============================================================

const int LEFT_TRIG  = 11;
const int LEFT_ECHO  = 10;
const int RIGHT_TRIG = 9;
const int RIGHT_ECHO = 8;


// ============================================================
// 기본 설정
// ============================================================

const int US_MAX_RANGE_CM = 60;          // 최대 감지 거리
const int US_CENTER_THRESHOLD_CM = 3;    // 이 차이 이하면 중앙
const unsigned long US_TIMEOUT_US = 10000UL;
const int US_SENSOR_GAP_MS = 20;         // 초음파 간섭 방지 딜레이

const float US_BASELINE_CM = 5.5;        // 좌우 초음파 센서 사이 거리
const int INVALID_DISTANCE = -1;


// ============================================================
// 결과 저장 구조체
// ============================================================

struct FrontTarget
{
    bool detected;       // 상대 감지 여부
    int direction;       // -1 왼쪽, 0 중앙, 1 오른쪽, 99 없음
    int distance;        // 속도 제어용 거리
    int steeringError;   // 좌우 회전 제어용 오차

    int leftDistance;    // 디버깅용
    int rightDistance;   // 디버깅용
};


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
// 실패: -1
// ============================================================

int measureDistance(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, US_TIMEOUT_US);

    if (duration == 0)
    {
        return INVALID_DISTANCE;
    }

    int distance = duration * 0.0343 / 2.0;

    if (distance <= 0 || distance > US_MAX_RANGE_CM)
    {
        return INVALID_DISTANCE;
    }

    return distance;
}


// ============================================================
// 삼각형 계산으로 정면 직선거리 구하기
//
// leftDistance  = 왼쪽 센서가 본 거리 L
// rightDistance = 오른쪽 센서가 본 거리 R
// US_BASELINE_CM = 두 센서 사이 거리 b
//
// 계산 성공: 정면 직선거리 반환
// 계산 실패: -1 반환
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

    // 삼각형이 만들어지려면 두 거리 차이가 센서 간격보다 작아야 함
    if (diff > b)
    {
        return INVALID_DISTANCE;
    }

    // 상대의 좌우 위치 x 계산
    float x = (L * L - R * R) / (2.0 * b);

    // 왼쪽 센서 기준 수평거리
    float horizontalFromLeft = x + (b / 2.0);

    // 정면 직선거리 y 계산
    float ySquared = L * L - horizontalFromLeft * horizontalFromLeft;

    if (ySquared < 0)
    {
        return INVALID_DISTANCE;
    }

    return (int)(sqrt(ySquared) + 0.5);
}


// ============================================================
// 정면 상대 감지
// ============================================================

FrontTarget detectFrontTarget()
{
    FrontTarget target;

    // 기본값
    target.detected = false;
    target.direction = 99;
    target.distance = INVALID_DISTANCE;
    target.steeringError = 0;
    target.leftDistance = INVALID_DISTANCE;
    target.rightDistance = INVALID_DISTANCE;

    // 1. 왼쪽 측정
    int leftDistance = measureDistance(LEFT_TRIG, LEFT_ECHO);

    // 2. 초음파 간섭 방지
    delay(US_SENSOR_GAP_MS);

    // 3. 오른쪽 측정
    int rightDistance = measureDistance(RIGHT_TRIG, RIGHT_ECHO);

    target.leftDistance = leftDistance;
    target.rightDistance = rightDistance;

    bool leftValid = (leftDistance > 0);
    bool rightValid = (rightDistance > 0);

    // ========================================================
    // 둘 다 감지 안 됨
    // ========================================================
    if (!leftValid && !rightValid)
    {
        return target;
    }

    target.detected = true;

    // ========================================================
    // 왼쪽만 감지
    // ========================================================
    if (leftValid && !rightValid)
    {
        target.direction = -1;
        target.distance = leftDistance;
        target.steeringError = 0;

        return target;
    }

    // ========================================================
    // 오른쪽만 감지
    // ========================================================
    if (!leftValid && rightValid)
    {
        target.direction = 1;
        target.distance = rightDistance;
        target.steeringError = 0;

        return target;
    }

    // ========================================================
    // 둘 다 감지됨
    // ========================================================

    // 1. 좌우 방향 판단
    int error = rightDistance - leftDistance;
    target.steeringError = error;

    if (abs(error) <= US_CENTER_THRESHOLD_CM)
    {
        target.direction = 0;
        target.steeringError = 0;
    }
    else if (error > 0)
    {
        // 왼쪽 센서 거리가 더 짧음
        // 상대가 왼쪽 쪽
        target.direction = -1;
    }
    else
    {
        // 오른쪽 센서 거리가 더 짧음
        // 상대가 오른쪽 쪽
        target.direction = 1;
    }

    // 2. 거리 계산
    int straightDistance = calculateStraightDistance(leftDistance, rightDistance);

    if (straightDistance > 0)
    {
        // 삼각형 계산 성공
        target.distance = straightDistance;
    }
    else
    {
        // 삼각형 계산 실패하면 더 가까운 거리 사용
        target.distance = min(leftDistance, rightDistance);
    }

    return target;
}


// ============================================================
// 디버깅 출력
// ============================================================

void printSensorValues()
{
    FrontTarget target = detectFrontTarget();

    Serial.print("L: ");
    Serial.print(target.leftDistance);

    Serial.print(" cm, R: ");
    Serial.print(target.rightDistance);

    Serial.print(" cm | detected: ");
    Serial.print(target.detected);

    Serial.print(" | direction: ");
    Serial.print(target.direction);

    Serial.print(" | distance: ");
    Serial.print(target.distance);

    Serial.print(" | steeringError: ");
    Serial.println(target.steeringError);
}


// ============================================================
// test
// ============================================================

void setup()
{
    Serial.begin(9600);
    initializeUltrasonicSensors();
}

void loop()
{
    printSensorValues();
    delay(100);
}
