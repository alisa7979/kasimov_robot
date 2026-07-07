// ============================================================
// Front Ultrasonic Sensor Control Code
// 담당: 로봇 정면 초음파 센서 2개(좌/우) - 상대 로봇 위치 감지
//
// 역할:
// - 좌/우 초음파 센서로 거리 측정
// - 상대 로봇이 정면 감지 범위 안에 있는지, 어느 쪽에 더 가까운지 판단
// - detectFrontTarget() 호출하여 추적 동작과 연결하면 됨
// ============================================================

const int LEFT_TRIG  = 11;
const int LEFT_ECHO  = 10;
const int RIGHT_TRIG = 9;
const int RIGHT_ECHO = 8;

// 상대 로봇으로 인정할 최대 감지 거리(cm). 로봇/경기장 크기에 맞게 조절 가능
const int US_MAX_RANGE_CM = 60;

// 좌우 거리 차이가 이 값(cm) 이하이면 정면에 있다고 판단. 조절 가능
const int US_CENTER_THRESHOLD_CM = 3;

void initializeUltrasonicSensors()
{
    pinMode(LEFT_TRIG, OUTPUT);
    pinMode(LEFT_ECHO, INPUT);

    pinMode(RIGHT_TRIG, OUTPUT);
    pinMode(RIGHT_ECHO, INPUT);
}

// 초음파 거리 측정(cm). 응답이 없으면 -1 반환
int measureDistance(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);

    if (duration == 0)
    {
        return US_MAX_RANGE_CM;
    }

    return duration * 0.0343 / 2.0;
}

int detectFrontTarget()
{
    int leftDistance  = measureDistance(LEFT_TRIG, LEFT_ECHO);
    int rightDistance = measureDistance(RIGHT_TRIG, RIGHT_ECHO);

    bool leftValid  = (leftDistance  > 0 && leftDistance  <= US_MAX_RANGE_CM);
    bool rightValid = (rightDistance > 0 && rightDistance <= US_MAX_RANGE_CM);

    // 감지 범위 안에 상대가 없음
    if (!leftValid && !rightValid)
    {
        return -1;
    }

    // 왼쪽 센서만 상대를 감지
    if (leftValid && !rightValid)
    {
        return -leftDistance;
    }

    // 오른쪽 센서만 상대를 감지
    if (!leftValid && rightValid)
    {
        return rightDistance;
    }

    // 좌우 둘 다 감지 -> 거리 차이로 더 가까운 쪽 판단
    int distanceDiff = leftDistance - rightDistance;

    // 왼쪽 거리가 더 짧음 -> 상대가 왼쪽에 더 가까움
    if (distanceDiff < 0  && abs(istanceDiff) > US_CENTER_THRESHOLD_CM)
    {
        return distanceDiff;
    }

    // 오른쪽 거리가 더 짧음 -> 상대가 오른쪽에 더 가까움
    if (distanceDiff > 0 && abs(distanceDiff) > US_CENTER_THRESHOLD_CM)
    {
        return distanceDiff;
    }

    // 좌우 거리가 비슷함 -> 정면 중앙에 있다고 판단
    return 0;
}


// ============================================================
// 아래는 디버깅용 시리얼 모니터 출력용
// ============================================================

void printSensorValues()
{
    int target = detectFrontTarget();

    Serial.println(target);
}


// test
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
