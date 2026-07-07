// ============================================================
// Front Ultrasonic Sensor Control Code
// 담당: 정면 초음파 센서 2개(좌/우) - 상대 로봇 위치 감지
//
// 개선 사항:
//  - pulseIn 타임아웃 추가 (없으면 빈 공간에서 최대 1초 멈춤)
//  - 중앙값(median) 필터로 순간 노이즈 제거
//  - 좌/우 판단 버그 수정 (원래 조건은 항상 거짓이었음)
//  - 반환을 visible / inRange / bearing 으로 분리 (int 하나로 안 뭉침)
//  - 두 센서 사이 짧은 간격 (크로스토크 방지)
//
// 실제 main에서는 updateUltrasonic()을 매 루프 1번 호출한 뒤,
// usVisible() / usInRange() / usBearing() 을 읽으면 됨.
// ============================================================

// ---- pins (Config.h 최종 핀맵) -----------------------------
const int LEFT_TRIG  = 7;
const int LEFT_ECHO  = 8;
const int RIGHT_TRIG = 11;
const int RIGHT_ECHO = 12;

// ---- tunables ----------------------------------------------
const int US_MAX_RANGE_CM        = 60;    // 이 안이면 "상대 보임"
const int US_ATTACK_CM           = 12;    // 이 안이면 "밀 수 있음"
const int US_CENTER_THRESHOLD_CM = 5;     // 좌우 차이 이 이하 -> 정면
const unsigned long US_TIMEOUT_US = 5000; // 약 85cm. 없으면 멈춤 방지
const int US_NO_ECHO = 9999;              // 에코 없음 = 아주 멀다로 취급

// 좌/우 최신 거리(cm) 캐시. 매 루프 updateUltrasonic()에서 갱신
static int lastLeft  = US_NO_ECHO;
static int lastRight = US_NO_ECHO;

void initializeUltrasonicSensors()
{
    pinMode(LEFT_TRIG, OUTPUT);
    pinMode(LEFT_ECHO, INPUT);
    pinMode(RIGHT_TRIG, OUTPUT);
    pinMode(RIGHT_ECHO, INPUT);
}

// 초음파 1회 측정(cm). 에코 없으면 US_NO_ECHO
int measureOnce(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
    if (duration == 0)
    {
        return US_NO_ECHO;
    }

    int cm = (int)(duration * 0.0343 / 2.0);
    if (cm <= 0)
    {
        return US_NO_ECHO;
    }
    return cm;
}

// 세 값의 중앙값
int median3(int a, int b, int c)
{
    if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
    if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
    return c;
}

// 3회 측정 후 중앙값 -> 순간 노이즈 제거
int measureDistance(int trigPin, int echoPin)
{
    int a = measureOnce(trigPin, echoPin);
    delayMicroseconds(800);
    int b = measureOnce(trigPin, echoPin);
    delayMicroseconds(800);
    int c = measureOnce(trigPin, echoPin);
    return median3(a, b, c);
}

// 매 루프 1번 호출: 좌/우 갱신
void updateUltrasonic()
{
    lastLeft  = measureDistance(LEFT_TRIG, LEFT_ECHO);
    delay(5);                       // 크로스토크 방지 간격
    lastRight = measureDistance(RIGHT_TRIG, RIGHT_ECHO);
}

// ============================================================
// main에서 사용할 판단 함수들
// ============================================================

// 정면에서 가장 가까운 유효 거리(cm). 아무것도 없으면 큰 값
int usFrontMinCm()
{
    return (lastLeft < lastRight) ? lastLeft : lastRight;
}

// 감지 범위 안에 상대가 있는가
bool usVisible()
{
    return usFrontMinCm() <= US_MAX_RANGE_CM;
}

// 밀어붙일 만큼 가까운가
bool usInRange()
{
    return usFrontMinCm() <= US_ATTACK_CM;
}

// -1 : 왼쪽이 더 가까움 / 0 : 정면 / +1 : 오른쪽이 더 가까움
// (원본 버그 수정: 'diff<0 && diff>3' 은 항상 거짓이었음)
int usBearing()
{
    bool leftValid  = (lastLeft  <= US_MAX_RANGE_CM);
    bool rightValid = (lastRight <= US_MAX_RANGE_CM);

    if (leftValid && !rightValid)  return -1;
    if (rightValid && !leftValid)  return  1;
    if (!leftValid && !rightValid) return  0;

    int diff = lastLeft - lastRight;
    if (diff < -US_CENTER_THRESHOLD_CM) return -1;
    if (diff >  US_CENTER_THRESHOLD_CM) return  1;
    return 0;
}

// ============================================================
// 아래는 디버깅용 시리얼 모니터 출력
// ============================================================
void printSensorValues()
{
    Serial.print("L: ");
    Serial.print(lastLeft);
    Serial.print("  R: ");
    Serial.print(lastRight);

    Serial.print("  | visible: ");
    Serial.print(usVisible());
    Serial.print("  inRange: ");
    Serial.print(usInRange());
    Serial.print("  bearing: ");
    Serial.println(usBearing());   // -1 왼쪽 / 0 정면 / +1 오른쪽
}

// ============================================================
// 아래는 단순 테스트용.
// 실제론 main loop()에서 updateUltrasonic() + 판단 함수만 호출.
// ============================================================
void setup()
{
    Serial.begin(115200);
    initializeUltrasonicSensors();
}

void loop()
{
    updateUltrasonic();
    printSensorValues();
    delay(100);
}
