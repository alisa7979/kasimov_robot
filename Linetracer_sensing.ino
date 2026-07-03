// ============================================================
// Line Tracer Sensor Control Code
// 라인트레이서 센서 값 읽기 + 라인 감지 방향 판단
// EscapeAction 값으로 반환한다.
//
// 모터 동작을 위해 decideLineEscapeAction() 함수를 호출 
// 반환된 ACTION 값을 본다.
// ============================================================
// 디지털 사용 가정
const int FRONT_LINE_PIN = 2;
const int REAR_LINE_PIN  = 3;
const int LEFT_LINE_PIN  = 4;
const int RIGHT_LINE_PIN = 5;

// 디지털 센서 기준: 감지되면 1, 감지 안 되면 0
const int LINE_DETECTED_LEVEL = HIGH;

// 같은 값이 이 시간 이상 유지되어야 실제 상태로 인정, 조절 가능
const unsigned long LINE_STABLE_TIME_MS = 50;

enum LineSensorIndex
{
    SENSOR_FRONT = 0,
    SENSOR_REAR,
    SENSOR_LEFT,
    SENSOR_RIGHT,
    SENSOR_COUNT
};

// 처음 시작은 경기장 안쪽 흰색이라고 가정
// 흰색은 0, 라인 감지는 1로 초기 상태를 고정
int lastRawLineState[SENSOR_COUNT] = {LOW, LOW, LOW, LOW};
bool stableLineState[SENSOR_COUNT] = {false, false, false, false};
unsigned long lastLineChangeTime[SENSOR_COUNT] = {0, 0, 0, 0};


enum EscapeAction
{
    ACTION_NO_LINE,                 // 라인 감지 없음

    ACTION_ESCAPE_BACKWARD,         // 앞쪽 라인 감지 -> 뒤로 탈출
    ACTION_ESCAPE_FORWARD,          // 뒤쪽 라인 감지 -> 앞으로 탈출
    ACTION_ESCAPE_TURN_RIGHT,       // 왼쪽 라인 감지 -> 오른쪽으로 탈출
    ACTION_ESCAPE_TURN_LEFT,        // 오른쪽 라인 감지 -> 왼쪽으로 탈출

    ACTION_ESCAPE_BACKWARD_RIGHT,   // 앞 + 왼쪽 라인 감지 -> 뒤 + 오른쪽으로 탈출
    ACTION_ESCAPE_BACKWARD_LEFT,    // 앞 + 오른쪽 라인 감지 -> 뒤 + 왼쪽으로 탈출
    ACTION_ESCAPE_FORWARD_RIGHT,    // 뒤 + 왼쪽 라인 감지 -> 앞 + 오른쪽으로 탈출
    ACTION_ESCAPE_FORWARD_LEFT,     // 뒤 + 오른쪽 라인 감지 -> 앞 + 왼쪽으로 탈출

    ACTION_ESCAPE_EMERGENCY         // 판단이 애매한 경우, 긴급 탈출 또는 정지 필요
};

int getLineSensorIndex(int pin)
{
    if (pin == FRONT_LINE_PIN)
    {
        return SENSOR_FRONT;
    }

    if (pin == REAR_LINE_PIN)
    {
        return SENSOR_REAR;
    }

    if (pin == LEFT_LINE_PIN)
    {
        return SENSOR_LEFT;
    }

    if (pin == RIGHT_LINE_PIN)
    {
        return SENSOR_RIGHT;
    }

    return SENSOR_FRONT;
}

void initializeLineSensors()
{
    pinMode(FRONT_LINE_PIN, INPUT);
    pinMode(REAR_LINE_PIN, INPUT);
    pinMode(LEFT_LINE_PIN, INPUT);
    pinMode(RIGHT_LINE_PIN, INPUT);

    // 처음 시작은 경기장 안쪽 흰색이라고 가정
    // 흰색은 0, 라인 감지는 1로 초기 상태를 고정
    unsigned long now = millis();

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        lastRawLineState[i] = LOW;
        stableLineState[i] = false;
        lastLineChangeTime[i] = now;
    }
}

int readLineRaw(int pin)
{
    return digitalRead(pin);
}


bool isLineDetected(int pin)
{
    int sensorIndex = getLineSensorIndex(pin);
    int value = readLineRaw(pin);
    unsigned long now = millis();

    // 입력값이 바뀐 순간부터 유지 시간을 다시 측정
    if (value != lastRawLineState[sensorIndex])
    {
        lastRawLineState[sensorIndex] = value;
        lastLineChangeTime[sensorIndex] = now;
    }

    // 같은 값이 일정 시간 이상 유지되면 실제 상태로 인정
    if (now - lastLineChangeTime[sensorIndex] >= LINE_STABLE_TIME_MS)
    {
        stableLineState[sensorIndex] = (value == LINE_DETECTED_LEVEL);
    }

    return stableLineState[sensorIndex];
}


EscapeAction decideLineEscapeAction()
{
    // 각 방향 라인트레이서가 라인을 감지했는지 확인
    bool frontLine = isLineDetected(FRONT_LINE_PIN);
    bool rearLine  = isLineDetected(REAR_LINE_PIN);
    bool leftLine  = isLineDetected(LEFT_LINE_PIN);
    bool rightLine = isLineDetected(RIGHT_LINE_PIN);


    // ------------------------------------------------------------
    // 아무 라인도 감지되지 않은 경우
    // -> 원래 주행 전략 계속 수행
    // ------------------------------------------------------------
    if (!frontLine && !rearLine && !leftLine && !rightLine)
    {
        return ACTION_NO_LINE;
    }


    // ------------------------------------------------------------
    // 단일 센서만 라인을 감지한 경우
    // ------------------------------------------------------------

    // 앞쪽 라인을 밟음
    // -> 뒤로 탈출
    if (frontLine && !rearLine && !leftLine && !rightLine)
    {
        return ACTION_ESCAPE_BACKWARD;
    }

    // 뒤쪽 라인을 밟음
    // -> 앞으로 탈출
    if (!frontLine && rearLine && !leftLine && !rightLine)
    {
        return ACTION_ESCAPE_FORWARD;
    }

    // 왼쪽 라인을 밟음
    // -> 오른쪽으로 탈출
    if (!frontLine && !rearLine && leftLine && !rightLine)
    {
        return ACTION_ESCAPE_TURN_RIGHT;
    }

    // 오른쪽 라인을 밟음
    // -> 왼쪽으로 탈출
    if (!frontLine && !rearLine && !leftLine && rightLine)
    {
        return ACTION_ESCAPE_TURN_LEFT;
    }


    // ------------------------------------------------------------
    // 코너 방향에서 라인을 감지한 경우
    //
    // 예:
    // frontLine + leftLine
    // -> 앞쪽과 왼쪽이 모두 위험
    // -> 뒤로 가면서 오른쪽으로 빠져야 함
    // ------------------------------------------------------------

    if (frontLine && !rearLine && leftLine && !rightLine)
    {
        return ACTION_ESCAPE_BACKWARD_RIGHT;
    }

    if (frontLine && !rearLine && !leftLine && rightLine)
    {
        return ACTION_ESCAPE_BACKWARD_LEFT;
    }

    if (!frontLine && rearLine && leftLine && !rightLine)
    {
        return ACTION_ESCAPE_FORWARD_RIGHT;
    }

    if (!frontLine && rearLine && !leftLine && rightLine)
    {
        return ACTION_ESCAPE_FORWARD_LEFT;
    }


    // ------------------------------------------------------------
    // 앞쪽 전체가 위험한 경우
    //
    // front + left + right 감지
    // -> 앞쪽 경계선에 걸쳐 있다고 판단
    // -> 단순 후진
    // ------------------------------------------------------------
    if (frontLine && !rearLine && leftLine && rightLine)
    {
        return ACTION_ESCAPE_BACKWARD;
    }


    // ------------------------------------------------------------
    // 뒤쪽 전체가 위험한 경우
    //
    // rear + left + right 감지
    // -> 뒤쪽 경계선에 걸쳐 있다고 판단
    // -> 단순 전진
    // ------------------------------------------------------------
    if (!frontLine && rearLine && leftLine && rightLine)
    {
        return ACTION_ESCAPE_FORWARD;
    }


    // ------------------------------------------------------------
    // 그 외 애매한 경우
    //
    // 예:
    // front + rear 동시 감지
    // left + right 동시 감지
    // 네 방향 전부 감지
    // 일단 비상 모드로 정의해놓음
    // ------------------------------------------------------------
    return ACTION_ESCAPE_EMERGENCY;
}


// ============================================================
// 아래는 디버깅 시리얼 모니터 출력용
// ============================================================

const char* escapeActionToText(EscapeAction action)
{
    switch (action)
    {
        case ACTION_NO_LINE:
            return "NO LINE";

        case ACTION_ESCAPE_BACKWARD:
            return "FRONT LINE - move backward";

        case ACTION_ESCAPE_FORWARD:
            return "REAR LINE - move forward";

        case ACTION_ESCAPE_TURN_RIGHT:
            return "LEFT LINE - turn right";

        case ACTION_ESCAPE_TURN_LEFT:
            return "RIGHT LINE - turn left";

        case ACTION_ESCAPE_BACKWARD_RIGHT:
            return "FRONT + LEFT LINE - move backward right";

        case ACTION_ESCAPE_BACKWARD_LEFT:
            return "FRONT + RIGHT LINE - move backward left";

        case ACTION_ESCAPE_FORWARD_RIGHT:
            return "REAR + LEFT LINE - move forward right";

        case ACTION_ESCAPE_FORWARD_LEFT:
            return "REAR + RIGHT LINE - move forward left";

        case ACTION_ESCAPE_EMERGENCY:
            return "EMERGENCY - unclear line pattern";

        default:
            return "UNKNOWN";
    }
}


void printLineValues()
{
    int frontValue = readLineRaw(FRONT_LINE_PIN);
    int rearValue  = readLineRaw(REAR_LINE_PIN);
    int leftValue  = readLineRaw(LEFT_LINE_PIN);
    int rightValue = readLineRaw(RIGHT_LINE_PIN);

    EscapeAction action = decideLineEscapeAction();

    Serial.print("Front: ");
    Serial.print(frontValue);

    Serial.print(" | Rear: ");
    Serial.print(rearValue);

    Serial.print(" | Left: ");
    Serial.print(leftValue);

    Serial.print(" | Right: ");
    Serial.print(rightValue);

    Serial.print(" | Action: ");
    Serial.println(escapeActionToText(action));
}

// ============================================================
// 아래는 단순 테스트용
// 실제론 메인 loop() 안에서 decideLineEscapeAction()만 호출해서 사용하면 됨.
// ============================================================

void setup()
{
    Serial.begin(115200);

    initializeLineSensors();
}

void loop()
{
    printLineValues();

    delay(200);
}
