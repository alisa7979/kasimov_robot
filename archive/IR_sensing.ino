// ============================================================
// Side IR Sensor Control Code
// 담당: 좌/우 측면 HW-201 적외선 센서 감지
//
// 역할:
// - 왼쪽 적외선 센서가 적을 감지했는지 확인
// - 오른쪽 적외선 센서가 적을 감지했는지 확인
// - 적이 어느 측면에 있는지 IrTarget 값으로 반환
// - detectIrTarget() 호출하여 회전 방향을 연결하면 됨
// ============================================================

const int LEFT_IR_PIN  = 2;
const int RIGHT_IR_PIN = 3;

const int IR_DETECTED_VALUE = HIGH; // 감지값이 0인지 1인지

// 같은 값이 이 시간 이상 유지되어야 실제 상태로 인정. 조절 가능
const unsigned long IR_STABLE_TIME_MS = 50;

enum IrSensorIndex
{
    IR_SENSOR_LEFT = 0,
    IR_SENSOR_RIGHT,
    IR_SENSOR_COUNT
};

// 처음 시작은 측면 적 감지가 없다고 가정
// 감지 없음은 0, 적 감지는 1로 초기 상태를 고정
int lastRawIrState[IR_SENSOR_COUNT] = {LOW, LOW};
bool stableIrState[IR_SENSOR_COUNT] = {false, false};
unsigned long lastIrChangeTime[IR_SENSOR_COUNT] = {0, 0};


// ============================================================
// 모터에게 넘겨줄 적외선 감지 결과
// ============================================================

enum IrTarget
{
    IR_TARGET_NONE,       // 측면 적 감지 없음
    IR_TARGET_LEFT,       // 왼쪽에서 적 감지
    IR_TARGET_RIGHT,      // 오른쪽에서 적 감지
    IR_TARGET_BOTH        // 좌우 둘 다 감지
};

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

void initializeIrSensors()
{
    pinMode(LEFT_IR_PIN, INPUT);
    pinMode(RIGHT_IR_PIN, INPUT);

    // 처음 시작은 측면 적 감지가 없다고 가정
    // 감지 없음은 0, 적 감지는 1로 초기 상태를 고정
    unsigned long now = millis();

    for (int i = 0; i < IR_SENSOR_COUNT; i++)
    {
        lastRawIrState[i] = LOW;
        stableIrState[i] = false;
        lastIrChangeTime[i] = now;
    }
}

int readIrRaw(int pin)
{
    return digitalRead(pin);
}

bool isIrDetected(int pin)
{
    int sensorIndex = getIrSensorIndex(pin);
    int value = readIrRaw(pin);
    unsigned long now = millis();

    // 입력값이 바뀐 순간부터 유지 시간을 다시 측정
    if (value != lastRawIrState[sensorIndex])
    {
        lastRawIrState[sensorIndex] = value;
        lastIrChangeTime[sensorIndex] = now;
    }

    // 같은 값이 일정 시간 이상 유지되면 실제 상태로 인정
    if (now - lastIrChangeTime[sensorIndex] >= IR_STABLE_TIME_MS)
    {
        stableIrState[sensorIndex] = (value == IR_DETECTED_VALUE);
    }

    return stableIrState[sensorIndex];
}


// ============================================================
// 좌/우 적외선 센서로 적 방향 판단
// main loop에서 이 함수를 호출하면 된다.
// ============================================================

IrTarget detectIrTarget()
{
    bool leftDetected  = isIrDetected(LEFT_IR_PIN);
    bool rightDetected = isIrDetected(RIGHT_IR_PIN);

    // 아무 측면 적도 감지되지 않음
    if (!leftDetected && !rightDetected)
    {
        return IR_TARGET_NONE;
    }

    // 왼쪽만 감지
    // -> 적이 왼쪽 측면에 있다고 판단
    if (leftDetected && !rightDetected)
    {
        return IR_TARGET_LEFT;
    }

    // 오른쪽만 감지
    // -> 적이 오른쪽 측면에 있다고 판단
    if (!leftDetected && rightDetected)
    {
        return IR_TARGET_RIGHT;
    }

    // 좌우 둘 다 감지
    // -> 적이 가까이 있거나 반사/센서 범위가 겹친 상황
    return IR_TARGET_BOTH;
}


// ============================================================
// 아래는 디버깅용 시리얼 모니터 출력용 
// ============================================================

const char* irTargetToText(IrTarget target)
{
    switch (target)
    {
        case IR_TARGET_NONE:
            return "NO SIDE IR TARGET";

        case IR_TARGET_LEFT:
            return "LEFT IR TARGET - enemy on left side";

        case IR_TARGET_RIGHT:
            return "RIGHT IR TARGET - enemy on right side";

        case IR_TARGET_BOTH:
            return "BOTH SIDE IR TARGETS";

        default:
            return "UNKNOWN";
    }
}

void printIrValues()
{
    int leftValue  = readIrRaw(LEFT_IR_PIN);
    int rightValue = readIrRaw(RIGHT_IR_PIN);

    IrTarget target = detectIrTarget();

    Serial.print("Left IR: ");
    Serial.print(leftValue);

    Serial.print(" | Right IR: ");
    Serial.print(rightValue);

    Serial.print(" | Target: ");
    Serial.println(irTargetToText(target));
}


// ============================================================
// 실제에선 아래가 아닌 main에서 detectIrTarget()만 함수 호출 사용
// ============================================================

void setup()
{
    Serial.begin(115200);

    initializeIrSensors();
}

void loop()
{
    printIrValues();

    delay(200);
}
