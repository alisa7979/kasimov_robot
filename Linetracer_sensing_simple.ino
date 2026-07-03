// ============================================================
// Line Tracer Sensor Code simple
//
// 역할:
// - 라인트레이서 4개의 아날로그 값을 읽는다.
// - 메인 제어 코드가 이 raw 값을 받아서 사용한다.
//
// 센서:
// HW-870 Line Tracer
//
// 연결:
// Front AO -> A0
// Rear  AO -> A1
// Left  AO -> A2
// Right AO -> A3
// VCC -> 5V
// GND -> GND
// DO는 현재 사용하지 않음
// ============================================================

const int FRONT_LINE_PIN = A0;
const int REAR_LINE_PIN  = A1;
const int LEFT_LINE_PIN  = A2;
const int RIGHT_LINE_PIN = A3;


// ------------------------------
// 라인트레이서 4개 값을 묶어서 저장할 구조체
//
// 이 구조체는 센서값을 한 번에 넘기기 위해 사용한다.
// front, rear, left, right는 각각 0~1023 사이의 analog 값이다.
// ------------------------------
struct LineSensorValues
{
    int front;
    int rear;
    int left;
    int right;
};

int readLineRaw(int pin)
{
    return analogRead(pin);
}

LineSensorValues readLineSensors()
{
    LineSensorValues values;

    values.front = readLineRaw(FRONT_LINE_PIN);
    values.rear  = readLineRaw(REAR_LINE_PIN);
    values.left  = readLineRaw(LEFT_LINE_PIN);
    values.right = readLineRaw(RIGHT_LINE_PIN);

    return values;
}


// ------------------------------
// 디버깅용 출력 함수
// ------------------------------
void printLineSensorValues()
{
    LineSensorValues values = readLineSensors();

    Serial.print("Line Front: ");
    Serial.print(values.front);

    Serial.print(" | Rear: ");
    Serial.print(values.rear);

    Serial.print(" | Left: ");
    Serial.print(values.left);

    Serial.print(" | Right: ");
    Serial.println(values.right);
}

void setup()
{
    Serial.begin(115200);
}


// ------------------------------
// loop()
//
// 테스트용. 실제론 이것 안쓰고 main 루프에서 값만 받아서 씀
// ------------------------------
void loop()
{
    printLineSensorValues();

    delay(200);
}