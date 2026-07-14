// ============================================================
// Side IR Raw Sensor Code
//
// 역할:
// - 측면 HW-201 적외선 센서 2개의 digital 값을 읽는다.
// - 메인 제어 코드가 이 raw 값을 받아서 사용한다.
//
// 센서:
// HW-201 IR Obstacle Sensor
//
// 연결:
// Left IR OUT  -> D2
// Right IR OUT -> D3
// VCC -> 5V
// GND -> GND
// ============================================================


// ------------------------------
// 적외선 센서 핀 설정
// ------------------------------
const int LEFT_IR_PIN  = 2;
const int RIGHT_IR_PIN = 3;


// ------------------------------
// 적외선 센서 2개 값을 묶어서 저장할 구조체
//
// left, right는 각각 digitalRead() 결과이다.
// 값은 HIGH(1) 또는 LOW(0)이다.
// ------------------------------
struct IrSensorValues
{
    int left;
    int right;
};

int readIrRaw(int pin)
{
    return digitalRead(pin);
}

IrSensorValues readSideIrSensors()
{
    IrSensorValues values;

    values.left  = readIrRaw(LEFT_IR_PIN);
    values.right = readIrRaw(RIGHT_IR_PIN);

    return values;
}


// ------------------------------
// 디버깅용 출력 함수
//
// 센서값이 제대로 들어오는지 시리얼 모니터로 확인하기 위한 함수.
// 최종 통합 코드에서는 없어도 된다.
// ------------------------------
void printIrSensorValues()
{
    IrSensorValues values = readSideIrSensors();

    Serial.print("Left IR: ");
    Serial.print(values.left);

    Serial.print(" | Right IR: ");
    Serial.println(values.right);
}

void setup()
{
    Serial.begin(115200);

    pinMode(LEFT_IR_PIN, INPUT);
    pinMode(RIGHT_IR_PIN, INPUT);
}


// ------------------------------
// 테스트용 loop() 실제론 값만 받아서 메인에서 사용
// ------------------------------
void loop()
{
    printIrSensorValues();

    delay(200);
}