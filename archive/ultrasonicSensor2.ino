// ============================================================
// Front Ultrasonic Sensor Control Code
// 담당: 로봇 정면 초음파 센서 2개(좌/우) - 상대 로봇 위치 감지
//
// 역할:
// - 좌/우 초음파 센서로 거리 측정
// - 상대 로봇이 정면 감지 범위 안에 있는지, 어느 쪽에 더 가까운지 판단
// - detectFrontTarget() 호출하여 추적 동작과 연결하면 됨
// ============================================================

const int LEFT_TRIG  = 10;
const int LEFT_ECHO  = 9;
const int RIGHT_TRIG = 12;
const int RIGHT_ECHO = 11;

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
        return -1;
    }

    return duration * 0.0343 / 2.0;
}

void printSensorValues()
{
    // int Distance  = measureDistance(LEFT_TRIG, LEFT_ECHO);
    int Distance = measureDistance(RIGHT_TRIG, RIGHT_ECHO);

    Serial.println(Distance);
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
