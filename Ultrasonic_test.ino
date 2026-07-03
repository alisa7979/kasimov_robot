const int ULTRASONIC_TRIG_PIN = 9;
const int ULTRASONIC_ECHO_PIN = 10;

long readUltrasonicDistance()
{
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000);

    if (duration == 0)
    {
        return -1;
    }

    long distanceCm = duration * 0.034 / 2;

    return distanceCm;
}

void printUltrasonic()
{
    long distance = readUltrasonicDistance();

    Serial.print("Distance : ");

    if (distance == -1)
    {
        Serial.println("Out of range");
    }
    else
    {
        Serial.print(distance);
        Serial.println(" cm");
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
}

void loop()
{
    printUltrasonic();
    delay(500);
}