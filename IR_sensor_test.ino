const int IR_SENSOR_PIN = 10;

int readIrSensor()
{
    return digitalRead(IR_SENSOR_PIN);
}

void printIrSensor()
{
    Serial.print("IR : ");
    Serial.println(readIrSensor());
}

void setup()
{
    Serial.begin(115200);

    pinMode(IR_SENSOR_PIN, INPUT);
}

void loop()
{
    printIrSensor();
    delay(100);
}