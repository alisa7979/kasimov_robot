const int LINE_TRACER_PIN = 2;

int readLineTracer()
{
    return digitalRead(LINE_TRACER_PIN);
}

void printLineTracer()
{
    int value = readLineTracer();

    Serial.print("Line : ");
    Serial.println(value);
}

void setup()
{
    Serial.begin(115200);

    pinMode(LINE_TRACER_PIN, INPUT);
}

void loop()
{
    printLineTracer();
    delay(100);
}