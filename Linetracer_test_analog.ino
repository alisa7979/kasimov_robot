const int LINE_TRACER_PIN = A2;

int readLineTracer()
{
    return analogRead(LINE_TRACER_PIN);
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
}

void loop()
{
    printLineTracer();
    delay(100);
}