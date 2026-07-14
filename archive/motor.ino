// ============================================================
// Motor Control Code
// 역할:
// - 좌/우 바퀴 속도/방향 제어 (전진/후진/회전/곡선)
// - 실제에선 main에서 driveForward() 같은 함수만 호출하면 됨
//
// ============================================================

// ---- pins--------------
const int M_L_A = 5;    // left  motor IN1 (PWM)
const int M_L_B = 6;    // left  motor IN2 (PWM)
const int M_R_A = 9;    // right motor IN3 (PWM)
const int M_R_B = 10;   // right motor IN4 (PWM)

// curve tightness: inner wheel = speed / this. bigger = gentler curve.
const int CURVE_INNER_DIV = 3;

// ---- low level: signed per-wheel drive, -255..255 -----------
void leftMotor(int v)
{
    v = constrain(v, -255, 255);
    if (v >= 0) { analogWrite(M_L_A, v);  analogWrite(M_L_B, 0);  }
    else        { analogWrite(M_L_A, 0);  analogWrite(M_L_B, -v); }
}

void rightMotor(int v)
{
    v = constrain(v, -255, 255);
    if (v >= 0) { analogWrite(M_R_A, v);  analogWrite(M_R_B, 0);  }
    else        { analogWrite(M_R_A, 0);  analogWrite(M_R_B, -v); }
}

void tankDrive(int left, int right)
{
    leftMotor(left);
    rightMotor(right);
}

// ============================================================
// main에서 호출할 이동 명령들
// ============================================================
void motorsInit()
{
    pinMode(M_L_A, OUTPUT);
    pinMode(M_L_B, OUTPUT);
    pinMode(M_R_A, OUTPUT);
    pinMode(M_R_B, OUTPUT);
    tankDrive(0, 0);
}

void motorsStop()              { tankDrive(0, 0); }
void driveForward(int s)       { tankDrive( s,  s); }
void driveBackward(int s)      { tankDrive(-s, -s); }
void turnLeft(int s)           { tankDrive(-s,  s); }   // 제자리 좌회전
void turnRight(int s)          { tankDrive( s, -s); }   // 제자리 우회전
void forwardLeft(int s)        { tankDrive( s / CURVE_INNER_DIV,  s); }
void forwardRight(int s)       { tankDrive( s,  s / CURVE_INNER_DIV); }
void backwardLeft(int s)       { tankDrive(-s / CURVE_INNER_DIV, -s); }
void backwardRight(int s)      { tankDrive(-s, -s / CURVE_INNER_DIV); }

// ============================================================
// 아래는 테스트용 (바퀴 공중에 띄우고 확인).
// ============================================================
void runPhase(const char* name, int left, int right, int ms)
{
    Serial.println(name);
    tankDrive(left, right);
    delay(ms);
    motorsStop();
    delay(600);
}

void setup()
{
    Serial.begin(115200);
    motorsInit();
    Serial.println("=== motor test (wheels off the ground) ===");
    delay(1500);
}

void loop()
{
    runPhase("LEFT forward",   180,   0, 1500);
    runPhase("RIGHT forward",    0, 180, 1500);
    runPhase("BOTH forward",   200, 200, 1500);
    runPhase("BOTH backward", -200,-200, 1500);
    runPhase("SPIN left",     -180, 180, 1200);
    runPhase("SPIN right",     180,-180, 1200);

    Serial.println("SPEED ramp (note pwm where it starts moving)");
    for (int s = 60; s <= 255; s += 15)
    {
        Serial.print("  pwm = "); Serial.println(s);
        tankDrive(s, s);
        delay(400);
    }
    motorsStop();

    Serial.println("--- done, repeat in 2 s ---\n");
    delay(2000);
}
