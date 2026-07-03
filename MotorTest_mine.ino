
const int L_A = 10;    // left  motor IN1 (PWM)
const int L_B = 9;   // left  motor IN2 (PWM)
const int R_A = 11;   // right motor IN3 (PWM)
const int R_B = 12;   // right motor IN4 (NOT PWM - on/off only)

// signed per-wheel drive: -255..255
void leftMotor(int v) {
  v = constrain(v, -255, 255);
  if (v >= 0) { analogWrite(L_A, v);  analogWrite(L_B, 0);  }
  else        { analogWrite(L_A, 0);  analogWrite(L_B, -v); }
}
void rightMotor(int v) {
  v = constrain(v, -255, 255);
  if (v >= 0) { analogWrite(R_A, v);  analogWrite(R_B, 0);  }
  else        { analogWrite(R_A, 0);  analogWrite(R_B, -v); }
}
void stopAll() { leftMotor(0); rightMotor(0); }

void phase(const char* name, int left, int right, int ms) {
  Serial.println(name);
  leftMotor(left); rightMotor(right);
  delay(ms);
  stopAll();
  delay(700);
}

void setup() {
  Serial.begin(115200);
  pinMode(L_A, OUTPUT); pinMode(L_B, OUTPUT);
  pinMode(R_A, OUTPUT); pinMode(R_B, OUTPUT);
  stopAll();
  Serial.println("=== Motor test start (wheels off the ground) ===");
  delay(1500);
}

void loop() {
  phase("1) LEFT wheel forward",  180,   0, 1500);
  phase("2) RIGHT wheel forward",   0, 180, 1500);
  phase("3) BOTH forward",        200, 200, 1500);
  phase("4) BOTH backward",      -200,-200, 1500);
  phase("5) SPIN left",          -180, 180, 1200);
  phase("6) SPIN right",          180,-180, 1200);

  Serial.println("7) SPEED ramp (watch when the wheels start turning)");
  for (int s = 60; s <= 255; s += 15) {
    Serial.print("   pwm = "); Serial.println(s);
    leftMotor(s); rightMotor(s);
    delay(400);
  }
  stopAll();

  Serial.println("--- cycle done, repeating in 2 s ---\n");
  delay(2000);
}
