// ==========================================
// 상수 정의 (UPPER_SNAKE_CASE)
// ==========================================
const int MOTOR_LEFT_ENA = 3;       // 왼쪽 모터 PWM
const int MOTOR_LEFT_IN1 = 2;       // 왼쪽 모터 방향 1
const int MOTOR_LEFT_IN2 = 4;       // 왼쪽 모터 방향 2

const int MOTOR_RIGHT_IN3 = 7;      // 오른쪽 모터 방향 1
const int MOTOR_RIGHT_IN4 = 8;      // 오른쪽 모터 방향 2
const int MOTOR_RIGHT_ENB = 5;      // 오른쪽 모터 PWM

const int TEST_SPEED = 150;         // 테스트할 모터 속도 (100 ~ 255)

void setup() {
  pinMode(MOTOR_LEFT_ENA, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);
  pinMode(MOTOR_RIGHT_ENB, OUTPUT);
}

void loop() {
  // 1. 2초간 전진
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_LEFT_ENA, TEST_SPEED);
  
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_RIGHT_ENB, TEST_SPEED);
  delay(2000);

  // 2. 1초간 정지
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  delay(1000);

  // 3. 2초간 후진
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  analogWrite(MOTOR_LEFT_ENA, TEST_SPEED);
  
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, HIGH);
  analogWrite(MOTOR_RIGHT_ENB, TEST_SPEED);
  delay(2000);

  // 4. 1초간 정지
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  delay(1000);
}