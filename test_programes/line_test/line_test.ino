// ==========================================
// 상수 정의 (UPPER_SNAKE_CASE)
// ==========================================
const int LINE_FRONT_LEFT = A0;
const int LINE_FRONT_RIGHT = A1;
const int LINE_REAR_LEFT = A2;
const int LINE_REAR_RIGHT = A3;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // 4개 센서의 아날로그 raw 데이터 읽기 (0 ~ 1023)
  int flValue = analogRead(LINE_FRONT_LEFT);
  int frValue = analogRead(LINE_FRONT_RIGHT);
  int rlValue = analogRead(LINE_REAR_LEFT);
  int rrValue = analogRead(LINE_REAR_RIGHT);

  // 시리얼 모니터에 가독성 좋게 출력
  Serial.print("FL: "); Serial.print(flValue);
  Serial.print(" | FR: "); Serial.print(frValue);
  Serial.print(" | RL: "); Serial.print(rlValue);
  Serial.print(" | RR: "); Serial.println(rrValue);

  delay(200); // 눈으로 읽기 편하도록 0.2초마다 출력
}