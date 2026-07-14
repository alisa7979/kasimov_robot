// ==========================================
// 상수 정의 (UPPER_SNAKE_CASE)
// ==========================================
const int SONIC_LEFT_TRIG = 6;
const int SONIC_LEFT_ECHO = 9;
const int SONIC_RIGHT_TRIG = 10;
const int SONIC_RIGHT_ECHO = 11;

const int FILTER_SIZE = 3;

// ==========================================
// 전역 변수 정의 (camelCase)
// ==========================================
long leftHistory[FILTER_SIZE] = {999, 999, 999};
long rightHistory[FILTER_SIZE] = {999, 999, 999};
int filterIndex = 0;

void setup() {
  Serial.begin(9600);
  pinMode(SONIC_LEFT_TRIG, OUTPUT);
  pinMode(SONIC_LEFT_ECHO, INPUT);
  pinMode(SONIC_RIGHT_TRIG, OUTPUT);
  pinMode(SONIC_RIGHT_ECHO, INPUT);
}

void loop() {
  // 필터링된 거리값 가져오기
  long leftDistance = getFilteredDistance(SONIC_LEFT_TRIG, SONIC_LEFT_ECHO, leftHistory);
  long rightDistance = getFilteredDistance(SONIC_RIGHT_TRIG, SONIC_RIGHT_ECHO, rightHistory);

  // 시리얼 모니터 출력
  Serial.print("Left Distance: ");
  Serial.print(leftDistance);
  Serial.print(" cm | Right Distance: ");
  Serial.print(rightDistance);
  Serial.println(" cm");

  delay(100); // 모니터링을 위한 최소한의 시간 간격
}

long getFilteredDistance(int trigPin, int echoPin, long* history) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 15000); 
  long distance = (duration == 0) ? 999 : duration * 0.034 / 2;

  if (distance <= 0) distance = 999;

  history[filterIndex] = distance;
  // 데이터 갱신 후 인덱스 순환 처리 (0, 1, 2, 0, 1, 2...)
  if (trigPin == SONIC_RIGHT_TRIG) { 
    filterIndex = (filterIndex + 1) % FILTER_SIZE;
  }

  long sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += history[i];
  }
  return sum / FILTER_SIZE;
}