# kasimov_robot

<pin numbers>

- left  DC motor:

- right DC motor:

- left  UC sensor:

- right UC sensor:

- left  IR sensor:

- right IR sensor:

- left  front Line sensor:

- right front Line sensor:

- left  back  Line sensor:

- right back  Line sensor:

//sensing_simple.ino 같은 경우 각 센서의 측정값 raw만을 받아내는 코드로, 베이스코드일 뿐 활용 X//

//Linetracer_sensing.ino, IR_sensing.ino는 각 센서의 측정값을 받아서 milis 사용하여 상태 유지를 측정하고, 이를 바탕으로 case를 정의함. 실제 main루프에서 활용//

<IR_sensing.ino>
좌/우 측면 HW-201 적외선 센서 감지
적이 왼쪽, 오른쪽, 양쪽, 또는 없음 중 어디에 있는지 판단

<Linetracer_sensing.ino>
전/후/좌/우 라인트레이서 센서 감지
경기장 라인을 밟았을 때 어느 방향으로 탈출해야 하는지 판단
Main Code에서 호출해야 하는 함수

1. 초기 설정

메인 코드의 setup()에서 센서 초기화 함수를 한 번씩 호출해야 한다.

initializeIrSensors();
initializeLineSensors();

예시:

void setup()
{
    Serial.begin(115200);

    initializeIrSensors();
    initializeLineSensors();

    // motor init 등 다른 초기화 코드
}
2. 측면 IR 센서 사용

메인 코드에서는 detectIrTarget() 함수를 호출해서 적외선 센서 판단 결과를 받으면 된다.

IrTarget irTarget = detectIrTarget();

반환되는 값은 다음과 같다.

IR_TARGET_NONE   // 측면 적 감지 없음
IR_TARGET_LEFT   // 왼쪽에서 적 감지
IR_TARGET_RIGHT  // 오른쪽에서 적 감지
IR_TARGET_BOTH   // 좌우 둘 다 감지

예시:

IrTarget irTarget = detectIrTarget();

if (irTarget == IR_TARGET_LEFT)
{
    // 왼쪽으로 회전 또는 왼쪽 적 추적
}
else if (irTarget == IR_TARGET_RIGHT)
{
    // 오른쪽으로 회전 또는 오른쪽 적 추적
}
else if (irTarget == IR_TARGET_BOTH)
{
    // 양쪽 감지 상황 처리
}
else
{
    // 측면 적 없음
}
3. 라인트레이서 센서 사용

메인 코드에서는 decideLineEscapeAction() 함수를 호출해서 라인 감지 시 필요한 탈출 동작을 받으면 된다.

EscapeAction lineAction = decideLineEscapeAction();

반환되는 값은 다음과 같다.

ACTION_NO_LINE                  // 라인 감지 없음

ACTION_ESCAPE_BACKWARD          // 앞쪽 라인 감지 -> 뒤로 탈출

ACTION_ESCAPE_FORWARD           // 뒤쪽 라인 감지 -> 앞으로 탈출

ACTION_ESCAPE_TURN_RIGHT        // 왼쪽 라인 감지 -> 오른쪽으로 탈출

ACTION_ESCAPE_TURN_LEFT         // 오른쪽 라인 감지 -> 왼쪽으로 탈출

ACTION_ESCAPE_BACKWARD_RIGHT    // 앞 + 왼쪽 라인 감지 -> 뒤 + 오른쪽 탈출

ACTION_ESCAPE_BACKWARD_LEFT     // 앞 + 오른쪽 라인 감지 -> 뒤 + 왼쪽 탈출

ACTION_ESCAPE_FORWARD_RIGHT     // 뒤 + 왼쪽 라인 감지 -> 앞 + 오른쪽 탈출

ACTION_ESCAPE_FORWARD_LEFT      // 뒤 + 오른쪽 라인 감지 -> 앞 + 왼쪽 탈출

ACTION_ESCAPE_EMERGENCY         // 판단이 애매한 경우

예시:

EscapeAction lineAction = decideLineEscapeAction();

if (lineAction == ACTION_ESCAPE_BACKWARD)
{
    // 후진
}
else if (lineAction == ACTION_ESCAPE_FORWARD)
{
    // 전진
}
else if (lineAction == ACTION_ESCAPE_TURN_RIGHT)
{
    // 오른쪽 회전
}
else if (lineAction == ACTION_ESCAPE_TURN_LEFT)
{
    // 왼쪽 회전
}
else if (lineAction == ACTION_NO_LINE)
{
    // 기존 주행 전략 유지
}
else
{
    // 복합 탈출 또는 비상 처리
}
//



4. 모터 사용

메인 코드의 setup()에서 모터 초기화 함수를 한 번 호출해야 한다.

    motorsInit();

예시:

    void setup() {
        Serial.begin(115200);

        initializeIrSensors();
        initializeLineSensors();
        initializeUltrasonicSensors();
        motorsInit();
    }

메인 코드에서는 판단 결과(IR / 라인 / 초음파)에 따라 아래 이동 함수를 호출하면 된다.
속도(speed)는 0~255 사이의 PWM 값이다.

    motorsStop();            // 정지
    driveForward(speed);     // 직진 전진
    driveBackward(speed);    // 직진 후진
    turnLeft(speed);         // 제자리 좌회전 (반시계)
    turnRight(speed);        // 제자리 우회전 (시계)
    forwardLeft(speed);      // 전진하면서 왼쪽으로 곡선
    forwardRight(speed);     // 전진하면서 오른쪽으로 곡선
    backwardLeft(speed);     // 후진하면서 왼쪽으로 곡선
    backwardRight(speed);    // 후진하면서 오른쪽으로 곡선
    tankDrive(left, right);  // 좌/우 바퀴 개별 제어 (-255~255), 저수준

예시 (라인 탈출 동작을 모터로 연결):

    EscapeAction lineAction = decideLineEscapeAction();

    if (lineAction == ACTION_ESCAPE_BACKWARD) {
        driveBackward(220);
    } else if (lineAction == ACTION_ESCAPE_FORWARD) {
        driveForward(220);
    } else if (lineAction == ACTION_ESCAPE_TURN_RIGHT) {
        turnRight(170);
    } else if (lineAction == ACTION_ESCAPE_TURN_LEFT) {
        turnLeft(170);
    } else if (lineAction == ACTION_ESCAPE_BACKWARD_RIGHT) {
        backwardRight(220);
    } else if (lineAction == ACTION_ESCAPE_BACKWARD_LEFT) {
        backwardLeft(220);
    } else if (lineAction == ACTION_ESCAPE_FORWARD_RIGHT) {
        forwardRight(220);
    } else if (lineAction == ACTION_ESCAPE_FORWARD_LEFT) {
        forwardLeft(220);
    } else if (lineAction == ACTION_NO_LINE) {
        // 기존 주행 전략 유지
    } else {
        turnRight(220); // ACTION_ESCAPE_EMERGENCY - 비상 회전
    }

예시 (측면 IR로 적 방향 추적):

    IrTarget irTarget = detectIrTarget();

    if (irTarget == IR_TARGET_LEFT)  turnLeft(170);
    else if (irTarget == IR_TARGET_RIGHT) turnRight(170);
    else if (irTarget == IR_TARGET_BOTH)  driveForward(180);
