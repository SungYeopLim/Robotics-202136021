/*
 * LiDAR 장애물 회피 주행 (ESP32 단독)
 *
 * 강사 예제 알고리즘(algorithm_template.tex)을 기준으로 다음 네 가지를 고쳤다.
 *   1. 매 주기마다 라이다 거리를 새로 읽는다.
 *   2. 한 점이 아니라 각도 구간의 최솟값으로 앞/왼쪽/오른쪽 거리를 구한다 (0은 측정 실패로 무시).
 *   3. 장애물을 만나면 먼저 멈추고(STOP), 그 다음 회전 방향을 정한다.
 *   4. 한 번 정한 회전 방향은 앞이 CLEAR_DISTANCE 이상 열릴 때까지 바꾸지 않는다 (좌우 떨림 방지).
 *
 * LiDAR 각도 기준 (lidar_test.ino와 동일)
 *   Back : 0°, Left : 90°, Front : 180°, Right : 270°
 */

// ====================== 현장에서 조절할 값 ======================

#define STOP_DISTANCE      250   // 이 거리(mm)보다 앞이 가까우면 멈춘다
#define CLEAR_DISTANCE     400   // 회전 중 앞이 이 거리(mm)보다 멀어지면 다시 직진한다 (STOP_DISTANCE보다 커야 함)

#define BASE_SPEED  180   // 직진 속도 (0~255)
#define TURN_SPEED  160   // 제자리 회전 속도 (0~255)
#define T_STOP      300   // 멈춘 뒤 방향을 정하기 전까지 기다리는 시간 (ms)

// 각도 구간 (도). 라이다 테스트 결과가 다르면 여기만 고친다.
#define FRONT_MIN   150
#define FRONT_MAX   210
#define LEFT_MIN     60
#define LEFT_MAX    120
#define RIGHT_MIN   240
#define RIGHT_MAX   300

// motor_test에서 "정회전"일 때 로봇이 뒤로 가는 바퀴는 true로 바꾼다.
#define FLIP_FR false
#define FLIP_FL false
#define FLIP_RR false
#define FLIP_RL false

// ====================== 핀 설정 ======================

#define LIDAR_RX 16

#define FR_IN1 26
#define FR_IN2 25
#define FL_IN1 19
#define FL_IN2 27
#define RR_IN1 23
#define RR_IN2 12
#define RL_IN1 0
#define RL_IN2 15

#define PWM_FREQ 20000   // 모터 PWM 주파수 (Hz)
#define PWM_RES  8       // 8bit: 0~255

#define DECISION_MS 50   // 판단 주기 (ms)
#define LOG_MS      200  // 시리얼 로그 출력 주기 (ms)
#define NO_DATA     9999 // 구간에 유효한 값이 없을 때 쓰는 거리 (열려 있다고 본다)

// ====================== 상태 ======================

enum State { FORWARD, STOP, TURN };
enum Dir   { DIR_NONE, DIR_LEFT, DIR_RIGHT };

State state = STOP;
Dir   dir   = DIR_NONE;
unsigned long stopStart = 0;

uint8_t  packet[47];
uint16_t distances[360] = {0};
float    previousAngle = -1.0;
bool     lidarReady = false;   // 라이다가 한 바퀴를 다 돌기 전에는 출발하지 않는다

uint16_t dF = NO_DATA, dL = NO_DATA, dR = NO_DATA;
int      vL = 0, vR = 0;

// ====================== LiDAR ======================

uint16_t readUInt16(uint8_t low, uint8_t high) {
    return low | (high << 8);
}

// lidar_test.ino의 패킷 해석 부분. 패킷 하나를 읽어 distances[]를 갱신한다.
void readLidarPacket() {
    if (!Serial2.available()) return;
    if (Serial2.read() != 0x54) return;

    while (!Serial2.available()) {}
    if (Serial2.read() != 0x2C) return;

    packet[0] = 0x54;
    packet[1] = 0x2C;
    for (int i = 2; i < 47; i++) {
        while (!Serial2.available()) {}
        packet[i] = Serial2.read();
    }

    float startAngle = readUInt16(packet[4], packet[5]) / 100.0;
    float endAngle   = readUInt16(packet[42], packet[43]) / 100.0;
    if (endAngle < startAngle) endAngle += 360.0;
    float angleStep = (endAngle - startAngle) / 11.0;

    for (int i = 0; i < 12; i++) {
        int index = 6 + i * 3;
        uint16_t distance = readUInt16(packet[index], packet[index + 1]);

        float angle = startAngle + angleStep * i;
        if (angle >= 360.0) angle -= 360.0;

        // 360°에서 0°로 넘어가면 한 바퀴를 다 돈 것
        if (previousAngle > 300.0 && angle < 60.0) lidarReady = true;

        int degree = (int)(angle + 0.5);
        if (degree >= 360) degree = 0;
        if (distance > 0) distances[degree] = distance;

        previousAngle = angle;
    }
}

// a~b도 구간에서 0(측정 실패)을 뺀 가장 가까운 거리
uint16_t sectorMin(int a, int b) {
    uint16_t m = NO_DATA;
    for (int deg = a; deg <= b; deg++) {
        if (distances[deg] > 0 && distances[deg] < m) m = distances[deg];
    }
    return m;
}

// ====================== 모터 ======================

// speed: -255 ~ 255 (양수 = 로봇 기준 앞으로)
void driveMotor(int in1, int in2, int speed, bool flip) {
    if (flip) speed = -speed;
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        ledcWrite(in1, speed);
        ledcWrite(in2, 0);
    } else if (speed < 0) {
        ledcWrite(in1, 0);
        ledcWrite(in2, -speed);
    } else {
        ledcWrite(in1, 0);
        ledcWrite(in2, 0);
    }
}

// 왼쪽 바퀴(FL, RL)와 오른쪽 바퀴(FR, RR)를 묶어서 제어한다
void setVelocity(int left, int right) {
    vL = left;
    vR = right;
    driveMotor(FL_IN1, FL_IN2, left,  FLIP_FL);
    driveMotor(RL_IN1, RL_IN2, left,  FLIP_RL);
    driveMotor(FR_IN1, FR_IN2, right, FLIP_FR);
    driveMotor(RR_IN1, RR_IN2, right, FLIP_RR);
}

void turnVelocity(Dir d) {
    if (d == DIR_LEFT) setVelocity(-TURN_SPEED,  TURN_SPEED);   // 제자리 좌회전
    else           setVelocity( TURN_SPEED, -TURN_SPEED);   // 제자리 우회전
}

// ====================== 알고리즘 ======================

void decide() {
    // 1, 2: 매 주기마다 구간 최솟값으로 거리를 새로 구한다
    dF = sectorMin(FRONT_MIN, FRONT_MAX);
    dL = sectorMin(LEFT_MIN,  LEFT_MAX);
    dR = sectorMin(RIGHT_MIN, RIGHT_MAX);

    if (state == TURN) {
        // 4: 앞이 충분히 열릴 때까지 정한 방향으로 계속 돈다
        if (dF > CLEAR_DISTANCE) {
            state = FORWARD;
            dir = DIR_NONE;
            setVelocity(BASE_SPEED, BASE_SPEED);
        } else {
            turnVelocity(dir);
        }
    } else if (state == STOP) {
        // 3: 멈춘 상태로 T_STOP 동안 기다린 뒤 더 넓은 쪽을 고른다
        setVelocity(0, 0);
        if (millis() - stopStart >= T_STOP) {
            if (dF > STOP_DISTANCE) {
                state = FORWARD;                 // 기다리는 동안 앞이 열렸으면 그대로 직진
            } else {
                dir = (dL > dR) ? DIR_LEFT : DIR_RIGHT;
                state = TURN;
                turnVelocity(dir);
            }
        }
    } else {  // FORWARD
        if (dF < STOP_DISTANCE) {
            state = STOP;                        // 3: 장애물 발견 → 먼저 정지
            stopStart = millis();
            setVelocity(0, 0);
        } else {
            setVelocity(BASE_SPEED, BASE_SPEED);
        }
    }
}

const char* stateName() {
    if (state == FORWARD) return "FORWARD";
    if (state == STOP)    return "STOP";
    return (dir == DIR_LEFT) ? "TURN_LEFT" : "TURN_RIGHT";
}

void printLog() {
    Serial.print(millis());
    Serial.print(" ms | dF: ");  Serial.print(dF);
    Serial.print(" dL: ");       Serial.print(dL);
    Serial.print(" dR: ");       Serial.print(dR);
    Serial.print(" | ");         Serial.print(stateName());
    Serial.print(" | vL: ");     Serial.print(vL);
    Serial.print(" vR: ");       Serial.println(vR);
}

// ====================== setup / loop ======================

void setup() {
    Serial.begin(115200);
    Serial2.begin(230400, SERIAL_8N1, LIDAR_RX, -1);

    int motorPins[8] = { FR_IN1, FR_IN2, FL_IN1, FL_IN2, RR_IN1, RR_IN2, RL_IN1, RL_IN2 };
    for (int i = 0; i < 8; i++) {
        ledcAttach(motorPins[i], PWM_FREQ, PWM_RES);
        ledcWrite(motorPins[i], 0);
    }

    stopStart = millis();
    Serial.println("obstacle avoidance start");
}

void loop() {
    static unsigned long lastDecision = 0;
    static unsigned long lastLog = 0;

    // 들어온 라이다 데이터를 모두 처리한다 (delay를 쓰지 않아 라이다를 계속 읽는다)
    while (Serial2.available() >= 47) {
        readLidarPacket();
    }

    if (!lidarReady) {
        setVelocity(0, 0);                       // 라이다가 한 바퀴 돌기 전에는 대기
        return;
    }

    if (millis() - lastDecision >= DECISION_MS) {
        lastDecision = millis();
        decide();
    }

    if (millis() - lastLog >= LOG_MS) {
        lastLog = millis();
        printLog();
    }
}
