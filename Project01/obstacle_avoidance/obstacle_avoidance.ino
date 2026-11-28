/*
 * LiDAR 장애물 회피 주행
 *
 * LiDAR 각도 기준 (lidar_test.ino와 동일)
 *   Back : 0°, Left : 90°, Front : 180°, Right : 270°
 */

// ====================== 조절할 값 ======================

#define STOP_DISTANCE   250   // 앞 거리가 이보다 가까우면 멈춘다 (mm)
#define CLEAR_DISTANCE  400   // 회전 중 앞 거리가 이보다 멀어지면 직진한다 (mm)
#define BASE_SPEED      180   // 직진 속도 (PWM 0~255)
#define TURN_SPEED      160   // 회전 속도 (PWM 0~255)
#define T_STOP          300   // 멈춘 뒤 기다리는 시간 (ms)

// ====================== 핀 (motor_test.ino와 동일) ======================

#define LIDAR_RX 16

#define FR_IN1 26
#define FR_IN2 25
#define FL_IN1 19
#define FL_IN2 27
#define RR_IN1 23
#define RR_IN2 12
#define RL_IN1 0
#define RL_IN2 15

// ====================== 변수 ======================

bool turning  = false;   // false: 직진 상태, true: 회전 상태
bool turnLeft = true;    // 회전 방향 (true: 왼쪽, false: 오른쪽)

uint8_t  packet[47];
uint16_t distances[360] = {0};

// ====================== LiDAR (lidar_test.ino와 동일) ======================

uint16_t readUInt16(uint8_t low, uint8_t high) {
    return low | (high << 8);
}

void readLidar() {
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
    float endAngle = readUInt16(packet[42], packet[43]) / 100.0;
    if (endAngle < startAngle) endAngle += 360.0;
    float angleStep = (endAngle - startAngle) / 11.0;

    for (int i = 0; i < 12; i++) {
        int index = 6 + i * 3;
        uint16_t distance = readUInt16(packet[index], packet[index + 1]);

        float angle = startAngle + angleStep * i;
        if (angle >= 360.0) angle -= 360.0;

        int degree = (int)(angle + 0.5);
        if (degree >= 360) degree = 0;

        if (distance > 0) distances[degree] = distance;
    }
}

// a~b도 구간에서 0을 뺀 가장 가까운 거리
uint16_t sectorMin(int a, int b) {
    uint16_t minDist = 9999;
    for (int deg = a; deg <= b; deg++) {
        if (distances[deg] > 0 && distances[deg] < minDist) {
            minDist = distances[deg];
        }
    }
    return minDist;
}

// ====================== 모터 ======================

// speed > 0: 정회전, speed < 0: 역회전, 0: 정지
void motor(int in1, int in2, int speed) {
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

// 왼쪽 바퀴(FL, RL)와 오른쪽 바퀴(FR, RR)
void setVelocity(int vL, int vR) {
    motor(FL_IN1, FL_IN2, vL);
    motor(RL_IN1, RL_IN2, vL);
    motor(FR_IN1, FR_IN2, vR);
    motor(RR_IN1, RR_IN2, vR);
}

// ====================== setup / loop ======================

void setup() {
    Serial.begin(115200);
    Serial2.begin(230400, SERIAL_8N1, LIDAR_RX, -1);

    int pins[8] = {FR_IN1, FR_IN2, FL_IN1, FL_IN2, RR_IN1, RR_IN2, RL_IN1, RL_IN2};
    for (int i = 0; i < 8; i++) {
        ledcAttach(pins[i], 1000, 8);   // 1000Hz, 8bit (0~255)
        ledcWrite(pins[i], 0);
    }
}

void loop() {
    // 들어온 LiDAR 데이터 읽기
    while (Serial2.available()) {
        readLidar();
    }

    // 앞, 왼쪽, 오른쪽 거리
    uint16_t dF = sectorMin(150, 210);
    uint16_t dL = sectorMin(60, 120);
    uint16_t dR = sectorMin(240, 300);

    if (turning) {
        // 앞이 충분히 열릴 때까지 정한 방향으로 계속 회전
        if (dF > CLEAR_DISTANCE) {
            turning = false;
            Serial.print("FORWARD  dF: ");
            Serial.println(dF);
        }
    } else if (dF < STOP_DISTANCE) {
        // 장애물 발견: 멈춘 뒤 더 넓은 쪽을 고른다
        setVelocity(0, 0);
        delay(T_STOP);

        turnLeft = (dL > dR);
        turning = true;

        Serial.print("STOP  dF: ");
        Serial.print(dF);
        Serial.print("  dL: ");
        Serial.print(dL);
        Serial.print("  dR: ");
        Serial.print(dR);
        Serial.println(turnLeft ? "  -> TURN LEFT" : "  -> TURN RIGHT");
    }

    if (!turning) {
        setVelocity(BASE_SPEED, BASE_SPEED);
    } else if (turnLeft) {
        setVelocity(-TURN_SPEED, TURN_SPEED);
    } else {
        setVelocity(TURN_SPEED, -TURN_SPEED);
    }
}
