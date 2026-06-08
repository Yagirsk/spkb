#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int
ENA = 3,
IN1 = 4,
IN2 = 6,
ENB = 5,
IN3 = 7,
IN4 = 8,
ENC = A0;

const float THRESHOLD = 4.85;
bool lastEncState = false;
unsigned long lastTriggerMs = 0;
const unsigned long MIN_INTERVAL_MS = 20;

const float WHEEL_DIAMETER_MM = 80.0;
const float WHEEL_CIRC_MM = WHEEL_DIAMETER_MM * 3.14;
const int   STRIPES_PER_REV = 14;
const float MM_PER_PULSE = WHEEL_CIRC_MM / STRIPES_PER_REV; // ~17.94 мм

const float SIDE_MM = 3000.0;
const int   TOTAL_SIDES = 4;
const float TURN_ANGLE = 90.0;

Adafruit_MPU6050 mpu;
float yaw = 0.0;
float turnStartYaw = 0.0;
float gz_offset = 0.0;
unsigned long gz_lastTime = 0;

const float GZ_FILTER_ALPHA = 0.15;
float gz_filtered = 0.0;

float Kp = 5.0, Kd = 3.0, prevError = 0.0;
const int MAX_STEER = 255, MIN_STEER = 0, DEADBAND = 1;

enum Phase { DRIVING, TURNING, DONE };
Phase phase = DRIVING;

enum Steer { RIGHT, LEFT, STRAIGHT };

int   sidesDone = 0;
float sideProgress = 0.0;
bool  brakingActive = false;
float targetYaw = 0.0;

void driveForward(int spd = 255) {
    analogWrite(ENB, spd);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void brakeMotor() {
    analogWrite(ENB, 255);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
}

void stopMotors() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void steering(Steer steer, int pw = 255) {
    switch (steer) {
    case STRAIGHT:
        digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
        analogWrite(ENA, 0);
        break;
    case RIGHT:
        analogWrite(ENA, pw);
        digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
        break;
    case LEFT:
        analogWrite(ENA, pw);
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
        break;
    }
}

void pdSteering() {
    float error = targetYaw - yaw;
    float derivative = error - prevError;
    prevError = error;
    float pdOut = constrain(Kp * error + Kd * derivative, -MAX_STEER, MAX_STEER);
    int pwm = abs((int)pdOut);
    if (pwm < DEADBAND) {
        steering(STRAIGHT, 0);
    }
    else if (pdOut >= 0) {
        steering(RIGHT, constrain(pwm, MIN_STEER, MAX_STEER));
    }
    else {
        steering(LEFT, constrain(pwm, MIN_STEER, MAX_STEER));
    }
}

void updateYaw() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long now = micros();
    float dt = (now - gz_lastTime) / 1000000.0;
    gz_lastTime = now;
    float gz_raw = g.gyro.z - gz_offset;
    gz_filtered = GZ_FILTER_ALPHA * gz_raw + (1.0 - GZ_FILTER_ALPHA) * gz_filtered;
    float gz_use = (fabs(gz_filtered) > 0.003) ? gz_filtered : 0.0;
    if (dt > 0.001 && dt < 0.1)
        yaw += gz_use * (180.0 / M_PI) * dt;
}

void waitYawStable() {
    float prev = yaw;
    int   count = 0;
    for (int i = 0; i < 40; i++) {
        delay(15);
        updateYaw();
        if (fabs(yaw - prev) < 0.10) {
            if (++count >= 3) break;
        }
        else {
            count = 0;
        }
        prev = yaw;
    }
}

void resetSideProgress() {
    noInterrupts(); sideProgress = 0.0; interrupts();
}

void startNextSide() {
    prevError = 0.0;
    resetSideProgress();
    brakingActive = false;
    phase = DRIVING;

    Serial.print("=== Сторона "); Serial.print(sidesDone + 1);
    Serial.print("  курс="); Serial.println(targetYaw, 1);

    driveForward();
}

void startArcTurn() {
    turnStartYaw = targetYaw;
    targetYaw = targetYaw - TURN_ANGLE;

    Serial.print("--- Дуга поворот #"); Serial.print(sidesDone);
    Serial.print("  цель="); Serial.println(targetYaw, 1);

    phase = TURNING;

    steering(RIGHT);
    driveForward();
}

void setup() {
    Serial.begin(9600);
    delay(500);

    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    stopMotors();

    resetSideProgress();

    if (!mpu.begin()) { Serial.println("MPU6050 не найден!"); while (1); }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("Калибровка (~2.5 сек, не двигать)...");
    float sumZ = 0.0;
    for (int i = 0; i < 500; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sumZ += g.gyro.z;
        delay(5);
    }
    gz_offset = sumZ / 500.0;
    gz_filtered = 0.0;
    gz_lastTime = micros();
    yaw = 0.0;

    targetYaw = 0.0;
    phase = DRIVING;
    Serial.println("=== Сторона 1  курс=0.0");
    driveForward();
}

void loop() {
    updateYaw();

    float voltage = analogRead(ENC) * (5.0 / 1023.0);
    bool encState = voltage > THRESHOLD;

    if (encState && !lastEncState) {
        unsigned long now = millis();
        if (now - lastTriggerMs >= MIN_INTERVAL_MS) {
            lastTriggerMs = now;
            sideProgress += MM_PER_PULSE * cos((yaw - targetYaw) * M_PI / 180.0);
        }
    }

    lastEncState = encState;

    switch (phase) {
    case DRIVING: {
        pdSteering();
        if (sideProgress >= SIDE_MM) {
            stopMotors();
            Serial.print("Начало дуги, прогресс стороны ="); Serial.print(sideProgress);
            Serial.print("  Курс="); Serial.print(yaw, 1);
            delay(150);
            startArcTurn();
        }
        break;
    }
    case TURNING: {
        float turnedSoFar = fabs(yaw - turnStartYaw);
        if (turnedSoFar >= TURN_ANGLE) {
            stopMotors();
            steering(STRAIGHT);

            Serial.print("Торможение дуги: повернули="); Serial.print(turnedSoFar, 1);
            Serial.print("  Курс="); Serial.println(yaw, 1);

            waitYawStable();

            sidesDone++;
            Serial.print("=== ПОВОРОТ #"); Serial.print(sidesDone);
            Serial.print(" завершён. Курс="); Serial.print(yaw, 1);
            Serial.print("  цель="); Serial.print(targetYaw, 1);
            Serial.print("  финал погрешность="); Serial.println(yaw - targetYaw, 1);

            if (sidesDone >= TOTAL_SIDES) {
                stopMotors();
                phase = DONE;
                Serial.println("=== КВАДРАТ ЗАВЕРШЁН! ===");
            }
            else {
                delay(300);
                startNextSide();
            }
            break;
        }

        // Диагностика каждые 200мс
        static unsigned long lastArcLog = 0;
        if (millis() - lastArcLog > 200) {
            Serial.print("  Дуга: повернули="); Serial.print(turnedSoFar, 1);
            Serial.print("/"); Serial.print(TURN_ANGLE, 1);
            Serial.print("  курс="); Serial.print(yaw, 1);
            lastArcLog = millis();
        }
        break;
    }

    case DONE:
        break;
    }

    // ── Лог каждые 150 мс ──
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 150) {
        if (phase == DRIVING) {
            Serial.print("Прямая: прогресс="); Serial.print(sideProgress, 1);
            Serial.print("/");  Serial.println(SIDE_MM, 1);
            Serial.print("  курс="); Serial.print(yaw, 1);
        }
        lastPrint = millis();
    }

    delay(8);
}