// Машинка едет по квадрату 3х3 метра
// Поворот по ДУГЕ радиусом 55 см, контроль угла по гироскопу
// Поворот начинается за 55 см до угла квадрата
// Плата: ARDUINO NANO

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ──────────── ПИНЫ ────────────
const int
ENA = 3,
IN1 = 4,
IN2 = 6,
ENB = 5,
IN3 = 7,
IN4 = 8,
ENC = 2;

// ──────────── ЭНКОДЕР ────────────
const float WHEEL_DIAMETER_MM = 80.0;
const float WHEEL_CIRC_MM = WHEEL_DIAMETER_MM * 3.14;
const int   STRIPES_PER_REV = 14;
const float MM_PER_PULSE = WHEEL_CIRC_MM / STRIPES_PER_REV; // ~17.94 мм

// ──────────────────────────────────────────────────────────────
// ГЕОМЕТРИЯ КВАДРАТА
// ──────────────────────────────────────────────────────────────
// Сторона квадрата (расстояние от угла до угла)
const float SIDE_MM = 3000.0;

// Радиус поворота 55 см.
// Машина начинает поворачивать за TURN_RADIUS_MM до угла,
// чтобы выйти точно на следующую сторону.
const float TURN_RADIUS_MM = 550.0;

// Прямой участок до начала поворота:
//   SIDE_MM - TURN_RADIUS_MM = 3000 - 550 = 2450 мм
const float STRAIGHT_MM = SIDE_MM - TURN_RADIUS_MM; // 2450 мм

// Тормозной путь прямой (250 мм до конца прямого участка)
const float STRAIGHT_BRAKE_MM = STRAIGHT_MM - 250.0; // 2200 мм

const long STRAIGHT_PULSES = (long)(STRAIGHT_MM / MM_PER_PULSE); // ~137
const long BRAKE_PULSES = (long)(STRAIGHT_BRAKE_MM / MM_PER_PULSE); // ~123

// Сколько сторон квадрата
const int TOTAL_SIDES = 4;

// ──────────────────────────────────────────────────────────────
// НАСТРОЙКИ ПОВОРОТА ПО ДУГЕ
// ──────────────────────────────────────────────────────────────

// Целевой угол поворота (90°)
const float TURN_ANGLE = 90.0;

// Скорость во время поворота (меньше = точнее)
// При повороте ENB задаёт скорость ходового, ENA — руля (полный поворот)
const int TURN_DRIVE_SPEED = 160;

// Тормозим когда набрали TURN_BRAKE_AT градусов.
// Инерция докрутит остаток до 90°.
// Подбирать: смотреть "после инерции ошибка=" в Serial Monitor.
//   перекручивает → увеличить (ближе к 90)
//   недокручивает → уменьшить
float TURN_BRAKE_AT = 90.0;

// Порог коррекции: если после инерции ошибка > этого — делаем доворот
const float CORRECTION_DEG = 4.0;

// Скорость микро-коррекции
const int CORRECTION_SPEED = 120;

// ──────────────────────────────────────────────────────────────

// ──────────── ГИРОСКОП ────────────
Adafruit_MPU6050 mpu;
float yaw = 0.0;
float gyroZ_offset = 0.0;
unsigned long lastTime = 0;

const float GZ_FILTER_ALPHA = 0.15;
float gz_filtered = 0.0;

// ──────────── ПИД (только для прямой) ────────────
float Kp = 5.0, Kd = 3.0;
float prevError = 0.0;
const int MAX_STEER = 220, MIN_STEER = 140, DEADBAND = 1;

// ──────────── СОСТОЯНИЕ ────────────
enum Phase { DRIVING, TURNING, DONE };
Phase phase = DRIVING;

int   sidesDone = 0;
bool  brakingActive = false;
float turnTarget = 0.0;   // целевой yaw после поворота
float pidTargetYaw = 0.0;   // целевой курс на прямой

volatile long encoderCount = 0;

// ──────────── ПРЕРЫВАНИЕ ────────────
void readEncoder() { encoderCount++; }

// ──────────── МОТОРЫ ────────────
void driveForward(int spd = 230) {
    analogWrite(ENB, spd);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void driveBackward(int spd = 160) {
    analogWrite(ENB, spd);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
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

// ──────────── РУЛЬ ────────────
// +1 = вправо полный, -1 = влево полный, 0 = прямо
void steerFull(int dir) {
    if (dir == 0) {
        digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
        analogWrite(ENA, 0);
    }
    else if (dir < 0) {
        analogWrite(ENA, MAX_STEER);
        digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    }
    else {
        analogWrite(ENA, MAX_STEER);
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    }
}

// ПИД-руление на прямой
void pidSteering() {
    float error = pidTargetYaw - yaw;
    float derivative = error - prevError;
    prevError = error;
    float pidOut = constrain(Kp * error + Kd * derivative, -MAX_STEER, MAX_STEER);
    int pwm = abs((int)pidOut);
    if (pwm < DEADBAND) {
        digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
        analogWrite(ENA, 0);
    }
    else if (pidOut < 0) {
        analogWrite(ENA, constrain(pwm, MIN_STEER, MAX_STEER));
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    }
    else {
        analogWrite(ENA, constrain(pwm, MIN_STEER, MAX_STEER));
        digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    }
}

// ──────────── ГИРОСКОП ────────────
void updateYaw() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    float gz_raw = g.gyro.z - gyroZ_offset;
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

void resetEncoder() {
    noInterrupts(); encoderCount = 0; interrupts();
}

// ──────────────────────────────────────────────────────────────
// МИКРО-КОРРЕКЦИЯ после дуги
// ──────────────────────────────────────────────────────────────
void correctTurn(int depth = 0) {
    if (depth > 1) return;  // не более 2 итераций

    float err = yaw - turnTarget;  // + недокрутил, - перекрутил
    if (fabs(err) < CORRECTION_DEG) {
        Serial.print("  Коррекция не нужна, err="); Serial.println(err, 2);
        return;
    }

    Serial.print("  Коррекция err="); Serial.println(err, 2);

    // err > 0 → недокрутили вправо → ещё вперёд с рулём вправо
    // err < 0 → перекрутили       → назад с рулём влево
    bool goFwd = (err > 0) ? false : true;  // влево = yaw растёт = err < 0 нужно ещё повернуть

    if (goFwd) {
        steerFull(-1);   // руль влево (недокрутили)
        driveForward(CORRECTION_SPEED);
    }
    else {
        steerFull(+1);   // руль вправо (перекрутили, сдаём назад)
        driveBackward(CORRECTION_SPEED);
    }

    // Следим в реальном времени
    unsigned long tStart = millis();
    while (millis() - tStart < 1500) {
        updateYaw();
        float rem = yaw - turnTarget;
        // Останавливаемся с небольшим упреждением (инерция на малой скорости ~0.5°)
        if (goFwd && rem >= -0.8) break;  // yaw догоняет turnTarget
        if (!goFwd && rem <= 0.8)  break;  // yaw возвращается к turnTarget
        delay(5);
    }

    brakeMotor();
    steerFull(0);
    delay(50);
    stopMotors();
    waitYawStable();

    float finalErr = yaw - turnTarget;
    Serial.print("  После коррекции err="); Serial.println(finalErr, 2);

    // Адаптация TURN_BRAKE_AT
    TURN_BRAKE_AT += finalErr * 0.4;
    TURN_BRAKE_AT = constrain(TURN_BRAKE_AT, 60.0, 89.0);
    Serial.print("  Новый TURN_BRAKE_AT="); Serial.println(TURN_BRAKE_AT, 1);

    // Если всё ещё промахнулись — ещё раз
    if (fabs(finalErr) > CORRECTION_DEG)
        correctTurn(depth + 1);
}

// ──────────────────────────────────────────────────────────────
// ЗАПУСК СЛЕДУЮЩЕЙ СТОРОНЫ
// ──────────────────────────────────────────────────────────────
void startNextSide() {
    pidTargetYaw = yaw;
    prevError = 0.0;
    resetEncoder();
    brakingActive = false;
    phase = DRIVING;

    Serial.print("=== Сторона "); Serial.print(sidesDone + 1);
    Serial.print("  курс="); Serial.println(pidTargetYaw, 1);

    driveForward();
}

// ──────────────────────────────────────────────────────────────
// ЗАПУСК ПОВОРОТА ПО ДУГЕ
// ──────────────────────────────────────────────────────────────
void startArcTurn() {
    // Поворот ВЛЕВО: yaw увеличивается на 90°
    // Если крутит в другую сторону — смените "+" на "-" и steerFull(-1) на steerFull(+1)
    turnTarget = pidTargetYaw - TURN_ANGLE;

    Serial.print("--- Дуга поворот #"); Serial.print(sidesDone);
    Serial.print("  цель yaw="); Serial.println(turnTarget, 1);

    phase = TURNING;

    // Руль полностью вправо + едем вперёд с уменьшенной скоростью
    steerFull(-1);   // руль влево = поворот влево
    driveForward(TURN_DRIVE_SPEED);
}

// ──────────── SETUP ────────────
void setup() {
    Serial.begin(9600);
    delay(500);

    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    stopMotors();

    pinMode(ENC, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC), readEncoder, RISING);
    resetEncoder();

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
    gyroZ_offset = sumZ / 500.0;
    gz_filtered = 0.0;
    lastTime = micros();
    yaw = 0.0;

    Serial.print("Offset: ");            Serial.println(gyroZ_offset, 5);
    Serial.print("MM/импульс: ");        Serial.println(MM_PER_PULSE, 2);
    Serial.print("Прямая до дуги мм: "); Serial.println(STRAIGHT_MM);
    Serial.print("Прямая импульсов: ");  Serial.println(STRAIGHT_PULSES);
    Serial.print("Торм с импульса: ");   Serial.println(BRAKE_PULSES);
    Serial.print("TURN_BRAKE_AT: ");     Serial.println(TURN_BRAKE_AT);
    Serial.println("Старт через 0.5 сек...");
    delay(500);

    pidTargetYaw = 0.0;
    phase = DRIVING;
    Serial.println("=== Сторона 1  курс=0.0");
    driveForward();
}

// ──────────── LOOP ────────────
void loop() {
    updateYaw();

    noInterrupts();
    long pulses = encoderCount;
    interrupts();

    switch (phase) {

        // ════════ ПРЯМОЛИНЕЙНОЕ ДВИЖЕНИЕ ════════
    case DRIVING: {
        pidSteering();

        // Начало дуги — просто останавливаемся и начинаем поворот
        if (pulses >= STRAIGHT_PULSES) {
            stopMotors();
            Serial.print("Начало дуги, имп="); Serial.print(pulses);
            Serial.print("  yaw="); Serial.print(yaw, 1);
            Serial.print("  мм="); Serial.println(pulses * MM_PER_PULSE, 0);
            delay(150);
            startArcTurn();
        }
        break;
    }

                // ════════ ПОВОРОТ ПО ДУГЕ ════════
    case TURNING: {
        // Используем abs() — работает независимо от знака yaw
        // Это защищает от бесконечного кружения если знак не тот
        float turnedSoFar = fabs(yaw - pidTargetYaw);

        // Таймаут: если за 10 секунд не повернули — аварийный стоп
        static unsigned long turnStartMs = 0;
        if (turnStartMs == 0) turnStartMs = millis();
        if (millis() - turnStartMs > 10000) {
            stopMotors(); steerFull(0);
            Serial.println("!!! ТАЙМАУТ ПОВОРОТА — аварийный стоп");
            Serial.print("yaw="); Serial.print(yaw, 1);
            Serial.print(" pidTargetYaw="); Serial.println(pidTargetYaw, 1);
            turnStartMs = 0;
            sidesDone++;
            if (sidesDone >= TOTAL_SIDES) { phase = DONE; }
            else { delay(300); startNextSide(); }
            break;
        }

        if (turnedSoFar >= TURN_BRAKE_AT) {
            turnStartMs = 0;  // сброс таймера
            stopMotors();
            steerFull(0);

            Serial.print("Торможение дуги: повернули="); Serial.print(turnedSoFar, 1);
            Serial.print("  yaw="); Serial.println(yaw, 1);

            // Ждём инерцию
            waitYawStable();

            float errAfterInertia = yaw - turnTarget;
            Serial.print("  После инерции ошибка="); Serial.println(errAfterInertia, 1);

            // Адаптация на следующий поворот
            TURN_BRAKE_AT -= errAfterInertia * 0.4;
            TURN_BRAKE_AT = constrain(TURN_BRAKE_AT, 60.0, 89.0);
            Serial.print("  TURN_BRAKE_AT → "); Serial.println(TURN_BRAKE_AT, 1);

            // Коррекция если нужна
            correctTurn();

            sidesDone++;
            Serial.print("=== ПОВОРОТ #"); Serial.print(sidesDone);
            Serial.print(" завершён. Итог yaw="); Serial.print(yaw, 1);
            Serial.print("  цель="); Serial.print(turnTarget, 1);
            Serial.print("  финал погрешность="); Serial.println(yaw - turnTarget, 1);

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
            Serial.print("/"); Serial.print(TURN_BRAKE_AT, 1);
            Serial.print("  yaw="); Serial.print(yaw, 1);
            Serial.print("  target="); Serial.println(pidTargetYaw + TURN_ANGLE, 1);
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
            Serial.print("DRIVE yaw=");  Serial.print(yaw, 1);
            Serial.print("  side=");     Serial.print(sidesDone + 1);
            Serial.print("  pulses=");   Serial.print(pulses);
            Serial.print("/");           Serial.println(STRAIGHT_PULSES);
        }
        lastPrint = millis();
    }

    delay(8);
}