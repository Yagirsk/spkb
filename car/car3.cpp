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
const float WHEEL_CIRC_MM     = WHEEL_DIAMETER_MM * 3.14;
const int   STRIPES_PER_REV   = 14;
const float MM_PER_PULSE      = WHEEL_CIRC_MM / STRIPES_PER_REV; // ~17.94 мм

// ──────────────────────────────────────────────────────────────
// ГЕОМЕТРИЯ КВАДРАТА
// ──────────────────────────────────────────────────────────────
// Сторона квадрата (расстояние от угла до угла)
const float SIDE_MM   = 3000.0;

// Радиус поворота 55 см.
// Машина начинает поворачивать за TURN_RADIUS_MM до угла,
// чтобы выйти точно на следующую сторону.
const float TURN_RADIUS_MM = 550.0;

// Прямой участок до начала поворота:
//   SIDE_MM - TURN_RADIUS_MM = 3000 - 550 = 2450 мм
const float STRAIGHT_MM = SIDE_MM - TURN_RADIUS_MM; // 2450 мм

// Тормозной путь прямой (250 мм до конца прямого участка)
const float STRAIGHT_BRAKE_MM = STRAIGHT_MM - 250.0; // 2200 мм

const long STRAIGHT_PULSES = (long)(STRAIGHT_MM       / MM_PER_PULSE); // ~137
const long BRAKE_PULSES    = (long)(STRAIGHT_BRAKE_MM / MM_PER_PULSE); // ~123

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
float yaw          = 0.0;
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

int   sidesDone    = 0;
bool  brakingActive = false;
float turnTarget   = 0.0;   // целевой yaw после поворота
float pidTargetYaw = 0.0;   // целевой курс на прямой

volatile long encoderCount = 0;

// Для плоттера
float arcProgress = 0.0;
float turnError = 0.0;

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
  } else if (dir < 0) {
    analogWrite(ENA, MAX_STEER);
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  } else {
    analogWrite(ENA, MAX_STEER);
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  }
}

// ПИД-руление на прямой
void pidSteering() {
  float error      = pidTargetYaw - yaw;
  float derivative = error - prevError;
  prevError        = error;
  float pidOut     = constrain(Kp * error + Kd * derivative, -MAX_STEER, MAX_STEER);
  int pwm = abs((int)pidOut);
  if (pwm < DEADBAND) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  } else if (pidOut < 0) {
    analogWrite(ENA, constrain(pwm, MIN_STEER, MAX_STEER));
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
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
  gz_filtered  = GZ_FILTER_ALPHA * gz_raw + (1.0 - GZ_FILTER_ALPHA) * gz_filtered;
  float gz_use = (fabs(gz_filtered) > 0.003) ? gz_filtered : 0.0;
  if (dt > 0.001 && dt < 0.1)
    yaw += gz_use * (180.0 / M_PI) * dt;
}

void waitYawStable() {
  float prev  = yaw;
  int   count = 0;
  for (int i = 0; i < 40; i++) {
    delay(15);
    updateYaw();
    if (fabs(yaw - prev) < 0.10) {
      if (++count >= 3) break;
    } else {
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
  if (fabs(err) < CORRECTION_DEG) return;

  // err > 0 → недокрутили вправо → ещё вперёд с рулём вправо
  // err < 0 → перекрутили       → назад с рулём влево
  bool goFwd = (err > 0) ? false : true;  // влево = yaw растёт = err < 0 нужно ещё повернуть

  if (goFwd) {
    steerFull(-1);   // руль влево (недокрутили)
    driveForward(CORRECTION_SPEED);
  } else {
    steerFull(+1);   // руль вправо (перекрутили, сдаём назад)
    driveBackward(CORRECTION_SPEED);
  }

  // Следим в реальном времени
  unsigned long tStart = millis();
  while (millis() - tStart < 1500) {
    updateYaw();
    float rem = yaw - turnTarget;
    // Останавливаемся с небольшим упреждением (инерция на малой скорости ~0.5°)
    if (goFwd  && rem >= -0.8) break;  // yaw догоняет turnTarget
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
  TURN_BRAKE_AT  = constrain(TURN_BRAKE_AT, 60.0, 89.0);
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
  prevError    = 0.0;
  resetEncoder();
  brakingActive = false;
  phase = DRIVING;
  driveForward();
}

// ──────────────────────────────────────────────────────────────
// ЗАПУСК ПОВОРОТА ПО ДУГЕ
// ──────────────────────────────────────────────────────────────
void startArcTurn() {
  // Поворот ВЛЕВО: yaw увеличивается на 90°
  // Если крутит в другую сторону — смените "+" на "-" и steerFull(-1) на steerFull(+1)
  turnTarget = pidTargetYaw - TURN_ANGLE;
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
  gz_filtered  = 0.0;
  lastTime     = micros();
  yaw          = 0.0;

  Serial.print("Offset: ");            Serial.println(gyroZ_offset, 5);
  Serial.print("MM/импульс: ");        Serial.println(MM_PER_PULSE, 2);
  Serial.print("Прямая до дуги мм: "); Serial.println(STRAIGHT_MM);
  Serial.print("Прямая импульсов: ");  Serial.println(STRAIGHT_PULSES);
  Serial.print("Торм с импульса: ");   Serial.println(BRAKE_PULSES);
  Serial.print("TURN_BRAKE_AT: ");     Serial.println(TURN_BRAKE_AT);
  Serial.println("Старт через 0.5 сек...");
  delay(500);

  Serial.println("yaw\tpidTarget\tencoder\tphase\tside\terror\tturnProgress\tturnError");

  pidTargetYaw = 0.0;
  phase        = DRIVING;
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
        delay(150);
        startArcTurn();
      }
      break;
    }

    // ════════ ПОВОРОТ ПО ДУГЕ ════════
    case TURNING: {
      float turnedSoFar = fabs(yaw - pidTargetYaw);

      static unsigned long turnStartMs = 0;
      if (turnStartMs == 0) turnStartMs = millis();
      if (millis() - turnStartMs > 10000) {
        stopMotors(); steerFull(0);
        Serial.println("! ТАЙМАУТ ПОВОРОТА");  // префикс '!' чтобы плоттер игнорировал
        turnStartMs = 0;
        sidesDone++;
        if (sidesDone >= TOTAL_SIDES) { phase = DONE; }
        else { delay(300); startNextSide(); }
        break;
      }

      if (turnedSoFar >= TURN_BRAKE_AT) {
        turnStartMs = 0;
        stopMotors();
        steerFull(0);
        waitYawStable();

        float errAfterInertia = yaw - turnTarget;
        TURN_BRAKE_AT -= errAfterInertia * 0.4;
        TURN_BRAKE_AT  = constrain(TURN_BRAKE_AT, 60.0, 89.0);

        correctTurn();

        sidesDone++;
        if (sidesDone >= TOTAL_SIDES) {
          stopMotors();
          phase = DONE;
        } else {
          delay(300);
          startNextSide();
        }
        break;
      }
      break;
    }

    case DONE:
      break;
  }

  // ── Единый вывод для плоттера каждые 100 мс ──
  static unsigned long lastPlot = 0;
  if (millis() - lastPlot >= 100) {
    lastPlot = millis();

    float err = 0.0;
    arcProgress = 0.0;
    turnError = 0.0;
    int currentSide = sidesDone + (phase == DONE ? 0 : 1);

    if (phase == DRIVING) {
      err = yaw - pidTargetYaw;
    } else if (phase == TURNING) {
      arcProgress = fabs(yaw - pidTargetYaw);
      turnError = yaw - turnTarget;
      err = turnError;
    }

    // Формат: значения через табуляцию
    Serial.print(yaw, 2);              Serial.print("\t");
    Serial.print(pidTargetYaw, 2);     Serial.print("\t");
    Serial.print(pulses);              Serial.print("\t");
    Serial.print(phase);               Serial.print("\t");  // 0=DRIVING, 1=TURNING, 2=DONE
    Serial.print(currentSide);         Serial.print("\t");
    Serial.print(err, 2);             Serial.print("\t");
    Serial.print(arcProgress, 2);     Serial.print("\t");
    Serial.println(turnError, 2);
  }

  delay(8);
}
