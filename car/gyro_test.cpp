//Плата ARDUINO NANO
#include <Adafruit_MPU6050.h> //Высокоуровневый драйвер конкретно для датчика MPU6050
#include <Adafruit_Sensor.h> //Единый формат представления данных для всех датчиков Adafruit
#include <Wire.h> //Встроенная библиотека для передачи битового обмена

Adafruit_MPU6050 mpu; //Создание объекта для вызова всех функций датчика

float ax = 0, ay = 0, az = 0; //Углы


void setup() {
    Serial.begin(115200); //Запуск последовательного порта (Serial) на скорости 115200 бит/сек
    Serial.printls("TEST!");

    //Попытка инициализации датчика по I^2C
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    //Установка диапазона акселерометра на +-8g
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

    //Установка диапазона гироскопа на +-500 градусов/сек
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    //Установка полосы пропускания встроенного цифрового фильтра датчика на 21 Гц
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.print("Accelerometer range set to: +-8G");
    Serial.print("Gyro range set to: +- 500 deg/s");
    Serial.print("Filter bandwidth set to: 21 Hz");
}

void loop() {
    //Чтение событий датчика
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    //Вывод значений
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");

    Serial.println("");
    delay(500);
}