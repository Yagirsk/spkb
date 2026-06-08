#include <Arduino.h>

const int ENC_PIN = A0;
const float THRESHOLD = 4.85;
const unsigned long MIN_INTERVAL_MS = 20; // 0.02 сек

long encoderCount = 0;
bool lastState = false;
unsigned long lastTriggerMs = 0;

void setup() {
    Serial.begin(9600);
}

void loop() {
    float voltage = analogRead(ENC_PIN) * (5.0 / 1023.0);
    bool state = voltage > THRESHOLD;

    if (state && !lastState) {
        unsigned long now = millis();
        if (now - lastTriggerMs >= MIN_INTERVAL_MS) {
            lastTriggerMs = now;
            encoderCount++;
            Serial.print(F("ENC: ")); Serial.println(encoderCount);
        }
    }

    lastState = state;
}