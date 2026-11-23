#include "sensors.h"
#include "config.h"
#include <Arduino.h>

// ยืนยันแล้ว (บทบาท 5, 23/11/2026): SN04-N ผ่าน DST-1R4P-N เป็น active-high
// เจอโลหะ = HIGH, ไม่เจอโลหะ = LOW
//
// Debounce: ค่าต้องอ่านได้เหมือนเดิมต่อเนื่องอย่างน้อย DEBOUNCE_MS
// ก่อนถึงจะยอมรับว่าเป็นสถานะจริง (กันสัญญาณรบกวนชั่วขณะ)

#define DEBOUNCE_MS 5

struct DebouncedInput {
    uint8_t pin;
    bool stableState;      // ค่าที่ผ่าน debounce แล้ว ให้ไฟล์อื่นเรียกใช้
    bool lastRawReading;    // ค่าดิบที่อ่านได้ล่าสุด (ยังไม่ผ่าน debounce)
    unsigned long lastChangeMs; // เวลาที่ค่าดิบเปลี่ยนล่าสุด
};

static DebouncedInput homeInput;
static DebouncedInput leftLimitInput;
static DebouncedInput rightLimitInput;
static DebouncedInput eStopInput;

static void initDebouncedInput(DebouncedInput &input, int pin) {
    input.pin = pin;
    pinMode(pin, INPUT);
    bool initial = digitalRead(pin) == HIGH;
    input.stableState = initial;
    input.lastRawReading = initial;
    input.lastChangeMs = millis();
}

static void updateDebouncedInput(DebouncedInput &input) {
    bool raw = digitalRead(input.pin) == HIGH;   // active-high: HIGH = เจอโลหะ/ทริกเกอร์
    unsigned long now = millis();

    if (raw != input.lastRawReading) {
        // ค่าดิบเปลี่ยน เริ่มจับเวลาใหม่
        input.lastRawReading = raw;
        input.lastChangeMs = now;
    } else if ((now - input.lastChangeMs) >= DEBOUNCE_MS) {
        // ค่าดิบคงที่นานพอแล้ว ยอมรับเป็นสถานะจริง
        input.stableState = raw;
    }
}

void sensorsInit() {
    initDebouncedInput(homeInput, PIN_SENSOR_HOME);
    initDebouncedInput(leftLimitInput, PIN_SENSOR_LEFT);
    initDebouncedInput(rightLimitInput, PIN_SENSOR_RIGHT);

    if (PIN_ESTOP >= 0) {
        initDebouncedInput(eStopInput, PIN_ESTOP);
    } else {
        // ยังไม่ได้กำหนดขา E-stop ใน config.h (รอตัดสินใจ)
        eStopInput.stableState = false;
    }
}

void sensorsUpdate() {
    updateDebouncedInput(homeInput);
    updateDebouncedInput(leftLimitInput);
    updateDebouncedInput(rightLimitInput);

    if (PIN_ESTOP >= 0) {
        updateDebouncedInput(eStopInput);
    }
}

bool sensorHomeTriggered() { return homeInput.stableState; }
bool sensorLeftLimitTriggered() { return leftLimitInput.stableState; }
bool sensorRightLimitTriggered() { return rightLimitInput.stableState; }
bool eStopTriggered() { return eStopInput.stableState; }