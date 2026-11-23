#include "encoder.h"
#include "config.h"

// TODO (บทบาท 5/6): ใช้ ESP32 PCNT (Pulse Counter) อ่านเฟส A/B จริง
// นับคำนวณ counts/mm ให้ตรงกับที่สอบเทียบใน config.h / calibration.cpp

void encoderInit() {
    // TODO: ตั้งค่า PCNT unit, กำหนดขา PIN_ENCODER_A/B, filter glitch
}

float encoderGetPositionMm() {
    // TODO: raw counts / counts_per_mm
    return 0.0f;
}

long encoderGetRawCounts() {
    // TODO
    return 0;
}

void encoderResetZero() {
    // TODO: pcnt_counter_clear(...)
}

bool encoderDetectStepLoss(float commandedMm, float thresholdMm) {
    float actual = encoderGetPositionMm();
    return (actual - commandedMm > thresholdMm) || (commandedMm - actual > thresholdMm);
}
