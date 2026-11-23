#include "motion.h"
#include "config.h"

// TODO (บทบาท 4): implement การสร้างพัลส์จริง (แนะนำ hardware timer/RMT/LEDC ของ ESP32-S2
// แทนการ digitalWrite ใน loop เพราะ loop ไม่ real-time พอสำหรับความถี่พัลส์สูง และกระทบ jitter)

static float commandedPositionMm = 0.0f;
static float targetSpeedMmS = DEFAULT_SPEED_MM_S;
static bool moving = false;

void motionInit() {
    // TODO: ตั้งค่า pinMode ของ PIN_PUL/PIN_DIR/PIN_ENA และตั้งค่า timer/RMT
}

void motionStartHoming() {
    // TODO: เคลื่อนเข้าหา PIN_SENSOR_HOME ด้วยความเร็วต่ำ แล้วเรียก encoderResetZero()
}

void motionMoveTo(float targetMm) {
    // TODO: ตรวจ soft limit (SOFT_LIMIT_MIN_MM/MAX_MM) ก่อนเริ่มเคลื่อนที่
    commandedPositionMm = targetMm;
    moving = true;
}

void motionSetSpeed(float speedMmS) {
    targetSpeedMmS = speedMmS;
}

void motionEmergencyStop() {
    moving = false;
    // TODO: หยุดพัลส์ทันที
}

void motionDisableDriver() {
    // TODO: ปลด ENA (PIN_ENA)
}

void motionUpdate() {
    // TODO: เดินทางลาดความเร็ว (accel/decel) และตรวจว่าถึงเป้าหมายหรือยัง
}

bool motionIsMoving() {
    return moving;
}

float motionGetCommandedPositionMm() {
    return commandedPositionMm;
}
