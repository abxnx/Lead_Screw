#include "sensors.h"
#include "config.h"

// TODO (บทบาท 5): อ่านขาจริงพร้อม debounce/filter
// ระวัง: สัญญาณจาก SN04-N ผ่าน DST-1R4P-N เป็นลอจิก 3.3V ฝั่งบอร์ดแล้ว
// แต่ต้องตรวจสอบ active-high/active-low ให้ตรงกับที่วัดจริง

static bool homeState = false;
static bool leftLimitState = false;
static bool rightLimitState = false;
static bool eStopState = false;

void sensorsInit() {
    // TODO: pinMode ของ PIN_SENSOR_* และ PIN_ESTOP
}

void sensorsUpdate() {
    // TODO: อ่านค่าจริงพร้อม debounce แล้วอัปเดตตัวแปร static ด้านบน
}

bool sensorHomeTriggered() { return homeState; }
bool sensorLeftLimitTriggered() { return leftLimitState; }
bool sensorRightLimitTriggered() { return rightLimitState; }
bool eStopTriggered() { return eStopState; }
