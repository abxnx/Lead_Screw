
#include "ui.h"
#include "config.h"
#include "motion.h"
#include "encoder.h"
#include "sensors.h"
#include "state_machine.h"
#include <Arduino.h>

// ===================================================================
// ตัวแปลคำสั่ง Serial ตาม R6: home, move, speed, stop, status (ครบ 5 คำสั่งบังคับ)
// เพิ่ม: help, sensors (debug) และการ validate/ยืนยันผลทุกคำสั่ง
//
// รูปแบบ: ส่งทีละบรรทัด จบด้วย newline, baud ตาม SERIAL_BAUD ใน config.h
//   home              -> เริ่ม homing
//   move <mm>         -> เช่น move 50.0
//   speed <mm/s>      -> เช่น speed 10.0
//   stop              -> หยุดฉุกเฉินทันที
//   status            -> รายงานสถานะ
//   sensors           -> (debug) ดูสถานะเซนเซอร์ดิบ
//   help              -> แสดงคำสั่งทั้งหมด
// ===================================================================

static bool parseFloatStrict(const String &text, float &outValue) {
    // ตรวจว่า argument เป็นตัวเลขจริง ไม่ใช่ค่าว่างหรือข้อความมั่ว
    // (toFloat() ของ Arduino คืน 0.0 ทั้งกรณี "0" และกรณี parse ไม่ได้ ต้องกันเองตรงนี้)
    if (text.length() == 0) return false;
    bool hasDigit = false;
    for (size_t i = 0; i < text.length(); i++) {
        char c = text[i];
        if (isDigit(c)) hasDigit = true;
        else if (c != '.' && c != '-' && c != '+') return false;
    }
    if (!hasDigit) return false;
    outValue = text.toFloat();
    return true;
}

static void printHelp() {
    Serial.println(F("commands: home | move <mm> | speed <mm/s> | stop | status | sensors | help"));
}

void uiInit() {
    Serial.begin(SERIAL_BAUD);
}

void uiUpdate() {
    if (!Serial.available()) return;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line == "home") {
        motionStartHoming();
        Serial.println(F("OK homing"));

    } else if (line.startsWith("move ")) {
        float target;
        if (!parseFloatStrict(line.substring(5), target)) {
            Serial.println(F("ERR invalid argument, usage: move <mm>"));
            return;
        }
        // R7: ห้ามเคลื่อนที่ต่อขณะอยู่ใน FAULT จนกว่าจะ home ใหม่
        if (getCurrentState() == SystemState::FAULT) {
            Serial.println(F("ERR system in FAULT, run 'home' first"));
            return;
        }
        if (target < SOFT_LIMIT_MIN_MM || target > SOFT_LIMIT_MAX_MM) {
            Serial.println(F("ERR target outside soft limit"));
            return;
        }
        motionMoveTo(target);
        Serial.print(F("OK move "));
        Serial.println(target);

    } else if (line.startsWith("speed ")) {
        float spd;
        if (!parseFloatStrict(line.substring(6), spd) || spd <= 0) {
            Serial.println(F("ERR invalid argument, usage: speed <mm/s> (must be > 0)"));
            return;
        }
        motionSetSpeed(spd);
        Serial.print(F("OK speed "));
        Serial.println(spd);

    } else if (line == "stop") {
        motionEmergencyStop();
        Serial.println(F("OK stopped"));

    } else if (line == "status") {
        uiReportStatus();

    } else if (line == "sensors") {
        Serial.print(F("home="));
        Serial.print(sensorHomeTriggered());
        Serial.print(F(" left="));
        Serial.print(sensorLeftLimitTriggered());
        Serial.print(F(" right="));
        Serial.print(sensorRightLimitTriggered());
        Serial.print(F(" estop="));
        Serial.println(eStopTriggered());

    } else if (line == "help") {
        printHelp();

    } else {
        Serial.println(F("ERR unknown command, type 'help'"));
    }
}

void uiReportStatus() {
    Serial.print(F("state="));
    Serial.print(stateToString(getCurrentState()));
    Serial.print(F(" cmd_mm="));
    Serial.print(motionGetCommandedPositionMm());
    Serial.print(F(" enc_mm="));
    Serial.print(encoderGetPositionMm());
    Serial.print(F(" moving="));
    Serial.println(motionIsMoving());
}
