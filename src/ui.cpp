#include "ui.h"
#include "config.h"
#include "motion.h"
#include "encoder.h"
#include "state_machine.h"
#include <Arduino.h>

// TODO (บทบาท 7): parser คำสั่ง Serial ให้ครบตาม R6 (home, move, speed, stop, status)
// แนะนำ: อ่านทีละบรรทัด (readStringUntil('\n')), trim, แยกคำสั่ง/argument ด้วย space

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
    } else if (line.startsWith("move ")) {
        float target = line.substring(5).toFloat();
        motionMoveTo(target);
    } else if (line.startsWith("speed ")) {
        float spd = line.substring(6).toFloat();
        motionSetSpeed(spd);
    } else if (line == "stop") {
        motionEmergencyStop();
    } else if (line == "status") {
        uiReportStatus();
    } else {
        Serial.println("ERR unknown command");
    }
}

void uiReportStatus() {
    Serial.print("state=");
    Serial.print(stateToString(getCurrentState()));
    Serial.print(" cmd_mm=");
    Serial.print(motionGetCommandedPositionMm());
    Serial.print(" enc_mm=");
    Serial.println(encoderGetPositionMm());
}
