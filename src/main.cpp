#include <Arduino.h>
#include "config.h"
#include "motion.h"
#include "encoder.h"
#include "sensors.h"
#include "ui.h"
#include "state_machine.h"

// =====================================================================
// main.cpp
// จุดรวม setup()/loop() — ไม่ควรใส่ logic หนัก ๆ ที่นี่ ให้เรียกใช้โมดูลต่าง ๆ แทน
// ผู้รับผิดชอบหลัก: บทบาท 1 (Project manager / system integrator) ดูแลการรวมระบบ
// =====================================================================

void setup() {
    uiInit();
    motionInit();
    encoderInit();
    sensorsInit();

    Serial.println("Lead Screw Linear Stage - Cucumber RS");
    Serial.println("พิมพ์ 'home' เพื่อเริ่ม homing, 'status' เพื่อดูสถานะ");

    requestState(SystemState::IDLE);
}

void loop() {
    sensorsUpdate();
    uiUpdate();

    // TODO: เช็ค eStopTriggered() / limit sensor ก่อนเสมอ แล้วค่อยอัปเดต motion/state
    if (eStopTriggered()) {
        motionEmergencyStop();
        motionDisableDriver();
        requestState(SystemState::FAULT);
    }

    motionUpdate();
    stateMachineUpdate();
}
