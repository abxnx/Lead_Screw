#include "state_machine.h"

// TODO (บทบาท 4): ใส่ logic การเปลี่ยน state จริง + เงื่อนไขตามคำถามออกแบบข้อ 7
// ตัวอย่างกฎที่ต้องคง: หลัง FAULT (เช่นจาก E-stop หรือ hard limit) ต้อง home ใหม่ก่อนจึงจะ MOVING ได้อีก (R7, R2)

static SystemState currentState = SystemState::IDLE;

const char* stateToString(SystemState state) {
    switch (state) {
        case SystemState::IDLE:   return "IDLE";
        case SystemState::HOMING: return "HOMING";
        case SystemState::READY:  return "READY";
        case SystemState::MOVING: return "MOVING";
        case SystemState::FAULT:  return "FAULT";
    }
    return "UNKNOWN";
}

void stateMachineUpdate() {
    // TODO: ใส่ transition logic จริงที่นี่
}

void requestState(SystemState next) {
    // TODO: ตรวจเงื่อนไขก่อนอนุญาตให้เปลี่ยน state (เช่น ห้ามออกจาก FAULT โดยไม่ home ใหม่)
    currentState = next;
}

SystemState getCurrentState() {
    return currentState;
}
