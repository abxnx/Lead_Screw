#pragma once
// =====================================================================
// state_machine.h
// ผู้รับผิดชอบหลัก: บทบาท 4 (Firmware การเคลื่อนที่)
// ตามหัวข้อ 3.1 ข้อ 7: ต้องมีแผนภาพ state machine และเงื่อนไขเปลี่ยนสถานะ
// ตัวอย่าง state: IDLE, HOMING, READY, MOVING, FAULT
// =====================================================================

enum class SystemState {
    IDLE,
    HOMING,
    READY,
    MOVING,
    FAULT
};

// คืนชื่อ state เป็น string สำหรับรายงานสถานะทาง Serial (R6)
const char* stateToString(SystemState state);

// เรียกทุกรอบ loop() เพื่ออัปเดต state machine
void stateMachineUpdate();

// ขอเปลี่ยน state (ฟังก์ชันควรตรวจเงื่อนไขก่อนเปลี่ยนจริง เช่น FAULT ต้อง home ใหม่ก่อนเสมอ ตาม R7)
void requestState(SystemState next);

SystemState getCurrentState();
