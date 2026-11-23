#pragma once
// =====================================================================
// encoder.h
// ผู้รับผิดชอบหลัก: บทบาท 5 (Firmware การตรวจวัด)
// นับ encoder ด้วย hardware PCNT ของ ESP32-S2 (เฟส A/B, decode x4)
// ใช้ตรวจจับสเต็ปหาย (R5) และเป็นฟีดแบ็กตำแหน่งจริง
// =====================================================================

void encoderInit();

// ตำแหน่งจริงปัจจุบัน หน่วย mm (คำนวณจาก counts/mm ใน config.h)
float encoderGetPositionMm();

// จำนวน count ดิบจาก PCNT (ไว้ debug / สอบเทียบ)
long encoderGetRawCounts();

// รีเซ็ต count เป็น 0 (เรียกตอน homing สำเร็จ)
void encoderResetZero();

// เทียบตำแหน่งจริงกับตำแหน่งที่สั่ง หากต่างเกินเกณฑ์ถือว่าสเต็ปหาย (R5, T5)
bool encoderDetectStepLoss(float commandedMm, float thresholdMm);
