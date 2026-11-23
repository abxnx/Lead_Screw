#pragma once
#include <stdint.h>
// =====================================================================
// motion.h
// ผู้รับผิดชอบหลัก: บทบาท 4 (Firmware การเคลื่อนที่)
// สร้างพัลส์ PUL/DIR/ENA, ทางลาดความเร็ว (accel/decel ramp), homing
// ใช้วิธีที่เหมาะสม เช่น hardware timer / RMT / LEDC แทน digitalWrite ใน loop()
// (ดูคำถามออกแบบข้อ 6 ในหัวข้อ 3.1)
// =====================================================================

void motionInit();

// เริ่ม homing ตามข้อกำหนด R1: เคลื่อนเข้าหาเซนเซอร์ home แล้วตั้งตำแหน่งเป็น 0 mm
void motionStartHoming();

// สั่งไปตำแหน่งเป้าหมาย (mm) ด้วยความเร็วที่ตั้งไว้ล่าสุด (R3)
void motionMoveTo(float targetMm);

// ตั้งความเร็วเป้าหมาย mm/s (R4)
void motionSetSpeed(float speedMmS);

// หยุดทันที (ใช้ตอน soft/hard limit หรือ E-stop ตาม R2, R7)
void motionEmergencyStop();

// ปลด ENA (หลัง E-stop ต้อง home ใหม่ก่อนทำงานต่อ ตาม R7)
void motionDisableDriver();

// ต้องเรียกถี่ ๆ ใน loop() เพื่อเดินทางลาดความเร็วและตรวจสถานะการเคลื่อนที่
void motionUpdate();

bool motionIsMoving();
float motionGetCommandedPositionMm();
