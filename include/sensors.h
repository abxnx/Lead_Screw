#pragma once
// =====================================================================
// sensors.h
// ผู้รับผิดชอบหลัก: บทบาท 5 (Firmware การตรวจวัด)
// อ่าน proximity sensor 3 ตัว (SN04-N ผ่าน DST-1R4P-N) และปุ่มหยุดฉุกเฉิน
// ต้องมีการกรองสัญญาณรบกวน (debounce / digital filter)
// =====================================================================

void sensorsInit();

// เรียกทุกรอบ loop() เพื่ออัปเดตค่าที่อ่านพร้อม debounce
void sensorsUpdate();

bool sensorHomeTriggered();
bool sensorLeftLimitTriggered();
bool sensorRightLimitTriggered();

// true ถ้าปุ่มหยุดฉุกเฉินถูกกด (ควรเช็คฮาร์ดแวร์โดยตรงด้วย ไม่พึ่งพาซอฟต์แวร์อย่างเดียว ตาม R7)
bool eStopTriggered();
