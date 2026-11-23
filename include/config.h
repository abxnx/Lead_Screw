#pragma once
// =====================================================================
// config.h
// รวม pin map, ค่าคาลิเบรต และค่าคงที่ของระบบไว้ที่เดียว
// ผู้รับผิดชอบหลัก: บทบาท 2 (ฮาร์ดแวร์และการต่อสาย) + บทบาท 6 (การสอบเทียบ)
//
// ***สำคัญ*** ห้ามใช้ขา GPIO0, GPIO45, GPIO46 (strapping),
// GPIO19/GPIO20 (USB D-/D+), GPIO26 (flash CS), GPIO18 (RGB LED),
// GPIO40/GPIO41 (I2C เซนเซอร์บนบอร์ด) เว้นแต่เข้าใจผลกระทบแล้ว (ดูหัวข้อ 2.3)
// =====================================================================

// ---------- Pin map: TB6600 (PUL/DIR/ENA) ----------
#define PIN_PUL        -1   // TODO: กำหนดขาจริง
#define PIN_DIR        -1   // TODO
#define PIN_ENA        -1   // TODO

// ---------- Pin map: Proximity sensor (ผ่าน DST-1R4P-N) ----------
#define PIN_SENSOR_LEFT    -1   // TODO: SN04-N ซ้าย
#define PIN_SENSOR_HOME    -1   // TODO: SN04-N กลาง (หรือกำหนดตัว home ตามที่เลือกจริง)
#define PIN_SENSOR_RIGHT   -1   // TODO: SN04-N ขวา

// ---------- Pin map: ปุ่มหยุดฉุกเฉิน ----------
#define PIN_ESTOP          -1   // TODO: ต่อฮาร์ดแวร์โดยตรง ต้องเทสแยกจากซอฟต์แวร์ด้วย

// ---------- Pin map: Encoder A/B (ผ่าน optocoupler PC817/EL817) ----------
#define PIN_ENCODER_A      -1   // TODO: ต้องใช้ขาที่รองรับ PCNT
#define PIN_ENCODER_B      -1   // TODO

// ---------- ค่าจากการสอบเทียบ (บทบาท 6 ต้องวัดจากชุดจริง) ----------
#define LEAD_MM_PER_REV        0.0   // TODO: ค่า L (mm ต่อรอบ) จากการวัดจริง
#define MICROSTEP              1    // TODO: ค่า microstep ที่ตั้งบน TB6600 (DIP switch)
#define ENCODER_PPR             600  // HN3806-AB600N: 600 พัลส์ต่อรอบ (x4 ถ้า decode แบบ 4 เท่า)
#define ENCODER_DECODE_X4       true

// steps/mm = (200 * MICROSTEP) / LEAD_MM_PER_REV
// counts/mm = (ENCODER_PPR * 4) / LEAD_MM_PER_REV   (ถ้า decode แบบ 4 เท่า)
// คำนวณจริงให้ทำใน calibration.cpp และยืนยันด้วยการวัดระยะจริง ห้าม hardcode เฉย ๆ

// ---------- ขีดจำกัดการเคลื่อนที่ (soft limit, นอกเหนือจาก hardware limit sensor) ----------
#define SOFT_LIMIT_MIN_MM     0.0
#define SOFT_LIMIT_MAX_MM     0.0   // TODO: ตั้งค่าตามระยะใช้งานจริงที่วัดได้ ให้เผื่อระยะก่อนถึงเซนเซอร์ limit จริง

// ---------- ความเร็ว/ความเร่งเริ่มต้น ----------
#define DEFAULT_SPEED_MM_S      5.0
#define DEFAULT_ACCEL_MM_S2     20.0

// ---------- Serial ----------
#define SERIAL_BAUD            115200
