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
#define PIN_PUL 10 // PUL+
#define PIN_DIR 11 // DIR+

// ---------- Pin map: Proximity sensor (ผ่าน DST-1R4P-N) ----------
#define PIN_SENSOR_LEFT 37  // เซนเซอร์ซ้ายสุด (Left End-Stop)
#define PIN_SENSOR_HOME 38  // เซนเซอร์ตรงกลาง = HOME (0.00 cm)
#define PIN_SENSOR_RIGHT 39 // เซนเซอร์ขวาสุด (Right End-Stop)

// ---------- Pin map: ปุ่มหยุดฉุกเฉิน ----------
#define PIN_ESTOP -1 // TODO: ยังไม่ได้กำหนด

// ---------- Pin map: Encoder A/B (ผ่าน optocoupler PC817/EL817) ----------
#define PIN_ENCODER_A 12 // OUTA (สีขาว)
#define PIN_ENCODER_B 13 // OUTB (สีเขียว)

// ---------- ค่าจากการสอบเทียบ (บทบาท 6 ต้องวัดจากชุดจริง) ----------
#define LEAD_MM_PER_REV 4.0 // วัดจริง 23/11/2026: หมุน 10 รอบ ได้ 4 cm (ยืนยันแล้ว)
#define MICROSTEP 1         // ตั้งบน TB6600 DIP switch (ยืนยันแล้ว)
#define ENCODER_PPR 600 // HN3806-AB600N: 600 พัลส์ต่อรอบ (x4 ถ้า decode แบบ 4 เท่า)
#define ENCODER_DECODE_X4 true

// steps/mm = (200 * 1) / 4 = 50 steps/mm
// counts/mm = (600 * 4) / 4 = 600 counts/mm
// คำนวณจริงให้ทำใน calibration.cpp และยืนยันด้วยการวัดระยะจริง ห้าม hardcode เฉย ๆ

// ---------- ขีดจำกัดการเคลื่อนที่ (soft limit, นอกเหนือจาก hardware limit sensor)
// ----------
#define SOFT_LIMIT_MIN_MM -100.0
#define SOFT_LIMIT_MAX_MM 120.0

// ---------- ความเร็ว/ความเร่งเริ่มต้น ----------
#define DEFAULT_SPEED_MM_S 5.0
#define DEFAULT_ACCEL_MM_S2 20.0

// ---------- Serial ----------
#define SERIAL_BAUD 115200
