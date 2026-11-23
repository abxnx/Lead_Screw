# Lead Screw Linear Stage — Cucumber RS (ESP32-S2)

2102633 Special Problems in Electrical Engineering III (Mechatronics) — 1/2026
Department of Electrical Engineering, Faculty of Engineering, Chulalongkorn University

**กลุ่มที่:** _____  **ชื่อโครงงาน:** _______________________________________

ระบบแกนเชิงเส้นแบบ lead screw ควบคุมตำแหน่งด้วยบอร์ด Cucumber RS (ESP32-S2)
ใช้สเต็ปเปอร์มอเตอร์ NEMA17 + ไดรเวอร์ TB6600, rotary encoder ที่ปลาย lead screw
และ proximity sensor 3 จุด (home / limit ซ้าย / limit ขวา)

## สารบัญเอกสารในโฟลเดอร์นี้

| ไฟล์ | เนื้อหา |
|---|---|
| `docs/01_design_review.md` | คำตอบคำถามออกแบบ 8 ข้อ สำหรับ Design Review (M1) |
| `docs/02_test_plan.md` | แผนการทดสอบ T1–T7 + ส่วนขยาย |
| `docs/03_roles.md` | ตารางบทบาท, contribution table, log การประชุม |
| `docs/04_risk_register.md` | รายการความเสี่ยง |
| `docs/05_calibration.md` | ค่า L, steps/mm, counts/mm และผลยืนยัน |
| `include/config.h` | **Pin map และค่าคาลิเบรตตัวจริงที่ firmware ใช้** (ต้องตรงกับ Design Review เสมอ) |
| `src/` | โค้ด firmware แยกเป็นโมดูล |
| `data/` | ข้อมูลดิบจากการทดสอบ T1–T7 (CSV ฯลฯ) |

## Pin Map (สรุป — รายละเอียดและเหตุผลอยู่ใน `docs/01_design_review.md`)

| สัญญาณ | GPIO | หมายเหตุ |
|---|---|---|
| PUL | TODO | |
| DIR | TODO | |
| ENA | TODO | |
| Sensor ซ้าย (limit) | TODO | SN04-N ผ่าน DST-1R4P-N |
| Sensor กลาง (home) | TODO | SN04-N ผ่าน DST-1R4P-N |
| Sensor ขวา (limit) | TODO | SN04-N ผ่าน DST-1R4P-N |
| Encoder A | TODO | ผ่าน PC817/EL817, ต้องรองรับ PCNT |
| Encoder B | TODO | ผ่าน PC817/EL817, ต้องรองรับ PCNT |
| E-Stop | TODO | ต่อฮาร์ดแวร์โดยตรง |

ห้ามใช้ (ตามข้อ 2.3 ของเอกสารโครงงาน): GPIO0, GPIO45, GPIO46 (strapping),
GPIO19/20 (USB D-/D+), GPIO26 (flash CS), GPIO18 (RGB LED บนบอร์ด),
GPIO40/41 (I2C เซนเซอร์บนบอร์ด) — ใช้ได้เฉพาะเมื่อเข้าใจผลกระทบแล้ว

## ค่าคาลิเบรตที่ใช้จริง

- L (lead, mm/รอบ): `4` — ดู `docs/05_calibration.md`
- Microstep (TB6600): `1`
- steps/mm: `50`
- counts/mm: `600`

## วิธี Build / Upload

โปรเจกต์นี้ตั้งเป็น [PlatformIO](https://platformio.org/) project (framework = Arduino)

```bash
# ติดตั้ง PlatformIO (ถ้ายังไม่มี) ผ่าน VS Code extension หรือ pip
pip install platformio

# build
pio run

# upload (ต่อ USB กับ Cucumber RS ก่อน และปิดการจ่ายไฟจาก power module เข้าขา 5V ถ้ายังไม่แน่ใจเรื่องไดโอดกันไฟย้อน)
pio run -t upload

# เปิด serial monitor
pio device monitor -b 115200
```

ถ้าใช้ Arduino IDE แทน: เลือกบอร์ด ESP32S2 ที่ตรงกับ Cucumber RS, คัดลอกไฟล์ใน `src/` และ `include/`
มารวมกันเป็น sketch เดียว (หรือปรับ `#include` path ให้เข้ากับโครงสร้าง Arduino sketch)

## ชุดคำสั่ง Serial (R6)

ส่งทาง Serial Monitor ที่ baud 115200, จบบรรทัดด้วย newline

| คำสั่ง | ความหมาย |
|---|---|
| `home` | เริ่ม homing |
| `move <mm>` | สั่งไปตำแหน่ง `<mm>` เช่น `move 50.0` |
| `speed <mm/s>` | ตั้งความเร็ว เช่น `speed 10.0` |
| `stop` | หยุดทันที |
| `status` | รายงานสถานะปัจจุบัน |

## โครงสร้าง state machine (สรุป)

`IDLE → HOMING → READY → MOVING → (กลับ READY หรือเข้า FAULT)`
`FAULT` ต้อง home ใหม่ก่อนจึงจะกลับไปทำงานต่อได้ (ตาม R7)
รายละเอียดเต็ม + แผนภาพ: ดู `docs/01_design_review.md` ข้อ 7

## ไลบรารีและโค้ดจากแหล่งภายนอก

ระบุทุกไลบรารีที่ใช้จริงที่นี่ พร้อมแหล่งที่มา (ข้อกำหนดบังคับของวิชา — ต้องอธิบายได้ว่าทำงานอย่างไร)

| ไลบรารี/โค้ด | เวอร์ชัน | แหล่งที่มา | ใช้ทำอะไร |
|---|---|---|---|
| | | | |

## แนวทาง Commit / Git

- ทุกคนต้องมี commit ในส่วนที่ตนรับผิดชอบ (ข้อกำหนดการให้คะแนน "คุณภาพโค้ดและ repository")
- ใช้ branch แยกตามโมดูล/บทบาท แล้ว merge เข้า `main` ผ่าน pull request ให้เพื่อนในทีม review
- Tag เวอร์ชันที่ใช้สาธิตจริง: `git tag m2-demo` ก่อน Prototype Demo และ `git tag m3-final` ก่อน Final Presentation
- ก่อนส่งงาน: `git archive` หรือ zip ทั้ง repo (รวมโฟลเดอร์ `.git`) ตามหัวข้อ 9.1 ของเอกสารโครงงาน

```bash
git log --oneline --graph --all   # ตรวจดูว่า commit ของทุกคนขึ้นครบ
```

## Checklist ความปลอดภัยก่อนจ่ายไฟ 12V (สรุปจากหัวข้อ 6)

- [ ] ผู้สอนตรวจผังต่อสายและการต่อจริงใน Design Review แล้ว
- [ ] วัดแรงดันขาเอาต์พุตของ power module ด้วยมัลติมิเตอร์ก่อนต่อเข้าบอร์ดทุกครั้ง
- [ ] ไม่ต่อขา 12V ของ power module เข้า Cucumber RS โดยตรง
- [ ] ตั้งกระแสบน TB6600 ไม่เกินพิกัดของมอเตอร์ (1.2 A)
- [ ] มีคนพร้อมกดปุ่มหยุดฉุกเฉินก่อนทดสอบทุกครั้ง โดยเฉพาะโปรแกรมใหม่ที่ยังไม่เคยรัน
- [ ] ยึดแกนกับโต๊ะมั่นคง ตรวจสกรูคัปปลิ้งทั้งสองฝั่ง
