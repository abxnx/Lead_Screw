# โฟลเดอร์เก็บข้อมูลดิบจากการทดสอบ

เก็บไฟล์ CSV/ข้อมูลดิบของแต่ละการทดสอบ ตั้งชื่อไฟล์ตามรูปแบบ:

```
T1_homing_repeatability.csv
T2_position_accuracy.csv
T3_speed_accuracy.csv
T4_travel_limits.csv
T5_step_loss_detection.csv
T6_estop.csv
T7_endurance.csv
X5_logged_positions.csv   (ถ้าเลือกส่วนขยาย X5)
```

แต่ละไฟล์ควรมีคอลัมน์อย่างน้อย: เวลา/ลำดับการทดลอง, ค่าที่สั่ง, ค่าที่วัดได้จริง, หน่วยวัด
ใช้ไฟล์เหล่านี้ทำกราฟในสไลด์และรายงาน (หัวข้อ 9.2, 9.3)
