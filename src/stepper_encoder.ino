/*
  Stepper (TB6600) + Encoder HN3806-600N  —  ESP32-S2
  ----------------------------------------------------
  Serial Monitor: 115200 baud, line ending = "Newline"

  คำสั่ง:
    <ตัวเลข>   หมุน stepper ตามจำนวน pulse เช่น 1000 หรือ -1000
    H          Homing → เลื่อนไปจน PX2 ON แล้วตั้งตำแหน่งเป็น 0
    Z          รีเซ็ตตำแหน่ง stepper และ encoder เป็น 0 (ตรงจุดที่อยู่ตอนนี้)
    S          แสดงสถานะ (steps / encoder / ระยะ cm / steps ต่อ cm / sensor)

  ลำดับ sensor บนแกน:   PX1 (DIR=LOW) ---- PX2 ---- PX3 (DIR=HIGH)

  ถ้าหมุนด้วยมือ (ไม่ได้สั่ง stepper) จะแสดงค่า encoder ทุก 100 ms เมื่อมีการเปลี่ยนแปลง
*/

// ======================= Stepper =======================
#define PX1 37   // limit ฝั่ง DIR = LOW
#define PX2 38   // จุด home (อยู่ระหว่าง PX1 กับ PX3)
#define PX3 39   // limit ฝั่ง DIR = HIGH
#define PUL 10
#define DIR 11

const int  LIMIT_ACTIVE  = HIGH;   // ระดับสัญญาณตอน sensor ทำงาน — เช็คของจริงด้วย!
const int  STEP_HALF_US  = 500;    // ความเร็วปกติ (~1000 steps/s)
const int  SLOW_HALF_US  = 2000;   // ความเร็วช้าตอนเข้าหา PX2 รอบสุดท้าย (~250 steps/s)
const long BACKOFF_STEPS = 200;
const long MAX_TRAVEL    = 300000; // กันวิ่งไม่หยุดถ้า sensor เสีย

// Homing: ทิศสุดท้ายที่เข้าหา PX2 (ให้เข้าทางเดียวกันทุกครั้ง จุด 0 จะตรงเดิม)
//   true  = เข้าจากฝั่ง PX1 วิ่งไปทาง PX3 (DIR = HIGH)
//   false = เข้าจากฝั่ง PX3 วิ่งไปทาง PX1 (DIR = LOW)
const bool HOME_APPROACH_POSITIVE = true;
const bool HOME_ON_BOOT           = false;   // true = homing อัตโนมัติตอนเปิดเครื่อง

long position = 0;                 // ตำแหน่ง stepper (steps)
bool homed    = false;

// ======================= Encoder =======================
const int encPinA = 12;            // สายสีขาว (OUTA)
const int encPinB = 13;            // สายสีเขียว (OUTB)

// 600 PPR นับขอบขาขึ้นของ A + B = 1,200 counts/รอบ
const float COUNTS_PER_REV  = 1200.0;
const float CM_PER_TWO_REVS = 0.9;
const float COUNTS_PER_CM   = (COUNTS_PER_REV * 2.0) / CM_PER_TWO_REVS;   // 2666.67 counts/cm

// ถ้า stepper หมุนทิศบวกแต่ encoder นับเป็นลบ ให้เปลี่ยนเป็น -1
const int ENC_SIGN = 1;

volatile long counter = 0;
long lastCounter = 0;
enum StopReason { DONE, HIT_SENSOR, HIT_LIMIT };

#if defined(ESP32)
  #define ISR_ATTR IRAM_ATTR
#else
  #define ISR_ATTR
#endif

void ISR_ATTR onPulseA() {         // A ขึ้น → ดู B
  if (digitalRead(encPinB) == LOW) counter++;
  else                             counter--;
}

void ISR_ATTR onPulseB() {         // B ขึ้น → ดู A
  if (digitalRead(encPinA) == LOW) counter--;
  else                             counter++;
}

long getCounter() {
  noInterrupts();
  long val = counter;
  interrupts();
  return val * ENC_SIGN;
}

void resetCounter() {
  noInterrupts();
  counter = 0;
  interrupts();
  lastCounter = 0;
}

// ======================= Stepper functions =======================


bool isActive(int pin) {
  return digitalRead(pin) == LIMIT_ACTIVE;
}

void pulse(int halfUs) {
  digitalWrite(PUL, HIGH);
  delayMicroseconds(halfUs);
  digitalWrite(PUL, LOW);
  delayMicroseconds(halfUs);
}

void setDir(bool positive) {
  digitalWrite(DIR, positive ? HIGH : LOW);
  delayMicroseconds(10);           // TB6600: DIR ต้องตั้งก่อน PUL > 5 µs
}

bool limitHit(bool positive) {     // เช็คเฉพาะ limit ฝั่งที่กำลังวิ่งไป
  return isActive(positive ? PX3 : PX1);
}

// หมุนไปทิศ positive จนกว่า sensorPin จะเป็นสถานะ wantActive
// หยุดก่อนถ้าชน limit ฝั่งที่วิ่งไป หรือครบ maxSteps
StopReason moveUntil(bool positive, int sensorPin, bool wantActive, long maxSteps, int halfUs) {
  setDir(positive);
  for (long i = 0; i < maxSteps; i++) {
    if (isActive(sensorPin) == wantActive) return HIT_SENSOR;
    if (limitHit(positive))                return HIT_LIMIT;
    pulse(halfUs);
    position += positive ? 1 : -1;
    if (i % 1000 == 999) delay(1); // กัน watchdog ของ ESP32-S2
  }
  return (isActive(sensorPin) == wantActive) ? HIT_SENSOR : DONE;
}

// สั่งหมุนตามจำนวน step (ถ้าชน limit จะถอยออก BACKOFF_STEPS)
void moveSteps(long steps) {
  bool positive = steps > 0;
  long n = labs(steps);
  setDir(positive);

  for (long i = 0; i < n; i++) {
    if (limitHit(positive)) {
      Serial.println(positive ? "FAULT: PX3 limit" : "FAULT: PX1 limit");
      setDir(!positive);
      for (long j = 0; j < BACKOFF_STEPS; j++) {
        pulse(STEP_HALF_US);
        position += positive ? -1 : 1;
      }
      return;
    }
    pulse(STEP_HALF_US);
    position += positive ? 1 : -1;
    if (i % 1000 == 999) delay(1);
  }
}

// ======================= Homing to PX2 =======================
bool homePX2() {
  Serial.println("Homing to PX2...");
  homed = false;
  const bool toward = HOME_APPROACH_POSITIVE;   // ทิศเข้าหา PX2 รอบสุดท้าย
  StopReason r;

  // 1) พาไปอยู่ฝั่ง "ก่อนถึง" PX2 (ฝั่งตรงข้ามกับทิศเข้าหา)
  if (isActive(PX2)) {
    // อยู่บน PX2 แล้ว → ถอยออกจน PX2 ดับ
    r = moveUntil(!toward, PX2, false, MAX_TRAVEL, STEP_HALF_US);
    if (r != HIT_SENSOR) { Serial.println("ERROR: ถอยออกจาก PX2 ไม่ได้"); return false; }
  } else {
    // ไม่รู้ว่าอยู่ฝั่งไหน → วิ่งไปทางฝั่งถอยก่อน
    r = moveUntil(!toward, PX2, true, MAX_TRAVEL, STEP_HALF_US);
    if (r == HIT_SENSOR) {
      // เจอ PX2 ระหว่างทาง = เดิมอยู่อีกฝั่ง → วิ่งต่อจนผ่าน PX2 ไป
      r = moveUntil(!toward, PX2, false, MAX_TRAVEL, STEP_HALF_US);
      if (r != HIT_SENSOR) { Serial.println("ERROR: ผ่าน PX2 ไม่ได้"); return false; }
    } else if (r == DONE) {
      Serial.println("ERROR: ไม่เจอ PX2 หรือ limit (เช็ค sensor / สาย)");
      return false;
    }
    // r == HIT_LIMIT → ชน limit ฝั่งถอย แปลว่า PX2 อยู่ข้างหน้า อยู่ถูกฝั่งแล้ว
  }

  // 2) วิ่งเร็วเข้าหา PX2
  r = moveUntil(toward, PX2, true, MAX_TRAVEL, STEP_HALF_US);
  if (r != HIT_SENSOR) {
    Serial.println("ERROR: ไม่เจอ PX2 (ชน limit อีกฝั่ง) — เช็ค PX2 หรือทิศ DIR");
    return false;
  }

  // 3) ถอยออกจน PX2 ดับ แล้วถอยเพิ่มอีก BACKOFF_STEPS
  r = moveUntil(!toward, PX2, false, MAX_TRAVEL, STEP_HALF_US);
  if (r != HIT_SENSOR) { Serial.println("ERROR: PX2 ไม่ดับตอนถอย"); return false; }
  moveUntil(!toward, PX2, true, BACKOFF_STEPS, STEP_HALF_US);   // ถอยเพิ่ม (หยุดเองถ้าชน limit)

  // 4) เข้าหาช้า ๆ เพื่อให้จุด 0 แม่นและตรงเดิมทุกครั้ง
  r = moveUntil(toward, PX2, true, BACKOFF_STEPS * 3, SLOW_HALF_US);
  for (int i=0; i<72; i++){
    pulse(STEP_HALF_US);
  }
  if (r != HIT_SENSOR) { Serial.println("ERROR: หา PX2 ไม่เจอตอนเข้าช้า"); return false; }

  // 5) ตั้งจุด 0
  position = 0;
  resetCounter();
  homed = true;
  Serial.println("Homed at PX2 → stepper = 0, encoder = 0");
  return true;
}

// ======================= Printing =======================
void printEncoder(long current) {
  Serial.print("Pulses: ");
  Serial.print(current);
  Serial.print(" | รอบ: ");
  Serial.print(current / COUNTS_PER_REV, 2);
  Serial.print(" rev | ระยะทาง: ");
  Serial.print(current / COUNTS_PER_CM, 2);
  Serial.println(" cm");
}

void printSensors() {
  Serial.print("  Sensors: PX1=");
  Serial.print(isActive(PX1) ? "ON" : "off");
  Serial.print("  PX2=");
  Serial.print(isActive(PX2) ? "ON" : "off");
  Serial.print("  PX3=");
  Serial.println(isActive(PX3) ? "ON" : "off");
}

void printStatus() {
  long current = getCounter();
  float distanceCm = current / COUNTS_PER_CM;

  Serial.print(homed ? "[homed] " : "[not homed] ");
  Serial.print("Stepper: ");
  Serial.print(position);
  Serial.print(" steps | ");
  printEncoder(current);

  // steps ต่อ cm จริง (วัดจาก encoder) — ใช้เช็ค/คาลิเบรต
  if (fabs(distanceCm) > 0.01) {
    Serial.print("  -> ");
    Serial.print(position / distanceCm, 2);
    Serial.println(" steps/cm");
  }
  printSensors();
  lastCounter = current;           // กันพิมพ์ซ้ำใน loop
}

// ======================= Setup / Loop =======================
void setup() {
  pinMode(PX1, INPUT);
  pinMode(PX2, INPUT);
  pinMode(PX3, INPUT);
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(PUL, LOW);
  digitalWrite(DIR, LOW);

  // ถ้าต่อผ่าน opto หรือหมุนเร็ว ควรใส่ pull-up ภายนอก 2.2k–4.7k ไป 3.3V ด้วย
  pinMode(encPinA, INPUT_PULLUP);
  pinMode(encPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encPinA), onPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(encPinB), onPulseB, RISING);

  Serial.begin(115200);
  Serial.setTimeout(50);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}   // รอ USB CDC ของ ESP32-S2
  delay(200);

  Serial.println("serial begin");
  Serial.println("คำสั่ง: ตัวเลข = หมุน (เช่น 1000, -1000) | H = homing ไป PX2 | Z = รีเซ็ต 0 | S = สถานะ");
  printSensors();

  if (HOME_ON_BOOT) {
    homePX2();
    printStatus();
  }
}

void loop() {
  // ----- รับคำสั่งจาก Serial -----
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;          // ไม่สนใจการกด Enter เปล่า

    char c = toupper(line.charAt(0));

    if (c == 'H') {
      homePX2();
      printStatus();
    }
    else if (c == 'Z') {
      position = 0;
      resetCounter();
      Serial.println("[RESET] stepper = 0 steps, encoder = 0 (0.00 cm)");
    }
    else if (c == 'S') {
      printStatus();
    }
    else {
      float cm = line.toFloat();  
      long steps = (cm / 0.4) * 200;             // "100" → 100, "-250" → -250
      if (steps == 0) {
        Serial.println("คำสั่งไม่ถูกต้อง: พิมพ์ตัวเลข, H, Z หรือ S");
        return;
      }
      Serial.print("Moving ");
      Serial.print(steps);
      Serial.println(" pulses");
      moveSteps(steps);
      printStatus();
    }
  }

  // ----- แสดงค่า encoder ทุก 100 ms เมื่อมีการหมุน (เช่น หมุนด้วยมือ) -----
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    long current = getCounter();
    if (current != lastCounter) {
      printEncoder(current);
      lastCounter = current;
    }
  }
}
