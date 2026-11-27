/*
  Stepper (TB6600) + Encoder HN3806-600N  —  ESP32-S2
  ----------------------------------------------------
  Serial Monitor: 115200 baud, line ending = "Newline"

  คำสั่ง:
    <number>   Move by distance in cm, e.g. 1.5 or -0.8  (cm → steps → show encoder count)
    P<number>  Move by raw steps/pulses, e.g. P200 or P-200
    L          Toggle per-step log on/off (default on)
    O          Zero calibration: after H, jog to the true 0, send O → prints new HOME_OFFSET_STEPS
    H          Homing → เลื่อนไปจน PX2 ON แล้วตั้งตำแหน่งเป็น 0
    Z          รีเซ็ตตำแหน่ง stepper และ encoder เป็น 0 (ตรงจุดที่อยู่ตอนนี้)
    S          แสดงสถานะ (steps / encoder count / count ที่ควรได้ / ส่วนต่าง / sensor)
    W          บันทึกค่า encoder + ตำแหน่ง stepper ลง flash ทันที
    R          แสดงค่าที่บันทึกไว้ใน flash
    C          ลบค่าที่บันทึกไว้ใน flash
    T          ทดสอบ: หมุน +1 รอบมอเตอร์ แล้ว −1 รอบ แสดง count ที่ได้เทียบกับที่ควรได้
    E          ตรวจสาย encoder: จำนวนขอบสัญญาณของ A และ B แยกกัน + ระดับสัญญาณตอนนี้

  Per-step log: during every cm / P move, the step position and encoder count are
  recorded at every step, then printed one line per step (absolute from home / Z):
    ตำแหน่ง : _ cm || pos_step : _ | expected_step : _ | err_step : _  ||  pos_count : _ || expected_count : _ ||  err_count : _
  The last line is marked [จุดสุดท้าย]. Manual rotation uses the same line format.
  plus a summary (stalled steps, min/max count per step).

  ลำดับ sensor บนแกน:   PX1 (DIR=LOW) ---- PX2 ---- PX3 (DIR=HIGH)

  ถ้าหมุนด้วยมือ (ไม่ได้สั่ง stepper) จะแสดงค่า encoder ทุก 100 ms เมื่อมีการเปลี่ยนแปลง

  การเก็บค่า encoder (Preferences / NVS ของ ESP32):
    - บันทึกอัตโนมัติหลังหมุนเสร็จ, หลัง Homing, หลัง Z และเมื่อหมุนด้วยมือแล้วหยุดนิ่ง 1 วินาที
    - เปิดเครื่องใหม่จะโหลดค่าเดิมกลับมาให้เอง
    - ไม่บันทึกระหว่างกำลังหมุน เพื่อถนอม flash
*/

#include <Preferences.h>

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

// ===== CALIBRATE ZERO HERE =====
// Steps moved after PX2 turns ON before setting 0 (1 step = 0.02 mm at 200 steps/rev, 4 mm/rev)
//   larger  = 0 moves further in the approach direction (toward PX3 when HOME_APPROACH_POSITIVE = true)
//   smaller / negative = 0 moves back toward PX1
// Use command O to get the suggested value (see below).
const long HOME_OFFSET_STEPS = 272;

long position = 0;                 // ตำแหน่ง stepper (steps)
bool homed    = false;

// ======================= Encoder =======================
const int encPinA = 12;            // สายสีขาว (OUTA)
const int encPinB = 13;            // สายสีเขียว (OUTB)

// ======================= Stepper / TB6600 =======================
// TB6600 set to 200 pulse/rev (full step) on the DIP switch
const long STEPS_PER_REV = 800;

// Lead screw travel per revolution (from stepper_encoder_1: 200 steps = 0.4 cm)
const float LEAD_MM = 4.0;

// 600 PPR, full quadrature (every edge of A and B) = 2,400 counts/rev
const float COUNTS_PER_REV = 2400.0;

const float STEPS_PER_MM  = STEPS_PER_REV / LEAD_MM;    // 50 steps/mm  = 500 steps/cm
const float COUNTS_PER_MM = COUNTS_PER_REV / LEAD_MM;   // 600 count/mm = 6000 count/cm
const float MM_PER_COUNT  = 1.0 / COUNTS_PER_MM;        // 0.001667 mm/count

// ถ้า stepper หมุนทิศบวกแต่ encoder นับเป็นลบ ให้เปลี่ยนเป็น -1
const int ENC_SIGN = 1;   // log: stepper +400 แต่ encoder ได้ -728 → กลับเครื่องหมาย

volatile long counter = 0;
long lastCounter = 0;
enum StopReason { DONE, HIT_SENSOR, HIT_LIMIT };

#if defined(ESP32)
  #define ISR_ATTR IRAM_ATTR
#else
  #define ISR_ATTR
#endif

#if defined(ESP32)
  #define ENC_TABLE_ATTR DRAM_ATTR
#else
  #define ENC_TABLE_ATTR
#endif

// ตาราง quadrature: index = (สถานะ AB เดิม << 2) | สถานะ AB ใหม่
// ถ้าสัญญาณเด้ง (ขึ้น-ลง-ขึ้น) จะได้ +1 −1 +1 หักล้างกันเอง ไม่นับเกิน
ENC_TABLE_ATTR const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};
volatile uint8_t encState = 0;

volatile unsigned long edgesA = 0;   // ใช้ตรวจสาย (คำสั่ง E)
volatile unsigned long edgesB = 0;

void ISR_ATTR decodeQuad() {
  uint8_t ab = (digitalRead(encPinA) << 1) | digitalRead(encPinB);
  encState = ((encState << 2) | ab) & 0x0F;
  counter += QUAD_TABLE[encState];
}

void ISR_ATTR onEncoderA() { edgesA++; decodeQuad(); }   // A เปลี่ยน
void ISR_ATTR onEncoderB() { edgesB++; decodeQuad(); }   // B เปลี่ยน

// แสดงจำนวนขอบของแต่ละสาย แล้วรีเซ็ตเพื่อทดสอบรอบถัดไป
// หมุน 1 รอบ ควรได้ A ≈ 1200 และ B ≈ 1200 (600 PPR × ขึ้น+ลง)
void printEncoderWiring() {
  noInterrupts();
  unsigned long a = edgesA, b = edgesB;
  edgesA = 0; edgesB = 0;
  interrupts();
  Serial.print("[ENC] edges A = ");
  Serial.print(a);
  Serial.print(" | edges B = ");
  Serial.print(b);
  Serial.print(" | ระดับตอนนี้ A=");
  Serial.print(digitalRead(encPinA));
  Serial.print(" B=");
  Serial.println(digitalRead(encPinB));
  if (a > 20 && b < a / 10) Serial.println("  → สาย B (เขียว) ไม่มีสัญญาณ: เช็คสาย/ขา 13/ไฟเลี้ยง encoder");
  else if (b > 20 && a < b / 10) Serial.println("  → สาย A (ขาว) ไม่มีสัญญาณ: เช็คสาย/ขา 12/ไฟเลี้ยง encoder");
  else if (a == 0 && b == 0) Serial.println("  → ไม่มีสัญญาณทั้ง 2 สาย: แกนไม่ได้หมุน หรือ encoder ไม่ได้ไฟเลี้ยง");
}

long getCounter() {
  noInterrupts();
  long val = counter;
  interrupts();
  return val * ENC_SIGN;
}

// count ที่ควรได้จากจำนวน step (200 steps/rev → 2400 count/rev = 12 count/step)
long stepsToCount(long steps) {
  return lround(steps * COUNTS_PER_REV / STEPS_PER_REV);
}

// ----- ค่าที่ return จาก encoder -----
long getEncoderCount() {           // จำนวน count (ใส่ ENC_SIGN แล้ว)
  return getCounter();
}

float countToMm(long count) {      // แปลง count → mm
  return count * MM_PER_COUNT;
}

float getEncoderMm() {             // ระยะจาก encoder หน่วย mm
  return countToMm(getEncoderCount());
}

void resetCounter() {
  noInterrupts();
  counter = 0;
  interrupts();
  lastCounter = 0;
}

// ======================= เก็บค่า encoder ลง flash =======================
Preferences prefs;

const unsigned long IDLE_SAVE_MS = 1000;   // หมุนด้วยมือแล้วหยุดนิ่งนานเท่านี้ค่อยบันทึก

long savedCounter  = 0;            // ค่า counter ดิบ (ยังไม่คูณ ENC_SIGN) ที่อยู่ใน flash
long savedPosition = 0;
bool savedHomed    = false;
bool pendingSave   = false;        // มีการเปลี่ยนแปลงที่ยังไม่ได้บันทึก
unsigned long lastChangeMs = 0;

long getCounterRaw() {
  noInterrupts();
  long val = counter;
  interrupts();
  return val;
}

void setCounterRaw(long val) {
  noInterrupts();
  counter = val;
  interrupts();
}

// force = true → เขียนแม้ค่าไม่เปลี่ยน
void saveState(bool force) {
  long raw = getCounterRaw();
  pendingSave = false;
  if (!force && raw == savedCounter && position == savedPosition && homed == savedHomed) return;

  prefs.putLong("enc", raw);
  prefs.putLong("pos", position);
  prefs.putBool("homed", homed);
  savedCounter  = raw;
  savedPosition = position;
  savedHomed    = homed;

  Serial.print("[SAVED] encoder = ");
  Serial.print(raw * ENC_SIGN);
  Serial.print(" | stepper = ");
  Serial.print(position);
  Serial.println(" steps");
}

void loadState() {
  if (!prefs.isKey("enc")) {
    Serial.println("[LOAD] ยังไม่มีค่าที่บันทึกไว้ → เริ่มที่ 0");
    return;
  }
  savedCounter  = prefs.getLong("enc", 0);
  savedPosition = prefs.getLong("pos", 0);
  savedHomed    = prefs.getBool("homed", false);

  setCounterRaw(savedCounter);
  position    = savedPosition;
  homed       = savedHomed;
  lastCounter = savedCounter * ENC_SIGN;

  Serial.print("[LOAD] encoder = ");
  Serial.print(savedCounter * ENC_SIGN);
  Serial.print(" count | stepper = ");
  Serial.print(savedPosition);
  Serial.println(" steps");
  Serial.println("       ถ้าแกนถูกขยับตอนปิดเครื่อง ค่านี้จะไม่ตรง → สั่ง H ใหม่");
}

void printSaved() {
  if (!prefs.isKey("enc")) {
    Serial.println("[FLASH] ไม่มีค่าที่บันทึกไว้");
    return;
  }
  long enc = prefs.getLong("enc", 0) * ENC_SIGN;
  Serial.print("[FLASH] encoder = ");
  Serial.print(enc);
  Serial.print(" count | stepper = ");
  Serial.print(prefs.getLong("pos", 0));
  Serial.print(" steps | ");
  Serial.println(prefs.getBool("homed", false) ? "homed" : "not homed");
}

void clearSaved() {
  prefs.clear();
  savedCounter  = 0;
  savedPosition = 0;
  savedHomed    = false;
  pendingSave   = false;
  Serial.println("[FLASH] ลบค่าที่บันทึกไว้แล้ว (ค่าปัจจุบันใน RAM ยังอยู่)");
}

// ======================= Stepper functions =======================

// รอให้ encoder นิ่ง (ค่าไม่เปลี่ยน STABLE_MS) ก่อนอ่านค่า หลัง stepper หยุด
// เพราะแกน/คัปปลิ้งยังสั่นหรือไหลต่ออีกนิดหลังหยุดส่ง pulse
const unsigned long SETTLE_STABLE_MS  = 50;
const unsigned long SETTLE_TIMEOUT_MS = 500;

void waitEncoderSettle() {
  unsigned long t0 = millis();
  unsigned long lastChange = millis();
  long last = getCounterRaw();
  while (millis() - t0 < SETTLE_TIMEOUT_MS) {
    long now = getCounterRaw();
    if (now != last) { last = now; lastChange = millis(); }
    if (millis() - lastChange >= SETTLE_STABLE_MS) return;
    delay(1);
  }
}


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

// ======================= Per-step log =======================
// Samples are stored in RAM during the move (printing while stepping would
// slow the motor and distort the timing), then printed after the move.
struct LogSample { int32_t step; int32_t count; };
const int  LOG_MAX = 8000;          // 8000 × 8 bytes = 64 KB
LogSample  logBuf[LOG_MAX];
int  logLen        = 0;
long logEvery      = 1;             // 1 = every step; larger moves are thinned so they fit
long logStepCount  = 0;
long logStartPos   = 0;
long logStartCount = 0;
bool logActive     = false;         // true only during a logged move
bool logEnabled    = true;          // toggled with L

void logAdd() {
  if (logLen >= LOG_MAX) return;
  logBuf[logLen].step  = position;            // absolute step position (0 = home / Z)
  logBuf[logLen].count = getEncoderCount();    // absolute encoder count (0 = home / Z)
  logLen++;
}

void logBegin(long nSteps) {
  logLen        = 0;
  logStepCount  = 0;
  logEvery      = (nSteps + LOG_MAX - 3) / (LOG_MAX - 2);   // leave room for first + settled samples
  if (logEvery < 1) logEvery = 1;
  logStartPos   = position;
  logStartCount = getEncoderCount();
  logActive     = logEnabled;
  if (logActive) logAdd();          // step 0, count 0
}

void logStep() {                    // called by moveSteps after every pulse
  if (!logActive) return;
  if (++logStepCount % logEvery == 0) logAdd();
}

// ===== Report line (used on every change and at the final point) =====
// ตำแหน่ง        = real position from encoder (cm)
// expected_step  = steps that match that encoder position  (pos_count / 12)
// expected_count = count that matches the steps sent        (pos_step × 12)
// err_*          = pos − expected
void printReport(long posStep, long posCount) {
  float cm        = countToMm(posCount) / 10.0;
  long  expStep   = lround(posCount * (float)STEPS_PER_REV / COUNTS_PER_REV);
  long  expCount  = stepsToCount(posStep);

  Serial.print("ตำแหน่ง : ");          Serial.print(cm, 4);
  Serial.print(" cm || pos_step : ");    Serial.print(posStep);
  Serial.print(" | expected_step : ");   Serial.print(expStep);
  Serial.print(" | err_step : ");        Serial.print(posStep - expStep);
  Serial.print("  ||  pos_count : ");    Serial.print(posCount);
  Serial.print(" || expected_count : "); Serial.print(expCount);
  Serial.print(" ||  err_count : ");     Serial.println(posCount - expCount);
}

void logFinish() {
  if (!logActive) return;
  logActive = false;
  logAdd();                         // last sample = after encoder settled

  const float expPerSample = (COUNTS_PER_REV / STEPS_PER_REV) * logEvery;

  Serial.println();
  Serial.print("----- Step log (every ");
  Serial.print(logEvery);
  Serial.print(" step");
  Serial.print(logEvery > 1 ? "s" : "");
  Serial.print(", expected d_count = ");
  Serial.print(expPerSample, 1);
  Serial.println(") -----");

  long minD = 0x7FFFFFFF, maxD = -0x7FFFFFFF;
  long stalled = 0;
  for (int i = 0; i < logLen; i++) {
    long  st    = logBuf[i].step;
    long  ct    = logBuf[i].count;
    long  d     = (i == 0) ? 0 : ct - logBuf[i - 1].count;

    if (i == logLen - 1) Serial.print("[จุดสุดท้าย] ");
    printReport(st, ct);

    // per-step stats (skip first row and the settled row)
    if (i > 0 && i < logLen - 1) {
      if (d < minD) minD = d;
      if (d > maxD) maxD = d;
      if (d == 0)   stalled++;
    }
    if (i % 200 == 199) delay(1);   // let USB catch up / feed watchdog
  }

  Serial.println("----- Summary -----");
  if (logLen > 2) {
    Serial.print("d_count per sample: min ");
    Serial.print(minD);
    Serial.print(" | max ");
    Serial.print(maxD);
    Serial.print(" | expected ");
    Serial.println(expPerSample, 1);
    Serial.print("Samples with no encoder change: ");
    Serial.print(stalled);
    Serial.print(" of ");
    Serial.println(logLen - 2);
  }
  Serial.println("-------------------");
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
        logStep();
      }
      return;
    }
    pulse(STEP_HALF_US);
    position += positive ? 1 : -1;
    logStep();
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
  if (r != HIT_SENSOR) { Serial.println("ERROR: หา PX2 ไม่เจอตอนเข้าช้า"); return false; }

  // 4.1) offset from PX2 trigger point to the real 0 (HOME_OFFSET_STEPS)
  if (HOME_OFFSET_STEPS != 0) {
    bool dir = (HOME_OFFSET_STEPS > 0) ? toward : !toward;
    setDir(dir);
    for (long i = 0; i < labs(HOME_OFFSET_STEPS); i++) {
      if (limitHit(dir)) { Serial.println("ERROR: limit hit during home offset"); return false; }
      pulse(SLOW_HALF_US);
    }
  }

  // 5) ตั้งจุด 0 (รอแกนนิ่งก่อน ไม่งั้น encoder จะนับต่อหลัง reset)
  waitEncoderSettle();
  position = 0;
  resetCounter();
  homed = true;
  Serial.println("Homed at PX2 → stepper = 0, encoder = 0");
  saveState(true);
  return true;
}

// ======================= ทดสอบ 1 รอบมอเตอร์ =======================
void testMove(long steps) {
  long c0 = getEncoderCount();
  moveSteps(steps);
  waitEncoderSettle();
  long d = getEncoderCount() - c0;
  float expected = COUNTS_PER_REV * steps / STEPS_PER_REV;

  Serial.print("  ");
  Serial.print(steps > 0 ? "+" : "");
  Serial.print(steps);
  Serial.print(" steps → encoder ");
  Serial.print(d);
  Serial.print(" count (ควรได้ ");
  Serial.print(expected, 0);
  Serial.print(") อัตราส่วน = ");
  Serial.println(d / expected, 3);
}

void testOneRev() {
  Serial.print("[TEST] หมุน 1 รอบมอเตอร์ = ");
  Serial.print(STEPS_PER_REV);
  Serial.println(" steps");
  long c0 = getEncoderCount();
  testMove(STEPS_PER_REV);
  testMove(-STEPS_PER_REV);
  Serial.print("  กลับมาคลาดจากจุดเริ่ม ");
  Serial.print(getEncoderCount() - c0);
  Serial.println(" count");
  Serial.println("  อัตราส่วน ≈ 1.000 = ตั้งค่าถูก | ≈ 0.5 หรือ 2 = pulse/rev ไม่ตรง DIP | ค่าแปลก ๆ = มีทดรอบหรือสัญญาณรบกวน");
  printStatus();
  saveState(false);
}

// ======================= Move by cm =======================
long cmToSteps(float cm) {
  return lround(cm * 10.0 * STEPS_PER_MM);       // cm → mm → steps (rounded)
}

void moveCm(float cm) {
  long steps = cmToSteps(cm);
  if (steps == 0) {
    Serial.println("Distance too small (less than 1 step = 0.002 cm)");
    return;
  }
  long expectedCount = lround(steps * COUNTS_PER_REV / STEPS_PER_REV);

  Serial.print("Target: ");
  Serial.print(cm, 4);
  Serial.print(" cm → ");
  Serial.print(steps);
  Serial.print(" steps (expected ");
  Serial.print(expectedCount);
  Serial.println(" count)");

  long c0 = getEncoderCount();
  long p0 = position;
  logBegin(labs(steps));
  moveSteps(steps);
  waitEncoderSettle();
  logFinish();
  long dCount = getEncoderCount() - c0;
  long dSteps = position - p0;                  // differs from target if a limit was hit

  long expCount = stepsToCount(dSteps);
  Serial.print("Result: ");
  Serial.print(dSteps);
  Serial.print(" steps | Count: ");
  Serial.print(dCount);
  Serial.print(" | Expected: ");
  Serial.print(expCount);
  Serial.print(" | Error: ");
  Serial.print(dCount - expCount);
  Serial.println(" count");

  printStatus();
  saveState(false);
}

// ======================= Printing =======================
void printEncoder(long current) {     // manual rotation: report on every change
  printReport(position, current);
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
  long current  = getEncoderCount();
  long expected = stepsToCount(position);

  (void)expected;
  Serial.print(homed ? "[homed] " : "[not homed] ");
  printReport(position, current);

  // count ต่อ step จริง (ควรได้ 12) — ใช้เช็ค/คาลิเบรต
  if (position != 0) {
    Serial.print("  -> ");
    Serial.print((float)current / position, 2);
    Serial.println(" count/step");
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
  encState = (digitalRead(encPinA) << 1) | digitalRead(encPinB);
  attachInterrupt(digitalPinToInterrupt(encPinA), onEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encPinB), onEncoderB, CHANGE);

  Serial.begin(115200);
  Serial.setTimeout(50);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}   // รอ USB CDC ของ ESP32-S2
  delay(200);

  Serial.println("serial begin");
  Serial.println("Commands: <cm> = move (e.g. 1.5, -0.8) | P<steps> = raw steps | H = home | Z = zero | S = status");
  Serial.println("        W = บันทึกค่า | R = ดูค่าที่บันทึก | C = ลบค่าที่บันทึก | T = ทดสอบ 1 รอบ");

  prefs.begin("stepenc2", false);  // namespace ใหม่ เพราะสเกล count เปลี่ยนเป็น 2400/รอบ
  loadState();
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
      waitEncoderSettle();
      printStatus();
    }
    else if (c == 'Z') {
      position = 0;
      resetCounter();
      Serial.println("[RESET] stepper = 0 steps, encoder = 0 count");
      saveState(true);
    }
    else if (c == 'S') {
      printStatus();
    }
    else if (c == 'W') {
      saveState(true);
    }
    else if (c == 'R') {
      printSaved();
    }
    else if (c == 'C') {
      clearSaved();
    }
    else if (c == 'T') {
      testOneRev();
    }
    else if (c == 'E') {
      printEncoderWiring();
    }
    else if (c == 'P') {                     // raw steps: P200, P-200
      long steps = line.substring(1).toInt();
      if (steps == 0) { Serial.println("Usage: P<steps>, e.g. P200"); return; }
      Serial.print("Moving ");
      Serial.print(steps);
      Serial.println(" pulses");
      logBegin(labs(steps));
      moveSteps(steps);
      waitEncoderSettle();
      logFinish();
      printStatus();
      saveState(false);
    }
    else if (c == 'O') {                     // zero calibration helper
      if (!homed) { Serial.println("Run H first, then jog to the true 0 with P<steps>, then send O"); return; }
      long suggested = HOME_OFFSET_STEPS + (HOME_APPROACH_POSITIVE ? position : -position);
      Serial.print("Current position from 0: ");
      Serial.print(position);
      Serial.print(" steps (");
      Serial.print(position / (STEPS_PER_MM * 10.0), 4);
      Serial.println(" cm)");
      Serial.print("Set  const long HOME_OFFSET_STEPS = ");
      Serial.print(suggested);
      Serial.println(";  then upload and run H");
    }
    else if (c == 'L') {
      logEnabled = !logEnabled;
      Serial.print("Per-step log: ");
      Serial.println(logEnabled ? "ON" : "OFF");
    }
    else {
      float cm = line.toFloat();             // "1.5" → 1.5 cm, "-0.8" → -0.8 cm
      if (cm == 0.0) {
        Serial.println("Invalid command: enter cm (e.g. 1.5), P<steps>, H, Z, S, W, R, C, T or E");
        return;
      }
      moveCm(cm);
    }
  }

  // ----- แสดงค่า encoder ทุก 100 ms เมื่อมีการหมุน (เช่น หมุนด้วยมือ) -----
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    long current = getEncoderCount();
    if (current != lastCounter) {
      printEncoder(current);
      lastCounter  = current;
      lastChangeMs = millis();
      pendingSave  = true;
    }
  }

  // ----- หมุนด้วยมือแล้วหยุดนิ่ง IDLE_SAVE_MS → บันทึกลง flash -----
  if (pendingSave && millis() - lastChangeMs >= IDLE_SAVE_MS) {
    saveState(false);
  }
}