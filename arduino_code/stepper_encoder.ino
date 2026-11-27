/*
  ================================================================
   Lead screw controller  —  ESP32-S2 + TB6600 + Encoder HN3806-600N
  ================================================================

  Library ที่ต้องติดตั้ง (Arduino IDE → Library Manager):
    1. FastAccelStepper   (by gin66)        — สร้าง pulse ด้วยฮาร์ดแวร์ + ทางลาดเร่ง/ลด
    2. ESP32Encoder       (by Kevin Harrington) — นับ encoder ด้วยฮาร์ดแวร์ PCNT
    
  Serial Monitor: 115200 baud, line ending = "Newline"

  ---------------- คำสั่ง ----------------
    G 25      ไปที่ตำแหน่ง 25 mm (นับจากจุด 0)
    M -5      เลื่อนจากตำแหน่งปัจจุบัน -5 mm
    V 10      ตั้งความเร็ว 10 mm/s
    A 50      ตั้งความเร่ง 50 mm/s²
    H         Homing → เลื่อนไปจน PX2 ON แล้วตั้งเป็น 0
    X         หยุด (ค่อย ๆ ลดความเร็วตามทางลาด)
    Z         ตั้งตำแหน่งปัจจุบันเป็น 0 (stepper + encoder)
    S         แสดงสถานะ

  ลำดับ sensor บนแกน:   PX1 (ฝั่งลบ) ---- PX2 (home) ---- PX3 (ฝั่งบวก)
*/

#include <FastAccelStepper.h>
#include <ESP32Encoder.h>


// ================================================================
//  1) ตั้งค่าฮาร์ดแวร์  (แก้ตรงนี้ให้ตรงกับของจริง)
// ================================================================

// ----- ขา -----
const int PIN_PUL = 10;
const int PIN_DIR = 11;
const int PIN_PX1 = 37;      // limit ฝั่งลบ
const int PIN_PX2 = 38;      // จุด home (อยู่ระหว่าง PX1 กับ PX3)
const int PIN_PX3 = 39;      // limit ฝั่งบวก
const int PIN_ENC_A = 13;    
const int PIN_ENC_B = 12;    

// ----- Sensor -----
const int SENSOR_ON = HIGH;  // ระดับสัญญาณตอน sensor เจอวัตถุ (NPN ส่วนใหญ่เป็น LOW)

// ----- มอเตอร์ + lead screw -----
const float STEPS_PER_REV = 200;   // มอเตอร์ 1.8° = 200 steps/รอบ
const float MICROSTEP     = 4;     // ตั้งให้ตรงกับ DIP switch ของ TB6600
const float MM_PER_REV    = 4.0;   // lead screw 4 mm ต่อรอบ
const float STEPS_PER_MM  = STEPS_PER_REV * MICROSTEP / MM_PER_REV;  

// ----- Encoder -----
const float ENC_PPR         = 600;                           // HN3806-600N
const float ENC_COUNTS_PER_REV = ENC_PPR * 4;                // นับทุกขอบ A+B = 2400 counts/รอบ
const float ENC_MM_PER_REV  = 4.0;                           // ระยะต่อรอบของแกนที่ encoder ติดอยู่
const float ENC_COUNTS_PER_MM = ENC_COUNTS_PER_REV / ENC_MM_PER_REV;
const int   ENC_SIGN        = 1;    // ถ้า stepper ไปทางบวกแต่ encoder ติดลบ → เปลี่ยนเป็น -1

// ----- ความเร็ว (หน่วย mm/s และ mm/s²) -----
float speedMM = 30.0;              // ความเร็วเริ่มต้น
float accelMM = 50.0;              // ความเร่งเริ่มต้น
const float MAX_SPEED_MM = 30.0;   // กันพิมพ์ค่าเกิน
const float MAX_ACCEL_MM = 500.0;

// ----- Homing -----
const float HOME_FAST_MM   = 30.0;   // ความเร็วตอนค้นหา PX2
const float HOME_SLOW_MM   = 0.5;   // ความเร็วตอนเข้าหา PX2 รอบสุดท้าย (ช้า = แม่น)
const float HOME_BACKOFF_MM = 1.0;  // ถอยออกจาก PX2 ก่อนเข้าหาช้า
const float HOME_OFFSET_MM = 1.9;
const bool  HOME_FROM_PX1_SIDE = true;   // true = เข้าหา PX2 จากฝั่ง PX1 ทุกครั้ง


// ----- Limit -----
const float LIMIT_BACKOFF_MM = 1.0;  // ชน limit แล้วถอยออกเท่านี้


// ================================================================
//  2) ตัวแปรของโปรแกรม
// ================================================================

FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;
ESP32Encoder encoder;

bool homed = false;
bool movingPositive = true;        // ทิศที่กำลังวิ่ง (ใช้เช็ค limit ฝั่งที่ถูก)
bool wasRunning = false;           // ใช้จับจังหวะ "เพิ่งหมุนเสร็จ"

// ผลลัพธ์ของ runUntil() — ต้องประกาศไว้ก่อนฟังก์ชันแรก (กัน error ของ Arduino IDE)
enum RunResult { FOUND, HIT_LIMIT, TIMEOUT };


// ================================================================
//  3) แปลงหน่วย mm <-> steps
// ================================================================

long mmToSteps(float mm) {
  return lround(mm * STEPS_PER_MM);
}

float stepsToMM(long steps) {
  return steps / STEPS_PER_MM;
}

float positionMM() {
  return stepsToMM(stepper->getCurrentPosition());
}

float encoderMM() {
  return ENC_SIGN * encoder.getCount() / ENC_COUNTS_PER_MM;
}


// ================================================================
//  4) Sensor
// ================================================================

bool isOn(int pin) {
  return digitalRead(pin) == SENSOR_ON;
}

// เช็คเฉพาะ limit ฝั่งที่กำลังวิ่งไป
bool limitAhead(bool positive) {
  return positive ? isOn(PIN_PX3) : isOn(PIN_PX1);
}


// ================================================================
//  5) สั่งมอเตอร์ (ไม่ block — มอเตอร์หมุนเองเบื้องหลัง)
// ================================================================

// ตั้งความเร็ว/ความเร่ง (หน่วย mm) ให้ library
void applySpeed(float mmPerSec, float mmPerSec2) {
  stepper->setSpeedInHz(lround(mmPerSec * STEPS_PER_MM));
  stepper->setAcceleration(lround(mmPerSec2 * STEPS_PER_MM));
  stepper->applySpeedAcceleration();     // มีผลทันทีแม้กำลังหมุนอยู่
}

// ไปตำแหน่ง absolute (mm)
void goToMM(float targetMM) {
  long target = mmToSteps(targetMM);
  long now    = stepper->getCurrentPosition();
  if (target == now) return;

  movingPositive = (target > now);
  if (limitAhead(movingPositive)) {
    Serial.println("ไม่เลื่อน: limit ฝั่งนั้นทำงานอยู่");
    return;
  }
  stepper->moveTo(target);
}

// เลื่อนแบบ relative (mm)
void moveByMM(float deltaMM) {
  goToMM(positionMM() + deltaMM);
}

// รอจนมอเตอร์หยุดสนิท
void waitUntilStopped() {
  while (stepper->isRunning()) {
    delay(1);
  }
}


// ================================================================
//  6) Limit switch — เรียกใน loop() ตลอดเวลา
// ================================================================

void checkLimits() {
  if (!stepper->isRunning()) return;
  if (!limitAhead(movingPositive)) return;

  // ชน limit → หยุดทันที แล้วถอยออก
  stepper->forceStop();
  waitUntilStopped();
  Serial.println(movingPositive ? "FAULT: ชน PX3" : "FAULT: ชน PX1");

  applySpeed(HOME_FAST_MM, accelMM);
  movingPositive = !movingPositive;
  stepper->move(movingPositive ? mmToSteps(LIMIT_BACKOFF_MM) : -mmToSteps(LIMIT_BACKOFF_MM));
  waitUntilStopped();
  applySpeed(speedMM, accelMM);
}


// ================================================================
//  7) Homing ไปที่ PX2
// ================================================================

// วิ่งไปทิศ positive ที่ความเร็ว mmPerSec จนกว่า sensorPin จะเป็น wantOn
// หยุดถ้าชน limit ฝั่งที่วิ่งไป  (ฟังก์ชันนี้ block จนหยุด)
RunResult runUntil(bool positive, int sensorPin, bool wantOn, float mmPerSec) {
  if (isOn(sensorPin) == wantOn) return FOUND;
  if (limitAhead(positive))      return HIT_LIMIT;

  applySpeed(mmPerSec, accelMM);
  movingPositive = positive;
  if (positive) stepper->runForward();
  else          stepper->runBackward();

  RunResult result = TIMEOUT;
  unsigned long start = millis();
  while (true) {
    if (isOn(sensorPin) == wantOn) { result = FOUND;     break; }
    if (limitAhead(positive))      { result = HIT_LIMIT; break; }
    if (millis() - start > 120000) { result = TIMEOUT;   break; }   // กันวิ่งไม่หยุด 2 นาที
    delay(1);
  }

  stepper->forceStop();
  waitUntilStopped();
  return result;
}

bool homePX2() {
  Serial.println("Homing ไป PX2...");
  homed = false;
  const bool toward = HOME_FROM_PX1_SIDE;   // ทิศสุดท้ายที่เข้าหา PX2 (true = ทางบวก)
  const bool away   = !toward;
  RunResult r;

  // ขั้น 1: พาไปอยู่ฝั่ง "ก่อนถึง" PX2
  if (isOn(PIN_PX2)) {
    r = runUntil(away, PIN_PX2, false, HOME_FAST_MM);          // อยู่บน PX2 → ถอยจนดับ
    if (r != FOUND) { Serial.println("ERROR: ถอยออกจาก PX2 ไม่ได้"); return false; }
  } else {
    r = runUntil(away, PIN_PX2, true, HOME_FAST_MM);           // ไม่รู้อยู่ฝั่งไหน → วิ่งไปทางถอยก่อน
    if (r == FOUND) {
      r = runUntil(away, PIN_PX2, false, HOME_FAST_MM);        // เจอ PX2 = เดิมอยู่อีกฝั่ง → วิ่งผ่านไป
      if (r != FOUND) { Serial.println("ERROR: ผ่าน PX2 ไม่ได้"); return false; }
    } else if (r == TIMEOUT) {
      Serial.println("ERROR: ไม่เจอทั้ง PX2 และ limit"); return false;
    }
    // r == HIT_LIMIT → ชน limit ฝั่งถอย = อยู่ถูกฝั่งแล้ว
  }

  // ขั้น 2: วิ่งเข้าหา PX2 (เร็ว)
  r = runUntil(toward, PIN_PX2, true, HOME_FAST_MM);
  if (r != FOUND) { Serial.println("ERROR: ไม่เจอ PX2 — เช็ค sensor หรือทิศมอเตอร์"); return false; }

  // ขั้น 3: ถอยออกจน PX2 ดับ แล้วถอยเพิ่มอีกนิด
  r = runUntil(away, PIN_PX2, false, HOME_SLOW_MM);
  if (r != FOUND) { Serial.println("ERROR: PX2 ไม่ดับตอนถอย"); return false; }
  applySpeed(HOME_FAST_MM, accelMM);
  stepper->move(away ? mmToSteps(HOME_BACKOFF_MM) : -mmToSteps(HOME_BACKOFF_MM));
  waitUntilStopped();

  // ขั้น 4: เข้าหา PX2 ช้า ๆ → ได้จุด 0 ที่ตรงเดิมทุกครั้ง
  r = runUntil(toward, PIN_PX2, true, HOME_SLOW_MM);
  if (r != FOUND) { Serial.println("ERROR: หา PX2 ไม่เจอตอนเข้าช้า"); return false; }

  // ขั้น 5: ตอนนี้อยู่ตรงจุดที่ PX2 ติด
  //   จุดนี้อยู่ห่างจากจุด 0 ที่ต้องการ = -HOME_OFFSET_MM
  //   → บอก library ว่า "ตอนนี้อยู่ที่ -offset" แล้วสั่งไปที่ 0
  stepper->setCurrentPosition(mmToSteps(-HOME_OFFSET_MM));
  encoder.setCount(lround(-HOME_OFFSET_MM * ENC_COUNTS_PER_MM) * ENC_SIGN);

  // ขั้น 6: เลื่อนไปจุด 0 จริง (ถ้า offset = 0 จะไม่ขยับ)
  applySpeed(HOME_FAST_MM, accelMM);
  movingPositive = (HOME_OFFSET_MM > 0);
  stepper->moveTo(0);
  waitUntilStopped();

  applySpeed(speedMM, accelMM);
  homed = true;
  Serial.printf("Homed → ตำแหน่ง = 0 mm (ห่างจาก PX2 %+.3f mm)\n", HOME_OFFSET_MM);
  return true;
}


// ================================================================
//  8) แสดงผล
// ================================================================

void printSensors() {
  Serial.printf("  Sensor:  PX1=%s  PX2=%s  PX3=%s\n",
                isOn(PIN_PX1) ? "ON" : "off",
                isOn(PIN_PX2) ? "ON" : "off",
                isOn(PIN_PX3) ? "ON" : "off");
}

void printPosition() {
  float cmd = positionMM();
  float enc = encoderMM();
  Serial.printf("  ตำแหน่ง: %8.3f mm | encoder: %8.3f mm | ต่างกัน: %+.3f mm\n",
                cmd, enc, enc - cmd);
}

void printStatus() {
  Serial.println("----------- สถานะ -----------");
  Serial.printf("  Homed:   %s\n", homed ? "yes" : "NO");
  printPosition();
  Serial.printf("  ความเร็ว: %.2f mm/s | ความเร่ง: %.1f mm/s²\n", speedMM, accelMM);
  Serial.printf("  สเกล:    %.1f steps/mm (microstep 1/%.0f)\n", STEPS_PER_MM, MICROSTEP);
  printSensors();
  Serial.println("-----------------------------");
}

void printHelp() {
  Serial.println();
  Serial.println("คำสั่ง:");
  Serial.println("  G <mm>     ไปที่ตำแหน่ง   เช่น G 25");
  Serial.println("  M <mm>     เลื่อนจากที่เดิม  เช่น M -5");
  Serial.println("  V <mm/s>   ตั้งความเร็ว   เช่น V 10");
  Serial.println("  A <mm/s²>  ตั้งความเร่ง   เช่น A 50");
  Serial.println("  H          homing ไป PX2");
  Serial.println("  X          หยุด (ลดความเร็วตามทางลาด)");
  Serial.println("  Z          ตั้งตรงนี้เป็น 0");
  Serial.println("  S          สถานะ");
  Serial.println();
}


// ================================================================
//  9) รับคำสั่งจาก Serial
// ================================================================

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  char cmd = toupper(line.charAt(0));
  String argText = line.substring(1);
  argText.trim();
  float value = argText.toFloat();
  bool hasValue = argText.length() > 0;

  switch (cmd) {
    case 'G':
      if (!hasValue) { Serial.println("ใส่ตำแหน่งด้วย เช่น G 25"); break; }
      if (!homed) Serial.println("(เตือน: ยังไม่ได้ homing — ตำแหน่ง 0 อาจไม่ตรง)");
      Serial.printf("ไปที่ %.3f mm\n", value);
      goToMM(value);
      break;

    case 'M':
      if (!hasValue) { Serial.println("ใส่ระยะด้วย เช่น M -5"); break; }
      Serial.printf("เลื่อน %+.3f mm\n", value);
      moveByMM(value);
      break;

    case 'V':
      if (value <= 0 || value > MAX_SPEED_MM) {
        Serial.printf("ความเร็วต้องอยู่ระหว่าง 0 – %.0f mm/s\n", MAX_SPEED_MM); break;
      }
      speedMM = value;
      applySpeed(speedMM, accelMM);
      Serial.printf("ความเร็ว = %.2f mm/s (%ld steps/s)\n", speedMM, mmToSteps(speedMM));
      break;

    case 'A':
      if (value <= 0 || value > MAX_ACCEL_MM) {
        Serial.printf("ความเร่งต้องอยู่ระหว่าง 0 – %.0f mm/s²\n", MAX_ACCEL_MM); break;
      }
      accelMM = value;
      applySpeed(speedMM, accelMM);
      Serial.printf("ความเร่ง = %.1f mm/s² (ถึงความเร็วเต็มใน %.2f s)\n", accelMM, speedMM / accelMM);
      break;

    case 'H':
      homePX2();
      printStatus();
      break;

    case 'X':
      stepper->stopMove();
      Serial.println("หยุด");
      break;

    case 'Z':
      if (stepper->isRunning()) { Serial.println("หยุดมอเตอร์ก่อน (X)"); break; }
      stepper->setCurrentPosition(0);
      encoder.clearCount();
      Serial.println("ตั้งตำแหน่งนี้เป็น 0 mm");
      break;

    case 'S':
      printStatus();
      break;

    default:
      Serial.println("ไม่รู้จักคำสั่ง");
      printHelp();
  }
}


// ================================================================
//  10) setup / loop
// ================================================================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}   // รอ USB ของ ESP32-S2
  delay(200);

  // Sensor
  pinMode(PIN_PX1, INPUT);
  pinMode(PIN_PX2, INPUT);
  pinMode(PIN_PX3, INPUT);

  // Encoder (นับด้วยฮาร์ดแวร์ ไม่ต้องใช้ interrupt เอง)
  ESP32Encoder::useInternalWeakPullResistors = puType::up;   // ESP32Encoder เวอร์ชันเก่าใช้ UP
  encoder.attachFullQuad(PIN_ENC_A, PIN_ENC_B);
  encoder.clearCount();

  // Stepper
  engine.init();
  stepper = engine.stepperConnectToPin(PIN_PUL);
  if (stepper == nullptr) {
    Serial.println("ERROR: เชื่อม stepper ไม่ได้ — เช็คขา PUL");
    while (true) delay(1000);
  }
  stepper->setDirectionPin(PIN_DIR);     // DIR = HIGH → ตำแหน่งเพิ่ม (ไปทาง PX3)
  applySpeed(speedMM, accelMM);

  Serial.println("=== Lead screw controller ===");
  printStatus();
  printHelp();
}

void loop() {
  // 1) รับคำสั่ง
  if (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  // 2) เช็ค limit ตลอดเวลาที่มอเตอร์หมุน
  checkLimits();

  // 3) แสดงตำแหน่งทุก 200 ms ระหว่างหมุน และสรุปตอนหมุนเสร็จ
  static unsigned long lastPrint = 0;
  bool running = stepper->isRunning();

  if (running && millis() - lastPrint >= 200) {
    lastPrint = millis();
    printPosition();
  }
  if (wasRunning && !running) {
    Serial.println("เสร็จ");
    printPosition();
  }
  wasRunning = running;
}