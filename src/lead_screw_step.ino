#define PX1 37   // limit at the DIR=LOW end
#define PX2 38   // unused for now
#define PX3 39   // limit at the DIR=HIGH end
#define PUL 10
#define DIR 11

const int  LIMIT_ACTIVE  = HIGH;  // level your sensor outputs when triggered — check this!
const int  STEP_HALF_US  = 500;
const long BACKOFF_STEPS = 200;

long position = 0;

void pulse() {
  digitalWrite(PUL, HIGH);
  delayMicroseconds(STEP_HALF_US);
  digitalWrite(PUL, LOW);
  delayMicroseconds(STEP_HALF_US);
}

void setDir(bool positive) {
  digitalWrite(DIR, positive ? HIGH : LOW);
  delayMicroseconds(10);          // TB6600: DIR must lead PUL by >5 µs
}

bool limitHit(bool positive) {    // only check the limit we're moving toward
  return digitalRead(positive ? PX3 : PX1) == LIMIT_ACTIVE;
}

void moveSteps(long steps) {
  bool positive = steps > 0;
  long n = labs(steps);
  setDir(positive);

  for (long i = 0; i < n; i++) {
    if (limitHit(positive)) {
      Serial.println(positive ? "FAULT: PX3 limit" : "FAULT: PX1 limit");
      setDir(!positive);
      for (long j = 0; j < BACKOFF_STEPS; j++) {
        pulse();
        position += positive ? -1 : 1;
      }
      return;
    }
    pulse();
    position += positive ? 1 : -1;
  }
}

void setup() {
  pinMode(PX1, INPUT);
  pinMode(PX2, INPUT);
  pinMode(PX3, INPUT);
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(PUL, LOW);
  digitalWrite(DIR, LOW);

  Serial.begin(115200);
  Serial.setTimeout(50);
  Serial.println("serial begin");
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;     // ignore blank Enter presses

    long steps = line.toInt();          // "100" → 100, "-250" → -250
    if (steps == 0) return;

    Serial.print("Moving ");
    Serial.print(steps);
    Serial.println(" pulses");
    moveSteps(steps);
    Serial.print("Position: ");
    Serial.println(position);
  }
}