/*
 * L298N all-direction bench test.
 *
 * Arduino UNO -> L298N
 *   IN1 -> D7     IN2 -> D11   (LEFT  side motors via OUT1/OUT2)
 *   IN3 -> D9     IN4 -> D8    (RIGHT side motors via OUT3/OUT4)
 *   ENA/ENB jumpers remain fitted.
 *
 * Lift chassis. Connect motor battery. Send GO over serial (115200 baud).
 * Sequence (5 s each, 5 s gap between each):
 *   1. FORWARD          – both sides forward
 *   2. BACKWARD         – both sides backward
 *   3. LEFT SLIDE       – left backward, right forward
 *   4. RIGHT SLIDE      – left forward, right backward
 *   5. LEFT SLIDE REV   – same as LEFT SLIDE (repeated)
 *   6. RIGHT SLIDE REV  – same as RIGHT SLIDE (repeated)
 *   7. LEFT SLIDE REV   – same as LEFT SLIDE (repeated)
 * Send STOP at any time to cut motors immediately.
 */

const uint8_t IN1 = 7;   // LEFT  forward
const uint8_t IN2 = 11;  // LEFT  backward
const uint8_t IN3 = 9;   // RIGHT forward
const uint8_t IN4 = 8;   // RIGHT backward

const unsigned long RUN_MS  = 5000UL;
const unsigned long GAP_MS  = 5000UL;

bool testFinished = false;
String commandBuffer;

// ── helpers ──────────────────────────────────────────────────────────────────

void stopAllMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setMotors(bool leftFwd, bool rightFwd) {
  digitalWrite(IN1, leftFwd  ? HIGH : LOW);
  digitalWrite(IN2, leftFwd  ? LOW  : HIGH);
  digitalWrite(IN3, rightFwd ? HIGH : LOW);
  digitalWrite(IN4, rightFwd ? LOW  : HIGH);
}

void runStep(const char* label, bool leftFwd, bool rightFwd) {
  Serial.print("STEP: ");
  Serial.println(label);
  setMotors(leftFwd, rightFwd);
  delay(RUN_MS);
  stopAllMotors();
  Serial.println("GAP");
  delay(GAP_MS);
}

// ── main sequence ─────────────────────────────────────────────────────────────

void runTest() {
  runStep("FORWARD",          true,  true );
  runStep("BACKWARD",         false, false);
  runStep("LEFT SLIDE",       false, true );
  runStep("RIGHT SLIDE",      true,  false);
  runStep("LEFT SLIDE REV",   false, true );
  runStep("RIGHT SLIDE REV",  true,  false);
  runStep("LEFT SLIDE REV",   false, true );
  Serial.println("DONE: all steps complete");
}

// ── Arduino boilerplate ───────────────────────────────────────────────────────

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopAllMotors();

  Serial.begin(115200);
  commandBuffer.reserve(8);
  Serial.println("READY: lift chassis, connect motor battery, then send GO");
}

void loop() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      commandBuffer.trim();
      if (commandBuffer == "GO" && !testFinished) {
        runTest();
        testFinished = true;
      } else if (commandBuffer == "STOP") {
        stopAllMotors();
        Serial.println("STOPPED");
      }
      commandBuffer = "";
    } else if (commandBuffer.length() < 7) {
      commandBuffer += c;
    } else {
      commandBuffer = "";
      stopAllMotors();
    }
  }
  if (!testFinished) stopAllMotors();
}
