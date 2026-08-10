/*
 * FRESH STANDALONE 4-MOTOR SIMULTANEOUS ROTATION SKETCH
 * 
 * Pin Allocations on Arduino UNO:
 * - IN1 = Pin 2 (Left Motors Direction 1)
 * - ENA = Pin 3 (Left Motors Enable)
 * - IN2 = Pin 4 (Left Motors Direction 2)
 * - IN3 = Pin 5 (Right Motors Direction 1)
 * - IN4 = Pin 6 (Right Motors Direction 2)
 * - ENB = Pin 10 (Right Motors Enable)
 */

void setup() {
  Serial.begin(115200);

  // Set all 6 L298N pins as OUTPUT
  pinMode(2, OUTPUT); // IN1
  pinMode(3, OUTPUT); // ENA
  pinMode(4, OUTPUT); // IN2
  pinMode(5, OUTPUT); // IN3
  pinMode(6, OUTPUT); // IN4
  pinMode(10, OUTPUT); // ENB

  // Immediately rotate ALL 4 MOTORS AT THE EXACT SAME TIME!
  rotateAllMotors();
  Serial.println("STATUS: ALL 4 MOTORS ROTATING SIMULTANEOUSLY!");
}

void loop() {
  // Continuously maintain full-power rotation on all 4 motors
  rotateAllMotors();

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "STOP") {
      stopAllMotors();
      while (true) {
        if (Serial.available() > 0) {
          String r = Serial.readStringUntil('\n');
          r.trim();
          if (r == "START" || r == "FORWARD") break;
        }
        delay(50);
      }
    }
  }
  delay(50);
}

void rotateAllMotors() {
  // 1. Enable 100% full power to BOTH L298N Channels
  digitalWrite(3, HIGH);  // ENA HIGH
  digitalWrite(10, HIGH); // ENB HIGH

  // 2. Drive Left Motors (OUT1/OUT2) FORWARD
  digitalWrite(2, HIGH);  // IN1 HIGH
  digitalWrite(4, LOW);   // IN2 LOW

  // 3. Drive Right Motors (OUT3/OUT4) FORWARD
  digitalWrite(5, HIGH);  // IN3 HIGH
  digitalWrite(6, LOW);   // IN4 LOW
}

void stopAllMotors() {
  digitalWrite(3, LOW);
  digitalWrite(10, LOW);
  digitalWrite(2, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
}
