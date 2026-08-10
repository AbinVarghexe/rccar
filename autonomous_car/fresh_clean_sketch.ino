/*
 * FRESH STANDALONE 4-MOTOR ROTATION CODE
 * 
 * Pin Connections:
 * - IN1 (L298N) -> Arduino Pin 2
 * - IN2 (L298N) -> Arduino Pin 4
 * - IN3 (L298N) -> Arduino Pin 5
 * - IN4 (L298N) -> Arduino Pin 6
 * - GND (L298N) -> Arduino GND (Shared Ground)
 * - ENA & ENB jumper caps ON
 */

const int IN1 = 2; // Left Motors Dir 1
const int IN2 = 4; // Left Motors Dir 2
const int IN3 = 5; // Right Motors Dir 1
const int IN4 = 6; // Right Motors Dir 2

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("=== FRESH 4-MOTOR ROTATION TEST STARTED ===");
}

void loop() {
  // ---------------------------------------------------------------------------
  // TEST 1: ALL 4 MOTORS FORWARD (3 SECONDS)
  // ---------------------------------------------------------------------------
  Serial.println(">>> SPINNING ALL 4 MOTORS FORWARD...");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(3000);

  // STOP (1.5 SECONDS)
  Serial.println(">>> STOPPING...");
  stopMotors();
  delay(1500);

  // ---------------------------------------------------------------------------
  // TEST 2: ALL 4 MOTORS REVERSE (3 SECONDS)
  // ---------------------------------------------------------------------------
  Serial.println(">>> SPINNING ALL 4 MOTORS REVERSE...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(3000);

  // STOP (2 SECONDS)
  Serial.println(">>> STOPPING...");
  stopMotors();
  delay(2000);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
