/*
 * BOTH MOTORS TEST: Channel A (OUT1/OUT2) + Channel B (OUT3/OUT4)
 * Drives BOTH sides of the RC car simultaneously.
 * 
 * Pin Connections:
 * - IN1 (L298N) -> Arduino Pin 2
 * - ENA (L298N) -> Arduino Pin 3 (or Jumper Cap ON)
 * - IN2 (L298N) -> Arduino Pin 4
 * - IN3 (L298N) -> Arduino Pin 5
 * - IN4 (L298N) -> Arduino Pin 6
 * - ENB (L298N) -> Arduino Pin 10 (or Jumper Cap ON)
 * - GND (L298N) -> Arduino GND
 */

const int IN1_PIN = 2; // Left Dir 1
const int ENA_PIN = 3; // Left Speed
const int IN2_PIN = 4; // Left Dir 2

const int IN3_PIN = 5;  // Right Dir 1
const int IN4_PIN = 6;  // Right Dir 2
const int ENB_PIN = 10; // Right Speed

void setup() {
  Serial.begin(115200);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);

  Serial.println("=== DUAL SIDE MOTOR TEST INITIALIZED ===");
}

void loop() {
  Serial.println(">>> SPINNING BOTH MOTORS (CHANNEL A & CHANNEL B)...");

  // Force 100% full power on Enable Pins
  digitalWrite(ENA_PIN, HIGH);
  digitalWrite(ENB_PIN, HIGH);
  analogWrite(ENA_PIN, 255);
  analogWrite(ENB_PIN, 255);

  // Drive Channel A (OUT1/OUT2)
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);

  // Drive Channel B (OUT3/OUT4)
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);

  delay(3000); // Spin for 3 seconds

  Serial.println(">>> STOPPING BOTH MOTORS...");

  digitalWrite(ENA_PIN, LOW);
  digitalWrite(ENB_PIN, LOW);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  delay(2000); // Pause for 2 seconds
}
