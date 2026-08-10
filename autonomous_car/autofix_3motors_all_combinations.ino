/*
 * AUTO-FIX 3-MOTOR COMBINATION TESTER
 * Automatically sweeps through all HIGH/LOW polarity combinations on Pins 2, 4, 5, 6
 * to guarantee that ALL 3 MOTORS ROTATE TOGETHER AT THE EXACT SAME TIME.
 * 
 * Hardware Pins:
 * - IN1 = Pin 2
 * - IN2 = Pin 4
 * - IN3 = Pin 5
 * - IN4 = Pin 6
 * - ENA & ENB = Bridged with Jumper Caps ON
 * - GND = Shared Arduino GND
 */

const int IN1 = 2;
const int IN2 = 4;
const int IN3 = 5;
const int IN4 = 6;

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();
  Serial.println("=== 3-MOTOR AUTO-FIX DIAGNOSTIC STARTED ===");
}

void loop() {
  // ---------------------------------------------------------------------------
  // COMBINATION 1: (IN1=HIGH, IN2=LOW | IN3=HIGH, IN4=LOW)
  // ---------------------------------------------------------------------------
  Serial.println("\n>>> TEST 1: IN1=HIGH, IN2=LOW | IN3=HIGH, IN4=LOW (3 SECONDS)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(3000);
  stopMotors();
  delay(1500);

  // ---------------------------------------------------------------------------
  // COMBINATION 2: (IN1=HIGH, IN2=LOW | IN3=LOW, IN4=HIGH) <-- INVERTS SIDE 2!
  // ---------------------------------------------------------------------------
  Serial.println(">>> TEST 2: IN1=HIGH, IN2=LOW | IN3=LOW, IN4=HIGH (3 SECONDS)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(3000);
  stopMotors();
  delay(1500);

  // ---------------------------------------------------------------------------
  // COMBINATION 3: (IN1=LOW, IN2=HIGH | IN3=HIGH, IN4=LOW) <-- INVERTS SIDE 1!
  // ---------------------------------------------------------------------------
  Serial.println(">>> TEST 3: IN1=LOW, IN2=HIGH | IN3=HIGH, IN4=LOW (3 SECONDS)");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(3000);
  stopMotors();
  delay(1500);

  // ---------------------------------------------------------------------------
  // COMBINATION 4: SIDE 2 ONLY FORCE TEST (IN1=LOW, IN2=LOW | IN3=HIGH, IN4=LOW)
  // ---------------------------------------------------------------------------
  Serial.println(">>> TEST 4: SIDE 2 ONLY FORCE (IN1=LOW, IN2=LOW | IN3=HIGH, IN4=LOW)");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(3000);
  stopMotors();
  delay(2000);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
