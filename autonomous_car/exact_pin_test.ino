/*
 * EXACT USER HARDWARE CONFIGURATION SKETCH
 * 
 * Hardware Layout:
 * - ENA and ENB are BRIDGED TOGETHER (Jumper caps ON - no enable wires needed).
 * - IN1 -> Arduino Digital Pin 2
 * - IN2 -> Arduino Digital Pin 4
 * - IN3 -> Arduino Digital Pin 5
 * - IN4 -> Arduino Digital Pin 6
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

  stopAll();
  Serial.println("=== EXACT HARDWARE PIN TEST INITIALIZED ===");
}

void loop() {
  // ---------------------------------------------------------------------------
  // STEP 1: TEST BOTH CHANNELS FORWARD (3 SECONDS)
  // ---------------------------------------------------------------------------
  Serial.println(">>> STEP 1: BOTH CHANNELS FORWARD (Pin 2=HIGH, 4=LOW | Pin 5=HIGH, 6=LOW)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(3000);

  stopAll();
  delay(1500);

  // ---------------------------------------------------------------------------
  // STEP 2: TEST CHANNEL B REVERSED POLARITY (3 SECONDS)
  // (Tests if Channel B needs opposite HIGH/LOW signal)
  // ---------------------------------------------------------------------------
  Serial.println(">>> STEP 2: CHANNEL B REVERSED POLARITY (Pin 2=HIGH, 4=LOW | Pin 5=LOW, 6=HIGH)");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(3000);

  stopAll();
  delay(1500);

  // ---------------------------------------------------------------------------
  // STEP 3: TEST CHANNEL B ONLY (3 SECONDS)
  // (Tests if Channel B / OUT3/OUT4 turns when Channel A is off)
  // ---------------------------------------------------------------------------
  Serial.println(">>> STEP 3: CHANNEL B ONLY (Pin 2=LOW, 4=LOW | Pin 5=HIGH, 6=LOW)");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(3000);

  stopAll();
  delay(1500);

  // ---------------------------------------------------------------------------
  // STEP 4: BOTH CHANNELS REVERSE (3 SECONDS)
  // ---------------------------------------------------------------------------
  Serial.println(">>> STEP 4: BOTH CHANNELS REVERSE (Pin 2=LOW, 4=HIGH | Pin 5=LOW, 6=HIGH)");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(3000);

  stopAll();
  delay(2000);
}

void stopAll() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
