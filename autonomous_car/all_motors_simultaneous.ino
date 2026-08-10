/*
 * ALL 4 MOTORS SIMULTANEOUS ROTATION (PURE 4-LINE CONTROL)
 * 
 * Pin Connections:
 * - IN1 (L298N) -> Arduino Pin 2
 * - IN2 (L298N) -> Arduino Pin 4
 * - IN3 (L298N) -> Arduino Pin 5
 * - IN4 (L298N) -> Arduino Pin 6
 * - ENA and ENB are BRIDGED TOGETHER (Jumper caps ON)
 */

void setup() {
  pinMode(2, OUTPUT); // IN1
  pinMode(4, OUTPUT); // IN2
  pinMode(5, OUTPUT); // IN3
  pinMode(6, OUTPUT); // IN4

  // ROTATE ALL 4 MOTORS AT THE EXACT SAME TIME INSTANTLY ON POWER-UP!
  digitalWrite(2, HIGH); // Channel A Forward
  digitalWrite(4, LOW);

  digitalWrite(5, HIGH); // Channel B Forward
  digitalWrite(6, LOW);
}

void loop() {
  // Keep ALL 4 motors rotating continuously at the exact same time
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);

  digitalWrite(5, HIGH);
  digitalWrite(6, LOW);
}
