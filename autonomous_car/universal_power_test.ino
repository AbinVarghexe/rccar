/*
 * UNIVERSAL L298N ALL-ENABLE MOTOR FORCE SKETCH
 * Powers BOTH internal logic AND external enable pins (Pins 2, 3, 4, 5, 6, 10).
 * 
 * Hardware Checklist:
 * 1. Wire from Arduino GND -> L298N GND (CRITICAL!)
 * 2. External Battery (7V-12V) connected to L298N 12V and GND screw terminals.
 * 3. IN1 -> Pin 2, ENA -> Pin 3, IN2 -> Pin 4, IN3 -> Pin 5, IN4 -> Pin 6, ENB -> Pin 10.
 */

void setup() {
  // Set all digital pins 2, 3, 4, 5, 6, 10 as OUTPUT
  pinMode(2, OUTPUT);  // IN1
  pinMode(3, OUTPUT);  // ENA (Enable A)
  pinMode(4, OUTPUT);  // IN2
  pinMode(5, OUTPUT);  // IN3
  pinMode(6, OUTPUT);  // IN4
  pinMode(10, OUTPUT); // ENB (Enable B)
}

void loop() {
  // FORWARD FORCE PULSE: Enable ENA (Pin 3) + ENB (Pin 10) AND set IN1=HIGH, IN3=HIGH
  digitalWrite(3, HIGH);  // Force ENA ON
  digitalWrite(10, HIGH); // Force ENB ON

  digitalWrite(2, HIGH);  // IN1 HIGH
  digitalWrite(4, LOW);   // IN2 LOW

  digitalWrite(5, HIGH);  // IN3 HIGH
  digitalWrite(6, LOW);   // IN4 LOW

  delay(3000); // Drive for 3 seconds

  // REVERSE FORCE PULSE
  digitalWrite(3, HIGH);
  digitalWrite(10, HIGH);

  digitalWrite(2, LOW);   // IN1 LOW
  digitalWrite(4, HIGH);  // IN2 HIGH

  digitalWrite(5, LOW);   // IN3 LOW
  digitalWrite(6, HIGH);  // IN4 HIGH

  delay(3000); // Drive reverse for 3 seconds
}
