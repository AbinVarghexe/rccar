/*
 * ALL 4 MOTORS SIMULTANEOUS RUN SKETCH
 * 
 * Hardware Connections:
 * - IN1 -> Arduino Pin 2
 * - ENA -> Arduino Pin 3 (or Jumper Cap ON)
 * - IN2 -> Arduino Pin 4
 * - IN3 -> Arduino Pin 5
 * - IN4 -> Arduino Pin 6
 * - ENB -> Arduino Pin 10 (or Jumper Cap ON)
 * - GND -> Arduino GND (Shared Ground)
 */

void setup() {
  pinMode(2, OUTPUT);  // IN1
  pinMode(3, OUTPUT);  // ENA
  pinMode(4, OUTPUT);  // IN2
  pinMode(5, OUTPUT);  // IN3
  pinMode(6, OUTPUT);  // IN4
  pinMode(10, OUTPUT); // ENB

  // DRIVE ALL 4 MOTORS AT THE EXACT SAME TIME INSTANTLY ON POWER-UP!
  digitalWrite(3, HIGH);  // ENA ON
  digitalWrite(10, HIGH); // ENB ON

  digitalWrite(2, HIGH);  // IN1 HIGH (Left Motors)
  digitalWrite(4, LOW);   // IN2 LOW

  digitalWrite(5, HIGH);  // IN3 HIGH (Right Motors)
  digitalWrite(6, LOW);   // IN4 LOW
}

void loop() {
  // Continuously drive ALL 4 MOTORS together at 100% full power
  digitalWrite(3, HIGH);
  digitalWrite(10, HIGH);

  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);

  digitalWrite(5, HIGH);
  digitalWrite(6, LOW);
}
