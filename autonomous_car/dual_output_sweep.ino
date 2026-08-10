/*
 * DUAL OUTPUT HARDWARE SWEEP SKETCH
 * Drives ALL pin pairs (2/4, 5/6, 7/8, 9/10) to find exact Left Motor working pins.
 */

void setup() {
  Serial.begin(115200);

  for (int p = 2; p <= 11; p++) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  Serial.println("=== DUAL OUTPUT HARDWARE SWEEP INITIALIZED ===");
}

void loop() {
  // PULSE 1: Pins 2, 3, 4 (Standard Channel A)
  Serial.println("PULSE 1: Driving Pins 2, 3, 4...");
  digitalWrite(3, HIGH);
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);
  delay(3000);
  allOff();
  delay(1000);

  // PULSE 2: Pins 5, 6, 10 (Standard Channel B - KNOWN WORKING)
  Serial.println("PULSE 2: Driving Pins 5, 6, 10...");
  digitalWrite(10, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, LOW);
  delay(3000);
  allOff();
  delay(1000);

  // PULSE 3: Pins 7, 8, 9 (Alternate Pins)
  Serial.println("PULSE 3: Driving Pins 7, 8, 9...");
  digitalWrite(9, HIGH);
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
  delay(3000);
  allOff();
  delay(1000);
}

void allOff() {
  for (int p = 2; p <= 11; p++) {
    digitalWrite(p, LOW);
  }
}
