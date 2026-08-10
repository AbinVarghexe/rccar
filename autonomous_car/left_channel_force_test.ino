/*
 * LEFT SIDE CHANNEL (OUT1 & OUT2) ISOLATION TEST SKETCH
 * Tests Left Motors on Pins 2, 3, 4, 7, 8 to force Left channel ON.
 */

void setup() {
  Serial.begin(115200);

  // Set all test pins as OUTPUT
  pinMode(2, OUTPUT); // IN1
  pinMode(3, OUTPUT); // ENA
  pinMode(4, OUTPUT); // IN2
  pinMode(7, OUTPUT); // Test Pin 7
  pinMode(8, OUTPUT); // Test Pin 8

  Serial.println("=== FORCE TESTING LEFT MOTOR CHANNEL (OUT1 & OUT2) ===");
}

void loop() {
  Serial.println(">>> TESTING LEFT MOTORS (Pin 2=HIGH, Pin 3=HIGH, Pin 4=LOW)...");

  // Force ENA (Pin 3) HIGH + IN1 (Pin 2) HIGH + IN2 (Pin 4) LOW
  digitalWrite(3, HIGH);
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);
  delay(3000);

  Serial.println(">>> REVERSING LEFT MOTORS (Pin 2=LOW, Pin 3=HIGH, Pin 4=HIGH)...");

  // Reverse Left Channel
  digitalWrite(3, HIGH);
  digitalWrite(2, LOW);
  digitalWrite(4, HIGH);
  delay(3000);

  Serial.println(">>> TESTING ALTERNATE PINS 7 & 8 FOR LEFT CHANNEL...");
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
  delay(3000);
}
