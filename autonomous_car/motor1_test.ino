/*
 * MOTOR 1 TEST: Channel A (OUT1 & OUT2)
 * Tests Motor 1 independently for 3 seconds ON, 2 seconds OFF.
 * 
 * Pin Connections on Arduino UNO:
 * - IN1 (L298N) -> Arduino Pin 2
 * - ENA (L298N) -> Arduino Pin 3
 * - IN2 (L298N) -> Arduino Pin 4
 * - GND (L298N) -> Arduino GND
 */

const int IN1_PIN = 2; // Motor 1 Direction 1
const int ENA_PIN = 3; // Motor 1 Speed Enable
const int IN2_PIN = 4; // Motor 1 Direction 2

void setup() {
  Serial.begin(115200);
  
  pinMode(IN1_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  Serial.println("=== MOTOR 1 (CHANNEL A) TEST INITIALIZED ===");
}

void loop() {
  Serial.println(">>> SPINNING MOTOR 1 (FORWARD)...");
  
  // Enable Motor 1 at 100% full power
  digitalWrite(ENA_PIN, HIGH);
  
  // Set direction FORWARD
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  
  delay(3000); // Spin for 3 seconds

  Serial.println(">>> STOPPING MOTOR 1...");
  
  // Stop Motor 1
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  
  delay(2000); // Pause for 2 seconds
}
