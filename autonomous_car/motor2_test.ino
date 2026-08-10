/*
 * MOTOR 2 TEST: Channel B (OUT3 & OUT4)
 * Tests Motor 2 (Channel B) independently to diagnose why only one side is working.
 * 
 * Hardware Connections on Arduino UNO:
 * - IN3 (L298N) -> Arduino Pin 5
 * - ENB (L298N) -> Arduino Pin 10 (or Black Jumper Cap ON)
 * - IN4 (L298N) -> Arduino Pin 6
 * - GND (L298N) -> Arduino GND
 */

const int IN3_PIN = 5;  // Motor 2 Direction 1
const int ENB_PIN = 10; // Motor 2 Speed Enable
const int IN4_PIN = 6;  // Motor 2 Direction 2

void setup() {
  Serial.begin(115200);
  
  pinMode(IN3_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  Serial.println("=== MOTOR 2 (CHANNEL B) ISOLATION TEST INITIALIZED ===");
}

void loop() {
  Serial.println(">>> SPINNING MOTOR 2 / CHANNEL B (OUT3 & OUT4)...");
  
  // Force 100% full power to ENB
  digitalWrite(ENB_PIN, HIGH);
  analogWrite(ENB_PIN, 255);
  
  // Set direction FORWARD for Channel B
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  
  delay(3000); // Spin for 3 seconds

  Serial.println(">>> STOPPING MOTOR 2...");
  
  // Stop Motor 2
  digitalWrite(ENB_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  
  delay(2000); // Pause for 2 seconds
}
