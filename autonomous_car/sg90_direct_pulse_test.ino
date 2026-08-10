/*
 * SG90 Micro Servo 9g Direct Bit-Bang Pulse Test (No Timers / No Libraries)
 * Pin 9 Signal
 */

const int SERVO_PIN = 9;

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("--- Starting SG90 Direct Pulse Test ---");
}

// Sends raw microsecond pulse to SG90 servo
void sendPulse(int pin, int pulseWidthUs) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseWidthUs);
  digitalWrite(pin, LOW);
  delayMicroseconds(20000 - pulseWidthUs); // 20ms total period (50Hz)
}

void loop() {
  // Move to 0 degrees (1000us pulse)
  Serial.println("Pulse: 1000us (Left 0 deg)");
  for (int i = 0; i < 50; i++) {
    sendPulse(SERVO_PIN, 1000);
  }
  delay(1000);

  // Move to 90 degrees (1500us pulse - Center)
  Serial.println("Pulse: 1500us (Center 90 deg)");
  for (int i = 0; i < 50; i++) {
    sendPulse(SERVO_PIN, 1500);
  }
  delay(1000);

  // Move to 180 degrees (2000us pulse - Right 180 deg)
  Serial.println("Pulse: 2000us (Right 180 deg)");
  for (int i = 0; i < 50; i++) {
    sendPulse(SERVO_PIN, 2000);
  }
  delay(1000);
}
