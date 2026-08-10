/*
 * Arduino UNO Multi-Pin Camera Servo Motor Hardware Test
 * Drives Servo on Digital Pins 5, 6, and 9 simultaneously.
 */

#include <Servo.h>

Servo servoPin5;
Servo servoPin6;
Servo servoPin9;

void setup() {
  Serial.begin(115200);
  Serial.println("--- Starting Multi-Pin Servo Motor Test ---");

  // Attach servos to Pins 5, 6, and 9
  servoPin5.attach(5);
  servoPin6.attach(6);
  servoPin9.attach(9);

  // Set initial position to Center (90 degrees)
  servoPin5.write(90);
  servoPin6.write(90);
  servoPin9.write(90);
  delay(1000);
}

void loop() {
  // Move to 30 degrees
  Serial.println("Moving Servos (Pins 5, 6, 9) to 30 degrees...");
  servoPin5.write(30);
  servoPin6.write(30);
  servoPin9.write(30);
  delay(1500);

  // Move to 150 degrees
  Serial.println("Moving Servos (Pins 5, 6, 9) to 150 degrees...");
  servoPin5.write(150);
  servoPin6.write(150);
  servoPin9.write(150);
  delay(1500);

  // Move to 90 degrees (Center)
  Serial.println("Moving Servos (Pins 5, 6, 9) to 90 degrees...");
  servoPin5.write(90);
  servoPin6.write(90);
  servoPin9.write(90);
  delay(1500);
}
