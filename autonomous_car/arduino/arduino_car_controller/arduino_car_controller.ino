/*
 * Arduino UNO Autonomous RC Car Controller with Camera Servo Pan
 * Interfaces:
 * - 2 HC-SR04 Ultrasonic Sensors (Front & Rear, 0 Resistors required)
 * - 1 Camera Pan Servo Motor (Digital Pin 5)
 * - Motion commands over USB Serial (FORWARD, BACKWARD, LEFT, RIGHT, STOP)
 * - Camera Servo commands over USB Serial (SERVO:angle, e.g. SERVO:90, SERVO:150, SERVO:30)
 */

#include <Servo.h>

// -----------------------------------------------------------------------------
// PIN DEFINITIONS
// -----------------------------------------------------------------------------
// Front HC-SR04 Ultrasonic Sensor
const int FRONT_TRIG_PIN = 9;
const int FRONT_ECHO_PIN = 10;

// Rear HC-SR04 Ultrasonic Sensor
const int REAR_TRIG_PIN = 11;
const int REAR_ECHO_PIN = 12;

// Camera Pan Servo Motor
const int SERVO_PIN = 5;
Servo cameraServo;

// Timing interval for sending sensor telemetry (ms)
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_INTERVAL_MS = 100;

void setup() {
  Serial.begin(115200);

  // Front Ultrasonic Pin Configuration
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  digitalWrite(FRONT_TRIG_PIN, LOW);

  // Rear Ultrasonic Pin Configuration
  pinMode(REAR_TRIG_PIN, OUTPUT);
  pinMode(REAR_ECHO_PIN, INPUT);
  digitalWrite(REAR_TRIG_PIN, LOW);

  // Attach Camera Servo Motor and set to Center (90 degrees)
  cameraServo.attach(SERVO_PIN);
  cameraServo.write(90);

  // (Motor Driver pins will be initialized here when driver arrives)
}

void loop() {
  // 1. Check for incoming Serial commands from Raspberry Pi
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    executeCommand(command);
  }

  // 2. Periodically read ultrasonic sensors and send data back to Pi
  unsigned long now = millis();
  if (now - lastSensorReadTime >= SENSOR_INTERVAL_MS) {
    lastSensorReadTime = now;
    readAndSendSensors();
  }
}

// -----------------------------------------------------------------------------
// SENSOR READING & TELEMETRY
// -----------------------------------------------------------------------------
float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) {
    return 400.0; // Out of range
  }
  float distanceCm = (duration * 0.0343) / 2.0;
  return distanceCm;
}

void readAndSendSensors() {
  float frontDist = measureDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  float rearDist = measureDistance(REAR_TRIG_PIN, REAR_ECHO_PIN);

  // Send format to Raspberry Pi: "DIST:front_cm,rear_cm"
  Serial.print("DIST:");
  Serial.print(frontDist, 1);
  Serial.print(",");
  Serial.println(rearDist, 1);
}

// -----------------------------------------------------------------------------
// COMMAND HANDLER (Motors & Camera Servo)
// -----------------------------------------------------------------------------
void executeCommand(String cmd) {
  // Handle Camera Servo Command Format: SERVO:angle (e.g. SERVO:90, SERVO:150)
  if (cmd.startsWith("SERVO:")) {
    int angle = cmd.substring(6).toInt();
    angle = constrain(angle, 0, 180);
    cameraServo.write(angle);
    return;
  }

  // Handle Motion Commands
  if (cmd == "FORWARD") {
    // Drive motors forward
  } else if (cmd == "BACKWARD") {
    // Drive motors backward
  } else if (cmd == "LEFT") {
    // Turn motors left
  } else if (cmd == "RIGHT") {
    // Turn motors right
  } else if (cmd == "STOP") {
    // Stop all motors
  }
}
