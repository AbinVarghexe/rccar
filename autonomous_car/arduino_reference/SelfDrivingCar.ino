/*
 * SelfDrivingCar.ino — Main RC Car Sketch
 *
 * Arduino UNO pinout:
 *   L298N motor driver:
 *     IN1 → D7   IN2 → D11   (LEFT  motors, OUT1/OUT2)
 *     IN3 → D9   IN4 → D8    (RIGHT motors, OUT3/OUT4)
 *     ENA/ENB jumpers fitted — full speed
 *
 *   HC-SR04 ultrasonic:
 *     TRIG → D4   ECHO → D3
 *
 *   Servo (camera tilt):
 *     Signal → D6
 *
 * Serial protocol (115200 baud):
 *   Pi → Arduino:  FORWARD | BACKWARD | LEFT | RIGHT | HARD_LEFT | HARD_RIGHT |
 *                  SLIDE_LEFT | SLIDE_RIGHT | STOP | SERVO:<angle>
 *   Arduino → Pi:  DIST:<cm>\n   (every ~60 ms)
 *                  ACK:<cmd>\n   (after each motion command)
 */

#include <Servo.h>

// ── Motor pins ────────────────────────────────────────────────────────────────
const uint8_t IN1 = 7;   // LEFT  forward
const uint8_t IN2 = 11;  // LEFT  backward
const uint8_t IN3 = 9;   // RIGHT forward
const uint8_t IN4 = 8;   // RIGHT backward

// ── Ultrasonic pins ───────────────────────────────────────────────────────────
const uint8_t TRIG_PIN = 4;
const uint8_t ECHO_PIN = 3;

// ── Servo ─────────────────────────────────────────────────────────────────────
const uint8_t SERVO_PIN = 6;
Servo camServo;

// ── Timing ────────────────────────────────────────────────────────────────────
const unsigned long SONAR_INTERVAL_MS = 60UL;
unsigned long lastSonarMs = 0;

// ── Standby & Serial command buffer ──────────────────────────────────────────
bool standbyMode = false;
String cmdBuffer;

// ── Motor helpers ─────────────────────────────────────────────────────────────

void motorsStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void motorsForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void motorsBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void motorsLeft() {
  // Left side backward, right side forward → spins left
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void motorsRight() {
  // Left side forward, right side backward → spins right
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void motorsSlideLeft() {
  // Gentle forward arc: right track drives, left track is stopped.
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void motorsSlideRight() {
  // Gentle forward arc: left track drives, right track is stopped.
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ── Ultrasonic ────────────────────────────────────────────────────────────────

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // pulseIn timeout = 30 ms → max range ~500 cm; returns 0 on timeout
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return 400.0f;  // out of range
  return duration * 0.01723f;        // cm = µs × (0.034 / 2)
}

// ── Command handler ───────────────────────────────────────────────────────────

void handleCommand(const String& cmd) {
  if (cmd == "STANDBY") {
    motorsStop();
    standbyMode = true;
    Serial.println("ACK:STANDBY");
  } else if (cmd == "RESUME") {
    standbyMode = false;
    Serial.println("ACK:RESUME");
  } else if (cmd == "FORWARD") {
    standbyMode = false;
    motorsForward();
    Serial.println("ACK:FORWARD");
  } else if (cmd == "BACKWARD") {
    standbyMode = false;
    motorsBackward();
    Serial.println("ACK:BACKWARD");
  } else if (cmd == "LEFT") {
    standbyMode = false;
    motorsLeft();
    Serial.println("ACK:LEFT");
  } else if (cmd == "RIGHT") {
    standbyMode = false;
    motorsRight();
    Serial.println("ACK:RIGHT");
  } else if (cmd == "HARD_LEFT") {
    standbyMode = false;
    motorsLeft();
    Serial.println("ACK:HARD_LEFT");
  } else if (cmd == "HARD_RIGHT") {
    standbyMode = false;
    motorsRight();
    Serial.println("ACK:HARD_RIGHT");
  } else if (cmd == "SLIDE_LEFT") {
    standbyMode = false;
    motorsSlideLeft();
    Serial.println("ACK:SLIDE_LEFT");
  } else if (cmd == "SLIDE_RIGHT") {
    standbyMode = false;
    motorsSlideRight();
    Serial.println("ACK:SLIDE_RIGHT");
  } else if (cmd == "STOP") {
    motorsStop();
    Serial.println("ACK:STOP");
  } else if (cmd.startsWith("SERVO:")) {
    int angle = cmd.substring(6).toInt();
    angle = constrain(angle, 0, 180);
    camServo.write(angle);
    Serial.print("ACK:SERVO:");
    Serial.println(angle);
  } else if (cmd == "PING") {
    Serial.println("PONG");
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  motorsStop();

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo
  camServo.attach(SERVO_PIN);
  camServo.write(90);  // center on boot

  // Serial
  Serial.begin(115200);
  cmdBuffer.reserve(16);

  Serial.println("READY");
}

// ── Loop ──────────────────────────────────────────────────────────────────────

void loop() {
  // 1. Read serial commands
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuffer.trim();
      if (cmdBuffer.length() > 0) {
        handleCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else if (cmdBuffer.length() < 20) {
      cmdBuffer += c;
    } else {
      // Oversized garbage — discard and stop for safety
      cmdBuffer = "";
      motorsStop();
    }
  }

  // 2. Send distance reading at fixed interval (non-blocking) - Cut off completely during Standby
  if (!standbyMode) {
    unsigned long now = millis();
    if (now - lastSonarMs >= SONAR_INTERVAL_MS) {
      lastSonarMs = now;
      float dist = readDistanceCm();
      Serial.print("DIST:");
      Serial.println((int)dist);
    }
  }
}
