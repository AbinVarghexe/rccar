/*
 * ============================================================================
 * 🏎️ PROFESSIONAL 4WD RC CAR MOTOR CONTROLLER FIRMWARE
 * ============================================================================
 * 
 * Hardware Pin Allocation Table:
 * ┌───────────┬──────────────────┬─────────────────┬─────────────────────────┐
 * │ L298N Pin │ Arduino UNO Pin  │ Signal Type     │ Description             │
 * ├───────────┼──────────────────┼─────────────────┼─────────────────────────┤
 * │ IN1       │ Digital Pin 2    │ Digital Output  │ Left Motors Dir 1       │
 * │ ENA       │ Jumper Cap (5V)  │ Power Enable    │ Left Motors 100% Power  │
 * │ IN2       │ Digital Pin 4    │ Digital Output  │ Left Motors Dir 2       │
 * │ IN3       │ Digital Pin 5    │ Digital Output  │ Right Motors Dir 1      │
 * │ IN4       │ Digital Pin 6    │ Digital Output  │ Right Motors Dir 2      │
 * │ ENB       │ Jumper Cap (5V)  │ Power Enable    │ Right Motors 100% Power │
 * │ GND       │ Arduino GND      │ Shared Ground   │ Common Reference Ground │
 * └───────────┴──────────────────┴─────────────────┴─────────────────────────┘
 */

// ============================================================================
// 1. PIN & HARDWARE CONSTANTS
// ============================================================================
const int PIN_LEFT_DIR1  = 2; // IN1
const int PIN_LEFT_DIR2  = 4; // IN2
const int PIN_RIGHT_DIR1 = 5; // IN3
const int PIN_RIGHT_DIR2 = 6; // IN4

// Communication Settings
const long SERIAL_BAUD_RATE = 115200;

// ============================================================================
// 2. SYSTEM INITIALIZATION (Runs once at startup)
// ============================================================================
void setup() {
  // Initialize Serial Communication for debugging and remote commands
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println(F("=== 4WD RC CAR SYSTEM INITIALIZED ==="));

  // Configure Motor Control Pins as Digital Outputs
  pinMode(PIN_LEFT_DIR1, OUTPUT);
  pinMode(PIN_LEFT_DIR2, OUTPUT);
  pinMode(PIN_RIGHT_DIR1, OUTPUT);
  pinMode(PIN_RIGHT_DIR2, OUTPUT);

  // Ensure motors start in a safe STOPPED state
  stopMotors();
}

// ============================================================================
// 3. MAIN CONTROL LOOP (Runs continuously)
// ============================================================================
void loop() {
  // Listen for incoming serial commands from PC / Raspberry Pi
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    executeCommand(command);
  }
}

// ============================================================================
// 4. COMMAND PROCESSOR
// ============================================================================
void executeCommand(String command) {
  if (command == "FORWARD" || command == "START" || command == "F") {
    moveForward();
    Serial.println(F("STATUS: FORWARD"));
  } 
  else if (command == "BACKWARD" || command == "REVERSE" || command == "B") {
    moveBackward();
    Serial.println(F("STATUS: BACKWARD"));
  } 
  else if (command == "LEFT" || command == "L") {
    turnLeft();
    Serial.println(F("STATUS: LEFT"));
  } 
  else if (command == "RIGHT" || command == "R") {
    turnRight();
    Serial.println(F("STATUS: RIGHT"));
  } 
  else if (command == "STOP" || command == "S") {
    stopMotors();
    Serial.println(F("STATUS: STOPPED"));
  }
}

// ============================================================================
// 5. MOTOR DRIVE PRIMITIVES (MODULAR FUNCTIONS)
// ============================================================================

/**
 * Moves all 4 motors FORWARD in unison.
 */
void moveForward() {
  // Left Motors (OUT1 / OUT2) -> Forward
  digitalWrite(PIN_LEFT_DIR1, HIGH);
  digitalWrite(PIN_LEFT_DIR2, LOW);

  // Right Motors (OUT3 / OUT4) -> Forward
  digitalWrite(PIN_RIGHT_DIR1, HIGH);
  digitalWrite(PIN_RIGHT_DIR2, LOW);
}

/**
 * Moves all 4 motors BACKWARD in unison.
 */
void moveBackward() {
  // Left Motors -> Backward
  digitalWrite(PIN_LEFT_DIR1, LOW);
  digitalWrite(PIN_LEFT_DIR2, HIGH);

  // Right Motors -> Backward
  digitalWrite(PIN_RIGHT_DIR1, LOW);
  digitalWrite(PIN_RIGHT_DIR2, HIGH);
}

/**
 * Rotates the car LEFT (Spin Turn).
 */
void turnLeft() {
  // Left Motors -> Reverse
  digitalWrite(PIN_LEFT_DIR1, LOW);
  digitalWrite(PIN_LEFT_DIR2, HIGH);

  // Right Motors -> Forward
  digitalWrite(PIN_RIGHT_DIR1, HIGH);
  digitalWrite(PIN_RIGHT_DIR2, LOW);
}

/**
 * Rotates the car RIGHT (Spin Turn).
 */
void turnRight() {
  // Left Motors -> Forward
  digitalWrite(PIN_LEFT_DIR1, HIGH);
  digitalWrite(PIN_LEFT_DIR2, LOW);

  // Right Motors -> Reverse
  digitalWrite(PIN_RIGHT_DIR1, LOW);
  digitalWrite(PIN_RIGHT_DIR2, HIGH);
}

/**
 * Stops all 4 motors cleanly.
 */
void stopMotors() {
  digitalWrite(PIN_LEFT_DIR1, LOW);
  digitalWrite(PIN_LEFT_DIR2, LOW);
  digitalWrite(PIN_RIGHT_DIR1, LOW);
  digitalWrite(PIN_RIGHT_DIR2, LOW);
}
