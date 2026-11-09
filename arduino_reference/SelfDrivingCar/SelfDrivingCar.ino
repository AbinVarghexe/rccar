/*
 * Minimal motor controller for the sanoguzhan/Self-Driving-Car hardware map.
 *
 * Arduino UNO -> L298N
 *   Left:  ENA D5, IN1 D7, IN2 D11
 *   Right: ENB D6, IN3 D9, IN4 D8
 *
 * Serial commands are newline-terminated at 115200 baud:
 *   FORWARD, BACKWARD, LEFT, RIGHT, HARD_LEFT, HARD_RIGHT,
 *   SLIDE_LEFT, SLIDE_RIGHT, STOP, PING
 *
 * The watchdog is intentional: if the Raspberry Pi stops sending commands,
 * both motor channels are disabled.
 */

const uint8_t EN_L = 5;
const uint8_t IN_L_A = 7;
const uint8_t IN_L_B = 11;

const uint8_t EN_R = 6;
const uint8_t IN_R_A = 9;
const uint8_t IN_R_B = 8;

const uint8_t DRIVE_SPEED = 180;       // 0..255; start conservatively
const unsigned long WATCHDOG_MS = 500; // stop if Pi link goes quiet

String commandBuffer;
unsigned long lastValidCommandMs = 0;

void setChannel(uint8_t enablePin, uint8_t pinA, uint8_t pinB,
                int16_t speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else if (speed < 0) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
  analogWrite(enablePin, abs(speed));
}

void stopMotors() {
  setChannel(EN_L, IN_L_A, IN_L_B, 0);
  setChannel(EN_R, IN_R_A, IN_R_B, 0);
}

void applyCommand(const String &command) {
  if (command == "FORWARD") {
    setChannel(EN_L, IN_L_A, IN_L_B, DRIVE_SPEED);
    setChannel(EN_R, IN_R_A, IN_R_B, DRIVE_SPEED);
  } else if (command == "BACKWARD") {
    setChannel(EN_L, IN_L_A, IN_L_B, -DRIVE_SPEED);
    setChannel(EN_R, IN_R_A, IN_R_B, -DRIVE_SPEED);
  } else if (command == "LEFT") {
    setChannel(EN_L, IN_L_A, IN_L_B, -DRIVE_SPEED);
    setChannel(EN_R, IN_R_A, IN_R_B, DRIVE_SPEED);
  } else if (command == "RIGHT") {
    setChannel(EN_L, IN_L_A, IN_L_B, DRIVE_SPEED);
    setChannel(EN_R, IN_R_A, IN_R_B, -DRIVE_SPEED);
  } else if (command == "HARD_LEFT") {
    setChannel(EN_L, IN_L_A, IN_L_B, -DRIVE_SPEED);
    setChannel(EN_R, IN_R_A, IN_R_B, DRIVE_SPEED);
  } else if (command == "HARD_RIGHT") {
    setChannel(EN_L, IN_L_A, IN_L_B, DRIVE_SPEED);
    setChannel(EN_R, IN_R_A, IN_R_B, -DRIVE_SPEED);
  } else if (command == "SLIDE_LEFT") {
    setChannel(EN_L, IN_L_A, IN_L_B, 0);
    setChannel(EN_R, IN_R_A, IN_R_B, DRIVE_SPEED);
  } else if (command == "SLIDE_RIGHT") {
    setChannel(EN_L, IN_L_A, IN_L_B, DRIVE_SPEED);
    setChannel(EN_R, IN_R_A, IN_R_B, 0);
  } else if (command == "STOP") {
    stopMotors();
  } else if (command == "STANDBY") {
    stopMotors();
    Serial.println("OK:STANDBY");
    return;
  } else if (command == "RESUME") {
    stopMotors();
    Serial.println("OK:RESUME");
    return;
  } else if (command == "PING") {
    Serial.println("READY");
    return;
  } else {
    stopMotors();
    Serial.println("ERROR:UNKNOWN_COMMAND");
    return;
  }

  lastValidCommandMs = millis();
  Serial.print("OK:");
  Serial.println(command);
}

void setup() {
  pinMode(EN_L, OUTPUT);
  pinMode(IN_L_A, OUTPUT);
  pinMode(IN_L_B, OUTPUT);
  pinMode(EN_R, OUTPUT);
  pinMode(IN_R_A, OUTPUT);
  pinMode(IN_R_B, OUTPUT);
  stopMotors();

  Serial.begin(115200);
  commandBuffer.reserve(24);
  lastValidCommandMs = millis();
  Serial.println("READY:SELF_DRIVING_CAR");
}

void loop() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      commandBuffer.trim();
      if (commandBuffer.length() > 0) {
        applyCommand(commandBuffer);
      }
      commandBuffer = "";
    } else if (commandBuffer.length() < 23) {
      commandBuffer += c;
    } else {
      commandBuffer = "";
      stopMotors();
    }
  }

  if (millis() - lastValidCommandMs > WATCHDOG_MS) {
    stopMotors();
  }
}
