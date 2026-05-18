// ============================================================
//  Micromouse - Step 1: Keyboard Control + Sensor Monitor
//
//  Serial commands (115200 baud, send single character):
//    W - Move forward  (press any key to stop)
//    S - Move backward (press any key to stop)
//    A - Rotate left 90 degrees  (blocks until done, then re-enables input)
//    D - Rotate right 90 degrees (blocks until done, then re-enables input)
//    X - Emergency stop
//    P - Print raw sensor values (for tuning thresholds)
//    E - Print encoder counts (for verifying encoders)
//
//  Sensor crossection is logged after every turn and every stop.
//  Tune FRONT_WALL_THRESHOLD and SIDE_WALL_THRESHOLD in config.h
//  based on "P" output before trusting crossection output.
//
//  TURN TUNING:
//    Adjust TURN_TIME_MS in config.h until A/D produces exactly
//    90 degrees of rotation on your surface.
// ============================================================

#include <Arduino.h>
#include "pins.h"
#include "config.h"
#include "motors.h"
#include "sensors.h"

// ---- State ----
enum State {
  STATE_IDLE,
  STATE_MOVING_FORWARD,
  STATE_MOVING_BACKWARD,
  STATE_TURNING_LEFT,
  STATE_TURNING_RIGHT
};

State currentState = STATE_IDLE;

// ---- LED helpers ----
void setLED1(bool on) { digitalWrite(PIN::DEBUG_LED_1, on ? HIGH : LOW); }
void setLED2(bool on) { digitalWrite(PIN::DEBUG_LED_2, on ? HIGH : LOW); }

// ---- Crossection detection and logging ----
// Reads sensors and logs which directions are open (no wall).
void logCrossection() {
  SensorValues sv = Sensors::readAll();
  WallInfo walls = Sensors::detectWalls();

  Serial.print("[Sensors] FL:");
  Serial.print(sv.frontLeft);
  Serial.print(" FR:"); Serial.print(sv.frontRight);
  Serial.print(" SL:"); Serial.print(sv.sideLeft);
  Serial.print(" SR:"); Serial.println(sv.sideRight);

  bool openFront = !walls.front;
  bool openLeft  = !walls.left;
  bool openRight = !walls.right;
  int openCount  = (openFront ? 1 : 0) + (openLeft ? 1 : 0) + (openRight ? 1 : 0);

  if (openCount == 0) {
    Serial.println("[Crossection] Dead end - all directions blocked");
    return;
  }

  Serial.print("[Crossection] Open: [");
  bool first = true;
  if (openFront) { Serial.print("Straight"); first = false; }
  if (openLeft)  { if (!first) Serial.print(", "); Serial.print("Left");  first = false; }
  if (openRight) { if (!first) Serial.print(", "); Serial.print("Right"); }
  Serial.println("]");
}

// ---- Timed turn ----
// Uses time instead of encoders because the right encoder ISR is not yet
// verified. Tune CONFIG::TURN_TIME_MS in config.h until turns are 90 degrees.
void executeTurnLeft() {
  Serial.println("[Turn] Rotating LEFT 90 degrees...");
  setLED1(true);
  setLED2(false);
  Motors::resetEncoders();

  // Right wheel forward, left wheel backward = rotate CCW (left)
  Motors::setLeftMotor(-CONFIG::TURN_SPEED);
  Motors::setRightMotor(CONFIG::TURN_SPEED);
  delay(CONFIG::TURN_TIME_MS);
  Motors::stop();
  delay(100);  // Settle

  Serial.print("[Encoders after turn] L:");
  Serial.print(Motors::getLeftEncoder());
  Serial.print(" R:");
  Serial.println(Motors::getRightEncoder());

  setLED1(false);
  Serial.println("[Turn] Done.");
}

void executeTurnRight() {
  Serial.println("[Turn] Rotating RIGHT 90 degrees...");
  setLED1(false);
  setLED2(true);
  Motors::resetEncoders();

  // Left wheel forward, right wheel backward = rotate CW (right)
  Motors::setLeftMotor(CONFIG::TURN_SPEED);
  Motors::setRightMotor(-CONFIG::TURN_SPEED);
  delay(CONFIG::TURN_TIME_MS);
  Motors::stop();
  delay(100);  // Settle

  Serial.print("[Encoders after turn] L:");
  Serial.print(Motors::getLeftEncoder());
  Serial.print(" R:");
  Serial.println(Motors::getRightEncoder());

  setLED2(false);
  Serial.println("[Turn] Done.");
}

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);  // Wait for USB CDC to connect

  Serial.println();
  Serial.println("======================================");
  Serial.println("  Micromouse - Step 1: Keyboard Mode");
  Serial.println("======================================");
  Serial.println("  W = Forward    S = Backward");
  Serial.println("  A = Left 90    D = Right 90");
  Serial.println("  X = Stop       P = Print sensors");
  Serial.println("  E = Print encoders");
  Serial.println("======================================");

  pinMode(PIN::DEBUG_LED_1, OUTPUT);
  pinMode(PIN::DEBUG_LED_2, OUTPUT);
  setLED1(false);
  setLED2(false);

  Motors::init();
  Sensors::init();
  Motors::enable();

  if (Motors::isFault()) {
    Serial.println("!!! MOTOR DRIVER FAULT at startup !!!");
    Serial.println("    Check EN/FAULT wiring. Sensors still work.");
  }

  int batt = Sensors::readBatteryRaw();
  Serial.print("Battery ADC: "); Serial.println(batt);
  if (Sensors::isBatteryLow()) {
    Serial.println("WARNING: Battery may be low.");
  }

  Serial.println();
  Serial.println("Ready. Send P first to read sensor values and tune thresholds.");
}

// ============================================================
void loop() {

  // ---- Read serial input ----
  if (Serial.available()) {
    char cmd = (char)Serial.read();
    while (Serial.available()) Serial.read();  // Flush newline/extra bytes

    // Emergency stop always works
    if (cmd == 'x' || cmd == 'X') {
      Motors::stop();
      currentState = STATE_IDLE;
      setLED1(false);
      setLED2(false);
      Serial.println("[Stop] Emergency stopped.");
      logCrossection();
      return;
    }

    // Debug commands work in any state
    if (cmd == 'p' || cmd == 'P') {
      SensorValues sv = Sensors::readAll();
      Serial.print("[Sensors] FL:"); Serial.print(sv.frontLeft);
      Serial.print(" FR:"); Serial.print(sv.frontRight);
      Serial.print(" SL:"); Serial.print(sv.sideLeft);
      Serial.print(" SR:"); Serial.print(sv.sideRight);
      Serial.print(" BAT:"); Serial.println(Sensors::readBatteryRaw());
      Serial.print("  FRONT_WALL_THRESHOLD="); Serial.print(CONFIG::FRONT_WALL_THRESHOLD);
      Serial.print("  SIDE_WALL_THRESHOLD="); Serial.println(CONFIG::SIDE_WALL_THRESHOLD);
      return;
    }

    if (cmd == 'e' || cmd == 'E') {
      Serial.print("[Encoders] L:");
      Serial.print(Motors::getLeftEncoder());
      Serial.print(" R:");
      Serial.println(Motors::getRightEncoder());
      return;
    }

    // Movement commands
    switch (currentState) {

      case STATE_IDLE:
        if (cmd == 'w' || cmd == 'W') {
          Serial.println("[Move] Moving FORWARD. Press any key to stop.");
          currentState = STATE_MOVING_FORWARD;
          setLED1(true); setLED2(false);
          Motors::resetEncoders();
          Motors::setLeftMotor(CONFIG::EXPLORE_SPEED);
          Motors::setRightMotor(CONFIG::EXPLORE_SPEED);

        } else if (cmd == 's' || cmd == 'S') {
          Serial.println("[Move] Moving BACKWARD. Press any key to stop.");
          currentState = STATE_MOVING_BACKWARD;
          setLED1(false); setLED2(true);
          Motors::resetEncoders();
          Motors::setLeftMotor(-CONFIG::EXPLORE_SPEED);
          Motors::setRightMotor(-CONFIG::EXPLORE_SPEED);

        } else if (cmd == 'a' || cmd == 'A') {
          currentState = STATE_TURNING_LEFT;
          executeTurnLeft();
          currentState = STATE_IDLE;
          logCrossection();

        } else if (cmd == 'd' || cmd == 'D') {
          currentState = STATE_TURNING_RIGHT;
          executeTurnRight();
          currentState = STATE_IDLE;
          logCrossection();

        } else {
          Serial.println("[?] W=Fwd S=Back A=Left90 D=Right90 X=Stop P=Sensors E=Encoders");
        }
        break;

      // Any key press while moving = stop
      case STATE_MOVING_FORWARD:
      case STATE_MOVING_BACKWARD:
        Motors::stop();
        currentState = STATE_IDLE;
        setLED1(false); setLED2(false);
        Serial.println("[Stop] Stopped.");
        Serial.print("[Encoders] L:");
        Serial.print(Motors::getLeftEncoder());
        Serial.print(" R:");
        Serial.println(Motors::getRightEncoder());
        logCrossection();
        break;

      case STATE_TURNING_LEFT:
      case STATE_TURNING_RIGHT:
        // Turning is blocking, this case is never reached in practice
        break;
    }
  }

  // ---- Periodic sensor readout while moving ----
  // Prints every 300ms so you can watch values change as the mouse
  // approaches walls. This helps tune FRONT_WALL_THRESHOLD.
  static unsigned long lastSensorPrint = 0;
  if (currentState == STATE_MOVING_FORWARD || currentState == STATE_MOVING_BACKWARD) {
    unsigned long now = millis();
    if (now - lastSensorPrint >= 300) {
      lastSensorPrint = now;
      SensorValues sv = Sensors::readAll();
      Serial.print("[Moving] FL:"); Serial.print(sv.frontLeft);
      Serial.print(" FR:"); Serial.print(sv.frontRight);
      Serial.print(" SL:"); Serial.print(sv.sideLeft);
      Serial.print(" SR:"); Serial.print(sv.sideRight);
      if ((sv.frontLeft + sv.frontRight) > CONFIG::FRONT_WALL_THRESHOLD) {
        Serial.print(" *** WALL AHEAD ***");
      }
      Serial.println();
    }
  }
}
