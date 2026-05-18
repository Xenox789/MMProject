#include "motors.h"
#include "pins.h"
#include "config.h"

namespace Motors {

// PWM channels
static const int CH_A1 = 0;
static const int CH_A2 = 1;
static const int CH_B1 = 2;
static const int CH_B2 = 3;

// Encoder counters (volatile for ISR access)
static volatile long encoderLeftCount = 0;
static volatile long encoderRightCount = 0;

// PID state
static float pidIntegral = 0;
static float pidLastError = 0;

// --- Encoder ISRs ---
static void IRAM_ATTR onEncoderLeftA() {
  if (digitalRead(PIN::MOTOR_A_ENC_B) == LOW) {
    encoderLeftCount++;
  } else {
    encoderLeftCount--;
  }
}

static void IRAM_ATTR onEncoderRightA() {
  if (digitalRead(PIN::MOTOR_B_ENC_B) == LOW) {
    encoderRightCount++;
  } else {
    encoderRightCount--;
  }
}

void init() {
  // Motor driver pins
  pinMode(PIN::MOTOR_DRV_EN, OUTPUT);
  pinMode(PIN::MOTOR_DRV_FAULT, INPUT);
  
  // Encoder pins
  pinMode(PIN::MOTOR_A_ENC_A, INPUT_PULLUP);
  pinMode(PIN::MOTOR_A_ENC_B, INPUT_PULLUP);
  pinMode(PIN::MOTOR_B_ENC_A, INPUT_PULLUP);
  pinMode(PIN::MOTOR_B_ENC_B, INPUT_PULLUP);
  
  // Add a small delay to allow pins to stabilize before attaching interrupts
  delay(100);
  
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(PIN::MOTOR_A_ENC_A), onEncoderLeftA, RISING);
  // attachInterrupt(digitalPinToInterrupt(PIN::MOTOR_B_ENC_A), onEncoderRightA, RISING); // <-- Temporarily disabled for testing
  
  // Setup PWM channels
  ledcSetup(CH_A1, CONFIG::PWM_FREQUENCY, CONFIG::PWM_RESOLUTION);
  ledcSetup(CH_A2, CONFIG::PWM_FREQUENCY, CONFIG::PWM_RESOLUTION);
  ledcSetup(CH_B1, CONFIG::PWM_FREQUENCY, CONFIG::PWM_RESOLUTION);
  ledcSetup(CH_B2, CONFIG::PWM_FREQUENCY, CONFIG::PWM_RESOLUTION);
  
  // Attach PWM to pins
  ledcAttachPin(PIN::MOTOR_A_1, CH_A1);
  ledcAttachPin(PIN::MOTOR_A_2, CH_A2);
  ledcAttachPin(PIN::MOTOR_B_1, CH_B1);
  ledcAttachPin(PIN::MOTOR_B_2, CH_B2);
  
  stop();
}

void enable() {
  digitalWrite(PIN::MOTOR_DRV_EN, HIGH);
}

void disable() {
  stop();
  digitalWrite(PIN::MOTOR_DRV_EN, LOW);
}

bool isFault() {
  return digitalRead(PIN::MOTOR_DRV_FAULT) == LOW;
}

// Set left motor speed: -255 (backward) to +255 (forward)
// Note: Motor A is physically reversed on this board
void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    // Forward for left motor (reversed wiring)
    ledcWrite(CH_A1, 0);
    ledcWrite(CH_A2, speed);
  } else {
    // Backward for left motor (reversed wiring)
    ledcWrite(CH_A1, -speed);
    ledcWrite(CH_A2, 0);
  }
}

// Set right motor speed: -255 (backward) to +255 (forward)
void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    // Forward for right motor
    ledcWrite(CH_B1, speed);
    ledcWrite(CH_B2, 0);
  } else {
    // Backward for right motor
    ledcWrite(CH_B1, 0);
    ledcWrite(CH_B2, -speed);
  }
}

void stop() {
  ledcWrite(CH_A1, 0);
  ledcWrite(CH_A2, 0);
  ledcWrite(CH_B1, 0);
  ledcWrite(CH_B2, 0);
}

void resetEncoders() {
  noInterrupts();
  encoderLeftCount = 0;
  encoderRightCount = 0;
  interrupts();
}

long getLeftEncoder() {
  noInterrupts();
  long val = encoderLeftCount;
  interrupts();
  return val;
}

long getRightEncoder() {
  noInterrupts();
  long val = encoderRightCount;
  interrupts();
  return val;
}

float getLeftDistanceMM() {
  return getLeftEncoder() * CONFIG::MM_PER_TICK;
}

float getRightDistanceMM() {
  return getRightEncoder() * CONFIG::MM_PER_TICK;
}

// --- PID ---

void pidReset() {
  pidIntegral = 0;
  pidLastError = 0;
}

// PID to keep both wheels at same speed (straight line driving)
// Call this in a tight loop while moving forward
void pidUpdate(int baseSpeed) {
  long left = getLeftEncoder();
  long right = getRightEncoder();
  
  // Error = difference in encoder counts (we want them equal)
  float error = (float)(left - right);
  
  pidIntegral += error;
  pidIntegral = constrain(pidIntegral, -1000.0f, 1000.0f);
  
  float derivative = error - pidLastError;
  pidLastError = error;
  
  float correction = CONFIG::PID_KP * error 
                   + CONFIG::PID_KI * pidIntegral 
                   + CONFIG::PID_KD * derivative;
  
  int leftSpeed  = baseSpeed - (int)(correction / 2);
  int rightSpeed = baseSpeed + (int)(correction / 2);
  
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

// --- High-Level Movements ---

void moveForwardCells(int cells, int baseSpeed) {
  long targetTicks = (long)cells * CONFIG::TICKS_PER_CELL;
  
  resetEncoders();
  pidReset();
  
  while (true) {
    long avgTicks = (getLeftEncoder() + getRightEncoder()) / 2;
    if (avgTicks >= targetTicks) break;
    
    pidUpdate(baseSpeed);
    delayMicroseconds(CONFIG::LOOP_INTERVAL_US);
  }
  
  stop();
}

void turnRight90(int speed) {
  long targetTicks = CONFIG::TICKS_PER_90_TURN;
  
  resetEncoders();
  
  // Left wheel forward, right wheel backward
  setLeftMotor(speed);
  setRightMotor(-speed);
  
  while (true) {
    // Left encoder goes positive, right encoder goes negative
    long leftTicks = getLeftEncoder();
    long rightTicks = -getRightEncoder();  // negate because going backward
    long avgTicks = (leftTicks + rightTicks) / 2;
    
    if (avgTicks >= targetTicks) break;
    delayMicroseconds(CONFIG::LOOP_INTERVAL_US);
  }
  
  stop();
  delay(50);  // Brief settle time
}

void turnLeft90(int speed) {
  long targetTicks = CONFIG::TICKS_PER_90_TURN;
  
  resetEncoders();
  
  // Right wheel forward, left wheel backward
  setLeftMotor(-speed);
  setRightMotor(speed);
  
  while (true) {
    long leftTicks = -getLeftEncoder();
    long rightTicks = getRightEncoder();
    long avgTicks = (leftTicks + rightTicks) / 2;
    
    if (avgTicks >= targetTicks) break;
    delayMicroseconds(CONFIG::LOOP_INTERVAL_US);
  }
  
  stop();
  delay(50);
}

void turnAround(int speed) {
  turnRight90(speed);
  turnRight90(speed);
}

} // namespace Motors
