// Motor control with encoder feedback and PID
#pragma once
#include <Arduino.h>

namespace Motors {

void init();
void enable();
void disable();
bool isFault();

// Basic motor control
void setLeftMotor(int speed);   // -255 to 255
void setRightMotor(int speed);  // -255 to 255
void stop();

// Encoder functions
void resetEncoders();
long getLeftEncoder();
long getRightEncoder();
float getLeftDistanceMM();
float getRightDistanceMM();

// High-level movement with PID
void moveForwardCells(int cells, int baseSpeed);
void turnRight90(int speed);
void turnLeft90(int speed);
void turnAround(int speed);

// PID update (call every loop iteration during movement)
void pidUpdate(int baseSpeed);
void pidReset();

} // namespace Motors
