// IR sensor reading and battery monitoring
#pragma once
#include <Arduino.h>

struct WallInfo {
  bool front;
  bool left;
  bool right;
};

struct SensorValues {
  int frontLeft;   // IR_2 (5 degree)
  int frontRight;  // IR_3 (5 degree)
  int sideLeft;    // IR_1 (60 degree)
  int sideRight;   // IR_4 (60 degree)
};

namespace Sensors {

void init();

// Read all four IR sensors
SensorValues readAll();

// Detect walls based on thresholds
WallInfo detectWalls();

// Get individual readings for wall-following correction
int getSideLeftError();   // Error from center for left wall following
int getSideRightError();  // Error from center for right wall following

// Battery monitoring
int readBatteryRaw();
bool isBatteryLow();

// Debug: print all sensor values
void printValues();

} // namespace Sensors
