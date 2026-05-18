#include "sensors.h"
#include "pins.h"
#include "config.h"

namespace Sensors {

void init() {
  // IR LED pins as outputs
  pinMode(PIN::IR_LED_1, OUTPUT);
  pinMode(PIN::IR_LED_2, OUTPUT);
  pinMode(PIN::IR_LED_3, OUTPUT);
  pinMode(PIN::IR_LED_4, OUTPUT);
  
  // All LEDs off initially
  digitalWrite(PIN::IR_LED_1, LOW);
  digitalWrite(PIN::IR_LED_2, LOW);
  digitalWrite(PIN::IR_LED_3, LOW);
  digitalWrite(PIN::IR_LED_4, LOW);
  
  // Battery pin
  pinMode(PIN::BATT, INPUT);
}

// Read a single IR sensor: LED on -> read -> LED off
// Subtracts ambient reading for better accuracy
static int readSingle(int ledPin, int sensorPin) {
  // Read ambient (LED off)
  int ambient = analogRead(sensorPin);
  
  // Read with LED on
  digitalWrite(ledPin, HIGH);
  delayMicroseconds(150);
  int active = analogRead(sensorPin);
  digitalWrite(ledPin, LOW);
  
  // Return difference (removes ambient light noise)
  int value = active - ambient;
  return (value > 0) ? value : 0;
}

SensorValues readAll() {
  SensorValues sv;
  sv.sideLeft   = readSingle(PIN::IR_LED_1, PIN::IR_1);
  sv.frontLeft  = readSingle(PIN::IR_LED_2, PIN::IR_2);
  sv.frontRight = readSingle(PIN::IR_LED_3, PIN::IR_3);
  sv.sideRight  = readSingle(PIN::IR_LED_4, PIN::IR_4);
  return sv;
}

WallInfo detectWalls() {
  SensorValues sv = readAll();
  WallInfo walls;
  
  walls.front = (sv.frontLeft + sv.frontRight) > CONFIG::FRONT_WALL_THRESHOLD;
  walls.left  = sv.sideLeft > CONFIG::SIDE_WALL_THRESHOLD;
  walls.right = sv.sideRight > CONFIG::SIDE_WALL_THRESHOLD;
  
  return walls;
}

int getSideLeftError() {
  int value = readSingle(PIN::IR_LED_1, PIN::IR_1);
  return value - CONFIG::SIDE_WALL_CENTER;
}

int getSideRightError() {
  int value = readSingle(PIN::IR_LED_4, PIN::IR_4);
  return value - CONFIG::SIDE_WALL_CENTER;
}

int readBatteryRaw() {
  return analogRead(PIN::BATT);
}

bool isBatteryLow() {
  return readBatteryRaw() < CONFIG::BATTERY_LOW_THRESHOLD;
}

void printValues() {
  SensorValues sv = readAll();
  Serial.print("FL:");  Serial.print(sv.frontLeft);
  Serial.print(" FR:"); Serial.print(sv.frontRight);
  Serial.print(" SL:"); Serial.print(sv.sideLeft);
  Serial.print(" SR:"); Serial.print(sv.sideRight);
  Serial.print(" BAT:"); Serial.println(readBatteryRaw());
}

} // namespace Sensors
