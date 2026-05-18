// Micromouse pinout
#pragma once

namespace PIN {

// Button
constexpr int BTN = 0;

// Debug LEDS
constexpr int DEBUG_LED_1 = 1;
constexpr int DEBUG_LED_2 = 2;

// Motors
constexpr int MOTOR_DRV_EN = 41;
constexpr int MOTOR_DRV_FAULT = 40;
constexpr int MOTOR_A_1 = 11;
constexpr int MOTOR_A_2 = 13;
constexpr int MOTOR_B_1 = 14;
constexpr int MOTOR_B_2 = 12;

constexpr int MOTOR_A_ENC_A = 9;
constexpr int MOTOR_A_ENC_B = 10;
constexpr int MOTOR_B_ENC_A = 48;
constexpr int MOTOR_B_ENC_B = 42;

// Infra distance sensors
constexpr int IR_LED_1 = 18;  // Side Left (60 degrees)
constexpr int IR_LED_2 = 17;  // Front Left (5 degrees)
constexpr int IR_LED_3 = 15;  // Front Right (5 degrees)
constexpr int IR_LED_4 = 16;  // Side Right (60 degrees)

constexpr int IR_1 = 5;   // Side Left sensor
constexpr int IR_2 = 7;   // Front Left sensor
constexpr int IR_3 = 4;   // Front Right sensor
constexpr int IR_4 = 6;   // Side Right sensor

// IMU
constexpr int IMU_SDA = 21;
constexpr int IMU_SCL = 47;
constexpr int IMU_INT = 38;

// Battery sensor
constexpr int BATT = 8;

} // namespace PIN
