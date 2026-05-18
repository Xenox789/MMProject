// Micromouse configuration and tuning parameters
#pragma once

namespace CONFIG {

// --- Maze Dimensions ---
constexpr int MAZE_SIZE = 16;  // Standard 16x16 micromouse maze
// Goal cells (center of maze)
constexpr int GOAL_X1 = 7;
constexpr int GOAL_Y1 = 7;
constexpr int GOAL_X2 = 8;
constexpr int GOAL_Y2 = 8;

// --- Physical Dimensions ---
constexpr float CELL_SIZE_MM = 180.0f;        // Standard cell size
constexpr float WHEEL_DIAMETER_MM = 50.0f;    // Adjust to your wheel
constexpr float WHEEL_BASE_MM = 60.0f;        // Distance between wheels
constexpr int ENCODER_CPR = 12;               // Counts per revolution of encoder
constexpr float GEAR_RATIO = 50.0f;           // Motor gear ratio

// Derived: mm per encoder tick
constexpr float MM_PER_TICK = (3.14159f * WHEEL_DIAMETER_MM) / (ENCODER_CPR * GEAR_RATIO);
// Ticks to travel one cell
constexpr int TICKS_PER_CELL = (int)(CELL_SIZE_MM / MM_PER_TICK);
// Ticks to turn 90 degrees (quarter of wheel-base circle)
constexpr int TICKS_PER_90_TURN = (int)((3.14159f * WHEEL_BASE_MM * 0.25f) / MM_PER_TICK);

// --- Motor PWM ---
constexpr int PWM_FREQUENCY = 20000;  // 20kHz - above audible range
constexpr int PWM_RESOLUTION = 8;     // 0-255

// --- Speed Profiles ---
constexpr int EXPLORE_SPEED = 120;     // PWM for exploration (slow and safe)
constexpr int SPEED_RUN_SPEED = 200;   // PWM for speed run
constexpr int TURN_SPEED = 100;        // PWM for turning

// --- Turn Timing (Step 1: timed turns, no encoder dependency) ---
// Increase if the mouse turns less than 90 degrees.
// Decrease if the mouse turns more than 90 degrees.
// Start with 500ms, observe result, and tune.
constexpr int TURN_TIME_MS = 500;

// --- PID Controller (Straight line) ---
constexpr float PID_KP = 2.0f;    // Proportional gain
constexpr float PID_KI = 0.0f;    // Integral gain (start at 0)
constexpr float PID_KD = 0.5f;    // Derivative gain

// --- PID Controller (Wall following) ---
constexpr float WALL_KP = 0.5f;
constexpr float WALL_KI = 0.0f;
constexpr float WALL_KD = 0.2f;

// --- IR Sensor Thresholds ---
// IMPORTANT: Tune these by reading sensor values with serial monitor!
constexpr int FRONT_WALL_THRESHOLD = 2000;  // Combined front sensors
constexpr int SIDE_WALL_THRESHOLD = 1200;   // Individual side sensors
constexpr int SIDE_WALL_CENTER = 800;       // Target value for wall following (centered in cell)

// --- Battery ---
// Minimum battery voltage before shutdown (in ADC counts)
constexpr int BATTERY_LOW_THRESHOLD = 2000;

// --- Timing ---
constexpr int LOOP_INTERVAL_US = 1000;       // Main loop interval (1ms)
constexpr int BUTTON_DEBOUNCE_MS = 200;      // Button debounce time
constexpr int STARTUP_DELAY_MS = 1000;       // Delay after button press before starting

} // namespace CONFIG
