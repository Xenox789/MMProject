// ============================================================================
//  main.cpp  --  Micromouse top-level state machine.
//
//  Architecture
//  ------------
//  Arduino loop() runs the BEHAVIOUR state machine (idle / explore / return /
//  speed-run / calibrate / error).  Whenever it needs to physically move,
//  it talks to mouse::g_mouse -- the facade that hides motors, encoders,
//  IR, IMU and the high-rate control loops.
//
//  Two FreeRTOS tasks run beneath us:
//      * "ctrl"   on core 1, CONTROL_LOOP_HZ : odometry + planner + motor PID
//      * "sense"  on core 0, SENSING_LOOP_HZ : IR sampling + wall detection
//
//  All tuning happens in include/constants.h.
// ============================================================================

#include <Arduino.h>

#include "pins.h"
#include "constants.h"
#include "mouse.h"
#include "maze.h"

namespace {

// ----- High-level state ------------------------------------------------------
enum class AppState : uint8_t {
    Idle,
    Exploring,
    Returning,
    Ready,
    SpeedRun,
    Calibrate,
    Error
};

AppState g_state = AppState::Idle;

// Mouse pose in MAZE coords (cell indices + cardinal heading) -- maintained
// by the behaviour layer.  The control layer's continuous pose is metric and
// independent.
int       g_cell_x  = 0;
int       g_cell_y  = 0;
Direction g_heading = NORTH;

// Button debouncing
unsigned long g_last_btn_ms = 0;
bool button_pressed() {
    if (digitalRead(PIN::BTN) == LOW &&
        millis() - g_last_btn_ms > CONFIG::BUTTON_DEBOUNCE_MS) {
        g_last_btn_ms = millis();
        return true;
    }
    return false;
}

inline void led1(bool on) { digitalWrite(PIN::DEBUG_LED_1, on ? HIGH : LOW); }
inline void led2(bool on) { digitalWrite(PIN::DEBUG_LED_2, on ? HIGH : LOW); }

// ----- Mapping from a maze RelativeDir to a Mouse turn primitive -------------
void execute_relative_turn(RelativeDir rd) {
    using mouse::TurnDirection;
    switch (rd) {
    case REL_FORWARD: /* no-op */ return;
    case REL_RIGHT:   mouse::g_mouse.turn(TurnDirection::Right,  CONFIG::TURN_SPEED_MMPS); break;
    case REL_LEFT:    mouse::g_mouse.turn(TurnDirection::Left,   CONFIG::TURN_SPEED_MMPS); break;
    case REL_BACK:    mouse::g_mouse.turn(TurnDirection::Around, CONFIG::TURN_SPEED_MMPS); break;
    }
    mouse::g_mouse.wait_until_done(2000);
}

// ----- One "cell-step": sense, decide, turn, advance ------------------------
void cell_step(float cruise_mmps) {
    // 1) sense walls AT current cell (already centred from previous move)
    WallInfo wi = mouse::g_mouse.walls();   // implicit ctor from WallView
    Maze::updateWalls(g_cell_x, g_cell_y, g_heading, wi);

    Serial.printf("Pos (%d,%d)  hdg=%d  walls F=%d L=%d R=%d\n",
                  g_cell_x, g_cell_y, (int)g_heading,
                  wi.front, wi.left, wi.right);

    // 2) plan
    Maze::floodFill(CONFIG::GOAL_X1, CONFIG::GOAL_Y1,
                    CONFIG::GOAL_X2, CONFIG::GOAL_Y2);
    Direction target = Maze::bestDirection(g_cell_x, g_cell_y, g_heading);
    RelativeDir rd   = Maze::getRelativeTurn(g_heading, target);

    // 3) execute turn (if any), then move forward one cell
    execute_relative_turn(rd);
    g_heading = target;

    mouse::g_mouse.forward_cells(1, cruise_mmps);
    auto res = mouse::g_mouse.wait_until_done(5000);
    if (res == mouse::MotionResult::BlockedByWall) {
        // Front wall surprise -- mark it and abort the step; flood-fill will
        // re-plan from the same cell on the next iteration.
        Maze::setWall(g_cell_x, g_cell_y, g_heading);
        return;
    }
    if (res == mouse::MotionResult::Aborted) {
        g_state = AppState::Error;
        return;
    }

    // 4) advance the maze-coord state
    int nx, ny;
    Maze::getNextCell(g_cell_x, g_cell_y, g_heading, nx, ny);
    if (nx < 0 || nx >= CONFIG::MAZE_SIZE || ny < 0 || ny >= CONFIG::MAZE_SIZE) {
        Serial.println("ERROR: out of bounds!");
        g_state = AppState::Error;
        return;
    }
    g_cell_x = nx;
    g_cell_y = ny;
}

void reset_pose_to_start() {
    g_cell_x = 0;
    g_cell_y = 0;
    g_heading = NORTH;
}

} // namespace


// ============================================================================
//  setup / loop
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n==============================");
    Serial.println("  Micromouse  --  layered fw  ");
    Serial.println("==============================");

    pinMode(PIN::BTN,         INPUT_PULLUP);
    pinMode(PIN::DEBUG_LED_1, OUTPUT);
    pinMode(PIN::DEBUG_LED_2, OUTPUT);

    Maze::init();

    if (!mouse::g_mouse.begin()) {
        Serial.println("[main] mouse bring-up failed; entering ERROR");
        g_state = AppState::Error;
        return;
    }

    mouse::g_mouse.enable_motors();

    if (mouse::g_mouse.motor_fault()) {
        Serial.println("[main] motor FAULT at boot");
        g_state = AppState::Error;
    } else if (mouse::g_mouse.battery_low()) {
        Serial.println("[main] battery LOW at boot");
        g_state = AppState::Error;
    } else {
        Serial.println("Ready. Press BTN to start exploration.");
    }
}

void loop() {
    switch (g_state) {

    // ---- IDLE ----
    case AppState::Idle: {
        led1((millis() / 500) % 2);
        led2(false);

        if (button_pressed()) {
            // long-press detection for calibrate mode
            const unsigned long start = millis();
            while (digitalRead(PIN::BTN) == LOW) {
                if (millis() - start > 2000) {
                    g_state = AppState::Calibrate;
                    Serial.println(">> CALIBRATE");
                    return;
                }
            }
            Serial.println(">> EXPLORE");
            led1(true);
            delay(CONFIG::STARTUP_DELAY_MS);

            reset_pose_to_start();
            Maze::init();
            g_state = AppState::Exploring;
        }
        break;
    }

    // ---- EXPLORING ----
    case AppState::Exploring: {
        led1(true); led2(false);

        if (mouse::g_mouse.motor_fault() || mouse::g_mouse.battery_low()) {
            mouse::g_mouse.emergency_brake();
            g_state = AppState::Error;
            break;
        }

        cell_step(CONFIG::EXPLORE_SPEED_MMPS);

        if (Maze::isGoal(g_cell_x, g_cell_y)) {
            mouse::g_mouse.stop();
            mouse::g_mouse.wait_until_done(1000);
            Serial.println("=== GOAL REACHED ===");
            Maze::printMaze();
            g_state = AppState::Returning;
        }
        break;
    }

    // ---- RETURNING ----
    case AppState::Returning: {
        led1((millis() / 200) % 2);
        led2(false);

        if (mouse::g_mouse.motor_fault() || mouse::g_mouse.battery_low()) {
            mouse::g_mouse.emergency_brake();
            g_state = AppState::Error;
            break;
        }

        // sense + plan toward start
        WallInfo wi = mouse::g_mouse.walls();
        Maze::updateWalls(g_cell_x, g_cell_y, g_heading, wi);
        Maze::floodFillToStart();
        Direction target = Maze::bestDirection(g_cell_x, g_cell_y, g_heading);
        execute_relative_turn(Maze::getRelativeTurn(g_heading, target));
        g_heading = target;
        mouse::g_mouse.forward_cells(1, CONFIG::EXPLORE_SPEED_MMPS);
        mouse::g_mouse.wait_until_done(5000);

        int nx, ny;
        Maze::getNextCell(g_cell_x, g_cell_y, g_heading, nx, ny);
        g_cell_x = nx; g_cell_y = ny;

        if (g_cell_x == 0 && g_cell_y == 0) {
            mouse::g_mouse.stop();
            mouse::g_mouse.wait_until_done(1000);
            Serial.println("=== BACK AT START -- press BTN for SPEED RUN ===");
            g_state = AppState::Ready;
        }
        break;
    }

    // ---- READY ----
    case AppState::Ready: {
        led1(false);
        led2((millis() / 500) % 2);

        if (button_pressed()) {
            Serial.println(">> SPEED RUN");
            led2(true);
            delay(CONFIG::STARTUP_DELAY_MS);
            reset_pose_to_start();
            Maze::floodFill(CONFIG::GOAL_X1, CONFIG::GOAL_Y1,
                            CONFIG::GOAL_X2, CONFIG::GOAL_Y2);
            g_state = AppState::SpeedRun;
        }
        break;
    }

    // ---- SPEED RUN ----
    case AppState::SpeedRun: {
        led1(false); led2(true);

        if (mouse::g_mouse.motor_fault() || mouse::g_mouse.battery_low()) {
            mouse::g_mouse.emergency_brake();
            g_state = AppState::Error;
            break;
        }

        cell_step(CONFIG::SPEED_RUN_MMPS);

        if (Maze::isGoal(g_cell_x, g_cell_y)) {
            mouse::g_mouse.stop();
            mouse::g_mouse.wait_until_done(1000);
            Serial.println("=== SPEED RUN COMPLETE ===");
            g_state = AppState::Idle;
        }
        break;
    }

    // ---- CALIBRATE ----
    case AppState::Calibrate: {
        led1(true); led2(true);
        mouse::g_mouse.print_diag();
        delay(200);
        if (button_pressed()) {
            mouse::g_mouse.recalibrate_imu();
            Serial.println(">> exit CALIBRATE");
            g_state = AppState::Idle;
        }
        break;
    }

    // ---- ERROR ----
    case AppState::Error: {
        mouse::g_mouse.emergency_brake();
        const bool on = (millis() / CONFIG::FAULT_BLINK_MS) % 2;
        led1(on); led2(on);

        static unsigned long last = 0;
        if (millis() - last > 2000) {
            last = millis();
            Serial.printf("ERROR fault=%d batt_low=%d\n",
                          (int)mouse::g_mouse.motor_fault(),
                          (int)mouse::g_mouse.battery_low());
        }
        if (button_pressed()) g_state = AppState::Idle;
        break;
    }

    } // switch
}
