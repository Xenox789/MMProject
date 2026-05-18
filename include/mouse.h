// ============================================================================
//  mouse.h  --  Top-level facade.  This is the ONLY header the maze-solving
//               code needs to include.
//
//  Design contract
//  ---------------
//  The maze solver and the high-level state machine think in terms of:
//      * "advance one cell forward, centred between walls"
//      * "turn 90 deg left/right" / "u-turn"
//      * "tell me what walls I see at the current cell"
//      * "stop"
//      * "are you idle yet?"
//
//  Everything below this facade (motors, encoders, IR, IMU, PIDs, FreeRTOS
//  tasks) is private.  Implementations can change without recompiling the
//  maze module.
//
//  Threading
//  ---------
//  The maze module runs on the Arduino loop() task.  It calls Mouse from
//  there.  The control task drives the actual motion in the background.
//  Mouse methods are non-blocking unless the name says wait_*.
// ============================================================================

#pragma once

#include "sensing/wall_view.h"

namespace mouse {

enum class TurnDirection : uint8_t {
    Left,        // 90 deg CCW
    Right,       // 90 deg CW
    Around       // 180 deg
};

enum class MotionResult : uint8_t {
    Idle,            // no command in progress
    Busy,            // motion still running
    DoneOk,
    BlockedByWall,   // unexpected front wall during forward()
    Aborted
};

class Mouse {
public:
    // One-time hardware/RTOS bring-up.  Spawns the control + sensing tasks
    // pinned per CONFIG::*_TASK_CORE.  Returns false on hardware failure
    // (IMU WHO_AM_I, motor fault at boot, etc.).
    bool begin();

    // Spin up motor driver enable / kill it.
    void enable_motors();
    void disable_motors();

    // Non-blocking commands.  Each returns immediately; poll status() or
    // call wait_until_done() afterwards.
    void forward_cells(int cells, float cruise_mmps);
    void forward_mm   (float mm,  float cruise_mmps);
    void turn(TurnDirection d, float wheel_speed_mmps);
    void stop();
    void emergency_brake();

    // Snapshot of the current control-side status.  Lock-free.
    MotionResult status() const;

    // Blocks the calling task until motion completes (or fault).
    // Returns the final result.  Caller should still check status() between
    // commands; e.g. don't issue a new forward() while busy().
    MotionResult wait_until_done(uint32_t timeout_ms = 0);

    // Walls seen RIGHT NOW at the mouse's current pose.  This is a thin
    // accessor over the latest wall-detector output -- always fresh
    // (updated at SENSING_LOOP_HZ).
    sensing::WallView walls() const;

    // Battery / fault helpers
    bool battery_low()  const;
    bool motor_fault()  const;

    // Diagnostic / calibration entry point.  Re-runs the IMU bias
    // calibration; mouse must be stationary.
    void recalibrate_imu();

    // Pretty-print one line of sensor / pose state to Serial -- for the
    // existing CALIBRATE state in main.cpp.
    void print_diag();
};

// Process-global instance.  Created in main.cpp; the maze module references
// it through this header.
extern Mouse g_mouse;

} // namespace mouse
