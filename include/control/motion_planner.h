// ============================================================================
//  control/motion_planner.h
//
//  Mid-level motion primitives that translate "advance one cell" or "turn
//  90 degrees right" into a trapezoidal velocity profile fed to
//  MotionController.
//
//  The planner is itself state-driven: kick off a primitive with a non-
//  blocking start_*() call and poll busy() / done() from the behaviour
//  layer.  Internally, the planner runs in the same control task as the
//  motion controller so that profile updates happen at the same rate as
//  PID updates.
// ============================================================================

#pragma once

#include "control/motion_controller.h"
#include "sensing/odometry.h"
#include "sensing/wall_view.h"

namespace control {

enum class PrimitiveResult : uint8_t {
    InProgress,
    DoneOk,
    DoneFrontWall,    // front wall reached unexpectedly -> mouse blocked
    Aborted
};

class MotionPlanner {
public:
    void begin(MotionController *mc, sensing::Odometry *o);

    // Start primitive moves.  Returns immediately; poll result().
    void start_forward_mm(float distance_mm, float cruise_mmps);
    void start_forward_cells(int cells, float cruise_mmps);
    void start_pivot_right(float angle_rad, float wheel_speed_mmps);
    void start_pivot_left (float angle_rad, float wheel_speed_mmps);
    void start_stop();           // decelerate to zero in place
    void abort();

    // Pass the latest wall view for safety (stop on unexpected front wall)
    // and for the controller's centring loop.
    void update_walls(const sensing::WallView &v);

    // Call from the control task at CONTROL_LOOP_HZ.
    PrimitiveResult tick(float dt_seconds);

    bool busy() const   { return m_state != State::Idle; }
    PrimitiveResult last_result() const { return m_last_result; }

private:
    enum class State : uint8_t {
        Idle,
        ForwardAccel,
        ForwardCruise,
        ForwardDecel,
        Pivot,
        Stopping
    };

    MotionController  *m_mc  = nullptr;
    sensing::Odometry *m_odo = nullptr;

    State m_state = State::Idle;
    PrimitiveResult m_last_result = PrimitiveResult::DoneOk;

    // Forward profile state
    float m_start_x_mm = 0.f, m_start_y_mm = 0.f;
    float m_target_mm  = 0.f;
    float m_cruise_mmps = 0.f;
    float m_current_v   = 0.f;

    // Pivot profile state
    float m_pivot_target_rad = 0.f;
    float m_pivot_start_rad  = 0.f;
    float m_pivot_dir        = 0.f; // +1 = CCW (left), -1 = CW (right)
    float m_pivot_omega      = 0.f;

    // Cached walls -- planner reads m_view.front for safety stop.
    sensing::WallView m_view;
};

} // namespace control
