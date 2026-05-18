// ============================================================================
//  control/motion_controller.h
//
//  Owns the high-rate (CONTROL_LOOP_HZ) control loop:
//
//        +--------------+
//        |  setpoint    |   v* = forward speed (mm/s)
//        |  ( v*, w* )  |   w* = yaw rate      (rad/s)  -- usually 0 unless
//        |              |        turning in place
//        +------+-------+
//               |
//               v
//        differential:  v_left*  = v* - w*(W/2)
//                       v_right* = v* + w*(W/2)
//
//        wheel-velocity PID per wheel  ->  duty in [-1, 1]
//        +  superimposed correction = headingPID + lateralPID
//
//  Inputs:
//      odometry     -- pose (heading, wheel velocities)
//      wall_view    -- lateral error for centring
//
//  Output:
//      motor_driver.set_left/set_right + tick()
//
//  Public API is just a few "commanded" setters; the controller is itself
//  consumed by the motion_planner (which sequences moves).
// ============================================================================

#pragma once

#include "control/pid.h"
#include "hal/motor_driver.h"
#include "hal/encoder.h"
#include "sensing/odometry.h"
#include "sensing/wall_view.h"

namespace control {

class MotionController {
public:
    void begin(hal::MotorDriver *m, hal::Encoders *e, sensing::Odometry *o);

    // Snapshot the latest WallView from the sensing task.  Cheap, lock-
    // free (caller copies scalars).  Setting view.left=view.right=false
    // disables lateral correction (heading-hold only).
    void set_wall_view(const sensing::WallView &v);

    // Commanded body-frame velocity setpoints.
    void set_velocity(float v_mmps, float w_radps);
    void set_target_heading(float heading_rad, bool enable_hold);

    // Halt and brake (zero setpoint + slew to zero + brake on engage).
    void brake();
    void coast();

    // Call from the control task every dt seconds.
    void tick(float dt_seconds);

    // Telemetry
    float commanded_v() const { return m_v_set; }
    float commanded_w() const { return m_w_set; }

private:
    hal::MotorDriver  *m_motor = nullptr;
    hal::Encoders     *m_enc   = nullptr;
    sensing::Odometry *m_odo   = nullptr;

    // Per-wheel velocity PIDs
    Pid m_pid_l, m_pid_r;

    // Higher-order PIDs
    Pid m_pid_heading;
    Pid m_pid_lateral;

    // Setpoints
    float m_v_set         = 0.f;
    float m_w_set         = 0.f;
    float m_heading_set   = 0.f;
    bool  m_hold_heading  = true;

    // Latest wall view (heading-priority fallback when corridor open)
    sensing::WallView m_view;

    // Cached tick state for derivative wheel speed estimation
    int64_t m_last_l_ticks = 0;
    int64_t m_last_r_ticks = 0;
};

} // namespace control
