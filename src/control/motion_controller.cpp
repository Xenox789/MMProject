#include "control/motion_controller.h"
#include "constants.h"

#include <cmath>

namespace control {

namespace {

inline float mmps_to_tickps(float mmps) { return mmps / CONFIG::MM_PER_TICK; }

float wrap_pi(float a) {
    while (a >  3.14159265f) a -= 6.2831853f;
    while (a < -3.14159265f) a += 6.2831853f;
    return a;
}

} // namespace

void MotionController::begin(hal::MotorDriver *m,
                             hal::Encoders *e,
                             sensing::Odometry *o) {
    m_motor = m;
    m_enc   = e;
    m_odo   = o;

    PidGains gw {
        CONFIG::WHEEL_PID_KP, CONFIG::WHEEL_PID_KI, CONFIG::WHEEL_PID_KD,
        CONFIG::WHEEL_PID_I_CLAMP,
        -CONFIG::MOTOR_MAX_DUTY, CONFIG::MOTOR_MAX_DUTY
    };
    m_pid_l.set_gains(gw);
    m_pid_r.set_gains(gw);

    m_pid_heading.set_gains({
        CONFIG::HEADING_PID_KP, CONFIG::HEADING_PID_KI, CONFIG::HEADING_PID_KD,
        500.f, -2000.f, 2000.f
    });
    m_pid_lateral.set_gains({
        CONFIG::LATERAL_PID_KP, CONFIG::LATERAL_PID_KI, CONFIG::LATERAL_PID_KD,
        500.f, -2000.f, 2000.f
    });

    m_last_l_ticks = e ? e->left_ticks()  : 0;
    m_last_r_ticks = e ? e->right_ticks() : 0;
}

void MotionController::set_wall_view(const sensing::WallView &v) { m_view = v; }

void MotionController::set_velocity(float v_mmps, float w_radps) {
    m_v_set = v_mmps;
    m_w_set = w_radps;
}

void MotionController::set_target_heading(float h_rad, bool hold) {
    m_heading_set  = h_rad;
    m_hold_heading = hold;
}

void MotionController::brake() {
    m_v_set = 0.f;
    m_w_set = 0.f;
    if (m_motor) m_motor->brake();
}

void MotionController::coast() {
    m_v_set = 0.f;
    m_w_set = 0.f;
    if (m_motor) m_motor->coast();
}

void MotionController::tick(float dt) {
    if (!m_motor || !m_enc || !m_odo || dt <= 0.f) return;

    // --- 1.  Measure wheel speeds (ticks/s) from raw encoder deltas. -----
    const int64_t l = m_enc->left_ticks();
    const int64_t r = m_enc->right_ticks();
    const int64_t dl = l - m_last_l_ticks;
    const int64_t dr = r - m_last_r_ticks;
    m_last_l_ticks = l;
    m_last_r_ticks = r;
    const float v_l_meas = dl / dt; // ticks/s
    const float v_r_meas = dr / dt;

    // --- 2.  Compose a "correction" yaw rate from heading/lateral PIDs. --
    float w_corr = 0.f;
    if (m_hold_heading) {
        const float herr = wrap_pi(m_heading_set - m_odo->pose().heading_rad);
        // ticks/s -- this becomes a differential bias added below
        w_corr += m_pid_heading.update(herr, 0.f, dt);
    }
    if (m_view.left || m_view.right) {
        // Negative lateral_error means we are too close to the right wall;
        // steer LEFT, i.e. yaw positive (CCW).  Sign chosen consistent with
        // wall_detector.cpp: lateral_error positive -> steer right.
        const float lerr = static_cast<float>(-m_view.lateral_error);
        w_corr += m_pid_lateral.update(lerr, 0.f, dt);
    }

    // --- 3.  Differential drive split. ----------------------------------
    const float half_W = 0.5f * CONFIG::WHEELBASE_MM; // mm
    const float v_set_mmps_l = m_v_set - m_w_set * half_W; // mm/s
    const float v_set_mmps_r = m_v_set + m_w_set * half_W;

    // convert to ticks/s, add correction (already in ticks/s)
    const float sp_l = mmps_to_tickps(v_set_mmps_l) - w_corr;
    const float sp_r = mmps_to_tickps(v_set_mmps_r) + w_corr;

    // --- 4.  Per-wheel velocity PID -> duty. ----------------------------
    const float duty_l = m_pid_l.update(sp_l, v_l_meas, dt);
    const float duty_r = m_pid_r.update(sp_r, v_r_meas, dt);

    m_motor->set_left (duty_l);
    m_motor->set_right(duty_r);

    // Let the motor driver slew + write its PWM registers.
    m_motor->tick(dt);
}

} // namespace control
