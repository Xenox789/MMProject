#include "control/motion_planner.h"
#include "constants.h"

#include <cmath>
#include <algorithm>

namespace control {

namespace {
float wrap_pi(float a) {
    while (a >  3.14159265f) a -= 6.2831853f;
    while (a < -3.14159265f) a += 6.2831853f;
    return a;
}
} // namespace

void MotionPlanner::begin(MotionController *mc, sensing::Odometry *o) {
    m_mc  = mc;
    m_odo = o;
}

void MotionPlanner::update_walls(const sensing::WallView &v) {
    m_view = v;
    if (m_mc) m_mc->set_wall_view(v);
}

void MotionPlanner::start_forward_mm(float distance_mm, float cruise_mmps) {
    if (!m_mc || !m_odo) return;

    const auto p = m_odo->pose();
    m_start_x_mm = p.x_mm;
    m_start_y_mm = p.y_mm;
    m_target_mm  = distance_mm;
    m_cruise_mmps = cruise_mmps;
    m_current_v   = std::max(0.f, m_mc->commanded_v());

    // Hold current heading throughout the move.
    m_mc->set_target_heading(p.heading_rad, true);

    m_state = State::ForwardAccel;
    m_last_result = PrimitiveResult::InProgress;
}

void MotionPlanner::start_forward_cells(int cells, float cruise_mmps) {
    start_forward_mm(cells * CONFIG::CELL_SIZE_MM, cruise_mmps);
}

void MotionPlanner::start_pivot_right(float angle_rad, float wheel_speed_mmps) {
    if (!m_mc || !m_odo) return;
    const auto p = m_odo->pose();
    m_pivot_start_rad  = p.heading_rad;
    m_pivot_target_rad = wrap_pi(p.heading_rad - std::fabs(angle_rad));
    m_pivot_dir        = -1.f;
    m_pivot_omega      = (2.f * wheel_speed_mmps) / CONFIG::WHEELBASE_MM;
    m_mc->set_target_heading(m_pivot_target_rad, false); // free heading PID
    m_mc->set_velocity(0.f, -m_pivot_omega);
    m_state = State::Pivot;
    m_last_result = PrimitiveResult::InProgress;
}

void MotionPlanner::start_pivot_left(float angle_rad, float wheel_speed_mmps) {
    if (!m_mc || !m_odo) return;
    const auto p = m_odo->pose();
    m_pivot_start_rad  = p.heading_rad;
    m_pivot_target_rad = wrap_pi(p.heading_rad + std::fabs(angle_rad));
    m_pivot_dir        = +1.f;
    m_pivot_omega      = (2.f * wheel_speed_mmps) / CONFIG::WHEELBASE_MM;
    m_mc->set_target_heading(m_pivot_target_rad, false);
    m_mc->set_velocity(0.f, +m_pivot_omega);
    m_state = State::Pivot;
    m_last_result = PrimitiveResult::InProgress;
}

void MotionPlanner::start_stop() {
    if (!m_mc) return;
    m_state = State::Stopping;
    m_last_result = PrimitiveResult::InProgress;
    m_current_v = m_mc->commanded_v();
}

void MotionPlanner::abort() {
    if (m_mc) m_mc->brake();
    m_state = State::Idle;
    m_last_result = PrimitiveResult::Aborted;
}

PrimitiveResult MotionPlanner::tick(float dt) {
    if (!m_mc || !m_odo || m_state == State::Idle) {
        return m_last_result;
    }

    const auto p = m_odo->pose();

    switch (m_state) {

    // ------------------------------------------------------------------ //
    case State::ForwardAccel:
    case State::ForwardCruise:
    case State::ForwardDecel: {
        // Distance travelled from where the primitive began.
        const float dx = p.x_mm - m_start_x_mm;
        const float dy = p.y_mm - m_start_y_mm;
        const float travelled = std::sqrt(dx*dx + dy*dy);

        // Safety abort on unexpected front wall while still far from target.
        if (m_view.front && (m_target_mm - travelled) > CONFIG::CELL_SIZE_MM*0.4f) {
            m_mc->brake();
            m_state = State::Idle;
            m_last_result = PrimitiveResult::DoneFrontWall;
            return m_last_result;
        }

        // Brake distance at current speed for the configured decel.
        const float brake_dist =
            (m_current_v * m_current_v) / (2.f * CONFIG::DECEL_MMPS2);

        if (travelled + brake_dist >= m_target_mm) {
            m_state = State::ForwardDecel;
            m_current_v -= CONFIG::DECEL_MMPS2 * dt;
            if (m_current_v < 0.f) m_current_v = 0.f;
        } else if (m_current_v < m_cruise_mmps) {
            m_state = State::ForwardAccel;
            m_current_v += CONFIG::ACCEL_MMPS2 * dt;
            if (m_current_v > m_cruise_mmps) m_current_v = m_cruise_mmps;
        } else {
            m_state = State::ForwardCruise;
        }

        m_mc->set_velocity(m_current_v, 0.f);

        // Done?
        if (travelled >= m_target_mm - CONFIG::POS_TOLERANCE_MM
            && m_current_v < 1.0f) {
            m_mc->set_velocity(0.f, 0.f);
            m_state = State::Idle;
            m_last_result = PrimitiveResult::DoneOk;
        }
        return m_last_result;
    }

    // ------------------------------------------------------------------ //
    case State::Pivot: {
        const float err = wrap_pi(m_pivot_target_rad - p.heading_rad);
        if (std::fabs(err) < CONFIG::HEADING_TOL_RAD) {
            m_mc->set_velocity(0.f, 0.f);
            // Re-engage heading hold at the new heading.
            m_mc->set_target_heading(m_pivot_target_rad, true);
            m_state = State::Idle;
            m_last_result = PrimitiveResult::DoneOk;
        } else {
            // Bang-bang-ish; the heading-hold PID will polish the final
            // approach once we engage it on Idle.
            const float omega = m_pivot_dir * m_pivot_omega;
            m_mc->set_velocity(0.f, omega);
        }
        return m_last_result;
    }

    // ------------------------------------------------------------------ //
    case State::Stopping: {
        m_current_v -= CONFIG::DECEL_MMPS2 * dt;
        if (m_current_v <= 0.f) {
            m_current_v = 0.f;
            m_mc->set_velocity(0.f, 0.f);
            m_state = State::Idle;
            m_last_result = PrimitiveResult::DoneOk;
        } else {
            m_mc->set_velocity(m_current_v, 0.f);
        }
        return m_last_result;
    }

    default:
        return m_last_result;
    }
}

} // namespace control
