// ============================================================================
//  control/pid.h  --  Reusable, header-only PID controller.
//
//  Features:
//      * Discrete-time PID with derivative on MEASUREMENT (not error) so it
//        doesn't kick on setpoint changes.
//      * Integral clamp (anti-windup).
//      * Optional output clamp; the integrator is also frozen when output
//        is saturated AND the integral term would push it further into
//        saturation (back-calculation).
// ============================================================================

#pragma once

#include <algorithm>

namespace control {

struct PidGains {
    float kp = 0.f, ki = 0.f, kd = 0.f;
    float i_clamp = 1e9f;        // max |integral term|
    float out_min = -1e9f;
    float out_max =  1e9f;

    PidGains() = default;
    PidGains(float kp_, float ki_, float kd_,
             float i_clamp_ = 1e9f, float out_min_ = -1e9f, float out_max_ = 1e9f)
        : kp(kp_), ki(ki_), kd(kd_), i_clamp(i_clamp_), out_min(out_min_), out_max(out_max_) {}
};

class Pid {
public:
    Pid() = default;
    explicit Pid(const PidGains &g) : m_g(g) {}

    void set_gains(const PidGains &g) { m_g = g; }
    void reset() { m_integ = 0.f; m_last_meas = 0.f; m_has_last = false; }

    // setpoint, measurement, dt seconds  ->  control output
    float update(float setpoint, float measurement, float dt) {
        const float err = setpoint - measurement;

        // P
        const float p = m_g.kp * err;

        // I (with clamp)
        m_integ += m_g.ki * err * dt;
        if (m_integ >  m_g.i_clamp) m_integ =  m_g.i_clamp;
        if (m_integ < -m_g.i_clamp) m_integ = -m_g.i_clamp;

        // D on measurement
        float d = 0.f;
        if (m_has_last && dt > 0.f) {
            d = -m_g.kd * (measurement - m_last_meas) / dt;
        }
        m_last_meas = measurement;
        m_has_last  = true;

        float out = p + m_integ + d;

        // Output clamp + back-calculate to undo wind-up.
        if (out > m_g.out_max) {
            m_integ -= (out - m_g.out_max);
            out = m_g.out_max;
        } else if (out < m_g.out_min) {
            m_integ -= (out - m_g.out_min);
            out = m_g.out_min;
        }
        return out;
    }

private:
    PidGains m_g;
    float m_integ      = 0.f;
    float m_last_meas  = 0.f;
    bool  m_has_last   = false;
};

} // namespace control
