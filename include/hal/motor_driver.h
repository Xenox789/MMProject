// ============================================================================
//  hal/motor_driver.h  --  Low-level interface to the BDR6622T dual H-bridge.
//
//  Responsibilities:
//      * Configure LEDC PWM on the four motor logic inputs
//      * Drive enable / read fault line
//      * Expose set_left(duty), set_right(duty) where duty is -1.0 ... +1.0
//      * Apply: side-swap, polarity invert, dead-band skip, max-duty clamp,
//        slew-rate limit
//
//  Anything higher level than "command a wheel duty" lives in the control
//  layer (motion_controller).  This file makes NO assumptions about the
//  direction of travel, encoder feedback, or maze semantics.
// ============================================================================

#pragma once

#include <cstdint>

namespace hal {

class MotorDriver {
public:
    // Set up GPIOs, LEDC channels, leave the bridge disabled.  Idempotent.
    void begin();

    // Drive the global enable line (BDR6622T nSLEEP-style).  Both wheels
    // coast when disabled regardless of duty commanded.
    void enable();
    void disable();

    // True while the BDR6622T pulls its FAULT line low (over-current,
    // over-temp, under-voltage).  Latched until the driver is power-cycled
    // or the controller toggles enable().
    bool fault_active() const;

    // Commanded duty: -1.0 (full reverse) ... +1.0 (full forward).  Out-of-
    // range values are clamped.  Side-swap / polarity-invert / dead-band /
    // max-duty / slew-rate are applied INSIDE this call before writing the
    // PWM registers.
    //
    // "left"/"right" refer to the mouse's body frame, not the IC pins.
    void set_left(float duty);
    void set_right(float duty);

    // Convenience.  Brake = both inputs HIGH; coast = both inputs LOW.
    void brake();
    void coast();

    // Call at the control-loop rate.  Drives the slew-rate limiter so commanded
    // duty changes are bounded by CONFIG::MOTOR_SLEW_PER_MS.
    void tick(float dt_seconds);

private:
    enum Side { LEFT = 0, RIGHT = 1 };

    // Physical IC channel for a body-frame side (after MOTOR_A_IS_LEFT)
    void resolve_pins(Side s, int &in1, int &in2, bool &invert) const;

    // Write the two LEDC duties for a given side given a signed duty.
    void write_side(Side s, float signed_duty);

    // Per-side state for slew rate limiting
    float m_target_duty[2] = {0.f, 0.f};   // last commanded by user
    float m_applied_duty[2] = {0.f, 0.f};  // actually written, slew-limited

    bool m_enabled = false;
};

} // namespace hal
