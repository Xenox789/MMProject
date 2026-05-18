// ============================================================================
//  sensing/odometry.h  --  Fuses encoders + IMU into a 2D pose.
//
//  We use the IMU for instantaneous yaw rate (high bandwidth, zero
//  scrubbing error) and the encoders for distance.  Heading is the time
//  integral of the IMU yaw rate; the encoder-derived heading is exposed
//  separately as a slow-but-bias-free reference that callers may use to
//  detrend IMU drift between cells.
//
//  All angles are radians.  Pose is in millimetres / radians.
// ============================================================================

#pragma once

#include "hal/encoder.h"
#include "hal/imu.h"

namespace sensing {

struct Pose {
    float x_mm = 0.f;
    float y_mm = 0.f;
    float heading_rad = 0.f;
    float v_mmps = 0.f;
    float w_radps = 0.f;
};

class Odometry {
public:
    void begin(hal::Encoders *enc, hal::Imu *imu);

    // Call from the control task at CONTROL_LOOP_HZ.  dt is the actual
    // elapsed seconds since the previous tick().
    void tick(float dt_seconds);

    // Snapshot of the current pose.  Safe to read from other tasks (the
    // values are scalar so torn reads are bounded; downstream logic deals
    // in tolerances much wider than scalar tearing).
    Pose pose() const { return m_pose; }

    // Cumulative wheel distances since reset(); useful for "advance N mm"
    // primitives.
    float left_mm() const  { return m_left_mm; }
    float right_mm() const { return m_right_mm; }

    void reset_pose();
    void reset_distance();   // zero left_mm / right_mm only, keep heading

private:
    hal::Encoders *m_enc = nullptr;
    hal::Imu      *m_imu = nullptr;

    int64_t m_last_l_ticks = 0;
    int64_t m_last_r_ticks = 0;

    Pose  m_pose;
    float m_left_mm  = 0.f;
    float m_right_mm = 0.f;
};

} // namespace sensing
