// ============================================================================
//  hal/imu.h  --  MPU-6000 driver, I2C, gyro + accel.
//
//  Only the bare minimum the control loop needs is exposed here:
//      * yaw_rate_dps()  -- Z gyro in deg/s, bias-subtracted
//      * read_raw()      -- full sample (gyro + accel) if you want it
//
//  Sensor fusion (yaw integration, complementary filter etc.) lives one
//  level up in sensing/odometry.h.
// ============================================================================

#pragma once

#include <cstddef>
#include <cstdint>

namespace hal {

struct ImuSample {
    float gx_dps, gy_dps, gz_dps;   // gyro, deg/s, bias-subtracted
    float ax_g, ay_g, az_g;         // accel, g
};

class Imu {
public:
    // Initialise I2C, wake the device, set ranges/filters, then measure the
    // gyro bias with the mouse stationary.  Blocks for ~0.5 s.  Returns
    // false if the WHO_AM_I check fails.
    bool begin();

    // Single synchronous sample.
    ImuSample read();

    // Quick path used by the control task: returns Z gyro rate only.
    float yaw_rate_dps();

    // Replace the bias estimate with a fresh measurement (assumes the
    // chassis is still).  Useful if the mouse warms up.
    void recalibrate_bias();

private:
    bool wr(uint8_t reg, uint8_t val);
    bool rd(uint8_t reg, uint8_t *buf, size_t n);

    float m_gx_bias = 0.f, m_gy_bias = 0.f, m_gz_bias = 0.f;
    float m_gyro_lsb_per_dps = 65.5f; // for +/- 500 dps full scale
    float m_accel_lsb_per_g  = 8192.f; // for +/- 4 g  full scale
};

} // namespace hal
