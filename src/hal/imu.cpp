// ============================================================================
//  hal/imu.cpp  --  MPU-6000 driver implementation.
//
//  Register references come straight out of the MPU-6000 register map; the
//  bare minimum we touch:
//      0x6B  PWR_MGMT_1     -- wake, clock select
//      0x1A  CONFIG         -- DLPF
//      0x1B  GYRO_CONFIG    -- gyro full scale
//      0x1C  ACCEL_CONFIG   -- accel full scale
//      0x19  SMPLRT_DIV     -- sample rate divider
//      0x75  WHO_AM_I       -- returns 0x68
//      0x3B  ACCEL_XOUT_H   -- 14 bytes burst read
// ============================================================================

#include "hal/imu.h"
#include "constants.h"
#include "pins.h"

#include <Arduino.h>
#include <Wire.h>

namespace hal {

namespace {

constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

inline int16_t be16(uint8_t hi, uint8_t lo) {
    return static_cast<int16_t>((hi << 8) | lo);
}

uint8_t gyro_fs_bits(int dps) {
    if (dps <= 250)  return 0x00;
    if (dps <= 500)  return 0x08;
    if (dps <= 1000) return 0x10;
    return 0x18; // 2000 dps
}
float gyro_lsb_per_dps(int dps) {
    if (dps <= 250)  return 131.0f;
    if (dps <= 500)  return 65.5f;
    if (dps <= 1000) return 32.8f;
    return 16.4f;
}

uint8_t accel_fs_bits(int g) {
    if (g <= 2)  return 0x00;
    if (g <= 4)  return 0x08;
    if (g <= 8)  return 0x10;
    return 0x18;
}
float accel_lsb_per_g(int g) {
    if (g <= 2)  return 16384.f;
    if (g <= 4)  return 8192.f;
    if (g <= 8)  return 4096.f;
    return 2048.f;
}

} // namespace

bool Imu::wr(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(CONFIG::IMU_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool Imu::rd(uint8_t reg, uint8_t *buf, size_t n) {
    Wire.beginTransmission(CONFIG::IMU_I2C_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    const size_t got = Wire.requestFrom(
        static_cast<int>(CONFIG::IMU_I2C_ADDRESS), static_cast<int>(n));
    if (got != n) return false;
    for (size_t i = 0; i < n; ++i) buf[i] = Wire.read();
    return true;
}

bool Imu::begin() {
    Wire.begin(PIN::IMU_SDA, PIN::IMU_SCL, CONFIG::IMU_I2C_FREQ_HZ);

    // WHO_AM_I check
    uint8_t who = 0;
    if (!rd(REG_WHO_AM_I, &who, 1) || who != 0x68) {
        return false;
    }

    // Wake + select gyro X PLL as the clock source
    wr(REG_PWR_MGMT_1, 0x01);
    // DLPF ~98 Hz (CONFIG = 2)
    wr(REG_CONFIG, 0x02);
    // Gyro full-scale
    wr(REG_GYRO_CONFIG, gyro_fs_bits(CONFIG::IMU_GYRO_FS_DPS));
    m_gyro_lsb_per_dps = gyro_lsb_per_dps(CONFIG::IMU_GYRO_FS_DPS);
    // Accel full-scale
    wr(REG_ACCEL_CONFIG, accel_fs_bits(CONFIG::IMU_ACCEL_FS_G));
    m_accel_lsb_per_g = accel_lsb_per_g(CONFIG::IMU_ACCEL_FS_G);
    // Sample-rate divider: 1 kHz / (1 + div) = IMU_SAMPLE_RATE_HZ
    const int div = (1000 / CONFIG::IMU_SAMPLE_RATE_HZ) - 1;
    wr(REG_SMPLRT_DIV, div < 0 ? 0 : static_cast<uint8_t>(div));

    delay(50);  // let the gyro start-up

    // Bias estimation: average N samples while the mouse is held still.
    double sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < CONFIG::IMU_GYRO_BIAS_SAMPLES; ++i) {
        ImuSample s = read();   // read() subtracts whatever bias is currently
                                // stored; on first run that's zero.
        sx += s.gx_dps;
        sy += s.gy_dps;
        sz += s.gz_dps;
        delayMicroseconds(1000);
    }
    m_gx_bias = static_cast<float>(sx / CONFIG::IMU_GYRO_BIAS_SAMPLES);
    m_gy_bias = static_cast<float>(sy / CONFIG::IMU_GYRO_BIAS_SAMPLES);
    m_gz_bias = static_cast<float>(sz / CONFIG::IMU_GYRO_BIAS_SAMPLES);
    return true;
}

ImuSample Imu::read() {
    uint8_t buf[14] = {0};
    rd(REG_ACCEL_XOUT_H, buf, 14);

    const int16_t ax = be16(buf[0], buf[1]);
    const int16_t ay = be16(buf[2], buf[3]);
    const int16_t az = be16(buf[4], buf[5]);
    // buf[6..7]: temperature -- ignored
    const int16_t gx = be16(buf[8],  buf[9]);
    const int16_t gy = be16(buf[10], buf[11]);
    const int16_t gz = be16(buf[12], buf[13]);

    ImuSample s;
    s.ax_g   = ax / m_accel_lsb_per_g;
    s.ay_g   = ay / m_accel_lsb_per_g;
    s.az_g   = az / m_accel_lsb_per_g;
    s.gx_dps = (gx / m_gyro_lsb_per_dps) - m_gx_bias;
    s.gy_dps = (gy / m_gyro_lsb_per_dps) - m_gy_bias;
    s.gz_dps = (gz / m_gyro_lsb_per_dps) - m_gz_bias;
    return s;
}

float Imu::yaw_rate_dps() {
    uint8_t buf[2] = {0};
    rd(0x47, buf, 2); // GYRO_ZOUT_H
    const int16_t gz = be16(buf[0], buf[1]);
    return (gz / m_gyro_lsb_per_dps) - m_gz_bias;
}

void Imu::recalibrate_bias() {
    double sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < CONFIG::IMU_GYRO_BIAS_SAMPLES; ++i) {
        ImuSample s = read();
        sx += s.gx_dps + m_gx_bias; // un-subtract the current bias estimate
        sy += s.gy_dps + m_gy_bias;
        sz += s.gz_dps + m_gz_bias;
        delayMicroseconds(1000);
    }
    m_gx_bias = static_cast<float>(sx / CONFIG::IMU_GYRO_BIAS_SAMPLES);
    m_gy_bias = static_cast<float>(sy / CONFIG::IMU_GYRO_BIAS_SAMPLES);
    m_gz_bias = static_cast<float>(sz / CONFIG::IMU_GYRO_BIAS_SAMPLES);
}

} // namespace hal
