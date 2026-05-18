#include "sensing/odometry.h"
#include "constants.h"

#include <cmath>

namespace sensing {

void Odometry::begin(hal::Encoders *enc, hal::Imu *imu) {
    m_enc = enc;
    m_imu = imu;
    reset_pose();
}

void Odometry::reset_pose() {
    m_pose = {};
    if (m_enc) {
        m_last_l_ticks = m_enc->left_ticks();
        m_last_r_ticks = m_enc->right_ticks();
    }
    m_left_mm = m_right_mm = 0.f;
}

void Odometry::reset_distance() {
    if (m_enc) {
        m_last_l_ticks = m_enc->left_ticks();
        m_last_r_ticks = m_enc->right_ticks();
    }
    m_left_mm = m_right_mm = 0.f;
}

void Odometry::tick(float dt) {
    if (!m_enc || !m_imu || dt <= 0.f) return;

    // --- Distance from encoders ---
    const int64_t l = m_enc->left_ticks();
    const int64_t r = m_enc->right_ticks();
    const int64_t dl = l - m_last_l_ticks;
    const int64_t dr = r - m_last_r_ticks;
    m_last_l_ticks = l;
    m_last_r_ticks = r;

    const float dl_mm = hal::Encoders::ticks_to_mm(dl);
    const float dr_mm = hal::Encoders::ticks_to_mm(dr);
    m_left_mm  += dl_mm;
    m_right_mm += dr_mm;

    const float ds = 0.5f * (dl_mm + dr_mm);          // forward step (mm)

    // --- Heading from IMU (radians) ---
    const float gz_dps = m_imu->yaw_rate_dps();
    const float dpsi   = gz_dps * (3.14159265f / 180.0f) * dt;

    m_pose.heading_rad += dpsi;
    // wrap to [-pi, pi]
    while (m_pose.heading_rad >  3.14159265f) m_pose.heading_rad -= 6.2831853f;
    while (m_pose.heading_rad < -3.14159265f) m_pose.heading_rad += 6.2831853f;

    m_pose.w_radps = dpsi / dt;
    m_pose.v_mmps  = ds / dt;

    // --- Position integration (mid-point) ---
    const float c = std::cos(m_pose.heading_rad - 0.5f * dpsi);
    const float s = std::sin(m_pose.heading_rad - 0.5f * dpsi);
    m_pose.x_mm += ds * c;
    m_pose.y_mm += ds * s;
}

} // namespace sensing
