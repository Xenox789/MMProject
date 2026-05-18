// ============================================================================
//  hal/encoder.h  --  Quadrature encoder reader using ESP32-S3 PCNT.
//
//  Each motor has a 2-channel encoder.  We use the PCNT (pulse counter)
//  hardware peripheral with x4 decoding so EVERY edge on A or B contributes
//  one tick, signed by direction.  No interrupts in the firmware, no CPU
//  cost per edge.
//
//  Public interface returns INT64 ticks because over a long run the int32
//  PCNT counter would wrap.  The driver handles wrap detection.
// ============================================================================

#pragma once

#include <cstdint>

namespace hal {

class Encoders {
public:
    void begin();

    // Body-frame ticks (sign reflects forward = positive).
    int64_t left_ticks();
    int64_t right_ticks();

    // Reset both counters to zero atomically.
    void reset();

    // Convert ticks <-> millimetres / radians.  Inlined so the maths is
    // exactly the one in constants.h.
    static float ticks_to_mm(int64_t ticks);
    static int64_t mm_to_ticks(float mm);

    // Body-frame heading change in radians, derived purely from differential
    // wheel travel (NOT the IMU).  Useful as a heading reference at slow
    // speed where the IMU bias dominates.
    //
    // Returns delta yaw since the LAST call -- so call it from one place.
    float consume_odometric_dyaw();

private:
    // Cached state for delta computation
    int64_t m_last_l = 0;
    int64_t m_last_r = 0;
};

} // namespace hal
