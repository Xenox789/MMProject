// ============================================================================
//  hal/ir_array.h  --  Four-channel reflective IR distance sensor array.
//
//  Channel layout (matches pins.h and CONFIG):
//      [0] FL_60  front-left  60 deg  (looks out the left wall)
//      [1] FL_05  front-left   5 deg  (looks nearly forward, left of axis)
//      [2] FR_05  front-right  5 deg  (looks nearly forward, right of axis)
//      [3] FR_60  front-right 60 deg  (looks out the right wall)
//
//  Each sample() call performs one full ambient-subtracted reading on every
//  channel.  Channels are sampled SEQUENTIALLY so two LEDs are never on at
//  once (avoids crosstalk between adjacent receivers).
// ============================================================================

#pragma once

#include <cstdint>

namespace hal {

enum IrChannel : uint8_t {
    IR_FL60 = 0,   // left side wall (~60 deg)
    IR_FL05 = 1,   // forward-left (~5 deg)
    IR_FR05 = 2,   // forward-right (~5 deg)
    IR_FR60 = 3,   // right side wall (~60 deg)
    IR_COUNT
};

struct IrReading {
    // Reflectance values after ambient subtraction (clamped to >= 0).
    // Higher value = stronger return = closer wall.
    int reflect[IR_COUNT] = {0, 0, 0, 0};

    // Raw ambient (LED off) -- useful for diagnostics.
    int ambient[IR_COUNT] = {0, 0, 0, 0};
};

class IrArray {
public:
    void begin();

    // Run one complete 4-channel sampling cycle.  Blocking for roughly
    // 4 * (settle + adc) microseconds -- target this from the sensing task.
    IrReading sample();

    // Convenience accessors used by wall_detector / motion controllers.
    const IrReading &last() const { return m_last; }

private:
    int read_channel_avg(int adc_pin);
    int sample_channel(IrChannel c);

    IrReading m_last;
};

} // namespace hal
