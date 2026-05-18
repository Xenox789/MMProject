// hal/battery.h  --  Battery voltage sensing.
#pragma once

namespace hal {

class Battery {
public:
    void begin();

    int  raw();         // raw 12-bit ADC counts
    float volts();      // back-calculated using BATTERY_DIVIDER_RATIO
    bool is_low();      // true when raw() < BATTERY_LOW_THRESHOLD
};

} // namespace hal
