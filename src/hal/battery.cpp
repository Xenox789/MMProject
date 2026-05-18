#include "hal/battery.h"
#include "constants.h"
#include "pins.h"

#include <Arduino.h>

namespace hal {

void Battery::begin() {
    pinMode(PIN::BATT, INPUT);
    analogSetPinAttenuation(PIN::BATT, ADC_11db);
}

int Battery::raw() {
    return analogRead(PIN::BATT);
}

float Battery::volts() {
    // 3.3 V reference, 12 bit ADC, post-divider voltage scaled back.
    return (raw() / 4095.0f) * 3.3f * CONFIG::BATTERY_DIVIDER_RATIO;
}

bool Battery::is_low() {
    return raw() < CONFIG::BATTERY_LOW_THRESHOLD;
}

} // namespace hal
