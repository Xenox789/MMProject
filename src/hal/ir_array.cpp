// ============================================================================
//  hal/ir_array.cpp  --  Reflective IR sampling with ambient subtraction.
// ============================================================================

#include "hal/ir_array.h"
#include "constants.h"
#include "pins.h"

#include <Arduino.h>

namespace hal {

namespace {

// Map an IrChannel to the LED and ADC GPIO numbers.
struct ChannelMap { int led_gpio; int adc_gpio; };

constexpr ChannelMap kChan[IR_COUNT] = {
    { PIN::IR_LED_1, PIN::IR_1 },   // IR_FL60
    { PIN::IR_LED_2, PIN::IR_2 },   // IR_FL05
    { PIN::IR_LED_3, PIN::IR_3 },   // IR_FR05
    { PIN::IR_LED_4, PIN::IR_4 },   // IR_FR60
};

} // namespace

void IrArray::begin() {
    for (int i = 0; i < IR_COUNT; ++i) {
        pinMode(kChan[i].led_gpio, OUTPUT);
        digitalWrite(kChan[i].led_gpio, LOW);
        // ADC pin is set automatically by analogRead on the ESP32-S3, but
        // setting it explicitly to INPUT is harmless and documents intent.
        pinMode(kChan[i].adc_gpio, INPUT);
    }
    // Increase ADC resolution to 12 bit (default) and set attenuation so
    // we can read up to ~3.3 V on the receiver phototransistor.
    analogReadResolution(12);
    analogSetPinAttenuation(PIN::IR_1, ADC_11db);
    analogSetPinAttenuation(PIN::IR_2, ADC_11db);
    analogSetPinAttenuation(PIN::IR_3, ADC_11db);
    analogSetPinAttenuation(PIN::IR_4, ADC_11db);
}

int IrArray::read_channel_avg(int adc_pin) {
    int sum = 0;
    for (int i = 0; i < CONFIG::IR_ADC_SAMPLES; ++i) {
        sum += analogRead(adc_pin);
    }
    return sum / CONFIG::IR_ADC_SAMPLES;
}

int IrArray::sample_channel(IrChannel c) {
    const ChannelMap &m = kChan[c];
    // 1) Ambient (LED off)
    digitalWrite(m.led_gpio, LOW);
    delayMicroseconds(CONFIG::IR_SETTLE_US);
    const int amb = read_channel_avg(m.adc_gpio);

    // 2) Drive LED, wait for settle
    digitalWrite(m.led_gpio, HIGH);
    delayMicroseconds(CONFIG::IR_SETTLE_US);
    const int on  = read_channel_avg(m.adc_gpio);
    digitalWrite(m.led_gpio, LOW);

    m_last.ambient[c] = amb;
    int r = on - amb;
    if (r < 0) r = 0;
    return r;
}

IrReading IrArray::sample() {
    for (int c = 0; c < IR_COUNT; ++c) {
        m_last.reflect[c] = sample_channel(static_cast<IrChannel>(c));
    }
    return m_last;
}

} // namespace hal
