// ============================================================================
//  hal/motor_driver.cpp  --  BDR6622T H-bridge driver implementation.
//
//  PWM strategy
//  ------------
//  Sign-magnitude: for each side we attach BOTH IN1 and IN2 to LEDC channels.
//  Forward  ->  IN1 = duty, IN2 = 0
//  Reverse  ->  IN1 = 0,    IN2 = duty
//  Brake    ->  IN1 = MAX,  IN2 = MAX
//  Coast    ->  IN1 = 0,    IN2 = 0
//
//  This avoids the dead-zone of locked-anti-phase PWM and lets the bridge
//  free-wheel through its body diodes on coast.
// ============================================================================

#include "hal/motor_driver.h"
#include "constants.h"
#include "pins.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>

namespace hal {

namespace {

// LEDC channels.  Eight channels are available per timer group on the S3;
// we use four.  Keep them adjacent so they share a timer.
constexpr int LEDC_CH_A1 = 0;
constexpr int LEDC_CH_A2 = 1;
constexpr int LEDC_CH_B1 = 2;
constexpr int LEDC_CH_B2 = 3;

// Resolve the user-side (LEFT/RIGHT) to chip pins / channels.
struct SidePins { int in1, in2, ch1, ch2; };

SidePins pins_for_chip_A() {
    return { PIN::MOTOR_A_1, PIN::MOTOR_A_2, LEDC_CH_A1, LEDC_CH_A2 };
}
SidePins pins_for_chip_B() {
    return { PIN::MOTOR_B_1, PIN::MOTOR_B_2, LEDC_CH_B1, LEDC_CH_B2 };
}

inline SidePins side_pins(MotorDriver*, bool body_is_left) {
    // MOTOR_A_IS_LEFT decides which physical channel maps to body-LEFT.
    const bool a_is_left = CONFIG::MOTOR_A_IS_LEFT;
    if (body_is_left)  return a_is_left ? pins_for_chip_A() : pins_for_chip_B();
    else               return a_is_left ? pins_for_chip_B() : pins_for_chip_A();
}

inline bool side_invert(bool body_is_left) {
    return body_is_left ? CONFIG::INVERT_LEFT : CONFIG::INVERT_RIGHT;
}

// LEDC compatibility shim.
// arduino-esp32 v2.x : ledcSetup(channel,freq,res); ledcAttachPin(pin,channel); ledcWrite(channel,duty)
// arduino-esp32 v3.x : ledcAttach(pin,freq,res);       ledcWrite(pin,duty)
// We keep a "channel" id around for v2; for v3 it doubles as the pin id
// (motor_driver maps channel -> pin via a small static table).
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define LEDC_V3 1
#else
  #define LEDC_V3 0
#endif

// Channel -> pin map (for v3 we route writes by pin).
int channel_to_pin(int ch) {
    switch (ch) {
        case 0: return PIN::MOTOR_A_1;
        case 1: return PIN::MOTOR_A_2;
        case 2: return PIN::MOTOR_B_1;
        case 3: return PIN::MOTOR_B_2;
    }
    return -1;
}

inline void ledc_write_duty(int channel, float duty01) {
    // duty01 in [0,1]; clamp & convert to LEDC ticks
    if (duty01 < 0.f) duty01 = 0.f;
    if (duty01 > 1.f) duty01 = 1.f;
    const uint32_t ticks =
        static_cast<uint32_t>(duty01 * CONFIG::MOTOR_PWM_MAX_DUTY);
#if LEDC_V3
    ledcWrite(channel_to_pin(channel), ticks);
#else
    ledcWrite(channel, ticks);
#endif
}

} // namespace

void MotorDriver::begin() {
    // FAULT is open-drain on the BDR6622T -> internal pull-up.
    pinMode(PIN::MOTOR_DRV_FAULT, INPUT_PULLUP);

    // Global enable -- start disabled.
    pinMode(PIN::MOTOR_DRV_EN, OUTPUT);
    digitalWrite(PIN::MOTOR_DRV_EN, LOW);
    m_enabled = false;

    // Configure LEDC -- one path per Arduino-ESP32 major version (see shim
    // above).  Functionally identical.
#if LEDC_V3
    ledcAttach(PIN::MOTOR_A_1, CONFIG::MOTOR_PWM_FREQ_HZ, CONFIG::MOTOR_PWM_RESOLUTION);
    ledcAttach(PIN::MOTOR_A_2, CONFIG::MOTOR_PWM_FREQ_HZ, CONFIG::MOTOR_PWM_RESOLUTION);
    ledcAttach(PIN::MOTOR_B_1, CONFIG::MOTOR_PWM_FREQ_HZ, CONFIG::MOTOR_PWM_RESOLUTION);
    ledcAttach(PIN::MOTOR_B_2, CONFIG::MOTOR_PWM_FREQ_HZ, CONFIG::MOTOR_PWM_RESOLUTION);
#else
    ledcSetup(LEDC_CH_A1, CONFIG::MOTOR_PWM_FREQ_HZ, CONFIG::MOTOR_PWM_RESOLUTION);
    ledcSetup(LEDC_CH_A2, CONFIG::MOTOR_PWM_FREQ_HZ, CONFIG::MOTOR_PWM_RESOLUTION);
    ledcSetup(LEDC_CH_B1, CONFIG::MOTOR_PWM_FREQ_HZ, CONFIG::MOTOR_PWM_RESOLUTION);
    ledcSetup(LEDC_CH_B2, CONFIG::MOTOR_PWM_FREQ_HZ, CONFIG::MOTOR_PWM_RESOLUTION);

    ledcAttachPin(PIN::MOTOR_A_1, LEDC_CH_A1);
    ledcAttachPin(PIN::MOTOR_A_2, LEDC_CH_A2);
    ledcAttachPin(PIN::MOTOR_B_1, LEDC_CH_B1);
    ledcAttachPin(PIN::MOTOR_B_2, LEDC_CH_B2);
#endif

    coast();
}

void MotorDriver::enable() {
    digitalWrite(PIN::MOTOR_DRV_EN, HIGH);
    m_enabled = true;
}

void MotorDriver::disable() {
    coast();
    digitalWrite(PIN::MOTOR_DRV_EN, LOW);
    m_enabled = false;
}

bool MotorDriver::fault_active() const {
    // BDR6622T FAULT is active LOW.
    return digitalRead(PIN::MOTOR_DRV_FAULT) == LOW;
}

void MotorDriver::set_left(float duty) {
    m_target_duty[LEFT] = duty;
}
void MotorDriver::set_right(float duty) {
    m_target_duty[RIGHT] = duty;
}

void MotorDriver::brake() {
    // Force both IN1 and IN2 to 100% on each side.
    ledc_write_duty(LEDC_CH_A1, 1.f);
    ledc_write_duty(LEDC_CH_A2, 1.f);
    ledc_write_duty(LEDC_CH_B1, 1.f);
    ledc_write_duty(LEDC_CH_B2, 1.f);
    m_applied_duty[LEFT] = m_applied_duty[RIGHT] = 0.f;
    m_target_duty[LEFT]  = m_target_duty[RIGHT]  = 0.f;
}

void MotorDriver::coast() {
    ledc_write_duty(LEDC_CH_A1, 0.f);
    ledc_write_duty(LEDC_CH_A2, 0.f);
    ledc_write_duty(LEDC_CH_B1, 0.f);
    ledc_write_duty(LEDC_CH_B2, 0.f);
    m_applied_duty[LEFT] = m_applied_duty[RIGHT] = 0.f;
    m_target_duty[LEFT]  = m_target_duty[RIGHT]  = 0.f;
}

void MotorDriver::write_side(Side s, float signed_duty) {
    // 1) clamp to MOTOR_MAX_DUTY
    const float maxd = CONFIG::MOTOR_MAX_DUTY;
    if (signed_duty >  maxd) signed_duty =  maxd;
    if (signed_duty < -maxd) signed_duty = -maxd;

    // 2) inversion (chassis wiring)
    if (side_invert(s == LEFT)) signed_duty = -signed_duty;

    // 3) dead-band skip: anything below the dead-band magnitude is treated
    //    as zero so the chip isn't pulsed below the start-up threshold of
    //    the motor.
    if (std::fabs(signed_duty) < CONFIG::MOTOR_DEADBAND_DUTY) {
        signed_duty = 0.f;
    }

    // 4) emit PWM (sign-magnitude)
    SidePins sp = side_pins(this, s == LEFT);
    if (signed_duty >= 0.f) {
        ledc_write_duty(sp.ch1, signed_duty);
        ledc_write_duty(sp.ch2, 0.f);
    } else {
        ledc_write_duty(sp.ch1, 0.f);
        ledc_write_duty(sp.ch2, -signed_duty);
    }
}

void MotorDriver::tick(float dt_seconds) {
    if (!m_enabled) return;

    const float max_step = CONFIG::MOTOR_SLEW_PER_MS * 1000.0f * dt_seconds;

    for (int i = 0; i < 2; ++i) {
        float err = m_target_duty[i] - m_applied_duty[i];
        if      (err >  max_step) m_applied_duty[i] += max_step;
        else if (err < -max_step) m_applied_duty[i] -= max_step;
        else                      m_applied_duty[i]  = m_target_duty[i];

        write_side(static_cast<Side>(i), m_applied_duty[i]);
    }
}

} // namespace hal
