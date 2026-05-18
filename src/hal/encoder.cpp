// ============================================================================
//  hal/encoder.cpp  --  PCNT-backed quadrature encoder reader.
//
//  Implementation notes
//  --------------------
//  * Uses the legacy ESP-IDF v4 "driver/pcnt.h" API which is available in
//    the Arduino-ESP32 framework bundled by PlatformIO.
//  * Quadrature x4 decoding: two channels per unit, A/B swapped on
//    edge/level inputs.
//  * The chip's hardware counter is 16-bit signed (-32768 ... +32767).  We
//    install H_LIM / L_LIM event interrupts to accumulate into a 64-bit
//    running total in software.
// ============================================================================

#include "hal/encoder.h"
#include "constants.h"
#include "pins.h"

#include "driver/pcnt.h"
#include <esp_attr.h>

namespace hal {

namespace {

constexpr int16_t PCNT_HIGH_LIMIT =  10000;
constexpr int16_t PCNT_LOW_LIMIT  = -10000;

struct UnitState {
    pcnt_unit_t unit;
    volatile int64_t accumulated = 0;     // total ticks since reset
};

UnitState g_left;
UnitState g_right;

static bool g_isr_installed = false;

static void IRAM_ATTR pcnt_isr_handler(void *arg) {
    auto *st = static_cast<UnitState*>(arg);
    uint32_t status = 0;
    pcnt_get_event_status(st->unit, &status);
    if (status & PCNT_EVT_H_LIM) {
        st->accumulated += PCNT_HIGH_LIMIT;
    }
    if (status & PCNT_EVT_L_LIM) {
        st->accumulated += PCNT_LOW_LIMIT;
    }
}

void install_unit(UnitState &st, pcnt_unit_t unit, int pin_a, int pin_b, bool invert) {
    st.unit = unit;
    st.accumulated = 0;

    // Channel 0: edge on pin_a, level on pin_b
    pcnt_config_t cfg_a = {};
    cfg_a.pulse_gpio_num = pin_a;
    cfg_a.ctrl_gpio_num  = pin_b;
    cfg_a.unit           = unit;
    cfg_a.channel        = PCNT_CHANNEL_0;
    cfg_a.pos_mode       = invert ? PCNT_COUNT_DEC : PCNT_COUNT_INC;
    cfg_a.neg_mode       = invert ? PCNT_COUNT_INC : PCNT_COUNT_DEC;
    cfg_a.lctrl_mode     = PCNT_MODE_REVERSE;
    cfg_a.hctrl_mode     = PCNT_MODE_KEEP;
    cfg_a.counter_h_lim  = PCNT_HIGH_LIMIT;
    cfg_a.counter_l_lim  = PCNT_LOW_LIMIT;
    pcnt_unit_config(&cfg_a);

    // Channel 1: edge on pin_b, level on pin_a (gives x4 decode)
    pcnt_config_t cfg_b = {};
    cfg_b.pulse_gpio_num = pin_b;
    cfg_b.ctrl_gpio_num  = pin_a;
    cfg_b.unit           = unit;
    cfg_b.channel        = PCNT_CHANNEL_1;
    cfg_b.pos_mode       = invert ? PCNT_COUNT_INC : PCNT_COUNT_DEC;
    cfg_b.neg_mode       = invert ? PCNT_COUNT_DEC : PCNT_COUNT_INC;
    cfg_b.lctrl_mode     = PCNT_MODE_REVERSE;
    cfg_b.hctrl_mode     = PCNT_MODE_KEEP;
    cfg_b.counter_h_lim  = PCNT_HIGH_LIMIT;
    cfg_b.counter_l_lim  = PCNT_LOW_LIMIT;
    pcnt_unit_config(&cfg_b);

    // Optional glitch filter (APB_CLK ticks, max 1023)
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    // Enable H_LIM and L_LIM events for overflow accumulation
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    // Install shared ISR service once, then add per-unit handler
    if (!g_isr_installed) {
        pcnt_isr_service_install(0);
        g_isr_installed = true;
    }
    pcnt_isr_handler_add(unit, pcnt_isr_handler, &st);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

int64_t read_total(UnitState &st) {
    int16_t current = 0;
    pcnt_get_counter_value(st.unit, &current);
    return st.accumulated + current;
}

} // namespace

void Encoders::begin() {
    install_unit(g_left, PCNT_UNIT_0,
                 PIN::MOTOR_A_ENC_A, PIN::MOTOR_A_ENC_B,
                 CONFIG::MOTOR_A_IS_LEFT ? CONFIG::ENCODER_LEFT_INVERT
                                         : CONFIG::ENCODER_RIGHT_INVERT);
    install_unit(g_right, PCNT_UNIT_1,
                 PIN::MOTOR_B_ENC_A, PIN::MOTOR_B_ENC_B,
                 CONFIG::MOTOR_A_IS_LEFT ? CONFIG::ENCODER_RIGHT_INVERT
                                         : CONFIG::ENCODER_LEFT_INVERT);
    reset();
}

int64_t Encoders::left_ticks()  {
    if (CONFIG::MOTOR_A_IS_LEFT) return read_total(g_left);
    return read_total(g_right);
}

int64_t Encoders::right_ticks() {
    if (CONFIG::MOTOR_A_IS_LEFT) return read_total(g_right);
    return read_total(g_left);
}

void Encoders::reset() {
    pcnt_counter_pause(g_left.unit);
    pcnt_counter_clear(g_left.unit);
    pcnt_counter_resume(g_left.unit);
    pcnt_counter_pause(g_right.unit);
    pcnt_counter_clear(g_right.unit);
    pcnt_counter_resume(g_right.unit);
    g_left.accumulated  = 0;
    g_right.accumulated = 0;
    m_last_l = 0;
    m_last_r = 0;
}

float Encoders::ticks_to_mm(int64_t ticks) {
    return static_cast<float>(ticks) * CONFIG::MM_PER_TICK;
}

int64_t Encoders::mm_to_ticks(float mm) {
    return static_cast<int64_t>(mm / CONFIG::MM_PER_TICK);
}

float Encoders::consume_odometric_dyaw() {
    const int64_t l = left_ticks();
    const int64_t r = right_ticks();
    const int64_t dl = l - m_last_l;
    const int64_t dr = r - m_last_r;
    m_last_l = l;
    m_last_r = r;

    // Differential drive: dyaw = (right_mm - left_mm) / wheelbase
    const float right_mm = ticks_to_mm(dr);
    const float left_mm  = ticks_to_mm(dl);
    return (right_mm - left_mm) / CONFIG::WHEELBASE_MM;
}

} // namespace hal
