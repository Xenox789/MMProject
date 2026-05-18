// ============================================================================
//  hal/encoder.cpp  --  PCNT-backed quadrature encoder reader.
//
//  Implementation notes
//  --------------------
//  * ESP-IDF v5 (used by Arduino-ESP32 v3) ships a new "pulse_cnt" driver
//    (driver/pulse_cnt.h) which is what we use here.  It supports x4
//    decoding natively via two channels per unit configured with the A/B
//    inputs swapped on edge/level inputs.
//  * The chip's hardware counter is 16-bit signed (-32768 ... +32767).  We
//    install an overflow watcher so we accumulate into a 64-bit running
//    total in software.
// ============================================================================

#include "hal/encoder.h"
#include "constants.h"
#include "pins.h"

#include "driver/pulse_cnt.h"
#include <esp_attr.h>

namespace hal {

namespace {

constexpr int PCNT_HIGH_LIMIT =  10000;
constexpr int PCNT_LOW_LIMIT  = -10000;

struct UnitState {
    pcnt_unit_handle_t unit = nullptr;
    pcnt_channel_handle_t chan_a = nullptr;
    pcnt_channel_handle_t chan_b = nullptr;
    volatile int64_t accumulated = 0;     // total ticks since reset
};

UnitState g_left;
UnitState g_right;

bool IRAM_ATTR on_watch(pcnt_unit_handle_t unit,
                        const pcnt_watch_event_data_t *edata,
                        void *user_ctx) {
    auto *st = static_cast<UnitState*>(user_ctx);
    // We installed watchpoints at +/- LIMIT; when fired, the hardware count
    // has already snapped back near zero, so credit the limit value.
    st->accumulated += edata->watch_point_value;
    return false; // do not yield from ISR
}

void install_unit(UnitState &st, int pin_a, int pin_b, bool invert) {
    pcnt_unit_config_t unit_cfg = {
        .low_limit  = PCNT_LOW_LIMIT,
        .high_limit = PCNT_HIGH_LIMIT,
        .intr_priority = 0,
        .flags = {.accum_count = 0},
    };
    pcnt_new_unit(&unit_cfg, &st.unit);

    // Channel A: edge = pin_a, level = pin_b
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = pin_a,
        .level_gpio_num = pin_b,
        .flags = {},
    };
    pcnt_new_channel(st.unit, &chan_a_cfg, &st.chan_a);

    // Channel B: edge = pin_b, level = pin_a  (this is what gives x4 decode)
    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num  = pin_b,
        .level_gpio_num = pin_a,
        .flags = {},
    };
    pcnt_new_channel(st.unit, &chan_b_cfg, &st.chan_b);

    // Edge/level actions: standard quadrature -- A leads B = +1 per edge
    // (or -1 if 'invert' is true).
    const pcnt_channel_edge_action_t edge_pos =
        invert ? PCNT_CHANNEL_EDGE_ACTION_DECREASE
               : PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    const pcnt_channel_edge_action_t edge_neg =
        invert ? PCNT_CHANNEL_EDGE_ACTION_INCREASE
               : PCNT_CHANNEL_EDGE_ACTION_DECREASE;

    pcnt_channel_set_edge_action(st.chan_a, edge_pos, edge_neg);
    pcnt_channel_set_level_action(st.chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_channel_set_edge_action(st.chan_b, edge_neg, edge_pos);
    pcnt_channel_set_level_action(st.chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // Watch the limits so we can roll the 16-bit count into our int64.
    pcnt_unit_add_watch_point(st.unit, PCNT_HIGH_LIMIT);
    pcnt_unit_add_watch_point(st.unit, PCNT_LOW_LIMIT);

    pcnt_event_callbacks_t cbs = { .on_reach = on_watch };
    pcnt_unit_register_event_callbacks(st.unit, &cbs, &st);

    pcnt_unit_enable(st.unit);
    pcnt_unit_clear_count(st.unit);
    pcnt_unit_start(st.unit);
}

int64_t read_total(UnitState &st) {
    int current = 0;
    pcnt_unit_get_count(st.unit, &current);
    return st.accumulated + current;
}

} // namespace

void Encoders::begin() {
    install_unit(g_left,
                 PIN::MOTOR_A_ENC_A, PIN::MOTOR_A_ENC_B,
                 CONFIG::MOTOR_A_IS_LEFT ? CONFIG::ENCODER_LEFT_INVERT
                                         : CONFIG::ENCODER_RIGHT_INVERT);
    install_unit(g_right,
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
    pcnt_unit_clear_count(g_left.unit);
    pcnt_unit_clear_count(g_right.unit);
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
