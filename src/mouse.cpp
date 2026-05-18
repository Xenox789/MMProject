// ============================================================================
//  mouse.cpp  --  Wires HAL + sensing + control behind the public Mouse API.
//
//  Lives in a single translation unit because:
//      * It owns all the singleton-ish state (hardware objects, tasks).
//      * The facade should not leak hal::/control::/sensing:: types -- the
//        less the maze module sees, the better.
// ============================================================================

#include "mouse.h"
#include "constants.h"
#include "pins.h"

#include "hal/motor_driver.h"
#include "hal/encoder.h"
#include "hal/ir_array.h"
#include "hal/imu.h"
#include "hal/battery.h"

#include "sensing/wall_detector.h"
#include "sensing/odometry.h"

#include "control/motion_controller.h"
#include "control/motion_planner.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>

namespace mouse {

namespace {

// --------------------------- system objects -------------------------------
hal::MotorDriver    g_motors;
hal::Encoders       g_encoders;
hal::IrArray        g_ir;
hal::Imu            g_imu;
hal::Battery        g_batt;

sensing::WallDetector g_wd;
sensing::Odometry     g_odo;

control::MotionController g_mc;
control::MotionPlanner    g_planner;

// Latest WallView -- updated by the sensing task, copied (struct of scalars)
// into the control task via a mutex-free volatile snapshot.  Slight tearing
// is acceptable because the consumer immediately runs it through the
// Schmitt-clean wall detector outputs.
volatile sensing::WallView g_wall_snapshot{};

// Cached planner result for the loop() thread.
std::atomic<MotionResult> g_status{MotionResult::Idle};

TaskHandle_t g_control_task = nullptr;
TaskHandle_t g_sensing_task = nullptr;

// --------------------------- tasks ----------------------------------------

void control_task(void *) {
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG::CONTROL_LOOP_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_us = micros();

    for (;;) {
        const uint32_t now_us = micros();
        const float dt = (now_us - last_us) * 1e-6f;
        last_us = now_us;

        // Pull the latest wall view.
        sensing::WallView v;
        v.front           = g_wall_snapshot.front;
        v.left            = g_wall_snapshot.left;
        v.right           = g_wall_snapshot.right;
        v.lateral_error   = g_wall_snapshot.lateral_error;
        v.front_metric    = g_wall_snapshot.front_metric;
        g_planner.update_walls(v);

        // Odometry then planner then controller -- order matters.
        g_odo.tick(dt);
        const auto res = g_planner.tick(dt);

        switch (res) {
        case control::PrimitiveResult::InProgress:
            g_status.store(MotionResult::Busy);     break;
        case control::PrimitiveResult::DoneOk:
            g_status.store(g_planner.busy() ? MotionResult::Busy
                                            : MotionResult::DoneOk);  break;
        case control::PrimitiveResult::DoneFrontWall:
            g_status.store(MotionResult::BlockedByWall); break;
        case control::PrimitiveResult::Aborted:
            g_status.store(MotionResult::Aborted);  break;
        }

        g_mc.tick(dt);

        vTaskDelayUntil(&last_wake, period);
    }
}

void sensing_task(void *) {
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG::SENSING_LOOP_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        const auto raw = g_ir.sample();
        const auto &v  = g_wd.update(raw);

        // publish snapshot
        g_wall_snapshot.front         = v.front;
        g_wall_snapshot.left          = v.left;
        g_wall_snapshot.right         = v.right;
        g_wall_snapshot.lateral_error = v.lateral_error;
        g_wall_snapshot.front_metric  = v.front_metric;

        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace

// ============================================================================

bool Mouse::begin() {
    // HAL bring-up
    g_motors.begin();
    g_encoders.begin();
    g_ir.begin();
    g_batt.begin();

    if (!g_imu.begin()) {
        Serial.println("[mouse] IMU WHO_AM_I failed");
        return false;
    }

    // Sensing/control wiring
    g_odo.begin(&g_encoders, &g_imu);
    g_mc.begin(&g_motors, &g_encoders, &g_odo);
    g_planner.begin(&g_mc, &g_odo);

    // FreeRTOS tasks
    xTaskCreatePinnedToCore(control_task, "ctrl",
                            CONFIG::CONTROL_TASK_STACK, nullptr,
                            CONFIG::CONTROL_TASK_PRIO, &g_control_task,
                            CONFIG::CONTROL_TASK_CORE);
    xTaskCreatePinnedToCore(sensing_task, "sense",
                            CONFIG::SENSING_TASK_STACK, nullptr,
                            CONFIG::SENSING_TASK_PRIO, &g_sensing_task,
                            CONFIG::SENSING_TASK_CORE);

    g_status.store(MotionResult::Idle);
    return true;
}

void Mouse::enable_motors()  { g_motors.enable(); }
void Mouse::disable_motors() { g_motors.disable(); }

// We always set the planner state BEFORE flipping g_status to Busy so the
// control task can never overwrite a stale DoneOk on top of our new Busy:
// once the planner's m_state is non-Idle, the next tick must return
// InProgress.
void Mouse::forward_cells(int cells, float cruise_mmps) {
    g_planner.start_forward_cells(cells, cruise_mmps);
    g_status.store(MotionResult::Busy);
}
void Mouse::forward_mm(float mm, float cruise_mmps) {
    g_planner.start_forward_mm(mm, cruise_mmps);
    g_status.store(MotionResult::Busy);
}
void Mouse::turn(TurnDirection d, float wheel_speed_mmps) {
    constexpr float k90  = 1.57079632f;
    constexpr float k180 = 3.14159265f;
    switch (d) {
    case TurnDirection::Left:   g_planner.start_pivot_left (k90,  wheel_speed_mmps); break;
    case TurnDirection::Right:  g_planner.start_pivot_right(k90,  wheel_speed_mmps); break;
    case TurnDirection::Around: g_planner.start_pivot_left (k180, wheel_speed_mmps); break;
    }
    g_status.store(MotionResult::Busy);
}
void Mouse::stop() {
    g_planner.start_stop();
}
void Mouse::emergency_brake() {
    g_planner.abort();
    g_motors.brake();
    g_status.store(MotionResult::Aborted);
}

MotionResult Mouse::status() const { return g_status.load(); }

MotionResult Mouse::wait_until_done(uint32_t timeout_ms) {
    const uint32_t t0 = millis();
    for (;;) {
        const auto s = g_status.load();
        if (s != MotionResult::Busy) return s;
        if (timeout_ms && (millis() - t0 > timeout_ms)) return MotionResult::Aborted;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

sensing::WallView Mouse::walls() const {
    sensing::WallView v;
    v.front         = g_wall_snapshot.front;
    v.left          = g_wall_snapshot.left;
    v.right         = g_wall_snapshot.right;
    v.lateral_error = g_wall_snapshot.lateral_error;
    v.front_metric  = g_wall_snapshot.front_metric;
    return v;
}

bool Mouse::battery_low() const { return g_batt.is_low(); }
bool Mouse::motor_fault() const { return g_motors.fault_active(); }

void Mouse::recalibrate_imu() { g_imu.recalibrate_bias(); }

void Mouse::print_diag() {
    const auto v  = walls();
    const auto p  = g_odo.pose();
    Serial.printf(
        "[diag] pose x=%.1f y=%.1f h=%.3f  v=%.0f w=%.2f  "
        "walls L=%d F=%d R=%d  lat=%d front=%d  batt=%d\n",
        p.x_mm, p.y_mm, p.heading_rad, p.v_mmps, p.w_radps,
        (int)v.left, (int)v.front, (int)v.right,
        v.lateral_error, v.front_metric,
        g_batt.raw());
}

// process-global instance
Mouse g_mouse;

} // namespace mouse
