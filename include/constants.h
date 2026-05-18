// ============================================================================
//  constants.h  --  Single source of truth for every tunable parameter.
//
//  The CONFIG namespace is split into clearly labelled sections.  Every value
//  carries a comment describing WHY you'd raise or lower it, so that future
//  tuning is a matter of editing this one file -- never the driver code.
//
//  Maintainer notes:
//    * Anything that is a function of geometry (wheel size, gear ratio, cell
//      size...) lives in the GEOMETRY section.  Distance/heading computations
//      derive from those constants -- do NOT hard-code distances elsewhere.
//    * PWM / PID / IR / IMU thresholds are kept in their own sections so that
//      tuning during a competition is a quick scroll, not a code dive.
//    * The CONFIG namespace name is preserved so existing code that pulls
//      e.g. CONFIG::MAZE_SIZE keeps compiling.
// ============================================================================

#pragma once

#include <cstdint>

namespace CONFIG {

// ---------------------------------------------------------------------------
//  1.  MAZE
// ---------------------------------------------------------------------------
//  Standard micromouse rules: a 16x16 grid of 180 mm cells with a 2x2 goal
//  in the centre.  These are typically not tuned -- they describe the
//  competition's physical maze.
// ---------------------------------------------------------------------------

constexpr int   MAZE_SIZE  = 16;        // 16x16 standard maze
constexpr int   GOAL_X1    = 7;         // goal area is the 2x2 block bounded
constexpr int   GOAL_Y1    = 7;         // by (GOAL_X1, GOAL_Y1) and
constexpr int   GOAL_X2    = 8;         // (GOAL_X2, GOAL_Y2) inclusive.
constexpr int   GOAL_Y2    = 8;


// ---------------------------------------------------------------------------
//  2.  GEOMETRY  (drivetrain & body)
// ---------------------------------------------------------------------------
//  Measure each of these on the real robot and update.  Every distance and
//  rotation computation in the firmware is derived from these numbers, so a
//  bad measurement here means the mouse over- or under-shoots every cell.
//
//  Tuning guide:
//    WHEEL_DIAMETER_MM      -> raise if the mouse undershoots forward moves
//                              by a constant percentage; lower if it
//                              overshoots.
//    WHEELBASE_MM           -> raise if 90 deg turns are SHORT (mouse stops
//                              rotated <90); lower if turns OVERSHOOT.
//    GEAR_RATIO             -> set to the exact reduction of your gearbox.
//                              Wrong values scale all distances/speeds.
//    ENCODER_PPR_PER_REV    -> base counts/rev of the encoder DISK.  With
//                              quadrature x4 decoding the effective ticks
//                              per motor-shaft rev = ENCODER_PPR_PER_REV*4.
// ---------------------------------------------------------------------------

constexpr float WHEEL_DIAMETER_MM   = 24.0f;   // typical micromouse wheel
constexpr float WHEELBASE_MM        = 70.0f;   // wheel-to-wheel centre dist.
constexpr float GEAR_RATIO          = 10.0f;   // motor : wheel reduction
constexpr int   ENCODER_PPR_PER_REV = 12;      // raw counts/rev (one channel)
constexpr int   QUADRATURE_MULT     = 4;       // x4 from PCNT, do not change

constexpr float CELL_SIZE_MM        = 180.0f;  // standard cell pitch

// --- Derived (do not edit, edit the inputs above) ---------------------------
constexpr float PI_F                = 3.14159265358979323846f;
// Distance the wheel surface travels per ENCODER TICK (post quadrature x4)
constexpr float MM_PER_TICK =
    (PI_F * WHEEL_DIAMETER_MM) /
    (ENCODER_PPR_PER_REV * QUADRATURE_MULT * GEAR_RATIO);
// Ticks for the mouse body to advance exactly one cell
constexpr int   TICKS_PER_CELL =
    static_cast<int>(CELL_SIZE_MM / MM_PER_TICK);
// Ticks per wheel for a 90 deg pivot in place (one wheel forward, one back
// -> each wheel sweeps a quarter-circle of radius WHEELBASE_MM/2)
constexpr int   TICKS_PER_90_PIVOT =
    static_cast<int>((PI_F * WHEELBASE_MM * 0.25f) / MM_PER_TICK);


// ---------------------------------------------------------------------------
//  3.  MOTOR DRIVER  (BDR6622T H-bridge over LEDC PWM)
// ---------------------------------------------------------------------------
//  The BDR6622T accepts two logic inputs per channel: we drive them with two
//  LEDC PWM channels in SIGN-MAGNITUDE mode (one PWM, other = 0 GND), which
//  gives the cleanest control and lowest dead-zone.
//
//  Tuning guide:
//    MOTOR_PWM_FREQ_HZ     -> 20 kHz is above the audible range; raising
//                             further reduces audible whine but increases
//                             switching losses in the driver.  Don't drop
//                             below ~16 kHz or the mouse will squeal.
//    MOTOR_PWM_RESOLUTION  -> 10 bits = 1024 levels.  Higher = smoother low
//                             speed, but max FREQ * 2^RES must stay within
//                             LEDC's clock limits.  10 bits @ 20 kHz is
//                             comfortable on the ESP32-S3.
//    MOTOR_DEADBAND_DUTY   -> below this duty the wheels don't actually
//                             turn (static friction).  The driver
//                             skips this band so a small commanded effort
//                             still produces motion.  Raise it if the mouse
//                             stalls on small corrections.
//    MOTOR_MAX_DUTY        -> safety cap; 1.0 = full battery voltage to the
//                             motor.  Lower (e.g. 0.85) protects the motor
//                             at the cost of top speed.
//    MOTOR_SLEW_PER_MS     -> max change in duty per millisecond (rate
//                             limit).  Lower = smoother starts, less wheel
//                             slip; higher = snappier response.
//
//  Wiring convention:
//    INVERT_MOTOR_LEFT / RIGHT flip the polarity for that side without you
//    touching the driver code -- handy when the chassis wiring puts a motor
//    on the wrong side or its IN1/IN2 are swapped.
// ---------------------------------------------------------------------------

constexpr int   MOTOR_PWM_FREQ_HZ      = 20000;    // 20 kHz
constexpr int   MOTOR_PWM_RESOLUTION   = 10;       // bits  -> 1024 levels
constexpr int   MOTOR_PWM_MAX_DUTY     = (1 << MOTOR_PWM_RESOLUTION) - 1;

constexpr float MOTOR_DEADBAND_DUTY    = 0.05f;    // 5 % is a safe start
constexpr float MOTOR_MAX_DUTY         = 0.95f;    // hard cap on commanded duty
constexpr float MOTOR_SLEW_PER_MS      = 0.01f;    // 1 %/ms -> 0..100 % in 100 ms

// --- Side / polarity selection (easy to flip during bring-up) --------------
//  MOTOR_A_IS_LEFT  : if true, MOTOR_A drives the LEFT wheel and MOTOR_B the
//                     RIGHT wheel; if false, the mapping is swapped.
//  INVERT_LEFT      : if true, the LEFT wheel's commanded direction is
//                     inverted (use when forward on that side actually
//                     drives the wheel backwards).
//  INVERT_RIGHT     : same for the RIGHT wheel.
//  Bring-up procedure: command a slow forward motion; if a wheel spins
//  backwards, flip the corresponding INVERT_* flag.  If the wheels are
//  swapped (turning right when commanded forward-then-right), flip
//  MOTOR_A_IS_LEFT.
constexpr bool  MOTOR_A_IS_LEFT        = true;
constexpr bool  INVERT_LEFT            = false;
constexpr bool  INVERT_RIGHT           = false;


// ---------------------------------------------------------------------------
//  4.  ENCODERS  (ESP32-S3 PCNT peripheral, x4 quadrature decoding)
// ---------------------------------------------------------------------------
//  ENCODER_A_INVERT / B_INVERT swap which channel the PCNT treats as the
//  quadrature direction input.  Use these when forward motion makes the
//  encoder count DOWN.  No code changes required.
// ---------------------------------------------------------------------------

constexpr bool  ENCODER_LEFT_INVERT    = false;
constexpr bool  ENCODER_RIGHT_INVERT   = false;


// ---------------------------------------------------------------------------
//  5.  IR DISTANCE SENSORS  (4 channels, ambient-subtracted)
// ---------------------------------------------------------------------------
//  Channel layout (matches pins.h):
//      FL_60  (IR_1, LED IR_LED_1) -- LEFT side, ~60 deg off forward axis
//      FL_05  (IR_2, LED IR_LED_2) -- LEFT front, ~5 deg
//      FR_05  (IR_3, LED IR_LED_3) -- RIGHT front, ~5 deg
//      FR_60  (IR_4, LED IR_LED_4) -- RIGHT side, ~60 deg
//
//  Sampling procedure (per channel, sequential to avoid cross-talk):
//      1. Read ADC with LED OFF             -> ambient
//      2. Drive LED ON, wait SETTLE_US
//      3. Read ADC with LED ON              -> ambient + reflection
//      4. Drive LED OFF
//      reflectance = (signal_on - ambient), clamped to >=0
//
//  Tuning guide:
//    IR_SETTLE_US           -> too short = sensor hasn't ramped, reading too
//                              low; too long = whole sampling cycle becomes
//                              slow.  100-300 us is typical.
//    IR_ADC_SAMPLES         -> averaging samples per phase.  More = quieter,
//                              slower.
//    IR_*_WALL_ON / _OFF    -> Schmitt-trigger thresholds.  Reading must
//                              cross _ON to register a wall, and fall back
//                              below _OFF to clear -- prevents flicker at
//                              the boundary.  Tune by running the calibrate
//                              mode and watching readings with/without
//                              walls present.
//    IR_CENTER_TARGET_LEFT  -> side-sensor reading when the mouse is exactly
//                              centred in a corridor (wall present).  Used
//                              by the lateral PID for wall-following.
// ---------------------------------------------------------------------------

constexpr int   IR_SETTLE_US           = 150;     // LED ramp / sensor stabilise
constexpr int   IR_ADC_SAMPLES         = 4;       // averaged per phase
constexpr int   IR_SAMPLE_PERIOD_US    = 2500;    // 400 Hz full 4-channel cycle

//  Schmitt-trigger thresholds (in raw 12-bit ADC counts after ambient sub)
//  -- raise *_ON to require a closer wall to register
//  -- raise *_OFF  to require the wall to be farther to clear
//                             ON   OFF
constexpr int   IR_FRONT_WALL_ON   =  900;
constexpr int   IR_FRONT_WALL_OFF  =  650;        // hysteresis band ~250
constexpr int   IR_SIDE_WALL_ON    =  500;
constexpr int   IR_SIDE_WALL_OFF   =  350;

//  When following a corridor, the lateral PID tries to keep each side
//  reading at its CENTER_TARGET.  These are calibrated values for the
//  60 deg sensor when the wall is exactly half-cell-width away.
constexpr int   IR_CENTER_TARGET_LEFT  = 420;
constexpr int   IR_CENTER_TARGET_RIGHT = 420;


// ---------------------------------------------------------------------------
//  6.  IMU  (MPU-6000 over I2C)
// ---------------------------------------------------------------------------
//  We use the gyro Z axis (yaw rate) for heading hold and for precise 90 deg
//  turns.  Accelerometer is currently unused but available.
//
//  Tuning guide:
//    IMU_I2C_FREQ_HZ        -> 400 kHz is reliable; raising to 1 MHz can
//                              be done on short PCB traces only.
//    IMU_GYRO_FS_DPS        -> full-scale range (deg/s).  Higher = less
//                              quantisation noise but lower resolution.
//                              500 dps is plenty for a micromouse.
//    IMU_GYRO_BIAS_SAMPLES  -> samples averaged during initialisation to
//                              estimate gyro bias.  More = lower bias error
//                              at the cost of a longer power-on calibration.
// ---------------------------------------------------------------------------

constexpr uint8_t IMU_I2C_ADDRESS      = 0x68;    // MPU-6000 default (AD0=0)
constexpr uint32_t IMU_I2C_FREQ_HZ     = 400000;  // 400 kHz
constexpr int    IMU_GYRO_FS_DPS       = 500;     // +/- 500 deg/s full-scale
constexpr int    IMU_ACCEL_FS_G        = 4;       // +/- 4 g full-scale
constexpr int    IMU_GYRO_BIAS_SAMPLES = 500;     // ~0.5 s @ 1 kHz
constexpr int    IMU_SAMPLE_RATE_HZ    = 1000;    // gyro/accel ODR


// ---------------------------------------------------------------------------
//  7.  CONTROL LOOPS  (PID gains & rates)
// ---------------------------------------------------------------------------
//  Two layers of PID:
//
//    A) WHEEL VELOCITY PID, one per wheel.  Input: target wheel speed in
//       ticks/s.  Output: motor duty.  Tighter gains here -> wheels track
//       commanded velocity closely.
//
//    B) HEADING / LATERAL PID, one each.  Input: heading error (from IMU
//       integration) or lateral error (from side IRs).  Output: a delta
//       added to the LEFT wheel's target speed and subtracted from the
//       RIGHT (a "differential" steering command).
//
//  Tuning order (classic): KP first with KI=KD=0 until response is snappy
//  but not oscillating.  Add KD to damp overshoot.  Add KI only if there
//  is steady-state error.
// ---------------------------------------------------------------------------

constexpr int   CONTROL_LOOP_HZ        = 1000;     // 1 kHz wheel-velocity PID
constexpr int   SENSING_LOOP_HZ        = 400;      // IR + IMU sampling rate

//  --- A) Wheel velocity PID -------------------------------------------------
constexpr float WHEEL_PID_KP           = 0.0012f;  // duty per tick/s of error
constexpr float WHEEL_PID_KI           = 0.0040f;  // raise if wheels lag
constexpr float WHEEL_PID_KD           = 0.0000f;  // usually 0
constexpr float WHEEL_PID_I_CLAMP      = 0.30f;    // anti-windup, max |I| term

//  --- B) Heading-hold PID (uses IMU yaw error) ------------------------------
//  Higher KP -> mouse snaps back to heading harder; too high causes weave.
constexpr float HEADING_PID_KP         = 6.0f;     // ticks/s per rad error
constexpr float HEADING_PID_KI         = 0.0f;
constexpr float HEADING_PID_KD         = 0.4f;     // damping

//  --- B') Lateral centring PID (uses IR side error) -------------------------
//  Active only while both side IRs see walls (corridor mode).  Otherwise we
//  fall back to heading-hold.
constexpr float LATERAL_PID_KP         = 0.8f;     // ticks/s per ADC count
constexpr float LATERAL_PID_KI         = 0.0f;
constexpr float LATERAL_PID_KD         = 0.05f;


// ---------------------------------------------------------------------------
//  8.  MOTION PROFILES  (high-level speeds in mm/s)
// ---------------------------------------------------------------------------
//  These are converted to ticks/s using MM_PER_TICK at runtime.  Keep speeds
//  conservative during early bring-up; the chassis usually limits these long
//  before the motors do.
//
//  Tuning guide:
//    EXPLORE_SPEED_MMPS     -> raise once wall detection is reliable at
//                              speed.  Too high makes IR sampling unreliable
//                              (you cross a cell between samples).
//    SPEED_RUN_MMPS         -> the headline "fast" speed for known paths.
//    TURN_SPEED_MMPS        -> tangential speed during in-place pivots.
//    ACCEL_MMPS2            -> longitudinal acceleration cap.  Higher = more
//                              wheel slip; lower = sluggish.
// ---------------------------------------------------------------------------

constexpr float EXPLORE_SPEED_MMPS     = 250.0f;   // exploration cruise
constexpr float SPEED_RUN_MMPS         = 700.0f;   // fast-run cruise
constexpr float TURN_SPEED_MMPS        = 200.0f;   // wheel tangential speed
constexpr float ACCEL_MMPS2            = 3000.0f;  // long. accel
constexpr float DECEL_MMPS2            = 4000.0f;  // braking (faster)

//  Tolerances for "movement complete"
constexpr float POS_TOLERANCE_MM       = 3.0f;     // ~ +/-3 mm at cell edge
constexpr float HEADING_TOL_RAD        = 0.035f;   // ~ +/- 2 deg


// ---------------------------------------------------------------------------
//  9.  BATTERY
// ---------------------------------------------------------------------------
//  BATT pin is a divided analogue voltage.  BATTERY_LOW_THRESHOLD is in raw
//  12-bit ADC counts; tune with a known voltage.  Below this the firmware
//  disables motors to avoid brown-out / damage to LiPo cells.
// ---------------------------------------------------------------------------

constexpr int   BATTERY_LOW_THRESHOLD  = 2000;     // raw ADC counts
constexpr int   BATTERY_SAMPLE_HZ      = 10;       // polling rate
constexpr float BATTERY_DIVIDER_RATIO  = 2.0f;     // 1:1 divider example


// ---------------------------------------------------------------------------
//  10. UX / TIMING
// ---------------------------------------------------------------------------

constexpr int   BUTTON_DEBOUNCE_MS     = 200;
constexpr int   STARTUP_DELAY_MS       = 1000;
constexpr int   FAULT_BLINK_MS         = 150;


// ---------------------------------------------------------------------------
//  11. RTOS  (FreeRTOS task placement & priorities)
// ---------------------------------------------------------------------------
//  ESP32-S3 has two cores.  We pin:
//     core 1 : the high-rate control task (deterministic timing)
//     core 0 : sensing + everything WiFi/UDP touches.
//  Priorities are relative to FreeRTOS conventions (higher = more urgent).
// ---------------------------------------------------------------------------

constexpr int   CONTROL_TASK_CORE      = 1;
constexpr int   CONTROL_TASK_PRIO      = 5;
constexpr int   CONTROL_TASK_STACK     = 4096;

constexpr int   SENSING_TASK_CORE      = 0;
constexpr int   SENSING_TASK_PRIO      = 4;
constexpr int   SENSING_TASK_STACK     = 4096;

} // namespace CONFIG
