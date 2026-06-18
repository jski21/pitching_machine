#pragma once

// ===========================================================================
// config.h
//
// Single place to edit for pin assignments, calibration, output mode, and
// safety limits. Field techs should be able to do almost all on-site
// adjustments by editing constants in this file only.
//
// NOTE: MODE_* flags (MODE_DIAGNOSTIC, MODE_PWM_TEST, MODE_SINGLE_POT,
// MODE_OPEN_LOOP, MODE_SAFE) are set by platformio.ini build_flags, not here.
// ===========================================================================

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Serial debug
// ---------------------------------------------------------------------------
#ifdef FORCE_SERIAL_DEBUG
    #define SERIAL_DEBUG_ENABLE true
#else
    #define SERIAL_DEBUG_ENABLE true   // set false to silence printDebug() output
#endif

#define SERIAL_BAUD            115200
#define DEBUG_PRINT_INTERVAL_MS 250    // how often printDebug() runs (millis-based)

// ---------------------------------------------------------------------------
// Pin assignments  (edit these to match actual wiring)
// ---------------------------------------------------------------------------
#define PIN_PITCH_SPEED_POT    34   // ADC1_CH6, input only
#define PIN_SPIN_RATE_POT      35   // ADC1_CH7, input only

// Spin-direction input. Two hardware options are supported (pick one with
// SPIN_DIRECTION_SOURCE below):
//   - Quadrature encoder on PIN_ENCODER_A / PIN_ENCODER_B
//   - 3-wire 360-degree rotation potentiometer on PIN_SPIN_DIR_POT (ADC)
#define PIN_ENCODER_A          25
#define PIN_ENCODER_B          26

#define PIN_SPIN_DIR_POT       39   // ADC1_CH3, input only (360-deg spin-dir pot)

#define PIN_MOTOR_A            32
#define PIN_MOTOR_B            33
#define PIN_MOTOR_C            27

// ---------------------------------------------------------------------------
// ADC calibration
//
// Raw ADC counts at the physical minimum and maximum knob positions.
// On-site calibration: rotate each pot to its end stops, read the raw
// values printed by esp32_diagnostic, and enter them here.
// ESP32 ADC defaults to 12-bit (0-4095) at 11dB attenuation (~0-3.3V).
// ---------------------------------------------------------------------------
#define ADC_RESOLUTION_BITS    12
#define ADC_MAX_COUNT          4095

#define PITCH_POT_ADC_MIN      120     // raw count at knob minimum
#define PITCH_POT_ADC_MAX      4000    // raw count at knob maximum

#define SPIN_POT_ADC_MIN       120
#define SPIN_POT_ADC_MAX       4000

// Plausibility window: readings outside [ -ADC_PLAUSIBLE_MARGIN, MAX+MARGIN ]
// are treated as a sensor fault and force a safe stop.
#define ADC_PLAUSIBLE_MARGIN   200

// ---------------------------------------------------------------------------
// Potentiometer direction inversion
//
// Set to true if turning a knob clockwise decreases its reading instead of
// increasing it (i.e. the wiper moves toward GND rather than toward 3.3V
// as you turn up). This applies AFTER normalization so the plausibility
// check and ADC calibration range are unaffected — just flip this flag
// instead of re-wiring the pot.
// ---------------------------------------------------------------------------
#define INVERT_PITCH_POT       true
#define INVERT_SPIN_POT        false

// ---------------------------------------------------------------------------
// Input filtering (moving average / low-pass)
//
// FILTER_SAMPLE_COUNT: number of samples averaged for the moving-average
// filter used on both potentiometer channels. Larger = smoother but slower
// to respond. Must be >= 1.
// ---------------------------------------------------------------------------
#define FILTER_SAMPLE_COUNT    16

// ---------------------------------------------------------------------------
// Spin-direction source selection
//
// SPIN_SOURCE_ENCODER  : quadrature encoder on PIN_ENCODER_A / PIN_ENCODER_B
// SPIN_SOURCE_POT_360  : 3-wire 360-degree rotation potentiometer (ADC) on
//                        PIN_SPIN_DIR_POT, voltage swept linearly maps to
//                        0..360 degrees of spin angle.
//
// Set SPIN_DIRECTION_SOURCE to whichever is physically installed.
// ---------------------------------------------------------------------------
#define SPIN_SOURCE_ENCODER    1
#define SPIN_SOURCE_POT_360    2

#define SPIN_DIRECTION_SOURCE  SPIN_SOURCE_ENCODER

// --- Quadrature encoder settings (used when SPIN_SOURCE_ENCODER) -----------
// Encoder counts per full mechanical revolution of the spin-direction dial.
// Used with DEGREES_PER_CLICK to convert encoder counts to an angle.
//
// For an EC11-style 11mm incremental encoder this firmware counts on every
// A/B edge (4 counts per quadrature cycle). A typical 20-detent EC11 yields
// ~80 counts per physical revolution, so 80 is a sensible starting point.
// VERIFY ON-SITE: run esp32_diagnostic, rotate the knob exactly one full
// turn, read the change in encCount, and set this to that number.
#define ENCODER_COUNTS_PER_REV   80

// Reverse the spin-direction sense without re-wiring: when true, the
// accumulated encoder angle is negated (clockwise vs counter-clockwise
// rotation is flipped) before being used by the mixing math.
#define INVERT_SPIN_DIR_ENCODER  true

// Minimum time (microseconds) between accepted A/B edges, used to reject
// mechanical contact bounce in the ISR. Cheap encoders like the EC11 can
// bounce for a few hundred microseconds to a couple of milliseconds per
// transition. Increase if angle readings still jitter; decrease if fast
// rotation feels like it's losing counts.
#define ENCODER_DEBOUNCE_US      800

// Degrees of spin-angle change per encoder click (quadrature edge count).
// Override this directly if the dial does not map 1:1 to 360 degrees,
// e.g. a dial with a gear ratio or a partial-turn range.
#define DEGREES_PER_CLICK      (360.0f / ENCODER_COUNTS_PER_REV)

// --- 360-degree pot settings (used when SPIN_SOURCE_POT_360) ---------------
// Raw ADC counts at the two ends of the pot's electrical travel. A 360-deg
// rotation pot sweeps voltage across (nearly) a full turn; calibrate the
// same way as the other pots using esp32_diagnostic. The normalized 0..1
// reading is then scaled to SPIN_DIR_ANGLE_SPAN degrees.
#define SPIN_DIR_POT_ADC_MIN   120
#define SPIN_DIR_POT_ADC_MAX   4000

// Degrees represented by the full electrical travel of the pot. 360 for a
// true single-turn 360-deg pot; reduce if the usable electrical range is
// less than a full turn.
#define SPIN_DIR_ANGLE_SPAN    360.0f

// Set true if turning the dial clockwise decreases the angle reading.
#define INVERT_SPIN_DIR_POT    false

// ---------------------------------------------------------------------------
// Output mode
//
// Choose exactly one. This selects how applyOutput() converts a normalized
// 0.0-1.0 command into a physical signal.
//   OUTPUT_MODE_DUTY_PWM : fixed-frequency PWM, variable duty cycle
//   OUTPUT_MODE_SERVO_US : 50 Hz servo-style pulse, variable pulse width
// ---------------------------------------------------------------------------
#define OUTPUT_MODE_DUTY_PWM   1
#define OUTPUT_MODE_SERVO_US   2

#define OUTPUT_MODE            OUTPUT_MODE_DUTY_PWM

// --- DUTY_PWM settings -----------------------------------------------------
#define PWM_FREQUENCY_HZ       20000   // fixed PWM frequency (Hz)
#define PWM_RESOLUTION_BITS    10      // LEDC duty resolution (1-15 bits typical)

// Normalized command 0.0 maps to MIN duty, 1.0 maps to MAX duty.
// Many ESCs/motor boards need a non-zero minimum duty to stay "awake".
#define PWM_DUTY_MIN           0.50f
#define PWM_DUTY_MAX           0.90f

// --- SERVO_US settings ------------------------------------------------------
#define SERVO_FREQUENCY_HZ     50      // standard servo refresh rate
#define SERVO_PWM_RESOLUTION_BITS 14   // LEDC resolution used to synthesize us pulses

// Default no-reverse pulse-width range (microseconds).
#define SERVO_US_NEUTRAL       1500    // stop / neutral
#define SERVO_US_MIN_THROW     1600    // normalized command 0.0
#define SERVO_US_MAX_THROW     1900    // normalized command 1.0
#define SERVO_US_HARD_MAX      2000    // absolute ceiling, never exceeded

// ---------------------------------------------------------------------------
// Throw command limits (normalized 0.0-1.0 internal command space)
//
// MIN_THROW_COMMAND / MAX_THROW_COMMAND bound the "useful" range that the
// pitch-speed pot maps onto. HARD_SAFETY_MAX_COMMAND is an absolute ceiling
// that can never be exceeded regardless of mixing/headroom math.
// ---------------------------------------------------------------------------
#define MIN_THROW_COMMAND      0.0f
#define MAX_THROW_COMMAND      1.0f
#define HARD_SAFETY_MAX_COMMAND 1.0f

// ---------------------------------------------------------------------------
// Per-wheel balance trim (normalized, added after mixing math)
//
// Use these to compensate for one wheel running physically slower than the
// others -- e.g. a motor that is slightly weaker, or belt tension differences.
// Applied after all mixing and before the hard safety ceiling, always,
// regardless of spin rate or angle.
//
// Start with 0.0 on all three. If the ball is drifting left/right with the
// spin knob at zero, bump the slow wheel up in small steps (e.g. 0.02-0.05)
// until the ball flies straight. Positive = faster, negative = slower.
//
//   Wheel A = bottom-middle
//   Wheel B = top-right
//   Wheel C = top-left
// ---------------------------------------------------------------------------
#define WHEEL_A_TRIM           0.00f
#define WHEEL_B_TRIM           0.00f
#define WHEEL_C_TRIM           0.00f

// ---------------------------------------------------------------------------
// Spin mixing limits
//
// SPIN_MAX_DELTA: maximum normalized contribution that the spin pot can add
// to (or subtract from) the base command on any single wheel before
// headroom clamping. Keep well below 1.0 so spin remains usable across the
// full pitch-speed range.
// ---------------------------------------------------------------------------
#define SPIN_MAX_DELTA         0.30f

// ---------------------------------------------------------------------------
// Reverse / direction safety
//
// ALLOW_REVERSE: when false (default), all computed commands are clamped to
// the forward-only range [0.0, 1.0] and SERVO_US output never goes below
// SERVO_US_NEUTRAL. Only enable reverse if the motor controllers and
// mechanical design explicitly support it.
// ---------------------------------------------------------------------------
#define ALLOW_REVERSE          false

// ---------------------------------------------------------------------------
// Boot / safety behavior
// ---------------------------------------------------------------------------
// Outputs are forced to safe-stop for this long after boot, regardless of
// mode, to give the operator time to react before any motor spins up.
#define BOOT_SAFETY_DELAY_MS   2000

// When true, outputs stay disabled at boot until explicitly armed in code
// (see ENABLE_OUTPUTS_AT_BOOT below combined with the speed-knob check).
#define REQUIRE_SPEED_KNOB_AT_MIN_ON_BOOT  false

// Knob is considered "at minimum" if its normalized value is below this
// threshold during the boot check.
#define SPEED_KNOB_MIN_THRESHOLD 0.05f

// Whether motor outputs are active immediately after the boot safety delay
// (and after the speed-knob-at-minimum check passes, if enabled).
// MODE_DIAGNOSTIC always overrides this to "outputs disabled".
#define ENABLE_OUTPUTS_AT_BOOT  true

// ---------------------------------------------------------------------------
// Fixed test commands for esp32_pwm_test (normalized 0.0-1.0)
// ---------------------------------------------------------------------------
#define PWM_TEST_COMMAND_A     0.60f
#define PWM_TEST_COMMAND_B     0.60f
#define PWM_TEST_COMMAND_C     0.60f

// ---------------------------------------------------------------------------
// esp32_safe overrides
//
// FORCE_SAFE_LIMITS is defined by the esp32_safe build environment. When
// present, these tighter limits replace the normal throw/spin limits above.
// ---------------------------------------------------------------------------
#ifdef FORCE_SAFE_LIMITS
    #undef MAX_THROW_COMMAND
    #define MAX_THROW_COMMAND   0.55f

    #undef SPIN_MAX_DELTA
    #define SPIN_MAX_DELTA      0.15f
#endif
