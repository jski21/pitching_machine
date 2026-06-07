#pragma once

// ===========================================================================
// output_driver.h
//
// Converts normalized 0.0-1.0 (or -1.0..1.0 if reverse enabled) commands
// into physical signals on the three motor channels using the ESP32 LEDC
// peripheral. Supports two interchangeable output modes selected at
// compile time via OUTPUT_MODE in config.h:
//
//   OUTPUT_MODE_DUTY_PWM - fixed-frequency PWM, variable duty cycle
//   OUTPUT_MODE_SERVO_US - 50 Hz servo-style pulse, variable pulse width
//
// Both modes use LEDC; SERVO_US simply runs LEDC at 50 Hz and converts a
// microsecond pulse width into the equivalent duty value.
// ===========================================================================

#include <Arduino.h>
#include "control_math.h"

// One entry per motor channel, used for logging/printDebug().
struct OutputValues {
    float normalizedA;
    float normalizedB;
    float normalizedC;

    // Final physical value actually written to hardware.
    // For DUTY_PWM this is a duty fraction 0.0-1.0.
    // For SERVO_US this is a pulse width in microseconds.
    float finalA;
    float finalB;
    float finalC;
};

// Configure LEDC channels/timers for the selected OUTPUT_MODE. Call once
// from setup(), after ioInit().
void outputDriverInit();

// Convert a WheelCommands set (normalized) into physical signals and write
// them to the LEDC channels. Also fills `out` for logging. Internally
// enforces the forward-only rule (ALLOW_REVERSE) and the hard safety
// ceiling (HARD_SAFETY_MAX_COMMAND).
void applyOutput(const WheelCommands &commands, OutputValues &out);

// Immediately drive all three channels to their safe-stop value:
//   DUTY_PWM -> PWM_DUTY_MIN
//   SERVO_US -> SERVO_US_NEUTRAL
// Used at boot, on sensor faults, and whenever outputs must be forced off.
void setAllOutputsSafe(OutputValues &out);
