#include "output_driver.h"
#include "config.h"

// LEDC channel assignment (0-15 available on ESP32).
static const int LEDC_CHANNEL_A = 0;
static const int LEDC_CHANNEL_B = 1;
static const int LEDC_CHANNEL_C = 2;

#if OUTPUT_MODE == OUTPUT_MODE_DUTY_PWM
static const uint32_t LEDC_FREQ_HZ      = PWM_FREQUENCY_HZ;
static const uint8_t  LEDC_RESOLUTION   = PWM_RESOLUTION_BITS;
#elif OUTPUT_MODE == OUTPUT_MODE_SERVO_US
static const uint32_t LEDC_FREQ_HZ      = SERVO_FREQUENCY_HZ;
static const uint8_t  LEDC_RESOLUTION   = SERVO_PWM_RESOLUTION_BITS;
#else
#error "OUTPUT_MODE must be OUTPUT_MODE_DUTY_PWM or OUTPUT_MODE_SERVO_US"
#endif

static uint32_t ledcMaxDutyTicks() {
    return (1UL << LEDC_RESOLUTION) - 1UL;
}

void outputDriverInit() {
    ledcSetup(LEDC_CHANNEL_A, LEDC_FREQ_HZ, LEDC_RESOLUTION);
    ledcSetup(LEDC_CHANNEL_B, LEDC_FREQ_HZ, LEDC_RESOLUTION);
    ledcSetup(LEDC_CHANNEL_C, LEDC_FREQ_HZ, LEDC_RESOLUTION);

    ledcAttachPin(PIN_MOTOR_A, LEDC_CHANNEL_A);
    ledcAttachPin(PIN_MOTOR_B, LEDC_CHANNEL_B);
    ledcAttachPin(PIN_MOTOR_C, LEDC_CHANNEL_C);
}

// ---------------------------------------------------------------------------
// DUTY_PWM mapping
//
// Normalized 0.0-1.0 maps linearly onto [PWM_DUTY_MIN, PWM_DUTY_MAX].
// DUTY_PWM boards in this system have no separate direction pin, so there
// is no physical way to represent "reverse" -- a negative normalized
// command (only possible if ALLOW_REVERSE is set) is clamped to 0.0
// (i.e. minimum/stop duty) rather than driving the motor backwards.
// ---------------------------------------------------------------------------
static float normalizedToDuty(float cmd) {
    cmd = clampf(cmd, 0.0f, 1.0f);
    float duty = PWM_DUTY_MIN + cmd * (PWM_DUTY_MAX - PWM_DUTY_MIN);
    return clampf(duty, PWM_DUTY_MIN, PWM_DUTY_MAX);
}

static void writeDuty(int channel, float duty, float &finalOut) {
    finalOut = duty;
    uint32_t ticks = (uint32_t)lroundf(duty * (float)ledcMaxDutyTicks());
    ledcWrite(channel, ticks);
}

// ---------------------------------------------------------------------------
// SERVO_US mapping
//
// Forward-only (default, ALLOW_REVERSE == false):
//   normalized 0.0 -> SERVO_US_MIN_THROW
//   normalized 1.0 -> SERVO_US_MAX_THROW
//   (note: this is intentionally ABOVE neutral -- motors always spin
//    forward at the configured "minimum throw" speed, never stopping
//    unless setAllOutputsSafe() drives SERVO_US_NEUTRAL directly)
//
// Reverse-enabled (ALLOW_REVERSE == true):
//   normalized -1.0 .. 0.0 maps to a mirror-image band below neutral
//   normalized  0.0 .. 1.0 maps to SERVO_US_NEUTRAL .. SERVO_US_MAX_THROW
//   This assumes the connected ESC supports bidirectional servo-style
//   pulses (centered on 1500us). Verify with the motor controller
//   documentation before enabling reverse.
//
// In all cases the result is clamped to the hard pulse-width limits.
// ---------------------------------------------------------------------------
static float normalizedToServoUs(float cmd) {
    float us;

    if (!ALLOW_REVERSE) {
        cmd = clampf(cmd, 0.0f, 1.0f);
        us = SERVO_US_MIN_THROW + cmd * (SERVO_US_MAX_THROW - SERVO_US_MIN_THROW);

        float hardLow = SERVO_US_NEUTRAL; // never command below neutral when reverse disallowed
        return clampf(us, hardLow, (float)SERVO_US_HARD_MAX);
    }

    cmd = clampf(cmd, -1.0f, 1.0f);
    float forwardSpan = (float)SERVO_US_MAX_THROW - (float)SERVO_US_NEUTRAL;

    if (cmd >= 0.0f) {
        us = SERVO_US_NEUTRAL + cmd * forwardSpan;
    } else {
        us = SERVO_US_NEUTRAL + cmd * forwardSpan; // mirror image below neutral
    }

    float hardLow  = (float)SERVO_US_NEUTRAL - forwardSpan;
    float hardHigh = (float)SERVO_US_HARD_MAX;
    return clampf(us, hardLow, hardHigh);
}

static void writeServoUs(int channel, float microseconds, float &finalOut) {
    finalOut = microseconds;
    float periodUs = 1000000.0f / (float)LEDC_FREQ_HZ;
    float dutyFraction = microseconds / periodUs;
    uint32_t ticks = (uint32_t)lroundf(dutyFraction * (float)ledcMaxDutyTicks());
    ledcWrite(channel, ticks);
}

// Apply the hard safety ceiling to a normalized command before conversion.
// HARD_SAFETY_MAX_COMMAND can never be exceeded regardless of mixing math.
static float applyHardSafetyCeiling(float cmd) {
    float lowerBound = ALLOW_REVERSE ? -HARD_SAFETY_MAX_COMMAND : 0.0f;
    return clampf(cmd, lowerBound, HARD_SAFETY_MAX_COMMAND);
}

void applyOutput(const WheelCommands &commands, OutputValues &out) {
    float a = applyHardSafetyCeiling(commands.a + WHEEL_A_TRIM);
    float b = applyHardSafetyCeiling(commands.b + WHEEL_B_TRIM);
    float c = applyHardSafetyCeiling(commands.c + WHEEL_C_TRIM);

    out.normalizedA = a;
    out.normalizedB = b;
    out.normalizedC = c;

#if OUTPUT_MODE == OUTPUT_MODE_DUTY_PWM
    writeDuty(LEDC_CHANNEL_A, normalizedToDuty(a), out.finalA);
    writeDuty(LEDC_CHANNEL_B, normalizedToDuty(b), out.finalB);
    writeDuty(LEDC_CHANNEL_C, normalizedToDuty(c), out.finalC);
#elif OUTPUT_MODE == OUTPUT_MODE_SERVO_US
    writeServoUs(LEDC_CHANNEL_A, normalizedToServoUs(a), out.finalA);
    writeServoUs(LEDC_CHANNEL_B, normalizedToServoUs(b), out.finalB);
    writeServoUs(LEDC_CHANNEL_C, normalizedToServoUs(c), out.finalC);
#endif
}

void setAllOutputsSafe(OutputValues &out) {
    out.normalizedA = 0.0f;
    out.normalizedB = 0.0f;
    out.normalizedC = 0.0f;

#if OUTPUT_MODE == OUTPUT_MODE_DUTY_PWM
    writeDuty(LEDC_CHANNEL_A, PWM_DUTY_MIN, out.finalA);
    writeDuty(LEDC_CHANNEL_B, PWM_DUTY_MIN, out.finalB);
    writeDuty(LEDC_CHANNEL_C, PWM_DUTY_MIN, out.finalC);
#elif OUTPUT_MODE == OUTPUT_MODE_SERVO_US
    writeServoUs(LEDC_CHANNEL_A, (float)SERVO_US_NEUTRAL, out.finalA);
    writeServoUs(LEDC_CHANNEL_B, (float)SERVO_US_NEUTRAL, out.finalB);
    writeServoUs(LEDC_CHANNEL_C, (float)SERVO_US_NEUTRAL, out.finalC);
#endif
}
