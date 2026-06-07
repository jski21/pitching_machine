// ===========================================================================
// main.cpp - 3-wheel pitching machine controller (ESP32 retrofit)
//
// Reads pitch-speed pot, spin-rate pot, and a spin-direction quadrature
// encoder, mixes them into three normalized wheel commands, and drives
// three motor controller boards via PWM or servo-style pulses.
//
// Build mode is selected at compile time via platformio.ini build_flags:
//   MODE_DIAGNOSTIC  - read & print sensors only, outputs forced safe
//   MODE_PWM_TEST    - ignore knobs, output fixed test values (bench test)
//   MODE_SINGLE_POT  - drive Wheel A only from the pitch-speed pot (bring-up)
//   MODE_OPEN_LOOP   - full control mode
//   MODE_SAFE        - open loop with conservative limits (defines
//                      MODE_OPEN_LOOP + FORCE_SAFE_LIMITS + FORCE_SERIAL_DEBUG)
//
// Exactly one of MODE_DIAGNOSTIC / MODE_PWM_TEST / MODE_SINGLE_POT /
// MODE_OPEN_LOOP should be active (MODE_SAFE layers on top of MODE_OPEN_LOOP).
// ===========================================================================

#include <Arduino.h>
#include "config.h"
#include "control_math.h"
#include "io.h"
#include "output_driver.h"

#if !defined(MODE_DIAGNOSTIC) && !defined(MODE_PWM_TEST) && \
    !defined(MODE_SINGLE_POT) && !defined(MODE_OPEN_LOOP)
#error "No MODE_* build flag defined -- select a PlatformIO environment (see platformio.ini)"
#endif

// ---------------------------------------------------------------------------
// Module-level state
// ---------------------------------------------------------------------------
static InputSnapshot g_inputs;
static OutputValues  g_outputs;
static bool          g_outputsArmed = false;
static unsigned long g_lastDebugPrintMs = 0;

// True only in modes where commanding the motors is meaningful.
// MODE_DIAGNOSTIC always forces outputs to safe-stop regardless of config.
#if defined(MODE_DIAGNOSTIC)
static const bool MODE_ALLOWS_OUTPUT = false;
#else
static const bool MODE_ALLOWS_OUTPUT = true;
#endif

static void printDebug(const InputSnapshot &in, const OutputValues &out);
static const char *modeName();

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    if (SERIAL_DEBUG_ENABLE) {
        Serial.begin(SERIAL_BAUD);
        delay(50); // let USB-serial enumerate; not a control-loop delay
        Serial.println();
        Serial.print(F("Pitching machine controller starting. Mode: "));
        Serial.println(modeName());
    }

    ioInit();
    outputDriverInit();

    // Force safe-stop immediately, before the boot delay even starts.
    setAllOutputsSafe(g_outputs);

    // -----------------------------------------------------------------
    // Boot safety delay: outputs stay at safe-stop for a fixed period so
    // the operator has time to step back / react. This is the ONE place
    // a blocking delay() is allowed in this firmware.
    // -----------------------------------------------------------------
    if (SERIAL_DEBUG_ENABLE) {
        Serial.print(F("Boot safety delay: "));
        Serial.print(BOOT_SAFETY_DELAY_MS);
        Serial.println(F(" ms..."));
    }
    delay(BOOT_SAFETY_DELAY_MS);

    // -----------------------------------------------------------------
    // Optional gate: require the speed knob to be at minimum before the
    // controller will arm outputs. This prevents a machine left with the
    // speed knob "hot" from immediately throwing on power-up.
    // -----------------------------------------------------------------
#if REQUIRE_SPEED_KNOB_AT_MIN_ON_BOOT
    if (MODE_ALLOWS_OUTPUT && ENABLE_OUTPUTS_AT_BOOT) {
        if (SERIAL_DEBUG_ENABLE) {
            Serial.println(F("Waiting for speed knob to be at minimum..."));
        }
        while (true) {
            readInputs(g_inputs);
            setAllOutputsSafe(g_outputs);
            if (g_inputs.pitchInPlausibleRange &&
                g_inputs.pitchNormalized <= SPEED_KNOB_MIN_THRESHOLD) {
                break;
            }
        }
        if (SERIAL_DEBUG_ENABLE) {
            Serial.println(F("Speed knob at minimum. Arming outputs."));
        }
    }
#endif

    g_outputsArmed = MODE_ALLOWS_OUTPUT && ENABLE_OUTPUTS_AT_BOOT;
}

// ---------------------------------------------------------------------------
// loop() -- millis()-based scheduling, no blocking delays here
// ---------------------------------------------------------------------------
void loop() {
    unsigned long now = millis();

#if defined(MODE_DIAGNOSTIC)
    // Sensors only. Outputs are always forced to safe-stop in this mode,
    // regardless of ENABLE_OUTPUTS_AT_BOOT, so the machine cannot throw
    // while a tech is probing wiring.
    readInputs(g_inputs);
    updateEncoder(g_inputs);
    setAllOutputsSafe(g_outputs);

#elif defined(MODE_PWM_TEST)
    // Knobs ignored entirely. Drive fixed test commands to all channels
    // so a tech can verify motor controller response with a known signal.
    g_inputs = InputSnapshot{};
    {
        WheelCommands testCmd;
        testCmd.a = PWM_TEST_COMMAND_A;
        testCmd.b = PWM_TEST_COMMAND_B;
        testCmd.c = PWM_TEST_COMMAND_C;

        if (g_outputsArmed) {
            applyOutput(testCmd, g_outputs);
        } else {
            setAllOutputsSafe(g_outputs);
        }
    }

#elif defined(MODE_SINGLE_POT)
    // Bring-up test: only the pitch-speed pot is read, and only Wheel A
    // is driven. Wheels B/C are held at safe-stop. Useful for bench
    // testing one motor channel before the rest of the system is wired.
    readInputs(g_inputs);
    updateEncoder(g_inputs);
    {
        bool sensorOk = g_inputs.pitchInPlausibleRange;
        WheelCommands singleCmd;
        singleCmd.a = sensorOk
            ? MIN_THROW_COMMAND + g_inputs.pitchNormalized * (MAX_THROW_COMMAND - MIN_THROW_COMMAND)
            : 0.0f;
        singleCmd.b = 0.0f;
        singleCmd.c = 0.0f;

        if (g_outputsArmed && sensorOk) {
            applyOutput(singleCmd, g_outputs);
        } else {
            setAllOutputsSafe(g_outputs);
        }
    }

#elif defined(MODE_OPEN_LOOP)
    // Full control mode (also used by MODE_SAFE, which layers on tighter
    // limits via FORCE_SAFE_LIMITS in config.h).
    readInputs(g_inputs);
    updateEncoder(g_inputs);
    {
        bool sensorsOk = g_inputs.pitchInPlausibleRange && g_inputs.spinInPlausibleRange;

        if (g_outputsArmed && sensorsOk) {
            float baseCommand = MIN_THROW_COMMAND +
                g_inputs.pitchNormalized * (MAX_THROW_COMMAND - MIN_THROW_COMMAND);
            baseCommand = clampf(baseCommand, MIN_THROW_COMMAND, MAX_THROW_COMMAND);

            WheelCommands cmds = computeWheelCommands(baseCommand,
                                                       g_inputs.spinNormalized,
                                                       g_inputs.spinAngleDegrees,
                                                       SPIN_MAX_DELTA,
                                                       ALLOW_REVERSE);
            applyOutput(cmds, g_outputs);
        } else {
            // Sensor fault or outputs not armed -> safe stop, no exceptions.
            setAllOutputsSafe(g_outputs);
        }
    }
#endif

    if (SERIAL_DEBUG_ENABLE && (now - g_lastDebugPrintMs >= DEBUG_PRINT_INTERVAL_MS)) {
        g_lastDebugPrintMs = now;
        printDebug(g_inputs, g_outputs);
    }
}

// ---------------------------------------------------------------------------
// printDebug() - human-readable field-debugging output
// ---------------------------------------------------------------------------
static void printDebug(const InputSnapshot &in, const OutputValues &out) {
    Serial.print(F("[" ));
    Serial.print(modeName());
    Serial.print(F("] "));

    Serial.print(F("rawPitch=")); Serial.print(in.pitchPotRaw);
    Serial.print(F(" rawSpin=")); Serial.print(in.spinPotRaw);

    Serial.print(F(" | normPitch=")); Serial.print(in.pitchNormalized, 3);
    Serial.print(F(" normSpin=")); Serial.print(in.spinNormalized, 3);

    Serial.print(F(" | encCount=")); Serial.print(in.encoderCount);
    Serial.print(F(" angle=")); Serial.print(in.spinAngleDegrees, 1);
    Serial.print(F("deg"));

    if (!in.pitchInPlausibleRange || !in.spinInPlausibleRange) {
        Serial.print(F(" | SENSOR FAULT"));
        if (!in.pitchInPlausibleRange) Serial.print(F(" [pitch out of range]"));
        if (!in.spinInPlausibleRange)  Serial.print(F(" [spin out of range]"));
    }

    Serial.print(F(" | wheelCmd A=")); Serial.print(out.normalizedA, 3);
    Serial.print(F(" B=")); Serial.print(out.normalizedB, 3);
    Serial.print(F(" C=")); Serial.print(out.normalizedC, 3);

#if OUTPUT_MODE == OUTPUT_MODE_DUTY_PWM
    Serial.print(F(" | duty A=")); Serial.print(out.finalA, 3);
    Serial.print(F(" B=")); Serial.print(out.finalB, 3);
    Serial.print(F(" C=")); Serial.print(out.finalC, 3);
#elif OUTPUT_MODE == OUTPUT_MODE_SERVO_US
    Serial.print(F(" | us A=")); Serial.print(out.finalA, 0);
    Serial.print(F(" B=")); Serial.print(out.finalB, 0);
    Serial.print(F(" C=")); Serial.print(out.finalC, 0);
#endif

    Serial.print(F(" | armed=")); Serial.print(g_outputsArmed ? F("YES") : F("no"));

    Serial.println();
}

static const char *modeName() {
#if defined(MODE_SAFE)
    return "SAFE";
#elif defined(MODE_DIAGNOSTIC)
    return "DIAGNOSTIC";
#elif defined(MODE_PWM_TEST)
    return "PWM_TEST";
#elif defined(MODE_SINGLE_POT)
    return "SINGLE_POT";
#elif defined(MODE_OPEN_LOOP)
    return "OPEN_LOOP";
#else
    return "UNKNOWN";
#endif
}
