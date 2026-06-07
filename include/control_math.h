#pragma once

// ===========================================================================
// control_math.h
//
// Pure math: normalization helpers and the 3-wheel vector mixing model.
// Nothing in this file touches hardware, so it can be bench-tested in
// isolation (e.g. with a host-side unit test runner) if desired.
//
// All "command" values in this module are normalized to 0.0 (minimum) .. 1.0
// (maximum). Conversion to raw PWM/servo signals happens in output_driver.
// ===========================================================================

#include <stdint.h>

// Result of mixing the base throw command with the spin command/angle.
struct WheelCommands {
    float a;   // normalized command for Wheel A, clamped to [0,1] (or [-1,1] if reverse allowed)
    float b;   // normalized command for Wheel B
    float c;   // normalized command for Wheel C
};

// Clamp a value into [lo, hi].
float clampf(float value, float lo, float hi);

// Map a raw ADC count to a normalized 0.0-1.0 value using the supplied
// calibration range. Result is clamped to [0,1] -- out-of-range raw values
// (within plausible limits) saturate rather than extrapolate.
float normalizeAdc(int rawCount, int adcMin, int adcMax);

// Wrap an angle in degrees into the continuous [0, 360) range.
float wrapAngleDegrees(float degrees);

// Compute normalized commands for all three wheels from:
//   baseCommand  - normalized pitch-speed command (0.0-1.0)
//   spinCommand  - normalized spin-rate command (0.0-1.0), scaled internally
//                  by spinMaxDelta to get the +/- contribution per wheel
//   thetaDegrees - spin direction angle (0-360, continuous)
//   spinMaxDelta - maximum +/- contribution any wheel may receive from spin
//   allowReverse - if false, results are clamped to [0,1] (forward only);
//                  if true, results are clamped to [-1,1]
//
// Headroom protection: the spin contribution is scaled down automatically
// when the base command is close to 0.0 or 1.0, so that adding spin cannot
// push a wheel command outside its valid range and "use up" all of the
// spin authority at the extremes of the throw range.
WheelCommands computeWheelCommands(float baseCommand,
                                   float spinCommand,
                                   float thetaDegrees,
                                   float spinMaxDelta,
                                   bool allowReverse);
