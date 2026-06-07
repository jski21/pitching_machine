#include "control_math.h"
#include <math.h>

float clampf(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

float normalizeAdc(int rawCount, int adcMin, int adcMax) {
    if (adcMax == adcMin) {
        return 0.0f;
    }
    float norm = (float)(rawCount - adcMin) / (float)(adcMax - adcMin);
    return clampf(norm, 0.0f, 1.0f);
}

float wrapAngleDegrees(float degrees) {
    float wrapped = fmodf(degrees, 360.0f);
    if (wrapped < 0.0f) {
        wrapped += 360.0f;
    }
    return wrapped;
}

static float degreesToRadians(float degrees) {
    return degrees * (float)M_PI / 180.0f;
}

WheelCommands computeWheelCommands(float baseCommand,
                                   float spinCommand,
                                   float thetaDegrees,
                                   float spinMaxDelta,
                                   bool allowReverse) {
    const float lowerBound = allowReverse ? -1.0f : 0.0f;
    const float upperBound = 1.0f;

    baseCommand = clampf(baseCommand, lowerBound, upperBound);
    spinCommand = clampf(spinCommand, 0.0f, 1.0f);

    // Requested spin contribution magnitude before headroom protection.
    float requestedDelta = spinCommand * spinMaxDelta;

    // Headroom protection: shrink the spin contribution so that
    // base +/- delta cannot leave the valid output range. This keeps
    // spin usable (at reduced authority) even when the base command is
    // pinned near the top or bottom of its range.
    float headroomUp   = upperBound - baseCommand;
    float headroomDown = baseCommand - lowerBound;
    float effectiveDelta = requestedDelta;
    effectiveDelta = fminf(effectiveDelta, headroomUp);
    effectiveDelta = fminf(effectiveDelta, headroomDown);
    effectiveDelta = fmaxf(effectiveDelta, 0.0f);

    float theta = wrapAngleDegrees(thetaDegrees);
    float thetaRad = degreesToRadians(theta);

    WheelCommands out;
    out.a = baseCommand + effectiveDelta * cosf(thetaRad);
    out.b = baseCommand + effectiveDelta * cosf(degreesToRadians(theta - 120.0f));
    out.c = baseCommand + effectiveDelta * cosf(degreesToRadians(theta - 240.0f));

    out.a = clampf(out.a, lowerBound, upperBound);
    out.b = clampf(out.b, lowerBound, upperBound);
    out.c = clampf(out.c, lowerBound, upperBound);

    return out;
}
