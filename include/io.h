#pragma once

// ===========================================================================
// io.h
//
// Hardware input handling: potentiometer ADC reads + filtering, and
// quadrature encoder decoding. Pin numbers and calibration come from
// config.h so field changes stay in one place.
// ===========================================================================

#include <Arduino.h>

// Snapshot of all sensor inputs for one control-loop iteration.
struct InputSnapshot {
    int   pitchPotRaw;       // raw ADC count, pitch speed pot
    int   spinPotRaw;        // raw ADC count, spin rate pot

    float pitchNormalized;   // filtered + normalized 0.0-1.0
    float spinNormalized;    // filtered + normalized 0.0-1.0

    long  encoderCount;      // raw signed quadrature count
    float spinAngleDegrees;  // continuous 0-360 angle derived from encoder

    bool  pitchInPlausibleRange; // false => sensor fault, force safe stop
    bool  spinInPlausibleRange;  // false => sensor fault, force safe stop
};

// Configure ADC pins and attach the quadrature encoder interrupt handlers.
// Call once from setup().
void ioInit();

// Read both potentiometers, run them through the moving-average filter,
// normalize against the calibration range in config.h, and check
// plausibility. Does not touch the encoder (see updateEncoder()).
void readInputs(InputSnapshot &snapshot);

// Service the quadrature encoder. The actual count is updated from ISRs;
// this function converts the latest count into a continuous 0-360 degree
// angle and writes it into the snapshot. Safe to call every loop iteration.
void updateEncoder(InputSnapshot &snapshot);

// Returns true once the moving-average filters have collected enough
// samples to produce a meaningful reading (avoids using zero-filled
// buffers immediately after boot).
bool ioFiltersWarmedUp();
