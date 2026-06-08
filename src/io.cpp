#include "io.h"
#include "config.h"
#include "control_math.h"

// ---------------------------------------------------------------------------
// Moving-average filter state (one ring buffer per potentiometer channel)
// ---------------------------------------------------------------------------
static int  pitchSampleBuf[FILTER_SAMPLE_COUNT];
static int  spinSampleBuf[FILTER_SAMPLE_COUNT];
static int  pitchSampleIndex = 0;
static int  spinSampleIndex  = 0;
static long pitchSampleSum   = 0;
static long spinSampleSum    = 0;
static int  samplesCollected = 0;

// ---------------------------------------------------------------------------
// Quadrature encoder state
//
// volatile because it is written from interrupt context (ISRs) and read
// from the main loop. encoderCount increments/decrements by 1 per valid
// quadrature edge transition.
// ---------------------------------------------------------------------------
static volatile long encoderCount = 0;
static volatile uint8_t lastEncoderState = 0;

// Quadrature transition table indexed by (previous state << 2 | new state).
// Valid forward transitions yield +1, valid reverse transitions yield -1,
// invalid/bounce transitions yield 0 and are ignored.
static const int8_t QUADRATURE_TABLE[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

static void IRAM_ATTR encoderISR() {
    uint8_t a = (uint8_t)digitalRead(PIN_ENCODER_A);
    uint8_t b = (uint8_t)digitalRead(PIN_ENCODER_B);
    uint8_t newState = (uint8_t)((a << 1) | b);
    uint8_t index = (uint8_t)((lastEncoderState << 2) | newState);
    encoderCount += QUADRATURE_TABLE[index & 0x0F];
    lastEncoderState = newState;
}

void ioInit() {
    // ESP32 ADC1 pins (32-39) work without conflicting with WiFi.
    analogReadResolution(ADC_RESOLUTION_BITS);
    analogSetPinAttenuation(PIN_PITCH_SPEED_POT, ADC_11db);
    analogSetPinAttenuation(PIN_SPIN_RATE_POT, ADC_11db);

    pinMode(PIN_PITCH_SPEED_POT, INPUT);
    pinMode(PIN_SPIN_RATE_POT, INPUT);

    pinMode(PIN_ENCODER_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_B, INPUT_PULLUP);

    uint8_t a = (uint8_t)digitalRead(PIN_ENCODER_A);
    uint8_t b = (uint8_t)digitalRead(PIN_ENCODER_B);
    lastEncoderState = (uint8_t)((a << 1) | b);

    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoderISR, CHANGE);

    for (int i = 0; i < FILTER_SAMPLE_COUNT; i++) {
        pitchSampleBuf[i] = 0;
        spinSampleBuf[i]  = 0;
    }
    pitchSampleSum = 0;
    spinSampleSum  = 0;
    pitchSampleIndex = 0;
    spinSampleIndex  = 0;
    samplesCollected = 0;
}

// Push a new raw sample into a ring-buffer moving average and return the
// updated average. Keeping a running sum avoids re-summing the whole
// buffer every call.
static int pushSample(int *buf, int &index, long &sum, int newSample) {
    sum -= buf[index];
    buf[index] = newSample;
    sum += newSample;
    index = (index + 1) % FILTER_SAMPLE_COUNT;
    return (int)(sum / FILTER_SAMPLE_COUNT);
}

static bool isPlausible(int rawCount, int adcMin, int adcMax) {
    int lo = adcMin - ADC_PLAUSIBLE_MARGIN;
    int hi = adcMax + ADC_PLAUSIBLE_MARGIN;
    if (lo < 0) lo = 0;
    if (hi > ADC_MAX_COUNT) hi = ADC_MAX_COUNT;
    return (rawCount >= lo) && (rawCount <= hi);
}

void readInputs(InputSnapshot &snapshot) {
    int pitchRaw = analogRead(PIN_PITCH_SPEED_POT);
    int spinRaw  = analogRead(PIN_SPIN_RATE_POT);

    int pitchFiltered = pushSample(pitchSampleBuf, pitchSampleIndex, pitchSampleSum, pitchRaw);
    int spinFiltered  = pushSample(spinSampleBuf, spinSampleIndex, spinSampleSum, spinRaw);

    if (samplesCollected < FILTER_SAMPLE_COUNT) {
        samplesCollected++;
    }

    snapshot.pitchPotRaw = pitchRaw;
    snapshot.spinPotRaw  = spinRaw;

    snapshot.pitchNormalized = normalizeAdc(pitchFiltered, PITCH_POT_ADC_MIN, PITCH_POT_ADC_MAX);
    snapshot.spinNormalized  = normalizeAdc(spinFiltered, SPIN_POT_ADC_MIN, SPIN_POT_ADC_MAX);

    if (INVERT_PITCH_POT) snapshot.pitchNormalized = 1.0f - snapshot.pitchNormalized;
    if (INVERT_SPIN_POT)  snapshot.spinNormalized  = 1.0f - snapshot.spinNormalized;

    snapshot.pitchInPlausibleRange = isPlausible(pitchRaw, PITCH_POT_ADC_MIN, PITCH_POT_ADC_MAX);
    snapshot.spinInPlausibleRange  = isPlausible(spinRaw, SPIN_POT_ADC_MIN, SPIN_POT_ADC_MAX);
}

void updateEncoder(InputSnapshot &snapshot) {
    // Snapshot the volatile counter atomically with interrupts paused.
    noInterrupts();
    long count = encoderCount;
    interrupts();

    snapshot.encoderCount = count;

    float degrees = (float)count * DEGREES_PER_CLICK;
    snapshot.spinAngleDegrees = wrapAngleDegrees(degrees);
}

bool ioFiltersWarmedUp() {
    return samplesCollected >= FILTER_SAMPLE_COUNT;
}
