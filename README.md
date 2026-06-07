# pitching_machine

ESP32 (Arduino framework / PlatformIO) replacement controller for a 3-wheel
pitching machine that originally used an aftermarket Arduino Nano board.

The controller reads a pitch-speed potentiometer, a spin-rate potentiometer,
and a quadrature rotary encoder (spin direction), mixes them into three
normalized wheel commands using a 3-wheel vector model, and drives three
independent motor controller boards via PWM or servo-style pulses.

This document is written for **field debugging first, elegance second**.
If something isn't working, the diagnostic sequence near the bottom of this
file is the fastest way to isolate the problem.

---

## 1. Wiring overview

| Signal                      | ESP32 pin (default, see `include/config.h`) | Notes |
|-----------------------------|----------------------------------------------|-------|
| Pitch speed potentiometer   | GPIO34 (`PIN_PITCH_SPEED_POT`)               | ADC1 input-only pin, wiper to pin, ends to 3.3V/GND |
| Spin rate potentiometer     | GPIO35 (`PIN_SPIN_RATE_POT`)                 | ADC1 input-only pin, wiper to pin, ends to 3.3V/GND |
| Spin direction encoder A    | GPIO25 (`PIN_ENCODER_A`)                     | Quadrature channel A, internal pull-up enabled |
| Spin direction encoder B    | GPIO26 (`PIN_ENCODER_B`)                     | Quadrature channel B, internal pull-up enabled |
| Motor A signal              | GPIO32 (`PIN_MOTOR_A`)                       | PWM or servo pulse to motor controller "Wheel A" |
| Motor B signal              | GPIO33 (`PIN_MOTOR_B`)                       | PWM or servo pulse to motor controller "Wheel B" |
| Motor C signal              | GPIO27 (`PIN_MOTOR_C`)                       | PWM or servo pulse to motor controller "Wheel C" |
| Common ground               | GND                                          | **Must be shared between ESP32 and every motor controller board** |

All pins above are edited in one place: `include/config.h`. If the retrofit
wiring differs from the table, change the `PIN_*` defines there — nothing
else in the code needs to change.

> **WARNING — read before connecting anything:**
> Verify the motor controller boards' signal ground is common with the ESP32
> ground, and confirm their input voltage tolerance (most accept 3.3V logic,
> but some older Nano-based aftermarket boards expect 5V signal levels).
> The ESP32 is **not 5V tolerant** on its GPIO pins. Use a level shifter if
> the existing boards require 5V logic. Double-check this *before* powering
> up with motors connected.

ADC pins 34/35 are input-only on the ESP32 and cannot drive outputs — that's
intentional, since they're only used for the potentiometers.

---

## 2. Choosing an output mode

Set `OUTPUT_MODE` in `include/config.h` to one of:

* `OUTPUT_MODE_DUTY_PWM` — fixed-frequency PWM with variable duty cycle.
  Configurable via `PWM_FREQUENCY_HZ`, `PWM_RESOLUTION_BITS`,
  `PWM_DUTY_MIN`, `PWM_DUTY_MAX`.
* `OUTPUT_MODE_SERVO_US` — 50 Hz servo-style pulses with variable pulse
  width. Configurable via `SERVO_US_NEUTRAL`, `SERVO_US_MIN_THROW`,
  `SERVO_US_MAX_THROW`, `SERVO_US_HARD_MAX`.

Both modes are implemented with the ESP32 LEDC peripheral (`output_driver.cpp`).
`SERVO_US` simply runs LEDC at 50 Hz and converts a microsecond pulse width
into the equivalent LEDC duty value — there is no separate "servo library"
dependency.

**`DUTY_PWM` vs `SERVO_US` — which do I need?**

* If the existing motor controller boards came from an RC/servo-style ESC
  background (e.g. they expect a ~1500us-centered pulse at 50Hz), use
  `SERVO_US`.
* If they expect a simple variable-duty PWM signal at a fixed higher
  frequency (common on brushed-motor speed controllers), use `DUTY_PWM`.
* **You can determine this empirically** — see the diagnostic sequence
  below (step 4). Run `esp32_pwm_test`, probe the signal with a meter or
  scope, and watch how the motor responds as you change `OUTPUT_MODE` and
  re-flash.

---

## 3. PlatformIO environments

This project defines five build environments in `platformio.ini`. Each one
sets a `MODE_*` compile-time flag that `src/main.cpp` branches on.

| Environment          | Build flag(s)                                              | Purpose |
|----------------------|------------------------------------------------------------|---------|
| `esp32_diagnostic`   | `-DMODE_DIAGNOSTIC`                                        | Read & print sensors only. Outputs are **always** forced to safe-stop. |
| `esp32_pwm_test`     | `-DMODE_PWM_TEST`                                          | Ignore knobs, drive fixed test commands (`PWM_TEST_COMMAND_*` in config.h) to all 3 channels. |
| `esp32_single_pot`   | `-DMODE_SINGLE_POT`                                        | Bring-up test: read only the pitch-speed pot, drive **Wheel A only**; B/C held at safe-stop. |
| `esp32_open_loop`    | `-DMODE_OPEN_LOOP`                                         | Full control: read both pots + encoder, mix 3-wheel vector commands, drive all motors. |
| `esp32_safe`         | `-DMODE_SAFE -DMODE_OPEN_LOOP -DFORCE_SAFE_LIMITS -DFORCE_SERIAL_DEBUG` | Same as open loop, but with a lower max throw command, lower spin delta, and serial debug forced on. |

Run any environment with the PlatformIO CLI, e.g.:

```sh
pio run -e esp32_diagnostic -t upload -t monitor
pio run -e esp32_pwm_test   -t upload -t monitor
pio run -e esp32_single_pot -t upload -t monitor
pio run -e esp32_safe       -t upload -t monitor
pio run -e esp32_open_loop  -t upload -t monitor
```

(Or use the PlatformIO IDE's environment selector / Build-Upload-Monitor
buttons.) Serial monitor runs at **115200 baud**.

---

## 4. On-site calibration procedure

All calibration constants live in `include/config.h`. Typical procedure:

1. **Flash `esp32_diagnostic`** with the motor outputs disconnected (see
   the diagnostic sequence below). Open the serial monitor.
2. **Calibrate the potentiometers:**
   - Rotate the pitch-speed pot to its physical minimum. Note the
     `rawPitch` value printed. Repeat at the physical maximum.
   - Enter those two numbers as `PITCH_POT_ADC_MIN` / `PITCH_POT_ADC_MAX`.
   - Repeat for the spin-rate pot → `SPIN_POT_ADC_MIN` / `SPIN_POT_ADC_MAX`.
3. **Check the encoder direction and scale:**
   - Rotate the spin-direction dial one full turn and confirm `angle`
     wraps cleanly through 0–360°.
   - If the angle increases when it should decrease (or vice versa), swap
     `PIN_ENCODER_A` / `PIN_ENCODER_B`.
   - If one physical revolution doesn't correspond to 360°, adjust
     `ENCODER_COUNTS_PER_REV` (or override `DEGREES_PER_CLICK` directly).
4. **Tune filtering:** if `normPitch`/`normSpin` are noisy/jittery, increase
   `FILTER_SAMPLE_COUNT` (smoother but slower to respond). If response feels
   sluggish, decrease it.
5. **Set the plausibility margin:** `ADC_PLAUSIBLE_MARGIN` defines how far
   outside the calibrated min/max a reading can drift before it's treated
   as a sensor fault (forcing safe-stop). Loosen it only if you're seeing
   false faults at the extreme ends of pot travel.
6. **Tune output ranges** once you know the output mode (see section 2):
   - `DUTY_PWM`: set `PWM_FREQUENCY_HZ`, `PWM_DUTY_MIN`/`PWM_DUTY_MAX` to
     match what the motor controller expects (e.g. 0.50–0.90).
   - `SERVO_US`: set `SERVO_US_MIN_THROW`/`SERVO_US_MAX_THROW` to the pulse
     widths that produce the desired minimum/maximum wheel speed, and
     `SERVO_US_HARD_MAX` to an absolute ceiling you never want to exceed.
7. **Tune throw and spin limits:**
   - `MIN_THROW_COMMAND` / `MAX_THROW_COMMAND` bound the normalized
     pitch-speed range.
   - `HARD_SAFETY_MAX_COMMAND` is an absolute ceiling on every wheel
     command, applied last, regardless of mixing.
   - `SPIN_MAX_DELTA` is the maximum +/- contribution spin can add to any
     one wheel (headroom-protected automatically — see `control_math.cpp`).
8. **Reverse:** leave `ALLOW_REVERSE` at `false` unless you have explicitly
   verified the motor controllers and mechanical design support reverse
   rotation. Changing it changes both the math clamping range and the
   `SERVO_US` pulse-width mapping (see comments in `output_driver.cpp`).

---

## 5. Suggested diagnostic sequence

Follow this order when bringing up a freshly retrofitted (or modified) machine:

1. **Run `esp32_diagnostic` with the motor controller outputs physically
   disconnected.** This guarantees nothing can spin while you're probing.
2. **Confirm pot and encoder readings** in the serial monitor: rotate each
   pot through its full range and watch `rawPitch`/`rawSpin` and
   `normPitch`/`normSpin`; rotate the spin dial and watch `encCount`/`angle`.
3. **Run `esp32_pwm_test` on ONE motor controller board only** (leave the
   other two disconnected). This sends a known, fixed signal
   (`PWM_TEST_COMMAND_*` in config.h) so you can observe exactly how that
   board responds without any knob variability.
4. **Determine whether that board expects `DUTY_PWM` or `SERVO_US`**: probe
   the signal pin with a meter/scope, try both `OUTPUT_MODE` settings (one
   at a time, re-flashing `esp32_pwm_test` between changes), and see which
   produces sensible motor behavior.
5. **Tune `include/config.h`** using the calibration procedure above, now
   that you know the correct output mode and have raw sensor ranges.
6. **Run `esp32_safe`.** This is full open-loop control but with reduced
   max speed and spin authority, and serial debug forced on — the safest
   way to see the whole system working together for the first time.
7. **Run `esp32_open_loop`** once `esp32_safe` behaves correctly and you're
   ready for full-range operation.

If at any point behavior looks wrong, power down, return to
`esp32_diagnostic`, and re-check the relevant readings before continuing.

---

## 6. Safety behavior (built into the firmware)

* **Boot delay:** outputs are forced to safe-stop for `BOOT_SAFETY_DELAY_MS`
  (default 2000ms) after every power-up/reset, regardless of mode.
* **Speed-knob-at-minimum gate:** when
  `REQUIRE_SPEED_KNOB_AT_MIN_ON_BOOT` is `true` (default), the controller
  will not arm outputs until the pitch-speed pot reads below
  `SPEED_KNOB_MIN_THRESHOLD`. This prevents an immediate throw if the
  machine is powered on with the speed knob already turned up.
* **Sensor plausibility check:** if either potentiometer's raw ADC reading
  falls outside its calibrated range (plus `ADC_PLAUSIBLE_MARGIN`), the
  firmware treats it as a fault and immediately forces all outputs to
  safe-stop (`setAllOutputsSafe()`), printing `SENSOR FAULT` to the serial
  log.
* **Hard safety ceiling:** `HARD_SAFETY_MAX_COMMAND` is enforced in
  `applyOutput()` as a final clamp that nothing — mixing math, calibration
  mistakes, etc. — can exceed.
* **No reverse by default:** `ALLOW_REVERSE` defaults to `false`. With it
  false, all wheel commands are clamped to forward-only ranges and
  `SERVO_US` output never drops below `SERVO_US_NEUTRAL`.
* **No blocking delays in the control loop:** all loop timing (debug print
  interval, etc.) uses `millis()`. The only `delay()` calls are the boot
  safety delay and a brief USB-serial settle delay at startup.

---

## 7. Code layout

| File | Responsibility |
|------|----------------|
| `include/config.h` | All pins, calibration constants, output-mode selection, and safety limits. **Edit this file for on-site changes.** |
| `include/control_math.h` / `src/control_math.cpp` | Pure math: normalization, angle wrapping, and the 3-wheel vector mixing model with headroom protection and clamping. No hardware access — testable in isolation. |
| `include/io.h` / `src/io.cpp` | Hardware input: ADC reads with moving-average filtering, calibration/normalization, plausibility checking, and quadrature encoder decoding (`readInputs()`, `updateEncoder()`). |
| `include/output_driver.h` / `src/output_driver.cpp` | Hardware output: ESP32 LEDC setup and conversion of normalized commands to PWM duty or servo microseconds (`applyOutput()`, `setAllOutputsSafe()`). |
| `src/main.cpp` | Mode selection, boot sequence, main loop, and `printDebug()`. |

Core functions (named to match the architecture, implemented across the
files above): `readInputs()`, `updateEncoder()`, `computeWheelCommands()`,
`applyOutput()`, `setAllOutputsSafe()`, `printDebug()`.
