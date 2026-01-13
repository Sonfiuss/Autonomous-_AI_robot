#pragma once

#include <Arduino.h>

// Pin and polarity mapping for a single DM556 channel
struct StepperPins {
  uint8_t dir;          // Direction output pin
  uint8_t step;         // Step pulse output pin
  uint8_t ena;          // Enable output pin
  bool dirActiveLow;    // true if DIR is active-low
  bool stepActiveLow;   // true if STEP is active-low
  bool enaActiveLow;    // true if ENA is active-low
};

// Three motors (A/B/C) pin assignments and polarities
static constexpr StepperPins motorA{2, 3, 4, false, false, true};
static constexpr StepperPins motorB{5, 6, 7, false, false, true};
static constexpr StepperPins motorC{8, 9, 10, false, false, true};

// Pulse timing (microseconds) and steps-per-rev per motor
static constexpr uint16_t pulseWidthUs = 20;   // STEP high duration
static constexpr uint16_t pulsePeriodUs = 500; // Total period per step
static constexpr int pulsesPerRev = 3200;      // Microsteps per full revolution