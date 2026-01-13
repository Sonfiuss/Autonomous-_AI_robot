#include <AccelStepper.h>
#include <Arduino.h>

#include "config/config.h"

// Motion settings
static constexpr unsigned long serialBaud = 115200;
static constexpr uint8_t ledPin = LED_BUILTIN;

// Speed limits (rev/s)
static constexpr float minRevPerSec = 0.3f;
static constexpr float maxRevPerSec = 2.0f;
static constexpr float revStep = 0.1f; // increment when pressing W/S

// Derived: steps/s
static constexpr float maxStepsPerSec = maxRevPerSec * pulsesPerRev;

AccelStepper stepper(AccelStepper::DRIVER, motorA.step, motorA.dir);

static void configureStepper(AccelStepper &s, const StepperPins &pins) {
  s.setEnablePin(pins.ena);
  s.setPinsInverted(pins.dirActiveLow, pins.stepActiveLow, pins.enaActiveLow);
  s.setMinPulseWidth(pulseWidthUs);
  s.setMaxSpeed(maxStepsPerSec);
  s.setAcceleration(maxStepsPerSec * 2);
  s.enableOutputs();
}

static void setEnabled(bool enabled) {
  const bool level = enabled ? LOW : HIGH;
  digitalWrite(motorA.ena, motorA.enaActiveLow ? level : !level);
}

void setup() {
  Serial.begin(serialBaud);

  pinMode(ledPin, OUTPUT);
  pinMode(motorA.ena, OUTPUT);

  configureStepper(stepper, motorA);
  setEnabled(true);

  // Start at minimum speed forward
  stepper.setSpeed(minRevPerSec * pulsesPerRev);
  Serial.println(F("Press 'W' to speed up, 'S' to slow down."));
}

void loop() {
  while (Serial.available() > 0) {
    const char c = tolower(Serial.read());
    float currentRevPerSec = stepper.speed() / pulsesPerRev;

    if (c == 'w') {
      currentRevPerSec += revStep;
    } else if (c == 's') {
      currentRevPerSec -= revStep;
    }

    currentRevPerSec = constrain(currentRevPerSec, minRevPerSec, maxRevPerSec);
    stepper.setSpeed(currentRevPerSec * pulsesPerRev);

    Serial.print(F("Speed: "));
    Serial.print(currentRevPerSec, 2);
    Serial.println(F(" rev/s"));

    digitalWrite(ledPin, (c == 'w') ? HIGH : LOW);
  }

  stepper.runSpeed();
}