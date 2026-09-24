// Servo_Driver — pan/tilt PWM on a LEDC LOW-speed timer.
//
// The low-speed group is deliberate: the step driver owns the high-speed
// timers, so the two cannot contend for the same peripheral. Callers pass an
// angle already inside the mechanical limits; the Peripheral task owns that
// clamp because it owns the integrator that would otherwise wind up.
#ifndef FW_SERVO_DRIVER_H
#define FW_SERVO_DRIVER_H

namespace fw {

class ServoDriver {
public:
    ServoDriver();

    bool begin();

    // Drives both servos to the given angles in degrees.
    void apply(float panDeg, float tiltDeg);

private:
    void write(int channel, float angleDeg);

    bool ready_;
};

}  // namespace fw

#endif  // FW_SERVO_DRIVER_H
