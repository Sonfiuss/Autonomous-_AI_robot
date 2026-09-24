#include "fw/drivers/servo_driver.h"

#include <cstdint>

#include "driver/ledc.h"
#include "fw/config.h"
#include "fw/fw_debug.h"

namespace fw {

namespace {

// The LOW-speed group: the step driver owns the high-speed timers, and the two
// groups are independent, so neither can disturb the other's frequency.
constexpr ledc_mode_t  SERVO_MODE  = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t SERVO_TIMER = LEDC_TIMER_0;
constexpr int          PAN_CHANNEL  = 0;
constexpr int          TILT_CHANNEL = 1;
constexpr int          SERVO_PINS[2] = {cfg::SERVO_PAN_PIN, cfg::SERVO_TILT_PIN};

}  // namespace

ServoDriver::ServoDriver() : ready_(false) {}

bool ServoDriver::begin() {
    ledc_timer_config_t timer = {};
    timer.speed_mode      = SERVO_MODE;
    timer.timer_num       = SERVO_TIMER;
    timer.duty_resolution = static_cast<ledc_timer_bit_t>(cfg::SERVO_DUTY_BITS);
    timer.freq_hz         = cfg::SERVO_PWM_HZ;
    timer.clk_cfg         = LEDC_AUTO_CLK;
    if (ledc_timer_config(&timer) != ESP_OK) {
        FW_DLOG("servo: timer config failed\n");
        return false;
    }

    for (int index = 0; index < 2; ++index) {
        ledc_channel_config_t channel = {};
        channel.gpio_num   = SERVO_PINS[index];
        channel.speed_mode = SERVO_MODE;
        channel.channel    = static_cast<ledc_channel_t>(index);
        channel.timer_sel  = SERVO_TIMER;
        channel.duty       = 0;
        channel.hpoint     = 0;
        if (ledc_channel_config(&channel) != ESP_OK) {
            FW_DLOG("servo: channel %d config failed\n", index);
            return false;
        }
    }
    ready_ = true;
    return true;
}

void ServoDriver::write(int channel, float angleDeg) {
    float ratio = (angleDeg - cfg::SERVO_MIN_DEG) / cfg::SERVO_SPAN_DEG;
    if (ratio < 0.0f) {
        ratio = 0.0f;
    }
    if (ratio > 1.0f) {
        ratio = 1.0f;
    }
    const float pulseUs = cfg::SERVO_MIN_PULSE_US +
                          ratio * (cfg::SERVO_MAX_PULSE_US - cfg::SERVO_MIN_PULSE_US);
    const uint32_t fullScale = (1u << cfg::SERVO_DUTY_BITS) - 1u;
    const uint32_t duty =
        static_cast<uint32_t>(pulseUs * static_cast<float>(fullScale) / cfg::SERVO_PERIOD_US);

    const ledc_channel_t target = static_cast<ledc_channel_t>(channel);
    ledc_set_duty(SERVO_MODE, target, duty);
    ledc_update_duty(SERVO_MODE, target);
}

void ServoDriver::apply(float panDeg, float tiltDeg) {
    if (!ready_) {
        return;
    }
    write(PAN_CHANNEL, panDeg);
    write(TILT_CHANNEL, tiltDeg);
}

}  // namespace fw
