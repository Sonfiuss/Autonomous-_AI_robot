#include "fw/drivers/step_dir_driver.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "fw/config.h"
#include "fw/fw_debug.h"

namespace fw {

namespace {

// The high-speed group, so the servo (low-speed group) can never take a timer
// this driver is using. Note: parts without a high-speed group — ESP32-S3, C3 —
// need this changed to LEDC_LOW_SPEED_MODE and a different timer split.
constexpr ledc_mode_t STEP_MODE = LEDC_HIGH_SPEED_MODE;

// Replaced by the first apply(); LEDC needs some frequency to configure with.
constexpr uint32_t INITIAL_FREQ_HZ = 1000;

ledc_timer_t timerOf(int wheel) {
    return static_cast<ledc_timer_t>(wheel);
}

ledc_channel_t channelOf(int wheel) {
    return static_cast<ledc_channel_t>(wheel);
}

gpio_num_t dirPinOf(int wheel) {
    return static_cast<gpio_num_t>(cfg::DIR_PIN[wheel]);
}

}  // namespace

StepDirDriver::StepDirDriver() : ready_(false) {}

bool StepDirDriver::begin() {
    for (int wheel = 0; wheel < rm::cfg::NUM_WHEELS; ++wheel) {
        // One timer per wheel: three wheels run at three different frequencies.
        ledc_timer_config_t timer = {};
        timer.speed_mode      = STEP_MODE;
        timer.timer_num       = timerOf(wheel);
        timer.duty_resolution = static_cast<ledc_timer_bit_t>(cfg::STEP_DUTY_BITS);
        timer.freq_hz         = INITIAL_FREQ_HZ;
        timer.clk_cfg         = LEDC_AUTO_CLK;
        if (ledc_timer_config(&timer) != ESP_OK) {
            FW_DLOG("steppers: timer %d config failed\n", wheel);
            return false;
        }

        ledc_channel_config_t channel = {};
        channel.gpio_num   = cfg::STEP_PIN[wheel];
        channel.speed_mode = STEP_MODE;
        channel.channel    = channelOf(wheel);
        channel.timer_sel  = timerOf(wheel);
        channel.duty       = 0;
        channel.hpoint     = 0;
        if (ledc_channel_config(&channel) != ESP_OK) {
            FW_DLOG("steppers: channel %d config failed\n", wheel);
            return false;
        }

        gpio_reset_pin(dirPinOf(wheel));
        if (gpio_set_direction(dirPinOf(wheel), GPIO_MODE_OUTPUT) != ESP_OK) {
            FW_DLOG("steppers: DIR pin %d config failed\n", wheel);
            return false;
        }
    }
    ready_ = true;
    return true;
}

void StepDirDriver::apply(int wheel, const rm::StepCommand& command) {
    if (!ready_ || wheel < 0 || wheel >= rm::cfg::NUM_WHEELS) {
        return;
    }
    // XOR with the wiring flag: the level the driver needs, not the one IK names.
    const bool forward = command.forward != cfg::DIR_INVERTED[wheel];
    gpio_set_level(dirPinOf(wheel), forward ? 1 : 0);

    if (command.freqHz < cfg::STEP_MIN_HZ) {
        ledc_set_duty(STEP_MODE, channelOf(wheel), 0);
        ledc_update_duty(STEP_MODE, channelOf(wheel));
        return;
    }
    ledc_set_freq(STEP_MODE, timerOf(wheel), static_cast<uint32_t>(command.freqHz));
    ledc_set_duty(STEP_MODE, channelOf(wheel), cfg::STEP_DUTY_HALF);
    ledc_update_duty(STEP_MODE, channelOf(wheel));
}

void StepDirDriver::stopAll() {
    if (!ready_) {
        return;
    }
    for (int wheel = 0; wheel < rm::cfg::NUM_WHEELS; ++wheel) {
        ledc_set_duty(STEP_MODE, channelOf(wheel), 0);
        ledc_update_duty(STEP_MODE, channelOf(wheel));
    }
}

}  // namespace fw
