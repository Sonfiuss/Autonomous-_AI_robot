#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fw/context.h"
#include "fw/tasks.h"

namespace fw {

namespace {

constexpr float SERVO_DT_S = static_cast<float>(cfg::SERVO_TICK_MS) / 1000.0f;

// Holds the angle inside the mechanical range. Returning whether it had to act
// is what keeps the integrator from winding far past the limit and then taking
// seconds to unwind.
bool clampAngle(float* angle) {
    if (*angle < cfg::SERVO_MIN_DEG) {
        *angle = cfg::SERVO_MIN_DEG;
        return true;
    }
    if (*angle > cfg::SERVO_MAX_DEG) {
        *angle = cfg::SERVO_MAX_DEG;
        return true;
    }
    return false;
}

}  // namespace

void peripheralTask(void* param) {
    RobotContext* context = static_cast<RobotContext*>(param);
    if (context == nullptr) {
        return;
    }

    float panDeg     = 0.0f;
    float tiltDeg    = 0.0f;
    bool  wasClamped = false;
    context->servos.apply(panDeg, tiltDeg);

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(cfg::SERVO_TICK_MS));

        if (context->servoReset.testAndClear()) {
            panDeg  = 0.0f;
            tiltDeg = 0.0f;
        }

        // A command that stopped arriving means a dead link or an e-stop; both
        // must leave the servo still rather than drifting to its end stop.
        float        panRate  = 0.0f;
        float        tiltRate = 0.0f;
        ServoCommand command;
        if (context->servoMailbox.peek(&command) &&
            nowMs() - command.stampMs <= cfg::SERVO_CMD_TIMEOUT_MS) {
            panRate  = command.panRateDegS;
            tiltRate = command.tiltRateDegS;
        }

        panDeg += panRate * SERVO_DT_S;
        tiltDeg += tiltRate * SERVO_DT_S;

        const bool panClamped  = clampAngle(&panDeg);
        const bool tiltClamped = clampAngle(&tiltDeg);
        const bool clampedNow  = panClamped || tiltClamped;
        // Counted on the edge, not every tick: parking against a limit is a
        // normal state, and a level-triggered count would emit an E every second
        // for as long as the operator held the stick over.
        if (clampedNow && !wasClamped) {
            context->errors.bump(link::ErrCode::SERVO_CLAMPED);
        }
        wasClamped = clampedNow;

        context->servos.apply(panDeg, tiltDeg);

        ServoSample sample;
        sample.panDeg  = panDeg;
        sample.tiltDeg = tiltDeg;
        context->servoSnapshot.set(sample);
    }
}

}  // namespace fw
