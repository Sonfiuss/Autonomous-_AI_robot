#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fw/context.h"
#include "fw/fw_debug.h"
#include "fw/motion_state.h"
#include "fw/tasks.h"

namespace fw {

namespace {

// The newest streamed velocity, or zero when there is none, when it has gone
// stale, or when it predates the stop currently latched.
//
// Zero-on-stale is a safety property, not an optimisation: without it the robot
// keeps driving on the last command after the cable is pulled, and resumes a
// long-dead command the instant an F/T leg ends.
rm::BodyVel freshVelocity(RobotContext* context, MotionState* motion) {
    MotionCommand command;
    if (!context->motionMailbox.peek(&command)) {
        return rm::BodyVel();
    }
    // Unsigned subtraction, so this stays correct across the 49-day wrap.
    if (nowMs() - command.stampMs > cfg::MOTION_CMD_TIMEOUT_MS) {
        return rm::BodyVel();
    }
    if (command.stampMs <= context->estopStampMs.load()) {
        return rm::BodyVel();
    }
    // Issued after the stop, so it is new intent and releases the latch.
    motion->clearStop();
    return command.velocity;
}

void applyOneShot(MotionState* motion, const OneShotCommand& command) {
    switch (command.kind) {
        case link::CmdKind::FORWARD:
            motion->beginForward(command.a, command.b);
            break;
        case link::CmdKind::TURN:
            motion->beginTurn(command.a, command.b);
            break;
        case link::CmdKind::RESET:
            motion->resetOdometry();
            break;
        default:
            break;
    }
}

}  // namespace

void motionTask(void* param) {
    RobotContext* context = static_cast<RobotContext*>(param);
    if (context == nullptr) {
        return;
    }

    MotionState motion;
    if (!motion.valid()) {
        // A singular wheel layout means every computed speed is meaningless.
        FW_DLOG("motion: singular wheel layout, refusing to drive\n");
        context->steppers.stopAll();
        vTaskDelete(nullptr);
        return;
    }

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(cfg::TICK_MS));

        // First thing in the tick, ahead of every other source of commands.
        if (context->estop.testAndClear()) {
            motion.requestStop();
        }

        OneShotCommand oneShot;
        if (!motion.busy() && context->oneShotQueue.pop(&oneShot)) {
            applyOneShot(&motion, oneShot);
        }

        MotionTick tick;
        motion.tick(freshVelocity(context, &motion), cfg::TICK_S, &tick);

        for (int wheel = 0; wheel < rm::cfg::NUM_WHEELS; ++wheel) {
            context->steppers.apply(wheel, tick.steps[wheel]);
        }

        PoseSample sample;
        sample.pose = tick.pose;
        context->poseSnapshot.set(sample);

        if (tick.legFinished && context->statusTask != nullptr) {
            xTaskNotifyGive(context->statusTask);
        }
    }
}

}  // namespace fw
