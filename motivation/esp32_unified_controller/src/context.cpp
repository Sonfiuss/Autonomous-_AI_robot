#include "fw/context.h"

#include "fw/fw_debug.h"

namespace fw {

RobotContext::RobotContext() : estopStampMs(0) {}

bool RobotContext::create() {
    if (!motionMailbox.create() || !oneShotQueue.create() || !servoMailbox.create() ||
        !poseSnapshot.create() || !servoSnapshot.create()) {
        FW_DLOG("context: queue creation failed\n");
        return false;
    }
    if (!uart.begin()) {
        return false;
    }
    if (!steppers.begin()) {
        // A partly configured driver could leave a channel running, so silence
        // every one of them before giving up.
        steppers.stopAll();
        return false;
    }
    if (!servos.begin()) {
        steppers.stopAll();
        return false;
    }
    return true;
}

}  // namespace fw
