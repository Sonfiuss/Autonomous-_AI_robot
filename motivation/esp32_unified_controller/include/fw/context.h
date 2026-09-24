// Robot_Context — every piece of state the four tasks share, in one object
// created once by main() and passed to each task by pointer. Grouping it here
// rather than scattering globals is what makes the ownership table in the task
// file checkable: each field below has exactly one writer.
#ifndef FW_CONTEXT_H
#define FW_CONTEXT_H

#include <atomic>
#include <cstdint>

#include "LINK/protocol.h"
#include "RM/types.h"
#include "fw/config.h"
#include "fw/drivers/servo_driver.h"
#include "fw/drivers/step_dir_driver.h"
#include "fw/drivers/uart_link.h"
#include "fw/rt_port.h"

#ifndef FW_HOST_BUILD
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

namespace fw {

// Mailbox payload: the newest streamed velocity, with the moment it arrived.
// The stamp is what lets Motion ignore a command that is stale, or one that was
// already in flight when a stop was requested.
struct MotionCommand {
    rm::BodyVel velocity;
    uint32_t    stampMs = 0;
};

// FIFO payload: a command that must run to completion (F, T) or take effect
// exactly once (R).
struct OneShotCommand {
    link::CmdKind kind = link::CmdKind::NONE;
    float         a    = 0.0f;
    float         b    = 0.0f;
};

// Mailbox payload: the newest servo rate command.
struct ServoCommand {
    float    panRateDegS  = 0.0f;
    float    tiltRateDegS = 0.0f;
    uint32_t stampMs      = 0;
};

struct PoseSample {
    rm::Pose pose;
};

struct ServoSample {
    float panDeg  = 0.0f;
    float tiltDeg = 0.0f;
};

struct RobotContext {
    // Comm writes, Motion reads.
    Mailbox<MotionCommand>                        motionMailbox;
    Queue<OneShotCommand, cfg::ONE_SHOT_QUEUE_LEN> oneShotQueue;
    Flag                                          estop;
    // The moment the latest stop was requested. Motion compares a velocity's
    // stamp against this so a command from before the stop can never restart
    // the robot.
    std::atomic<uint32_t> estopStampMs;

    // Comm writes, Peripheral reads.
    Mailbox<ServoCommand> servoMailbox;
    Flag                  servoReset;

    // Owner writes, Status reads.
    Snapshot<PoseSample>  poseSnapshot;
    Snapshot<ServoSample> servoSnapshot;

    // Anyone bumps, Status reports.
    link::ErrorCounters errors;

    UartLink      uart;
    StepDirDriver steppers;
    ServoDriver   servos;

#ifndef FW_HOST_BUILD
    // Motion notifies this handle when a leg finishes, so `K` goes out at once
    // instead of waiting for the next periodic slot.
    TaskHandle_t statusTask = nullptr;
#endif

    RobotContext();

    // Creates every queue and brings up every driver. False if anything failed,
    // in which case no task may be started.
    bool create();
};

}  // namespace fw

#endif  // FW_CONTEXT_H
