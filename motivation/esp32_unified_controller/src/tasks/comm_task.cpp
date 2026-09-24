#include "LINK/protocol.h"
#include "fw/context.h"
#include "fw/fw_debug.h"
#include "fw/tasks.h"

namespace fw {

namespace {

// Hands one parsed command to whichever task owns it. This is the only place
// that mapping exists.
void dispatch(RobotContext* context, const link::Command& command) {
    switch (command.kind) {
        case link::CmdKind::STOP: {
            // Straight to the flag. A stop queued behind other commands is not
            // a stop, so it never touches a queue.
            const uint32_t stamp = nowMs();
            context->estopStampMs.store(stamp);
            context->estop.set();
            // Zero rates travel the normal servo path, so S halts the camera
            // as well as the wheels without the servo task knowing about e-stop.
            ServoCommand servo;
            servo.stampMs = stamp;
            context->servoMailbox.post(servo);
            break;
        }

        case link::CmdKind::VELOCITY: {
            MotionCommand motion;
            motion.velocity.u = command.a;
            motion.velocity.v = command.b;
            motion.velocity.r = command.c;
            motion.stampMs    = nowMs();
            context->motionMailbox.post(motion);
            break;
        }

        case link::CmdKind::FORWARD:
        case link::CmdKind::TURN:
        case link::CmdKind::RESET: {
            OneShotCommand oneShot;
            oneShot.kind = command.kind;
            oneShot.a    = command.a;
            oneShot.b    = command.b;
            if (!context->oneShotQueue.push(oneShot)) {
                context->errors.bump(link::ErrCode::QUEUE_FULL);
                FW_DLOG("comm: one-shot queue full, command dropped\n");
            }
            if (command.kind == link::CmdKind::RESET) {
                context->servoReset.set();
            }
            break;
        }

        case link::CmdKind::SERVO: {
            ServoCommand servo;
            servo.panRateDegS  = command.a;
            servo.tiltRateDegS = command.b;
            servo.stampMs      = nowMs();
            context->servoMailbox.post(servo);
            break;
        }

        default:
            break;
    }
}

}  // namespace

void commTask(void* param) {
    RobotContext* context = static_cast<RobotContext*>(param);
    if (context == nullptr) {
        return;
    }

    link::LineAssembler assembler;
    char                chunk[cfg::UART_READ_CHUNK];

    for (;;) {
        // The only blocking call in this task, which is why no byte is missed.
        const int count =
            context->uart.read(chunk, cfg::UART_READ_CHUNK, cfg::UART_READ_TIMEOUT_MS);

        for (int index = 0; index < count; ++index) {
            if (!assembler.push(chunk[index])) {
                continue;
            }
            if (assembler.overflowed()) {
                context->errors.bump(link::ErrCode::LINE_OVERFLOW);
                continue;
            }
            if (assembler.length() == 0) {
                continue;   // a bare newline is idle chatter, not an error
            }

            link::Command command;
            if (!link::parseCommand(assembler.line(), &command)) {
                // A known letter with bad arguments is malformed; anything else
                // is a command this firmware does not implement.
                context->errors.bump(link::isCommandChar(assembler.line()[0])
                                         ? link::ErrCode::MALFORMED_LINE
                                         : link::ErrCode::UNKNOWN_COMMAND);
                FW_DLOG("comm: rejected \"%s\"\n", assembler.line());
                continue;
            }
            dispatch(context, command);
        }
    }
}

}  // namespace fw
