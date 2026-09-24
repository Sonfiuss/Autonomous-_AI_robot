#include <cstdint>

#include "LINK/protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fw/context.h"
#include "fw/tasks.h"

namespace fw {

namespace {

// One reusable line buffer: the longest wire line, plus its '\n' and NUL.
constexpr int LINE_CAP  = link::cfg::MAX_LINE_LEN + 2;
constexpr int ERR_SLOTS = static_cast<int>(link::ErrCode::COUNT);

// Writes one formatted line, counting the loss when the TX buffer cannot take
// it. Dropping a periodic sample is harmless — another follows in 20 ms — but
// silently dropping it would not be, hence the counter.
void sendLine(RobotContext* context, const char* line, int length) {
    if (length <= 0) {
        return;
    }
    if (!context->uart.writeLine(line, length)) {
        context->errors.bump(link::ErrCode::TX_DROPPED);
    }
}

}  // namespace

void statusTask(void* param) {
    RobotContext* context = static_cast<RobotContext*>(param);
    if (context == nullptr) {
        return;
    }

    char     line[LINE_CAP];
    uint32_t reported[ERR_SLOTS] = {0};
    uint32_t ticks               = 0;

    sendLine(context, line, link::formatReady(line, LINE_CAP));

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(cfg::TICK_MS);
    for (;;) {
        // Wait for a motion-complete notification, but never past the next
        // periodic deadline: K goes out the moment it happens, while P and O
        // stay on their cadence.
        const TickType_t now  = xTaskGetTickCount();
        const TickType_t wait = (deadline > now) ? (deadline - now) : 0;
        if (ulTaskNotifyTake(pdTRUE, wait) > 0) {
            sendLine(context, line, link::formatAck(line, LINE_CAP));
            continue;
        }

        deadline += pdMS_TO_TICKS(cfg::TICK_MS);
        ++ticks;

        const ServoSample servo = context->servoSnapshot.get();
        sendLine(context, line, link::formatServo(servo.panDeg, servo.tiltDeg, line, LINE_CAP));

        if (ticks % cfg::ODOM_EVERY_N_TICKS == 0) {
            const PoseSample sample = context->poseSnapshot.get();
            sendLine(context, line,
                     link::formatOdom(sample.pose.x, sample.pose.y, sample.pose.theta, line,
                                      LINE_CAP));
        }

        if (ticks % cfg::FAULT_EVERY_N_TICKS == 0) {
            for (int code = 1; code < ERR_SLOTS; ++code) {
                const link::ErrCode which = static_cast<link::ErrCode>(code);
                const uint32_t      count = context->errors.get(which);
                if (count == reported[code]) {
                    continue;   // nothing moved: stay silent
                }
                reported[code] = count;
                sendLine(context, line, link::formatFault(which, count, line, LINE_CAP));
            }
        }
    }
}

}  // namespace fw
