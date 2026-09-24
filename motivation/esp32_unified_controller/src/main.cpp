// Boot: bring the hardware up, then start the four tasks and get out of the way.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fw/context.h"
#include "fw/fw_debug.h"
#include "fw/tasks.h"

namespace {

// Static, not heap: this firmware allocates nothing after boot.
fw::RobotContext g_context;

}  // namespace

extern "C" void app_main(void) {
    if (!g_context.create()) {
        // No task starts on a failed init: a half-configured driver must never
        // be handed a wheel to turn.
        FW_DLOG("boot: initialisation failed, no task started\n");
        return;
    }

    // Status first, because Motion notifies its handle and the handle has to
    // exist before Motion's first tick can fire.
    xTaskCreatePinnedToCore(fw::statusTask, "status", fw::cfg::STATUS_STACK_BYTES, &g_context,
                            fw::cfg::STATUS_PRIORITY, &g_context.statusTask,
                            fw::cfg::SERVICE_CORE);
    xTaskCreatePinnedToCore(fw::commTask, "comm", fw::cfg::COMM_STACK_BYTES, &g_context,
                            fw::cfg::COMM_PRIORITY, nullptr, fw::cfg::SERVICE_CORE);
    xTaskCreatePinnedToCore(fw::peripheralTask, "servo", fw::cfg::PERIPHERAL_STACK_BYTES,
                            &g_context, fw::cfg::PERIPHERAL_PRIORITY, nullptr,
                            fw::cfg::SERVICE_CORE);
    // Motion last and alone on its core: nothing else may jitter the step timing.
    xTaskCreatePinnedToCore(fw::motionTask, "motion", fw::cfg::MOTION_STACK_BYTES, &g_context,
                            fw::cfg::MOTION_PRIORITY, nullptr, fw::cfg::MOTION_CORE);
}
