// The four FreeRTOS entry points. Each takes a RobotContext* as its parameter
// and never returns. Who owns what is documented on RobotContext.
#ifndef FW_TASKS_H
#define FW_TASKS_H

namespace fw {

void commTask(void* param);
void motionTask(void* param);
void peripheralTask(void* param);
void statusTask(void* param);

}  // namespace fw

#endif  // FW_TASKS_H
