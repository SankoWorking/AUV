#ifndef __STARTUP_TASK_H
#define __STARTUP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os2.h"

#define STARTUP_TASK_STACK_SIZE_BYTES (512U * 4U)
#define STARTUP_TASK_PRIORITY         osPriorityAboveNormal

BaseType_t Startup_Task_Init(void);
BaseType_t Startup_WaitSystemReady(TickType_t timeout_ticks);

#ifdef __cplusplus
}
#endif

#endif /* __STARTUP_TASK_H */
