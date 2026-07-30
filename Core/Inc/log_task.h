#ifndef __LOG_TASK_H
#define __LOG_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include <stddef.h>
#include <stdint.h>

#define LOG_BUFFER_SIZE_BYTES          4096U
#define LOG_MAX_MESSAGE_LENGTH         256U
#define LOG_TASK_STACK_SIZE_BYTES      (512U * 4U)
#define LOG_TASK_PRIORITY              osPriorityLow
#define LOG_SEND_TIMEOUT_TICKS         0U
#define LOG_MUTEX_TIMEOUT_TICKS        0U

BaseType_t Log_Task_Init(void);
int Log_printf(const char *format, ...);
int Log_write(const char *data, size_t length);
uint32_t Log_GetDroppedCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __LOG_TASK_H */
