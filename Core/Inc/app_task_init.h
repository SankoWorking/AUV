#ifndef __APP_TASK_INIT_H
#define __APP_TASK_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"


void App_Task_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_INIT_H */
