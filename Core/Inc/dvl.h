#ifndef __DVL_H
#define __DVL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include <stdint.h>

#define DVL_TASK_STACK_SIZE_BYTES (512U * 4U)
#define DVL_TASK_PRIORITY         osPriorityAboveNormal1
// DVL启动时需要等待连续有效的帧数。
#define DVL_STARTUP_VALID_FRAME_COUNT 5U

typedef struct{
	float raw_vx;
	float raw_vy;
	float raw_vz;
	float raw_ve;
	float vx;
	float vy;
	uint16_t status;
	uint8_t velocity_invalid_count;
	uint16_t filter_failure_count;
	uint32_t frame_count;
	uint32_t timestamp;
	uint32_t filter_timestamp;
}DVL_Data_t;

BaseType_t DVL_Init(void);
BaseType_t DVL_ConfigureStartup(void);
BaseType_t DVL_WaitReady(TickType_t timeout_ticks);
BaseType_t DVL_GetData(DVL_Data_t *data);
BaseType_t DVL_GetDataTimeout(DVL_Data_t *data, TickType_t timeout_ticks);

#ifdef __cplusplus
}
#endif

#endif /* __DVL_H */
