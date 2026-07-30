#ifndef __DVL_H
#define __DVL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include <stdint.h>

#define DVL_TASK_STACK_SIZE_BYTES (512U * 4U)
#define DVL_TASK_PRIORITY         osPriorityNormal

typedef struct
{
  int32_t velocity_mm_s[3];
  int32_t velocity_error_mm_s;
  uint32_t timestamp_ms;
  uint32_t frame_count;
  uint32_t parse_error_count;
  uint32_t invalid_velocity_count;
  uint32_t rx_drop_count;
  uint32_t uart_error_count;
  uint8_t velocity_valid;
} DVL_Data_t;

BaseType_t DVL_Init(void);
BaseType_t DVL_GetData(DVL_Data_t *data);
BaseType_t DVL_GetDataTimeout(DVL_Data_t *data, TickType_t timeout_ticks);

#ifdef __cplusplus
}
#endif

#endif /* __DVL_H */
