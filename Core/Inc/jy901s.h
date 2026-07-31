#ifndef __JY901S_H
#define __JY901S_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include <stdint.h>

#define JY901S_TASK_STACK_SIZE_BYTES (512U * 4U)
#define JY901S_TASK_PRIORITY         osPriorityAboveNormal

typedef struct
{
  float acc_g[3];
  float gyro_dps[3];
  float angle_deg[3];
  int16_t temperature_raw;
  uint32_t timestamp_ms;
  uint32_t frame_count;
  uint32_t checksum_error_count;
  uint32_t rx_drop_count;
  uint32_t uart_error_count;
  uint8_t acc_valid;
  uint8_t gyro_valid;
  uint8_t angle_valid;
} JY901S_Data_t;

BaseType_t JY901S_Init(void);
BaseType_t JY901S_GetData(JY901S_Data_t *data);
BaseType_t JY901S_GetDataTimeout(JY901S_Data_t *data, TickType_t timeout_ticks);

#ifdef __cplusplus
}
#endif

#endif /* __JY901S_H */
