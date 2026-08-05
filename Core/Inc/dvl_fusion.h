#ifndef __DVL_FUSION_H
#define __DVL_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "dvl.h"
#include <stdint.h>

#define DVL_FUSION_TASK_STACK_SIZE_BYTES (512U * 4U)
#define DVL_FUSION_TASK_PRIORITY         osPriorityNormal1

typedef struct
{
  float x_m;
  float y_m;
  float vn_mps;
  float ve_mps;
  float yaw_deg;
  float dvl_vx_mm_s;
  float dvl_vy_mm_s;
  uint32_t dvl_frame_count;
	uint32_t invalid_velocity_count;
	
  uint32_t imu_invalid_count;
  uint32_t imu_timeout_count;
  uint32_t nav_timestamp_ms;
  uint32_t integrated_count;
  uint32_t filter_failure_count;
	uint32_t filter_timeout_count;
} Nav_State_t;

void DVL_Fusion_Reset(float x_m, float y_m);
void DVL_Fusion_GetState(Nav_State_t *state);
BaseType_t DVL_Fusion_Task_Init(void);
BaseType_t DVL_Fusion_SubmitDvlData(const DVL_Data_t *dvl_snapshot);

#ifdef __cplusplus
}
#endif

#endif /* __DVL_FUSION_H */
