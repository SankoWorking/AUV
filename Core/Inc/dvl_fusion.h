#ifndef __DVL_FUSION_H
#define __DVL_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include <stdint.h>

#define DVL_FUSION_TASK_STACK_SIZE_BYTES (512U * 4U)
#define DVL_FUSION_TASK_PRIORITY         osPriorityNormal1

typedef enum
{
  DVL_FUSION_UPDATE_INTEGRATED = 0,
  DVL_FUSION_UPDATE_BASELINE_SET,
  DVL_FUSION_UPDATE_NO_NEW_DVL,
  DVL_FUSION_UPDATE_DVL_READ_FAIL,
  DVL_FUSION_UPDATE_DVL_INVALID,
  DVL_FUSION_UPDATE_IMU_READ_FAIL,
  DVL_FUSION_UPDATE_IMU_INVALID,
  DVL_FUSION_UPDATE_RECOVERY_SKIP,
  DVL_FUSION_UPDATE_FILTER_FAIL,
  DVL_FUSION_UPDATE_SPEED_JUMP,
  DVL_FUSION_UPDATE_DT_INVALID,
  DVL_FUSION_UPDATE_SPEED_DEADBAND
} DVL_FusionUpdate_t;

typedef struct
{
  float x_m;
  float y_m;
  float vn_mps;
  float ve_mps;
  float body_vx_mps;
  float body_vy_mps;
  float yaw_deg;
  uint32_t dvl_frame_count;
  uint32_t dvl_filter_timestamp;
  uint32_t integrated_count;
  uint32_t rejected_count;
  DVL_FusionUpdate_t last_update;
  uint8_t velocity_valid;
} DVL_FusionState_t;

void DVL_Fusion_Reset(float x_m, float y_m);
DVL_FusionUpdate_t DVL_Fusion_Update(void);
void DVL_Fusion_GetState(DVL_FusionState_t *state);
BaseType_t DVL_Fusion_Task_Init(void);
void DVL_Fusion_NotifyFromDvlTask(void);

#ifdef __cplusplus
}
#endif

#endif /* __DVL_FUSION_H */
