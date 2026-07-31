#include "dvl_fusion.h"

#include "dvl.h"
#include "jy901s.h"
#include "startup_task.h"

#include <math.h>
#include <string.h>

#define DVL_FUSION_EVENT_DVL_FILTERED        (1UL << 0)
#define DVL_FUSION_RECOVERY_SKIP_FRAMES       2U
#define DVL_FUSION_MAX_SPEED_JUMP_MM_S        50.0f
#define DVL_FUSION_MIN_DT_MS                  100U
#define DVL_FUSION_MAX_DT_MS                  400U
#define DVL_FUSION_MM_S_TO_M_S                0.001f
#define DVL_FUSION_DEG_TO_RAD                 0.017453292519943295f

static void DVL_Fusion_Task(void *argument);

static DVL_FusionState_t fusionState;
static osThreadId_t fusionTaskHandle;
static float lastVnMps;
static float lastVeMps;
static float lastBodyVxMmS;
static float lastBodyVyMmS;
static uint32_t lastDvlFrameCount;
static uint32_t lastFilterTimestamp;
static uint16_t lastFilterFailureCount;
static uint8_t haveLastVelocity;
static uint8_t recoverySkipRemaining;

static const osThreadAttr_t dvlFusionTaskAttributes = {
  .name = "dvl_fusion",
  .stack_size = DVL_FUSION_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)DVL_FUSION_TASK_PRIORITY,
};

static float DVL_Fusion_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static void DVL_Fusion_SetLastUpdate(DVL_FusionUpdate_t update)
{
  fusionState.last_update = update;
  if ((update != DVL_FUSION_UPDATE_INTEGRATED) &&
      (update != DVL_FUSION_UPDATE_BASELINE_SET) &&
      (update != DVL_FUSION_UPDATE_NO_NEW_DVL))
  {
    fusionState.rejected_count++;
  }
}

static void DVL_Fusion_BodyToNav(float body_vx_mps,
                                 float body_vy_mps,
                                 float yaw_deg,
                                 float *vn_mps,
                                 float *ve_mps)
{
  float yaw_rad = yaw_deg * DVL_FUSION_DEG_TO_RAD;
  float c = cosf(yaw_rad);
  float s = sinf(yaw_rad);

  *vn_mps = (c * body_vx_mps) - (s * body_vy_mps);
  *ve_mps = (s * body_vx_mps) + (c * body_vy_mps);
}

static void DVL_Fusion_UpdateVelocityState(const DVL_Data_t *dvl,
                                           const JY901S_Data_t *imu,
                                           float vn_mps,
                                           float ve_mps)
{
  fusionState.vn_mps = vn_mps;
  fusionState.ve_mps = ve_mps;
  fusionState.body_vx_mps = dvl->vx * DVL_FUSION_MM_S_TO_M_S;
  fusionState.body_vy_mps = dvl->vy * DVL_FUSION_MM_S_TO_M_S;
  fusionState.yaw_deg = imu->angle_deg[2];
  fusionState.dvl_frame_count = dvl->frame_count;
  fusionState.dvl_filter_timestamp = dvl->filter_timestamp;
}

static void DVL_Fusion_SetBaseline(const DVL_Data_t *dvl,
                                   const JY901S_Data_t *imu,
                                   float vn_mps,
                                   float ve_mps)
{
  DVL_Fusion_UpdateVelocityState(dvl, imu, vn_mps, ve_mps);
  lastVnMps = vn_mps;
  lastVeMps = ve_mps;
  lastBodyVxMmS = dvl->vx;
  lastBodyVyMmS = dvl->vy;
  lastFilterTimestamp = dvl->filter_timestamp;
  haveLastVelocity = 1U;
  fusionState.velocity_valid = 1U;
}

void DVL_Fusion_Reset(float x_m, float y_m)
{
  memset(&fusionState, 0, sizeof(fusionState));
  fusionState.x_m = x_m;
  fusionState.y_m = y_m;
  fusionState.last_update = DVL_FUSION_UPDATE_BASELINE_SET;

  lastVnMps = 0.0f;
  lastVeMps = 0.0f;
  lastBodyVxMmS = 0.0f;
  lastBodyVyMmS = 0.0f;
  lastDvlFrameCount = 0U;
  lastFilterTimestamp = 0U;
  lastFilterFailureCount = 0U;
  haveLastVelocity = 0U;
  recoverySkipRemaining = 0U;
}

BaseType_t DVL_Fusion_Task_Init(void)
{
  if (fusionTaskHandle != NULL)
  {
    return pdPASS;
  }

  DVL_Fusion_Reset(0.0f, 0.0f);

  fusionTaskHandle = osThreadNew(DVL_Fusion_Task, NULL, &dvlFusionTaskAttributes);
  if (fusionTaskHandle == NULL)
  {
    return pdFAIL;
  }

  return pdPASS;
}

void DVL_Fusion_NotifyFromDvlTask(void)
{
  if (fusionTaskHandle == NULL)
  {
    return;
  }

  (void)osThreadFlagsSet(fusionTaskHandle, DVL_FUSION_EVENT_DVL_FILTERED);
}

DVL_FusionUpdate_t DVL_Fusion_Update(void)
{
  DVL_Data_t dvl;
  JY901S_Data_t imu;
  float body_vx_mps;
  float body_vy_mps;
  float vn_mps;
  float ve_mps;
  uint32_t dt_ms;
  float dt_s;
  float jump_vx;
  float jump_vy;

  if (DVL_GetData(&dvl) != pdPASS)
  {
    fusionState.velocity_valid = 0U;
    DVL_Fusion_SetLastUpdate(DVL_FUSION_UPDATE_DVL_READ_FAIL);
    return DVL_FUSION_UPDATE_DVL_READ_FAIL;
  }

  if (dvl.frame_count == lastDvlFrameCount)
  {
    DVL_Fusion_SetLastUpdate(DVL_FUSION_UPDATE_NO_NEW_DVL);
    return DVL_FUSION_UPDATE_NO_NEW_DVL;
  }
  lastDvlFrameCount = dvl.frame_count;
  fusionState.dvl_frame_count = dvl.frame_count;

  if ((dvl.status != (uint8_t)'A') ||
      (dvl.raw_vx == 88888.0f) ||
      (dvl.raw_vy == 88888.0f) ||
      (dvl.filter_timestamp == 0U))
  {
    haveLastVelocity = 0U;
    fusionState.velocity_valid = 0U;
    recoverySkipRemaining = DVL_FUSION_RECOVERY_SKIP_FRAMES;
    lastFilterFailureCount = dvl.filter_failure_count;
    DVL_Fusion_SetLastUpdate(DVL_FUSION_UPDATE_DVL_INVALID);
    return DVL_FUSION_UPDATE_DVL_INVALID;
  }

  if (JY901S_GetData(&imu) != pdPASS)
  {
    haveLastVelocity = 0U;
    fusionState.velocity_valid = 0U;
    lastFilterFailureCount = dvl.filter_failure_count;
    DVL_Fusion_SetLastUpdate(DVL_FUSION_UPDATE_IMU_READ_FAIL);
    return DVL_FUSION_UPDATE_IMU_READ_FAIL;
  }

  if (imu.angle_valid == 0U)
  {
    haveLastVelocity = 0U;
    fusionState.velocity_valid = 0U;
    lastFilterFailureCount = dvl.filter_failure_count;
    DVL_Fusion_SetLastUpdate(DVL_FUSION_UPDATE_IMU_INVALID);
    return DVL_FUSION_UPDATE_IMU_INVALID;
  }

  body_vx_mps = dvl.vx * DVL_FUSION_MM_S_TO_M_S;
  body_vy_mps = dvl.vy * DVL_FUSION_MM_S_TO_M_S;
  DVL_Fusion_BodyToNav(body_vx_mps,
                       body_vy_mps,
                       imu.angle_deg[2],
                       &vn_mps,
                       &ve_mps);

  if (dvl.filter_failure_count != lastFilterFailureCount)
  {
    lastFilterFailureCount = dvl.filter_failure_count;
    DVL_Fusion_SetBaseline(&dvl, &imu, vn_mps, ve_mps);
    DVL_Fusion_SetLastUpdate(DVL_FUSION_UPDATE_FILTER_FAIL);
    return DVL_FUSION_UPDATE_FILTER_FAIL;
  }
  lastFilterFailureCount = dvl.filter_failure_count;

  if (recoverySkipRemaining > 0U)
  {
    recoverySkipRemaining--;
    haveLastVelocity = 0U;
    fusionState.velocity_valid = 0U;
    DVL_Fusion_UpdateVelocityState(&dvl, &imu, vn_mps, ve_mps);
    DVL_Fusion_SetLastUpdate(DVL_FUSION_UPDATE_RECOVERY_SKIP);
    return DVL_FUSION_UPDATE_RECOVERY_SKIP;
  }

  if (haveLastVelocity != 0U)
  {
    jump_vx = DVL_Fusion_AbsFloat(dvl.vx - lastBodyVxMmS);
    jump_vy = DVL_Fusion_AbsFloat(dvl.vy - lastBodyVyMmS);
    if ((jump_vx > DVL_FUSION_MAX_SPEED_JUMP_MM_S) ||
        (jump_vy > DVL_FUSION_MAX_SPEED_JUMP_MM_S))
    {
      DVL_Fusion_SetBaseline(&dvl, &imu, vn_mps, ve_mps);
      DVL_Fusion_SetLastUpdate(DVL_FUSION_UPDATE_SPEED_JUMP);
      return DVL_FUSION_UPDATE_SPEED_JUMP;
    }

    dt_ms = dvl.filter_timestamp - lastFilterTimestamp;
    if ((dt_ms < DVL_FUSION_MIN_DT_MS) || (dt_ms > DVL_FUSION_MAX_DT_MS))
    {
      DVL_Fusion_SetBaseline(&dvl, &imu, vn_mps, ve_mps);
      DVL_Fusion_SetLastUpdate(DVL_FUSION_UPDATE_DT_INVALID);
      return DVL_FUSION_UPDATE_DT_INVALID;
    }

    dt_s = (float)dt_ms / 1000.0f;
    fusionState.x_m += 0.5f * (lastVnMps + vn_mps) * dt_s;
    fusionState.y_m += 0.5f * (lastVeMps + ve_mps) * dt_s;
    fusionState.integrated_count++;
    DVL_Fusion_SetBaseline(&dvl, &imu, vn_mps, ve_mps);
    DVL_Fusion_SetLastUpdate(DVL_FUSION_UPDATE_INTEGRATED);
    return DVL_FUSION_UPDATE_INTEGRATED;
  }

  DVL_Fusion_SetBaseline(&dvl, &imu, vn_mps, ve_mps);
  DVL_Fusion_SetLastUpdate(DVL_FUSION_UPDATE_BASELINE_SET);
  return DVL_FUSION_UPDATE_BASELINE_SET;
}

void DVL_Fusion_GetState(DVL_FusionState_t *state)
{
  if (state == NULL)
  {
    return;
  }

  *state = fusionState;
}

static void DVL_Fusion_Task(void *argument)
{
  (void)argument;

  if (Startup_WaitSystemReady(osWaitForever) != pdPASS)
  {
    for (;;)
    {
      osDelay(1000U);
    }
  }

  DVL_Fusion_Reset(0.0f, 0.0f);
  (void)osThreadFlagsClear(DVL_FUSION_EVENT_DVL_FILTERED);

  for (;;)
  {
    uint32_t flags = osThreadFlagsWait(DVL_FUSION_EVENT_DVL_FILTERED,
                                       osFlagsWaitAny,
                                       osWaitForever);

    if ((flags & osFlagsError) == 0U)
    {
      (void)DVL_Fusion_Update();
    }
  }
}
