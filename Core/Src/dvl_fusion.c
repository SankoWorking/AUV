#include "dvl_fusion.h"

#include "dvl.h"
#include "jy901s.h"
#include "queue.h"
#include "semphr.h"
#include "startup_task.h"

#include <math.h>
#include <string.h>

#define DVL_FUSION_DVL_QUEUE_LENGTH           1U
#define DVL_FUSION_MIN_DT_MS                  100U
#define DVL_FUSION_MAX_DT_MS                  400U
#define DVL_FUSION_MM_S_TO_M_S                0.001f
#define DVL_FUSION_DEG_TO_RAD                 0.017453292519943295f
#define DVL_FUSION_SPEED_DEADBAND_MM_S        10.0f
#define DVL_FUSION_DVL_OFFSET_X_M             0.15f
#define DVL_FUSION_DVL_OFFSET_Y_M             0.20f
#define DVL_FUSION_IMU_SYNC_THRESHOLD_MS      200U
#define DVL_FUSION_LOST_FRAME_THRESHOLD       3U
#define DVL_FUSION_REACQUIRE_VALID_COUNT      3U
#define DVL_FUSION_BASE_NOISE_M_S             0.03f
#define DVL_FUSION_MAX_ACCEL_M_S2             0.80f
#define DVL_FUSION_TURN_MARGIN_M_S            0.02f
#define DVL_FUSION_HARD_JUMP_M_S              1.50f
#define DVL_FUSION_MOTION_REJECT_LIMIT        3U

/**
 * @brief  融合任务在计算时储存中间数据时用到的结构体。
 * 
 * invalid_velocity_count 总计出现DVL失锁的次数
 * consecutive_invalid 是否出现连续失锁
 * consecutive_invalid_count 出现连续失锁的次数统计
 * 
 */
typedef struct
{
  uint32_t invalid_velocity_count;
  uint8_t consecutive_valid_count;
  float last_filter_vx;
  float last_filter_vy;
  uint32_t filter_timestamp_ms;
  uint8_t consecutive_invalid; 
  uint32_t consecutive_invalid_count;
	uint32_t processed_dvl_frame_count;
} DVL_FusionCalcState_t;

static void DVL_Fusion_Task(void *argument);

static DVL_FusionState_t fusionState;
static DVL_FusionCalcState_t fusionCalcState;
static SemaphoreHandle_t fusionStateMutex;
static osThreadId_t fusionTaskHandle;
static QueueHandle_t fusionDvlQueue;
static float lastVnMps;
static float lastVeMps;
static uint8_t consecutiveMotionRejects;
static uint8_t haveLastVelocity;

static const osThreadAttr_t dvlFusionTaskAttributes = {
  .name = "dvl_fusion",
  .stack_size = DVL_FUSION_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)DVL_FUSION_TASK_PRIORITY,
};

static BaseType_t DVL_Fusion_LockState(TickType_t timeout_ticks)
{
  if (fusionStateMutex == NULL)
  {
    return pdFAIL;
  }

  return xSemaphoreTake(fusionStateMutex, timeout_ticks);
}

static void DVL_Fusion_UnlockState(void)
{
  if (fusionStateMutex != NULL)
  {
    xSemaphoreGive(fusionStateMutex);
  }
}

static void DVL_Fusion_PublishCounters(uint32_t dvl_frame_count)
{
  if (DVL_Fusion_LockState(portMAX_DELAY) == pdTRUE)
  {
    fusionState.dvl_frame_count = dvl_frame_count;
    fusionState.invalid_velocity_count = fusionCalcState.invalid_velocity_count;
    fusionState.consecutive_invalid_count = fusionCalcState.consecutive_invalid_count;
    DVL_Fusion_UnlockState();
  }
}

static void DVL_Fusion_IncrementImuInvalid(void)
{
  if (DVL_Fusion_LockState(portMAX_DELAY) == pdTRUE)
  {
    fusionState.imu_invalid_count++;
    DVL_Fusion_UnlockState();
  }
}

static void DVL_Fusion_IncrementImuTimeout(void)
{
  if (DVL_Fusion_LockState(portMAX_DELAY) == pdTRUE)
  {
    fusionState.imu_timeout_count++;
    DVL_Fusion_UnlockState();
  }
}

static void DVL_Fusion_IncrementFilterFailure(void)
{
  if (DVL_Fusion_LockState(portMAX_DELAY) == pdTRUE)
  {
    fusionState.filter_failure_count++;
    DVL_Fusion_UnlockState();
  }
}

static void DVL_Fusion_PublishKinematics(const DVL_Data_t *dvl,
                                         const JY901S_Data_t *imu,
                                         float body_vx_mps,
                                         float body_vy_mps,
                                         float vn_mps,
                                         float ve_mps,
                                         uint8_t integrate,
                                         float dt_s)
{
  if (DVL_Fusion_LockState(portMAX_DELAY) == pdTRUE)
  {
    if (integrate != 0U)
    {
      fusionState.x_m += 0.5f * (lastVnMps + vn_mps) * dt_s;
      fusionState.y_m += 0.5f * (lastVeMps + ve_mps) * dt_s;
      fusionState.integrated_count++;
    }

    fusionState.vn_mps = vn_mps;
    fusionState.ve_mps = ve_mps;
    fusionState.body_vx_mps = body_vx_mps;
    fusionState.body_vy_mps = body_vy_mps;
    fusionState.yaw_deg = imu->angle_deg[2];
    fusionState.dvl_frame_count = dvl->frame_count;
    fusionState.dvl_filter_timestamp = fusionCalcState.filter_timestamp_ms;
    fusionState.invalid_velocity_count = fusionCalcState.invalid_velocity_count;
    fusionState.consecutive_invalid_count = fusionCalcState.consecutive_invalid_count;
    DVL_Fusion_UnlockState();
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

static void DVL_Fusion_CompensateLeverArm(float dvl_vx_mps,
                                          float dvl_vy_mps,
                                          float yaw_rate_dps,
                                          float *body_vx_mps,
                                          float *body_vy_mps)
{
  float yaw_rate_rad_s = yaw_rate_dps * DVL_FUSION_DEG_TO_RAD;

  *body_vx_mps = dvl_vx_mps + (yaw_rate_rad_s * DVL_FUSION_DVL_OFFSET_Y_M);
  *body_vy_mps = dvl_vy_mps - (yaw_rate_rad_s * DVL_FUSION_DVL_OFFSET_X_M);
}

static uint8_t DVL_Fusion_IsVelocityBelowDeadband(float body_vx_mps,
                                                  float body_vy_mps)
{
  float threshold_mps = DVL_FUSION_SPEED_DEADBAND_MM_S * DVL_FUSION_MM_S_TO_M_S;
  float speed_sq = (body_vx_mps * body_vx_mps) + (body_vy_mps * body_vy_mps);

  return (speed_sq < (threshold_mps * threshold_mps)) ? 1U : 0U;
}

static uint32_t DVL_Fusion_TickAbsDiff(uint32_t a, uint32_t b)
{
  int32_t diff = (int32_t)(a - b);

  return (diff < 0) ? (uint32_t)(-diff) : (uint32_t)diff;
}

static uint8_t DVL_Fusion_IsDvlVelocityValid(const DVL_Data_t *dvl)
{
  return ((dvl->status == (uint8_t)'A') &&
          (dvl->raw_vx != 88888.0f) &&
          (dvl->raw_vy != 88888.0f)) ? 1U : 0U;
}
/**
 * @brief  出现DVL失锁后,递增失锁总数与连续失锁计数。连续失锁3帧后，进入连续失锁状态。
 *				 后续会触发连续失锁恢复。同时清零积分基准，清零滤波基准。最后更新FusionState
 *				 相关计数器。
 *
 * @param dvl dvl数据快照
 */
static void DVL_Fusion_HandleDvlLost(const DVL_Data_t *dvl)
{
  fusionCalcState.consecutive_invalid_count++;
	fusionCalcState.invalid_velocity_count++;

  fusionCalcState.consecutive_valid_count = 0U;

  if ((fusionCalcState.consecutive_invalid_count >= DVL_FUSION_LOST_FRAME_THRESHOLD) &&
      (fusionCalcState.consecutive_invalid == 0U))
  {
    fusionCalcState.consecutive_invalid = 1U;
    fusionCalcState.consecutive_invalid_count++;
		haveLastVelocity = 0U;
		fusionCalcState.filter_timestamp_ms = 0U;
		consecutiveMotionRejects = 0U;
  }

  DVL_Fusion_PublishCounters(dvl->frame_count);
}

static BaseType_t DVL_Fusion_FilterMotion(float *body_vx_mps,
                                          float *body_vy_mps,
                                          float yaw_rate_dps,
                                          float dt_s)
{
  float yaw_rate_rad_s = fabsf(yaw_rate_dps * DVL_FUSION_DEG_TO_RAD);
  float limit_x_mps = DVL_FUSION_BASE_NOISE_M_S +
                      (DVL_FUSION_MAX_ACCEL_M_S2 * dt_s) +
                      (yaw_rate_rad_s * DVL_FUSION_DVL_OFFSET_Y_M) +
                      DVL_FUSION_TURN_MARGIN_M_S;
  float limit_y_mps = DVL_FUSION_BASE_NOISE_M_S +
                      (DVL_FUSION_MAX_ACCEL_M_S2 * dt_s) +
                      (yaw_rate_rad_s * DVL_FUSION_DVL_OFFSET_X_M) +
                      DVL_FUSION_TURN_MARGIN_M_S;
  float delta_x_mps = *body_vx_mps - fusionCalcState.last_filter_vx;
  float delta_y_mps = *body_vy_mps - fusionCalcState.last_filter_vy;

  if ((fabsf(delta_x_mps) > DVL_FUSION_HARD_JUMP_M_S) ||
      (fabsf(delta_y_mps) > DVL_FUSION_HARD_JUMP_M_S))
  {
    if (consecutiveMotionRejects < 255U)
    {
      consecutiveMotionRejects++;
    }
    return pdFAIL;
  }

  if (delta_x_mps > limit_x_mps)
  {
    *body_vx_mps = fusionCalcState.last_filter_vx + limit_x_mps;
  }
  else if (delta_x_mps < -limit_x_mps)
  {
    *body_vx_mps = fusionCalcState.last_filter_vx - limit_x_mps;
  }

  if (delta_y_mps > limit_y_mps)
  {
    *body_vy_mps = fusionCalcState.last_filter_vy + limit_y_mps;
  }
  else if (delta_y_mps < -limit_y_mps)
  {
    *body_vy_mps = fusionCalcState.last_filter_vy - limit_y_mps;
  }

  consecutiveMotionRejects = 0U;
  return pdPASS;
}

static BaseType_t DVL_Fusion_ReadValidImu(JY901S_Data_t *imu,
                                          uint32_t dvl_timestamp_ms)
{
  if (JY901S_GetData(imu) != pdPASS)
  {
    DVL_Fusion_IncrementImuInvalid();
    return pdFAIL;
  }

  if ((imu->angle_valid == 0U) || (imu->gyro_valid == 0U))
  {
    DVL_Fusion_IncrementImuInvalid();
    return pdFAIL;
  }

  if (DVL_Fusion_TickAbsDiff(dvl_timestamp_ms, imu->timestamp_ms) >
      DVL_FUSION_IMU_SYNC_THRESHOLD_MS)
  {
    DVL_Fusion_IncrementImuTimeout();
    return pdFAIL;
  }

  return pdPASS;
}

static void DVL_Fusion_SetBaseline(const DVL_Data_t *dvl,
                                   const JY901S_Data_t *imu,
                                   float body_vx_mps,
                                   float body_vy_mps,
                                   float vn_mps,
                                   float ve_mps)
{
  fusionCalcState.last_filter_vx = body_vx_mps;
  fusionCalcState.last_filter_vy = body_vy_mps;
  fusionCalcState.filter_timestamp_ms = dvl->timestamp;
  lastVnMps = vn_mps;
  lastVeMps = ve_mps;
  haveLastVelocity = 1U;
  consecutiveMotionRejects = 0U;

  DVL_Fusion_PublishKinematics(dvl,
                               imu,
                               body_vx_mps,
                               body_vy_mps,
                               vn_mps,
                               ve_mps,
                               0U,
                               0.0f);
}

void DVL_Fusion_Reset(float x_m, float y_m)
{
  if (DVL_Fusion_LockState(portMAX_DELAY) == pdTRUE)
  {
    memset(&fusionState, 0, sizeof(fusionState));
    fusionState.x_m = x_m;
    fusionState.y_m = y_m;
    DVL_Fusion_UnlockState();
  }
  else
  {
    memset(&fusionState, 0, sizeof(fusionState));
    fusionState.x_m = x_m;
    fusionState.y_m = y_m;
  }

  memset(&fusionCalcState, 0, sizeof(fusionCalcState));
  lastVnMps = 0.0f;
  lastVeMps = 0.0f;
  consecutiveMotionRejects = 0U;
  haveLastVelocity = 0U;
}

BaseType_t DVL_Fusion_Task_Init(void)
{
  if (fusionTaskHandle != NULL)
  {
    return pdPASS;
  }

  if (fusionStateMutex == NULL)
  {
    fusionStateMutex = xSemaphoreCreateMutex();
    if (fusionStateMutex == NULL)
    {
      return pdFAIL;
    }
  }

  if (fusionDvlQueue == NULL)
  {
    fusionDvlQueue = xQueueCreate(DVL_FUSION_DVL_QUEUE_LENGTH, sizeof(DVL_Data_t));
    if (fusionDvlQueue == NULL)
    {
      return pdFAIL;
    }
  }

  DVL_Fusion_Reset(0.0f, 0.0f);

  fusionTaskHandle = osThreadNew(DVL_Fusion_Task, NULL, &dvlFusionTaskAttributes);
  if (fusionTaskHandle == NULL)
  {
    return pdFAIL;
  }

  return pdPASS;
}

BaseType_t DVL_Fusion_SubmitDvlData(const DVL_Data_t *dvl_snapshot)
{
  if ((dvl_snapshot == NULL) || (fusionDvlQueue == NULL))
  {
    return pdFAIL;
  }

  return xQueueOverwrite(fusionDvlQueue, dvl_snapshot);
}

BaseType_t DVL_Fusion_Update(const DVL_Data_t *dvl_snapshot)
{
  DVL_Data_t dvl;
  JY901S_Data_t imu;
  float body_vx_mps;
  float body_vy_mps;
  float vn_mps;
  float ve_mps;
  uint32_t dt_ms;
  float dt_s;

	//防御
  if (dvl_snapshot == NULL)
  {
    return pdFAIL;
  }
  dvl = *dvl_snapshot;

	//防御
  if (dvl.frame_count == fusionCalcState.processed_dvl_frame_count)
  {
    return pdFAIL;
  }
  fusionCalcState.processed_dvl_frame_count = dvl.frame_count;
	
	//判断DVL是否失锁
  if (DVL_Fusion_IsDvlVelocityValid(&dvl) == 0U)
  {
    DVL_Fusion_HandleDvlLost(&dvl);
    return pdFAIL;
  }
	//DVL连续失锁后的状态恢复
  if (fusionCalcState.consecutive_invalid != 0U){
    fusionCalcState.consecutive_invalid_count = 0U;
    
    fusionCalcState.consecutive_valid_count++;

    if (fusionCalcState.consecutive_valid_count < DVL_FUSION_REACQUIRE_VALID_COUNT)
    {
      DVL_Fusion_PublishCounters(dvl.frame_count);
      return pdFAIL;
    }
  }else{
    fusionCalcState.consecutive_invalid_count = 0U;
    fusionCalcState.consecutive_valid_count = 0U;
  }

  

  if (DVL_Fusion_ReadValidImu(&imu, dvl.timestamp) != pdPASS)
  {
    haveLastVelocity = 0U;
    fusionCalcState.consecutive_valid_count = 0U;
		DVL_Fusion_PublishCounters(dvl.frame_count);
    return pdFAIL;
  }

  DVL_Fusion_CompensateLeverArm(dvl.raw_vx * DVL_FUSION_MM_S_TO_M_S,
                                dvl.raw_vy * DVL_FUSION_MM_S_TO_M_S,
                                imu.gyro_dps[2],
                                &body_vx_mps,
                                &body_vy_mps);

  if (fusionCalcState.consecutive_invalid != 0U)
  {
    DVL_Fusion_BodyToNav(body_vx_mps,
                         body_vy_mps,
                         imu.angle_deg[2],
                         &vn_mps,
                         &ve_mps);

    if (DVL_Fusion_IsVelocityBelowDeadband(body_vx_mps, body_vy_mps) != 0U)
    {
      body_vx_mps = 0.0f;
      body_vy_mps = 0.0f;
      vn_mps = 0.0f;
      ve_mps = 0.0f;
    }

    DVL_Fusion_SetBaseline(&dvl, &imu, body_vx_mps, body_vy_mps, vn_mps, ve_mps);
    fusionCalcState.consecutive_invalid = 0U;
    fusionCalcState.consecutive_valid_count = 0U;
    DVL_Fusion_PublishCounters(dvl.frame_count);
    return pdFAIL;
  }

  if (haveLastVelocity == 0U)
  {
    DVL_Fusion_BodyToNav(body_vx_mps,
                         body_vy_mps,
                         imu.angle_deg[2],
                         &vn_mps,
                         &ve_mps);

    if (DVL_Fusion_IsVelocityBelowDeadband(body_vx_mps, body_vy_mps) != 0U)
    {
      body_vx_mps = 0.0f;
      body_vy_mps = 0.0f;
      vn_mps = 0.0f;
      ve_mps = 0.0f;
    }

    DVL_Fusion_SetBaseline(&dvl, &imu, body_vx_mps, body_vy_mps, vn_mps, ve_mps);
    return pdFAIL;
  }

  dt_ms = dvl.timestamp - fusionCalcState.filter_timestamp_ms;
  if ((dt_ms < DVL_FUSION_MIN_DT_MS) || (dt_ms > DVL_FUSION_MAX_DT_MS))
  {
    DVL_Fusion_BodyToNav(body_vx_mps,
                         body_vy_mps,
                         imu.angle_deg[2],
                         &vn_mps,
                         &ve_mps);
    DVL_Fusion_SetBaseline(&dvl, &imu, body_vx_mps, body_vy_mps, vn_mps, ve_mps);
    return pdFAIL;
  }

  dt_s = (float)dt_ms / 1000.0f;
  if (DVL_Fusion_FilterMotion(&body_vx_mps,
                              &body_vy_mps,
                              imu.gyro_dps[2],
                              dt_s) != pdPASS)
  {
    DVL_Fusion_IncrementFilterFailure();
    if (consecutiveMotionRejects >= DVL_FUSION_MOTION_REJECT_LIMIT)
    {
      haveLastVelocity = 0U;
      fusionCalcState.consecutive_valid_count = 0U;
    }
    return pdFAIL;
  }

  DVL_Fusion_BodyToNav(body_vx_mps,
                       body_vy_mps,
                       imu.angle_deg[2],
                       &vn_mps,
                       &ve_mps);

  if (DVL_Fusion_IsVelocityBelowDeadband(body_vx_mps, body_vy_mps) != 0U)
  {
    body_vx_mps = 0.0f;
    body_vy_mps = 0.0f;
    vn_mps = 0.0f;
    ve_mps = 0.0f;
    DVL_Fusion_SetBaseline(&dvl, &imu, body_vx_mps, body_vy_mps, vn_mps, ve_mps);
    return pdFAIL;
  }

  fusionCalcState.last_filter_vx = body_vx_mps;
  fusionCalcState.last_filter_vy = body_vy_mps;
  fusionCalcState.filter_timestamp_ms = dvl.timestamp;
  DVL_Fusion_PublishKinematics(&dvl,
                               &imu,
                               body_vx_mps,
                               body_vy_mps,
                               vn_mps,
                               ve_mps,
                               1U,
                               dt_s);
  lastVnMps = vn_mps;
  lastVeMps = ve_mps;
  haveLastVelocity = 1U;

  return pdPASS;
}

void DVL_Fusion_GetState(DVL_FusionState_t *state)
{
  if (state == NULL)
  {
    return;
  }

  if (DVL_Fusion_LockState(portMAX_DELAY) == pdTRUE)
  {
    *state = fusionState;
    DVL_Fusion_UnlockState();
  }
}

static void DVL_Fusion_Task(void *argument)
{
  DVL_Data_t dvl;

  (void)argument;

  if (Startup_WaitSystemReady(osWaitForever) != pdPASS)
  {
    for (;;)
    {
      osDelay(1000U);
    }
  }

  DVL_Fusion_Reset(0.0f, 0.0f);
  if (fusionDvlQueue != NULL)
  {
    (void)xQueueReset(fusionDvlQueue);
  }

  for (;;)
  {
    if ((fusionDvlQueue != NULL) &&
        (xQueueReceive(fusionDvlQueue, &dvl, portMAX_DELAY) == pdPASS))
    {
      (void)DVL_Fusion_Update(&dvl);
    }
  }
}
