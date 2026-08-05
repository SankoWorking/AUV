#include "dvl_fusion.h"

#include "dvl.h"
#include "jy901s.h"
#include "queue.h"
#include "semphr.h"
#include "startup_task.h"

#include <math.h>
#include <string.h>

#define DVL_FUSION_DVL_QUEUE_LENGTH           1U
#define DVL_FUSION_MIN_DT_MS                  50U
#define DVL_FUSION_MAX_DT_MS                  300U
#define DVL_FUSION_MM_S_TO_M_S                0.001f
#define DVL_FUSION_DEG_TO_RAD                 0.017453292519943295f
#define DVL_FUSION_SPEED_DEADBAND_MM_S        10.0f

#define DVL_FUSION_DVL_OFFSET_X_M             0.15f
#define DVL_FUSION_DVL_OFFSET_Y_M             0.20f
#define DVL_FUSION_DVL_TO_IMU_YAW_DEG         0.0f

#define DVL_FUSION_IMU_SYNC_THRESHOLD_MS      200U

#define DVL_FUSION_BASE_NOISE_M_S             0.03f
#define DVL_FUSION_MAX_ACCEL_M_S2             0.80f
#define DVL_FUSION_TURN_MARGIN_M_S            0.02f
#define DVL_FUSION_HARD_JUMP_M_S              1.50f
#define DVL_FUSION_MOTION_REJECT_LIMIT        3U
#define DVL_FUSION_FILTER_TIMEOUT_RECOVERY_LIMIT 1U

typedef struct
{
  uint32_t invalid_velocity_count;
  float last_filter_vx;
  float last_filter_vy;
  float last_vn_mps;
  float last_ve_mps;
  uint32_t filter_timestamp_ms;
  uint8_t consecutive_motion_rejects;
  uint8_t have_last_velocity;
  uint8_t filter_timeout_recovering;
  uint8_t filter_timeout_recovery_count;
	uint32_t processed_dvl_frame_count;
	uint32_t imu_invalid_count;
	uint32_t imu_timeout_count;
	uint32_t filter_failure_count;
	uint32_t filter_timeout_count;
} DVL_FusionState_t;

static void DVL_Fusion_Task(void *argument);

static Nav_State_t navState;
static DVL_FusionState_t fusionState;
static SemaphoreHandle_t fusionStateMutex;
static osThreadId_t fusionTaskHandle;
static QueueHandle_t fusionDvlQueue;

static const osThreadAttr_t dvlFusionTaskAttributes = {
  .name = "dvl_fusion",
  .stack_size = DVL_FUSION_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)DVL_FUSION_TASK_PRIORITY,
};

static BaseType_t DVL_Fusion_Lock(TickType_t timeout_ticks)
{
  if (fusionStateMutex == NULL)
  {
    return pdFAIL;
  }

  return xSemaphoreTake(fusionStateMutex, timeout_ticks);
}

static void DVL_Fusion_Unlock(void)
{
  if (fusionStateMutex != NULL)
  {
    xSemaphoreGive(fusionStateMutex);
  }
}

static void DVL_Fusion_UpdateNavState(const DVL_Data_t *dvl,
                                      const JY901S_Data_t *imu,
                                      float vn_mps,
                                      float ve_mps,
                                      uint8_t integrate,
                                      float dt_s)
{
  if (DVL_Fusion_Lock(portMAX_DELAY) == pdTRUE)
  {
    if (integrate != 0U)
    {
      navState.x_m += 0.5f * (fusionState.last_vn_mps + vn_mps) * dt_s;
      navState.y_m += 0.5f * (fusionState.last_ve_mps + ve_mps) * dt_s;
      navState.integrated_count++;
      navState.nav_timestamp_ms = dvl->timestamp;
    }

    navState.vn_mps = vn_mps;
    navState.ve_mps = ve_mps;
    navState.yaw_deg = imu->angle_deg[2];
    navState.dvl_frame_count = dvl->frame_count;
    navState.invalid_velocity_count = fusionState.invalid_velocity_count;
		navState.imu_invalid_count = fusionState.imu_invalid_count;
		navState.imu_timeout_count = fusionState.imu_timeout_count;
		navState.filter_failure_count = fusionState.filter_failure_count;
		navState.filter_timeout_count = fusionState.filter_timeout_count;
    DVL_Fusion_Unlock();
  }
}

/*
 * @brief  将载体坐标系速度转换到导航系
 */
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

static void DVL_Fusion_CompensateDvlToImuRotation(float dvl_vx_mps,
                                                  float dvl_vy_mps,
                                                  float *imu_vx_mps,
                                                  float *imu_vy_mps)
{
  float yaw_rad = DVL_FUSION_DVL_TO_IMU_YAW_DEG * DVL_FUSION_DEG_TO_RAD;
  float cy = cosf(yaw_rad);
  float sy = sinf(yaw_rad);

  *imu_vx_mps = (cy * dvl_vx_mps) - (sy * dvl_vy_mps);
  *imu_vy_mps = (sy * dvl_vx_mps) + (cy * dvl_vy_mps);
}

/**
 * @brief 杆臂补偿
 */

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
static void DVL_Fusion_HandleDvlLost(void)
{
	fusionState.invalid_velocity_count++;
}

static BaseType_t DVL_Fusion_Filter(float *body_vx_mps,
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
  float delta_x_mps = *body_vx_mps - fusionState.last_filter_vx;
  float delta_y_mps = *body_vy_mps - fusionState.last_filter_vy;

  if ((fabsf(delta_x_mps) > DVL_FUSION_HARD_JUMP_M_S) ||
      (fabsf(delta_y_mps) > DVL_FUSION_HARD_JUMP_M_S))
  {
    if (fusionState.consecutive_motion_rejects < 255U)
    {
      fusionState.consecutive_motion_rejects++;
    }
    return pdFAIL;
  }

  if (delta_x_mps > limit_x_mps)
  {
    *body_vx_mps = fusionState.last_filter_vx + limit_x_mps;
  }
  else if (delta_x_mps < -limit_x_mps)
  {
    *body_vx_mps = fusionState.last_filter_vx - limit_x_mps;
  }

  if (delta_y_mps > limit_y_mps)
  {
    *body_vy_mps = fusionState.last_filter_vy + limit_y_mps;
  }
  else if (delta_y_mps < -limit_y_mps)
  {
    *body_vy_mps = fusionState.last_filter_vy - limit_y_mps;
  }

  fusionState.consecutive_motion_rejects = 0U;
  return pdPASS;
}

static BaseType_t DVL_Fusion_ImuSync(JY901S_Data_t *imu,
                                     uint32_t dvl_timestamp_ms)
{
  if (JY901S_GetData(imu) != pdPASS)
  {
    fusionState.imu_invalid_count++;
    return pdFAIL;
  }

  if ((imu->angle_valid == 0U) || (imu->gyro_valid == 0U))
  {
    fusionState.imu_invalid_count++;;
    return pdFAIL;
  }

  if ((DVL_Fusion_TickAbsDiff(dvl_timestamp_ms, imu->gyro_timestamp_ms) >
       DVL_FUSION_IMU_SYNC_THRESHOLD_MS) ||
      (DVL_Fusion_TickAbsDiff(dvl_timestamp_ms, imu->angle_timestamp_ms) >
       DVL_FUSION_IMU_SYNC_THRESHOLD_MS))
  {
    fusionState.imu_timeout_count++;;
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
  fusionState.last_filter_vx = body_vx_mps;
  fusionState.last_filter_vy = body_vy_mps;
  fusionState.filter_timestamp_ms = dvl->timestamp;
  fusionState.last_vn_mps = vn_mps;
  fusionState.last_ve_mps = ve_mps;
  fusionState.have_last_velocity = 1U;
  fusionState.consecutive_motion_rejects = 0U;
  fusionState.filter_timeout_recovering = 0U;
  fusionState.filter_timeout_recovery_count = 0U;

  DVL_Fusion_UpdateNavState(dvl,
                            imu,
                            vn_mps,
                            ve_mps,
                            0U,
                            0.0f);
}

void DVL_Fusion_Reset(float x_m, float y_m)
{
  if (DVL_Fusion_Lock(portMAX_DELAY) == pdTRUE)
  {
    memset(&navState, 0, sizeof(navState));
    navState.x_m = x_m;
    navState.y_m = y_m;
    DVL_Fusion_Unlock();
  }
  else
  {
    memset(&navState, 0, sizeof(navState));
    navState.x_m = x_m;
    navState.y_m = y_m;
  }

  memset(&fusionState, 0, sizeof(fusionState));
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

static BaseType_t DVL_Fusion_Update(const DVL_Data_t *dvl_snapshot)
{
  DVL_Data_t dvl;
  JY901S_Data_t imu;
  float imu_vx_mps;
  float imu_vy_mps;
  float body_vx_mps;
  float body_vy_mps;
  float vn_mps;
  float ve_mps;
  uint32_t dt_ms;
  float dt_s;

  if (dvl_snapshot == NULL)
  {
    return pdFAIL;
  }
  dvl = *dvl_snapshot;

  if (dvl.frame_count == fusionState.processed_dvl_frame_count)
  {
    return pdFAIL;
  }
  fusionState.processed_dvl_frame_count = dvl.frame_count;
	
	//判断DVL是否失锁
  if (DVL_Fusion_IsDvlVelocityValid(&dvl) == 0U)
  {
    DVL_Fusion_HandleDvlLost();
    if (fusionState.filter_timeout_recovering != 0U)
    {
      fusionState.filter_timeout_recovery_count = 0U;
    }
    return pdFAIL;
  }
	//没有出现过读取IMU失败的情况，先不考虑这里的逻辑。
  if (DVL_Fusion_ImuSync(&imu, dvl.timestamp) != pdPASS)
  {
    if (fusionState.filter_timeout_recovering != 0U)
    {
      fusionState.filter_timeout_recovery_count = 0U;
    }
    else
    {
      fusionState.have_last_velocity = 0U;
    }
    return pdFAIL;
  }

  /* Filter-timeout recovery requires consecutive IMU-synced DVL frames. */
  if (fusionState.filter_timeout_recovering != 0U)
  {
    if (fusionState.filter_timeout_recovery_count < 255U)
    {
      fusionState.filter_timeout_recovery_count++;
    }

    if (fusionState.filter_timeout_recovery_count < DVL_FUSION_FILTER_TIMEOUT_RECOVERY_LIMIT)
    {
      return pdFAIL;
    }

    fusionState.filter_timeout_recovering = 0U;
    fusionState.filter_timeout_recovery_count = 0U;
    fusionState.have_last_velocity = 0U;
  }

  DVL_Fusion_CompensateDvlToImuRotation(dvl.raw_vx * DVL_FUSION_MM_S_TO_M_S,
                                        dvl.raw_vy * DVL_FUSION_MM_S_TO_M_S,
                                        &imu_vx_mps,
                                        &imu_vy_mps);

	//杆臂补偿
  DVL_Fusion_CompensateLeverArm(imu_vx_mps,
                                imu_vy_mps,
                                imu.gyro_dps[2],
                                &body_vx_mps,
                                &body_vy_mps);

	//失去积分标准后重新建立积分标准
  if (fusionState.have_last_velocity == 0U)
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

	//检查当前帧滤波超时
  dt_ms = dvl.timestamp - fusionState.filter_timestamp_ms;
  if ((dt_ms < DVL_FUSION_MIN_DT_MS) || (dt_ms > DVL_FUSION_MAX_DT_MS))
  {
		fusionState.filter_timeout_count++;
    fusionState.filter_timeout_recovering = 1U;
    fusionState.filter_timeout_recovery_count = 0U;
    return pdFAIL;
  }

	//滤波
  dt_s = (float)dt_ms / 1000.0f;
  if (DVL_Fusion_Filter(&body_vx_mps,
                        &body_vy_mps,
                        imu.gyro_dps[2],
                        dt_s) != pdPASS)
  {
    fusionState.filter_failure_count++;
    if (fusionState.consecutive_motion_rejects >= DVL_FUSION_MOTION_REJECT_LIMIT)
    {
      fusionState.have_last_velocity = 0U;
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

  fusionState.last_filter_vx = body_vx_mps;
  fusionState.last_filter_vy = body_vy_mps;
  fusionState.filter_timestamp_ms = dvl.timestamp;
  DVL_Fusion_UpdateNavState(&dvl,
                            &imu,
                            vn_mps,
                            ve_mps,
                            1U,
                            dt_s);
  fusionState.last_vn_mps = vn_mps;
  fusionState.last_ve_mps = ve_mps;
  fusionState.have_last_velocity = 1U;

  return pdPASS;
}

void DVL_Fusion_GetState(Nav_State_t *state)
{
  if (state == NULL)
  {
    return;
  }

  if (DVL_Fusion_Lock(portMAX_DELAY) == pdTRUE)
  {
    *state = navState;
    DVL_Fusion_Unlock();
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
