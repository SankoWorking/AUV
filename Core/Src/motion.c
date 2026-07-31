#include "motion.h"

#include "cmsis_os2.h"
#include "jy901s.h"
#include "log_task.h"
#include "motor.h"
#include "startup_task.h"

#define MOTION_FORWARD_SPEED_PERCENT 40
#define MOTION_FORWARD_TIME_MS       5000U
#define MOTION_TURN_TARGET_DEG       180.0f
#define MOTION_TURN_KP               0.75f
#define MOTION_TURN_KI               0.02f
#define MOTION_TURN_KD               0.12f
#define MOTION_TURN_MIN_SPEED        30
#define MOTION_TURN_MAX_SPEED        55
#define MOTION_TURN_TOLERANCE_DEG    5.0f
#define MOTION_TURN_RATE_TOLERANCE_DPS 8.0f
#define MOTION_TURN_STABLE_COUNT     5U
#define MOTION_TURN_CONTROL_PERIOD_MS 20U
#define MOTION_TURN_TIMEOUT_MS       15000U
#define MOTION_IMU_WAIT_TIMEOUT_MS   1000U
#define MOTION_YAW_CONTROL_SIGN      1.0f
#define MOTION_TURN_INTEGRAL_LIMIT   250.0f
#define MOTION_TURN_PROGRESS_DEADBAND_DEG 0.15f
#define MOTION_TURN_LOG_INTERVAL_MS  200U
#define MOTION_TASK_STACK_SIZE_BYTES (512U * 4U)
#define MOTION_TASK_PRIORITY         osPriorityNormal

static void MotionTask(void *argument);
static BaseType_t Motion_RunForwardTurn180Forward(void);

static osThreadId_t motionTaskHandle;

static const osThreadAttr_t motionTaskAttributes = {
  .name = "MotionTask",
  .stack_size = MOTION_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)MOTION_TASK_PRIORITY,
};

static float Motion_NormalizeAngleDeg(float angle_deg)
{
  while (angle_deg > 180.0f)
  {
    angle_deg -= 360.0f;
  }

  while (angle_deg < -180.0f)
  {
    angle_deg += 360.0f;
  }

  return angle_deg;
}

static float Motion_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static int16_t Motion_AbsInt16(int16_t value)
{
  return (value < 0) ? (int16_t)-value : value;
}

static int16_t Motion_ClampTurnSpeed(float command)
{
  int16_t speed = (int16_t)command;

  if (speed > MOTION_TURN_MAX_SPEED)
  {
    speed = MOTION_TURN_MAX_SPEED;
  }
  else if (speed < -MOTION_TURN_MAX_SPEED)
  {
    speed = -MOTION_TURN_MAX_SPEED;
  }

  if ((speed != 0) && (Motion_AbsInt16(speed) < MOTION_TURN_MIN_SPEED))
  {
    speed = (speed > 0) ? MOTION_TURN_MIN_SPEED : -MOTION_TURN_MIN_SPEED;
  }

  return speed;
}

static BaseType_t Motion_GetImu(JY901S_Data_t *imu)
{
  if (imu == NULL)
  {
    return pdFAIL;
  }

  if (JY901S_GetDataTimeout(imu, pdMS_TO_TICKS(20U)) != pdPASS)
  {
    return pdFAIL;
  }

  if (imu->angle_valid == 0U)
  {
    return pdFAIL;
  }

  return pdPASS;
}

static BaseType_t Motion_WaitForImu(JY901S_Data_t *imu)
{
  uint32_t startTick = HAL_GetTick();

  while ((HAL_GetTick() - startTick) < MOTION_IMU_WAIT_TIMEOUT_MS)
  {
    if (Motion_GetImu(imu) == pdPASS)
    {
      return pdPASS;
    }

    osDelay(MOTION_TURN_CONTROL_PERIOD_MS);
  }

  return pdFAIL;
}

static BaseType_t Motion_TurnRelative(float relative_deg)
{
  JY901S_Data_t imu;
  float targetAbs = Motion_AbsFloat(relative_deg);
  float direction = (relative_deg >= 0.0f) ? 1.0f : -1.0f;
  float progressDirection = 0.0f;
  float previousYaw;
  float turnedAbs = 0.0f;
  float integral = 0.0f;
  float previousError;
  uint32_t startTick;
  uint32_t previousTick;
  uint32_t lastLogTick;
  uint8_t stableCount = 0U;

  Log_printf("[MOTION] phase=turn_wait_imu target=%.1f\r\n", relative_deg);

  if (Motion_WaitForImu(&imu) != pdPASS)
  {
    Log_printf("[MOTION] phase=turn_wait_imu result=fail\r\n");
    Motor_Stop();
    return pdFAIL;
  }

  previousYaw = imu.angle_deg[2];
  previousError = targetAbs;
  previousTick = HAL_GetTick();
  startTick = HAL_GetTick();
  lastLogTick = startTick;

  Log_printf("[MOTION] phase=turn_start start_yaw=%.2f target_abs=%.2f\r\n",
             previousYaw,
             targetAbs);

  while ((HAL_GetTick() - startTick) < MOTION_TURN_TIMEOUT_MS)
  {
    uint32_t currentTick;
    float dt;
    float yawDelta;
    float yawRate = 0.0f;
    float error;
    float derivative;
    float command;
    int16_t turnSpeed = 0;

    if (Motion_GetImu(&imu) != pdPASS)
    {
      Log_printf("[MOTION] phase=turn result=fail reason=imu_lost\r\n");
      Motor_Stop();
      return pdFAIL;
    }

    currentTick = HAL_GetTick();
    dt = (float)(currentTick - previousTick) / 1000.0f;
    if (dt <= 0.0f)
    {
      dt = (float)MOTION_TURN_CONTROL_PERIOD_MS / 1000.0f;
    }

    yawDelta = Motion_NormalizeAngleDeg(imu.angle_deg[2] - previousYaw);

    if ((progressDirection == 0.0f) &&
        (Motion_AbsFloat(yawDelta) >= MOTION_TURN_PROGRESS_DEADBAND_DEG))
    {
      progressDirection = (yawDelta >= 0.0f) ? 1.0f : -1.0f;
      Log_printf("[MOTION] phase=turn_detect_dir yaw_delta=%.2f progress_dir=%.0f\r\n",
                 yawDelta,
                 progressDirection);
    }

    if (progressDirection != 0.0f)
    {
      turnedAbs += yawDelta * progressDirection;
    }

    previousYaw = imu.angle_deg[2];
    previousTick = currentTick;

    if (turnedAbs < 0.0f)
    {
      turnedAbs = 0.0f;
    }

    error = targetAbs - turnedAbs;

    if (imu.gyro_valid != 0U)
    {
      yawRate = (progressDirection == 0.0f) ?
                imu.gyro_dps[2] :
                (imu.gyro_dps[2] * progressDirection);
    }

    if ((Motion_AbsFloat(error) <= MOTION_TURN_TOLERANCE_DEG) &&
        (Motion_AbsFloat(yawRate) <= MOTION_TURN_RATE_TOLERANCE_DPS))
    {
      stableCount++;
      Motor_SetDrive(0, 0);

      if (stableCount >= MOTION_TURN_STABLE_COUNT)
      {
        Motor_SetDrive(0, 0);
        Log_printf("[MOTION] phase=turn_done yaw=%.2f turned=%.2f error=%.2f rate=%.2f stable=%u\r\n",
                   imu.angle_deg[2],
                   turnedAbs,
                   error,
                   yawRate,
                   stableCount);
        return pdPASS;
      }
    }
    else
    {
      stableCount = 0U;

      integral += error * dt;
      if (integral > MOTION_TURN_INTEGRAL_LIMIT)
      {
        integral = MOTION_TURN_INTEGRAL_LIMIT;
      }
      else if (integral < -MOTION_TURN_INTEGRAL_LIMIT)
      {
        integral = -MOTION_TURN_INTEGRAL_LIMIT;
      }

      derivative = (error - previousError) / dt;
      previousError = error;

      command = (error * MOTION_TURN_KP) +
                (integral * MOTION_TURN_KI) +
                (derivative * MOTION_TURN_KD);
      turnSpeed = Motion_ClampTurnSpeed(command);

      if (error < 0.0f)
      {
        turnSpeed = -Motion_AbsInt16(turnSpeed);
      }
      else
      {
        turnSpeed = Motion_AbsInt16(turnSpeed);
      }

      turnSpeed = (int16_t)((float)turnSpeed * direction * MOTION_YAW_CONTROL_SIGN);
      Motor_SetDrive((int16_t)-turnSpeed, turnSpeed);
    }

    if ((currentTick - lastLogTick) >= MOTION_TURN_LOG_INTERVAL_MS)
    {
      Log_printf("[MOTION] phase=turn yaw=%.2f delta=%.2f turned=%.2f error=%.2f rate=%.2f cmd=%d stable=%u\r\n",
                 imu.angle_deg[2],
                 yawDelta,
                 turnedAbs,
                 error,
                 yawRate,
                 turnSpeed,
                 stableCount);
      lastLogTick = currentTick;
    }

    osDelay(MOTION_TURN_CONTROL_PERIOD_MS);
  }

  Motor_Stop();
  Log_printf("[MOTION] phase=turn_timeout turned=%.2f target=%.2f stable=%u\r\n",
             turnedAbs,
             targetAbs,
             stableCount);
  return pdFAIL;
}

BaseType_t Motion_Task_Init(void)
{
  if (motionTaskHandle != NULL)
  {
    return pdPASS;
  }

  motionTaskHandle = osThreadNew(MotionTask, NULL, &motionTaskAttributes);
  if (motionTaskHandle == NULL)
  {
    return pdFAIL;
  }

  return pdPASS;
}

static void MotionTask(void *argument)
{
  (void)argument;

  Log_printf("[MOTION] phase=wait_system_ready\r\n");
  if (Startup_WaitSystemReady(osWaitForever) != pdPASS)
  {
    Motor_Stop();
    Log_printf("[MOTION] phase=abort reason=system_not_ready\r\n");
    for (;;)
    {
      osDelay(1000U);
    }
  }
  Log_printf("[MOTION] phase=system_ready\r\n");

  (void)Motion_RunForwardTurn180Forward();

  for (;;)
  {
    osDelay(1000U);
  }
}

static BaseType_t Motion_RunForwardTurn180Forward(void)
{
  Log_printf("[MOTION] phase=forward_1 speed=%d duration_ms=%u\r\n",
             MOTION_FORWARD_SPEED_PERCENT,
             MOTION_FORWARD_TIME_MS);
  Motor_Forward(MOTION_FORWARD_SPEED_PERCENT);
  osDelay(MOTION_FORWARD_TIME_MS);
  Motor_SetDrive(0, 0);
  Log_printf("[MOTION] phase=forward_1_done\r\n");
  osDelay(100U);

  if (Motion_TurnRelative(MOTION_TURN_TARGET_DEG) != pdPASS)
  {
    Motor_Stop();
    Log_printf("[MOTION] phase=abort reason=turn_failed\r\n");
    return pdFAIL;
  }

  Log_printf("[MOTION] phase=forward_2 speed=%d duration_ms=%u\r\n",
             MOTION_FORWARD_SPEED_PERCENT,
             MOTION_FORWARD_TIME_MS);
  Motor_Forward(MOTION_FORWARD_SPEED_PERCENT);
  osDelay(MOTION_FORWARD_TIME_MS);
  Motor_Stop();
  Log_printf("[MOTION] phase=done\r\n");

  return pdPASS;
}
