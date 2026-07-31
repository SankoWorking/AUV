#include "startup_task.h"

#include "dvl.h"
#include "log_task.h"
#include "motor.h"

#define STARTUP_EVENT_SYSTEM_READY          (1U << 0)
#define STARTUP_TOP_SUCTION_ARM_DELAY_MS   3000U
#define STARTUP_TOP_SUCTION_START_PERCENT  50U

static void StartupTask(void *argument);

static osThreadId_t startupTaskHandle;
static osEventFlagsId_t startupEventFlags;

static const osThreadAttr_t startupTaskAttributes = {
  .name = "StartupTask",
  .stack_size = STARTUP_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)STARTUP_TASK_PRIORITY,
};

BaseType_t Startup_Task_Init(void)
{
  if (startupTaskHandle != NULL)
  {
    return pdPASS;
  }

  startupEventFlags = osEventFlagsNew(NULL);
  if (startupEventFlags == NULL)
  {
    return pdFAIL;
  }

  startupTaskHandle = osThreadNew(StartupTask, NULL, &startupTaskAttributes);
  if (startupTaskHandle == NULL)
  {
    return pdFAIL;
  }

  return pdPASS;
}

BaseType_t Startup_WaitSystemReady(TickType_t timeout_ticks)
{
  uint32_t flags;

  if (startupEventFlags == NULL)
  {
    return pdFAIL;
  }

  flags = osEventFlagsWait(startupEventFlags,
                           STARTUP_EVENT_SYSTEM_READY,
                           osFlagsWaitAny,
                           timeout_ticks);
  if ((flags & osFlagsError) != 0U)
  {
    return pdFAIL;
  }

  return pdPASS;
}

static void Startup_Fail(const char *reason)
{
  Motor_Stop();
  Log_printf("[STARTUP] phase=fail reason=%s\r\n", reason);

  for (;;)
  {
    osDelay(1000U);
  }
}

static void StartupTask(void *argument)
{
  (void)argument;

  if (DVL_ConfigureStartup() != pdPASS)
  {
    Startup_Fail("dvl_config");
  }

  Log_printf("[STARTUP] phase=start\r\n");
  Log_printf("[STARTUP] phase=dvl_wait_ready valid_frames=%u\r\n",
             DVL_STARTUP_VALID_FRAME_COUNT);
  if (DVL_WaitReady(osWaitForever) != pdPASS)
  {
    Startup_Fail("dvl_ready_timeout");
  }
  Log_printf("[STARTUP] phase=dvl_ready\r\n");

  Log_printf("[STARTUP] phase=top_suction_arm speed=0 duration_ms=%u\r\n",
             STARTUP_TOP_SUCTION_ARM_DELAY_MS);
  Motor_SetTopSuction(0U);
  osDelay(STARTUP_TOP_SUCTION_ARM_DELAY_MS);
  Motor_SetTopSuction(STARTUP_TOP_SUCTION_START_PERCENT);
  Log_printf("[STARTUP] phase=top_suction_start speed=%u\r\n",
             STARTUP_TOP_SUCTION_START_PERCENT);

  (void)osEventFlagsSet(startupEventFlags, STARTUP_EVENT_SYSTEM_READY);
  Log_printf("[STARTUP] phase=system_ready\r\n");

  osThreadExit();
}
