#include "app_task_init.h"
#include "dvl.h"
#include "dvl_fusion.h"
#include "log_task.h"
#include "main.h"
#include "motor.h"
#include "jy901s.h"
#include "motion.h"
#include "startup_task.h"

static const osThreadAttr_t testTaskAttributes = {
  .name = "test_task",
  .stack_size = LOG_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)LOG_TASK_PRIORITY,
};


static void TestTask(void *argument)
{
  (void)argument;
	//DVL_FusionState_t nav={0};
	DVL_Data_t dvl={0};
  if (Startup_WaitSystemReady(osWaitForever) != pdPASS)
  {
    for (;;)
    {
      osDelay(1500U);
    }
  }

  for (;;)
  {
		/*
		DVL_Fusion_GetState(&nav);
		
    Log_printf("[NAV]x_m: %.3f y_m: %.3f vn_mps: %.3f ve_mps: %.3f body_vx_mps: %.3f body_vy_mps: %.3f yaw_deg: %.2f frame: %lu filter_timestamp: %lu integrated: %lu rejected: %lu last_update: %u valid: %u\r\n",
               nav.x_m,
               nav.y_m,
               nav.vn_mps,
               nav.ve_mps,
               nav.body_vx_mps,
               nav.body_vy_mps,
               nav.yaw_deg,
               (unsigned long)nav.dvl_frame_count,
               (unsigned long)nav.dvl_filter_timestamp,
               (unsigned long)nav.integrated_count,
               (unsigned long)nav.rejected_count,
               (unsigned int)nav.last_update,
               (unsigned int)nav.velocity_valid);
		*/
		DVL_GetData(&dvl);
    Log_printf("[DVL]raw_vx: %.2f raw_vy: %.2f filt_vx: %.2f filt_vy: %.2f status: %c invalid: %u filter_fail: %u frame: %lu timestamp: %lu\r\n",
               dvl.raw_vx,
               dvl.raw_vy,
               dvl.vx,
               dvl.vy,
               (char)dvl.status,
               dvl.velocity_invalid_count,
               (unsigned int)dvl.filter_failure_count,
               (unsigned long)dvl.frame_count,
               (unsigned long)dvl.timestamp);
		
		
		osDelay(150);
  }
}

void App_Task_Init(void){
	if (JY901S_Init() != pdPASS)
  {
    Error_Handler();
  }
	if (Log_Task_Init() != pdPASS)
	{
		Error_Handler();
	}
	if (DVL_Init() != pdPASS)
	{
		Error_Handler();
	}

	if (Startup_Task_Init() != pdPASS)
	{
		Error_Handler();
	}

	if (DVL_Fusion_Task_Init() != pdPASS)
	{
		Error_Handler();
	}
	
	if (Motion_Task_Init() != pdPASS)
	{
		Error_Handler();
	}

	osThreadId_t testTaskHandle = osThreadNew(TestTask, NULL, &testTaskAttributes);
}
