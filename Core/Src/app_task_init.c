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
	Nav_State_t nav={0};
	//JY901S_Data_t imu={0};
	//DVL_Data_t dvl={0};
  if (Startup_WaitSystemReady(osWaitForever) != pdPASS)
  {
    for (;;)
    {
      osDelay(1500U);
    }
  }

  for (;;)
  {

		DVL_Fusion_GetState(&nav);
		
	  Log_printf("[NAV]x_m: %.3f y_m: %.3f yaw_deg: %.1f wz_dps: %.1f dvl_vx: %.0f dvl_vy: %.0f rot_vx_mps: %.3f rot_vy_mps: %.3f body_vx_mps: %.3f body_vy_mps: %.3f vn_mps: %.3f ve_mps: %.3f invalid: %lu frame: %lu\r\n",
					 nav.x_m,
					 nav.y_m,
					 nav.yaw_deg,
					 nav.yaw_rate_dps,
					 nav.dvl_vx_mm_s,
					 nav.dvl_vy_mm_s,
					 nav.rot_vx_mps,
					 nav.rot_vy_mps,
					 nav.body_vx_mps,
					 nav.body_vy_mps,
					 nav.vn_mps,
					 nav.ve_mps,
					 (unsigned long)nav.invalid_velocity_count,
					 (unsigned long)nav.dvl_frame_count);
/*
		if (JY901S_GetData(&imu) == pdPASS)
		{
			Log_printf("[IMU]a gyro_z_dps: %.3f yaw_deg: %.2f\r\n",
								 imu.gyro_dps[2],
								 imu.angle_deg[2]);
		}
		else
		{
			Log_printf("[IMU]get_data: fail\r\n");
		}
		if (DVL_GetData(&dvl) == pdPASS)
		{
			Log_printf("[DVL]vx_mm_s: %.2f vy_mm_s: %.2f vz_mm_s: %.2f status: %c frame: %lu\r\n",
								 dvl.raw_vx,
								 dvl.raw_vy,
								 dvl.raw_vz,
								 (char)dvl.status,
								 (unsigned long)dvl.frame_count);
		}
		else
		{
			Log_printf("[DVL]get_data: fail\r\n");
		}
*/
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
