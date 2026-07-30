#include "app_task_init.h"
#include "log_task.h"
#include "main.h"
#include "motor.h"
#include "jy901s.h"

static const osThreadAttr_t testTaskAttributes = {
  .name = "test_task",
  .stack_size = LOG_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)LOG_TASK_PRIORITY,
};


static void TestTask(void *argument)
{
  (void)argument;
	/*
	Motor_SetTopSuction(0);
	osDelay(5000);
	Motor_Forward(40);
	Motor_SetTopSuction(70);
	*/
	JY901S_Data_t imu={0};
  for (;;)
  {
		JY901S_GetData(&imu);
		
    Log_printf("[IMU]yaw: %.2f",imu.angle_deg[2]);
		
		osDelay(1000);
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
	osThreadId_t testTaskHandle = osThreadNew(TestTask, NULL, &testTaskAttributes);
}
