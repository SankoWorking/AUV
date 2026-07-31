#include "app_task_init.h"
#include "dvl.h"
#include "log_task.h"
#include "main.h"
#include "motor.h"
#include "jy901s.h"
#include "motion.h"

static const osThreadAttr_t testTaskAttributes = {
  .name = "test_task",
  .stack_size = LOG_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)LOG_TASK_PRIORITY,
};


static void TestTask(void *argument)
{
  (void)argument;
	DVL_Data_t dvl={0};
	/*
	Motor_SetTopSuction(0);
	osDelay(5000);
	Motor_Forward(40);
	Motor_SetTopSuction(70);
	*/
  for (;;)
  {
		DVL_GetData(&dvl);
		
    Log_printf("[DVL]vx: %.2f vy: %.2f status: %c invalid: %u frame: %lu timestamp: %lu\r\n",
               dvl.raw_vx,
               dvl.raw_vy,
               (char)dvl.status,
               dvl.velocity_invalid_count,
               (unsigned long)dvl.frame_count,
               (unsigned long)dvl.timestamp);
		
		osDelay(100);
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
	
	if (Motion_Task_Init() != pdPASS)
	{
		Error_Handler();
	}
	
	osThreadId_t testTaskHandle = osThreadNew(TestTask, NULL, &testTaskAttributes);
}
