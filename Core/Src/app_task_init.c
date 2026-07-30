#include "app_task_init.h"
#include "log_task.h"
#include "main.h"
#include "motor.h"

static const osThreadAttr_t testTaskAttributes = {
  .name = "test_task",
  .stack_size = LOG_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)LOG_TASK_PRIORITY,
};


static void TestTask(void *argument)
{
  (void)argument;
	Motor_SetTopSuction(0);
	osDelay(5000);
	Motor_Forward(40);
	
	Motor_SetTopSuction(70);
  for (;;)
  {
    Log_printf("hello");
		
		osDelay(1000);
  }
}

void App_Task_Init(void){
	if (Log_Task_Init() != pdPASS)
	{
		Error_Handler();
	}
	osThreadId_t testTaskHandle = osThreadNew(TestTask, NULL, &testTaskAttributes);
}
