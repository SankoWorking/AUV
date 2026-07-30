#include "app_task_init.h"
#include "main.h"

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void DefaultTask(void *argument);

void DefaultTask(void *argument)
{
	(void)argument;
  for(;;)
  {
    osDelay(1);
  }
}


void App_Task_Init(void){
	defaultTaskHandle = osThreadNew(DefaultTask, NULL, &defaultTask_attributes);
}
