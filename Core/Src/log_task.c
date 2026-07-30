#include "log_task.h"

#include "cmsis_os2.h"
#include "main.h"
#include "message_buffer.h"
#include "semphr.h"

#include <stdarg.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;

static void Log_Task(void *argument);

static MessageBufferHandle_t logMessageBuffer;
static SemaphoreHandle_t logWriteMutex;
static osThreadId_t logTaskHandle;
static volatile uint32_t logDroppedCount;

static const osThreadAttr_t logTaskAttributes = {
  .name = "log_task",
  .stack_size = LOG_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)LOG_TASK_PRIORITY,
};

BaseType_t Log_Task_Init(void)
{
  if (logTaskHandle != NULL)
  {
    return pdPASS;
  }

  logMessageBuffer = xMessageBufferCreate(LOG_BUFFER_SIZE_BYTES);
  if (logMessageBuffer == NULL)
  {
    return pdFAIL;
  }

  logWriteMutex = xSemaphoreCreateMutex();
  if (logWriteMutex == NULL)
  {
    return pdFAIL;
  }

  logTaskHandle = osThreadNew(Log_Task, NULL, &logTaskAttributes);
  if (logTaskHandle == NULL)
  {
    return pdFAIL;
  }

  return pdPASS;
}

int Log_printf(const char *format, ...)
{
  char message[LOG_MAX_MESSAGE_LENGTH];
  va_list args;
  int formattedLength;
  size_t sendLength;

  if (format == NULL)
  {
    return -1;
  }

  va_start(args, format);
  formattedLength = vsnprintf(message, sizeof(message), format, args);
  va_end(args);

  if (formattedLength < 0)
  {
    logDroppedCount++;
    return -1;
  }

  sendLength = (formattedLength >= (int)sizeof(message)) ?
               (sizeof(message) - 1U) :
               (size_t)formattedLength;

  return Log_write(message, sendLength);
}

int Log_write(const char *data, size_t length)
{
  size_t sentLength;

  if ((data == NULL) || (length == 0U))
  {
    return 0;
  }

  if ((logMessageBuffer == NULL) || (logWriteMutex == NULL) || (__get_IPSR() != 0U))
  {
    logDroppedCount++;
    return -1;
  }

  if (length > LOG_MAX_MESSAGE_LENGTH)
  {
    length = LOG_MAX_MESSAGE_LENGTH;
  }

  if (xSemaphoreTake(logWriteMutex, (TickType_t)LOG_MUTEX_TIMEOUT_TICKS) != pdTRUE)
  {
    logDroppedCount++;
    return -1;
  }

  sentLength = xMessageBufferSend(logMessageBuffer, data, length, (TickType_t)LOG_SEND_TIMEOUT_TICKS);
  xSemaphoreGive(logWriteMutex);

  if (sentLength != length)
  {
    logDroppedCount++;
    return -1;
  }

  return (int)sentLength;
}

uint32_t Log_GetDroppedCount(void)
{
  return logDroppedCount;
}

static void Log_Task(void *argument)
{
  char message[LOG_MAX_MESSAGE_LENGTH];

  (void)argument;

  for (;;)
  {
    size_t receivedLength = xMessageBufferReceive(logMessageBuffer,
                                                  message,
                                                  sizeof(message),
                                                  portMAX_DELAY);

    if (receivedLength > 0U)
    {
      (void)HAL_UART_Transmit(&huart1,
                              (uint8_t *)message,
                              (uint16_t)receivedLength,
                              HAL_MAX_DELAY);
    }
  }
}
