#include "dvl.h"

#include "main.h"
#include "semphr.h"
#include "stream_buffer.h"
#include <stdlib.h>
#include <string.h>

#define DVL_RX_STREAM_SIZE      256U
#define DVL_RX_TASK_CHUNK_SIZE  32U
#define DVL_LINE_BUFFER_SIZE    96U

extern UART_HandleTypeDef huart1;

static void DVL_Task(void *argument);
static void DVL_ParseByte(uint8_t byte);
static void DVL_ParseLine(char *line);
static void DVL_RestartReceiveFromIsr(void);

static DVL_Data_t dvlData;
static SemaphoreHandle_t dvlDataMutex;
static StreamBufferHandle_t dvlRxStream;
static osThreadId_t dvlTaskHandle;
static uint8_t dvlRxByte;

static const osThreadAttr_t dvlTaskAttributes = {
  .name = "dvl_task",
  .stack_size = DVL_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)DVL_TASK_PRIORITY,
};

BaseType_t DVL_Init(void)
{
  if (dvlTaskHandle != NULL)
  {
    return pdPASS;
  }

  dvlDataMutex = xSemaphoreCreateMutex();
  if (dvlDataMutex == NULL)
  {
    return pdFAIL;
  }

  dvlRxStream = xStreamBufferCreate(DVL_RX_STREAM_SIZE, 1U);
  if (dvlRxStream == NULL)
  {
    return pdFAIL;
  }

  dvlTaskHandle = osThreadNew(DVL_Task, NULL, &dvlTaskAttributes);
  if (dvlTaskHandle == NULL)
  {
    return pdFAIL;
  }

  return pdPASS;
}

BaseType_t DVL_GetData(DVL_Data_t *data)
{
  return DVL_GetDataTimeout(data, 0U);
}

BaseType_t DVL_GetDataTimeout(DVL_Data_t *data, TickType_t timeout_ticks)
{
  if ((data == NULL) || (dvlDataMutex == NULL))
  {
    return pdFAIL;
  }

  if (xSemaphoreTake(dvlDataMutex, timeout_ticks) != pdTRUE)
  {
    return pdFAIL;
  }

  *data = dvlData;
  xSemaphoreGive(dvlDataMutex);

  return pdPASS;
}

void DVL_UART_RxCpltCallback(UART_HandleTypeDef *huart,
                             BaseType_t *higher_priority_task_woken)
{
  if (huart->Instance == USART1)
  {
    BaseType_t localTaskWoken = pdFALSE;
    BaseType_t *taskWoken = (higher_priority_task_woken != NULL) ?
                            higher_priority_task_woken :
                            &localTaskWoken;

    if (dvlRxStream != NULL)
    {
      (void)xStreamBufferSendFromISR(dvlRxStream,
                                     &dvlRxByte,
                                     1U,
                                     taskWoken);
    }

    DVL_RestartReceiveFromIsr();
  }
}

void DVL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    DVL_RestartReceiveFromIsr();
  }
}

static void DVL_Task(void *argument)
{
  uint8_t rxChunk[DVL_RX_TASK_CHUNK_SIZE];

  (void)argument;

  while (HAL_UART_Receive_IT(&huart1, &dvlRxByte, 1U) != HAL_OK)
  {
    osDelay(10U);
  }

  for (;;)
  {
    size_t receivedLength = xStreamBufferReceive(dvlRxStream,
                                                 rxChunk,
                                                 sizeof(rxChunk),
                                                 portMAX_DELAY);
    size_t i;

    for (i = 0U; i < receivedLength; i++)
    {
      DVL_ParseByte(rxChunk[i]);
    }
  }
}

static void DVL_ParseByte(uint8_t byte)
{
  static char line[DVL_LINE_BUFFER_SIZE];
  static uint8_t index;

  if (byte == (uint8_t)':')
  {
    index = 0U;
    line[index++] = (char)byte;
    return;
  }

  if (byte == (uint8_t)'\n')
  {
    if (index > 0U)
    {
      line[index] = '\0';
      DVL_ParseLine(line);
    }
    index = 0U;
    return;
  }

  if (byte == (uint8_t)'\r')
  {
    return;
  }

  if (index >= (DVL_LINE_BUFFER_SIZE - 1U))
  {
    index = 0U;
    return;
  }

  line[index++] = (char)byte;
}

static void DVL_ParseLine(char *line)
{
  char *p;
  float raw_vx;
  float raw_vy;
  float raw_vz;
  float raw_ve;
  uint8_t raw_status = 0U;

  if ((line == NULL) || (line[0] == '\0'))
  {
    return;
  }

  if (strncmp(line, ":BI,", 4) != 0)
  {
    return;
  }

  p = line + 4;
  raw_vx = strtof(p, &p); if (*p == ',') p++;
  raw_vy = strtof(p, &p); if (*p == ',') p++;
  raw_vz = strtof(p, &p); if (*p == ',') p++;
  raw_ve = strtof(p, &p); if (*p == ',') p++;

  while (*p == ' ')
  {
    p++;
  }
  if ((*p != '\0') && (*p != '*'))
  {
    raw_status = (uint8_t)*p;
  }

  if ((dvlDataMutex != NULL) &&
      (xSemaphoreTake(dvlDataMutex, portMAX_DELAY) == pdTRUE))
  {
    dvlData.raw_vx = raw_vx;
    dvlData.raw_vy = raw_vy;
    dvlData.raw_vz = raw_vz;
    dvlData.raw_ve = raw_ve;
    dvlData.status = raw_status;
    if (raw_status != (uint8_t)'A')
    {
      dvlData.velocity_invalid_count++;
    }
    dvlData.frame_count++;
    dvlData.timestamp = HAL_GetTick();
    xSemaphoreGive(dvlDataMutex);
  }
}

static void DVL_RestartReceiveFromIsr(void)
{
  (void)HAL_UART_Receive_IT(&huart1, &dvlRxByte, 1U);
}
