#include "dvl.h"

#include "dvl_fusion.h"
#include "log_task.h"
#include "main.h"
#include "semphr.h"
#include "stream_buffer.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart5;
#define DVL_UART_HANDLE   huart5
#define DVL_UART_INSTANCE UART5

#define DVL_EVENT_RX_STARTED    (1U << 0)
#define DVL_EVENT_READY         (1U << 1)

#define DVL_RX_STREAM_SIZE      256U
#define DVL_RX_TASK_CHUNK_SIZE  32U

#define DVL_LINE_BUFFER_SIZE    96U
#define DVL_STARTUP_COMMAND_1    "PM"
#define DVL_STARTUP_COMMAND_2     "ML"
#define Append_CRLF(s) s"\r\n"
#define DVL_STARTUP_TX_TIMEOUT_MS 100U
#define DVL_RX_START_TIMEOUT_MS 5000U
#define DVL_RAW_CAPTURE_BUFFER_SIZE 24576U
#define DVL_RAW_CAPTURE_LOG_DELAY_MS 20U
#define DVL_RAW_CAPTURE_FORWARD_CHUNK_SIZE 128U



static void DVL_Task(void *argument);
static void DVL_ParseByte(uint8_t byte);
static void DVL_ParseLine(char *line);
static void DVL_RawCaptureStart(void);
static void DVL_RawCaptureStop(void);
static uint8_t DVL_RawCaptureIsActive(void);
static void DVL_RawCaptureOnByte(uint8_t byte);
static void DVL_RawCaptureDump(void);
static BaseType_t DVL_SendStartupCommand(const uint8_t *command,
                                         const char *label);
static void DVL_RestartReceiveFromIsr(void);

static DVL_Data_t dvlData;
static SemaphoreHandle_t dvlDataMutex;
static StreamBufferHandle_t dvlRxStream;
static osThreadId_t dvlTaskHandle;
static osEventFlagsId_t dvlEventFlags;
static uint8_t dvlRxByte;
static uint8_t dvlConsecutiveValidFrames;
static uint8_t dvlRawCaptureBuffer[DVL_RAW_CAPTURE_BUFFER_SIZE];
static volatile uint8_t dvlRawCaptureActive;
static volatile uint32_t dvlRawCaptureLength;
static volatile uint32_t dvlRawCaptureDropped;

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

  dvlEventFlags = osEventFlagsNew(NULL);
  if (dvlEventFlags == NULL)
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

BaseType_t DVL_ConfigureStartup(void)
{
  uint32_t flags;
  const uint8_t startupCommand_1[] = Append_CRLF(DVL_STARTUP_COMMAND_1);
  const uint8_t startupCommand_2[] = Append_CRLF(DVL_STARTUP_COMMAND_2);

  if (dvlEventFlags == NULL)
  {
    return pdFAIL;
  }

  flags = osEventFlagsWait(dvlEventFlags,
                           DVL_EVENT_RX_STARTED,
                           osFlagsWaitAny,
                           pdMS_TO_TICKS(DVL_RX_START_TIMEOUT_MS));
  if ((flags & osFlagsError) != 0U)
  {
    return pdFAIL;
  }

  osDelay(3000);
  DVL_RawCaptureStart();
  dvlConsecutiveValidFrames = 0U;
  (void)osEventFlagsClear(dvlEventFlags, DVL_EVENT_READY);

  if (DVL_SendStartupCommand(startupCommand_1,
                             DVL_STARTUP_COMMAND_1) != pdPASS)
  {
    return pdFAIL;
  }
	osDelay(1000);
  if (DVL_SendStartupCommand(startupCommand_2,
                             DVL_STARTUP_COMMAND_2) != pdPASS)
  {
    return pdFAIL;
  }
	osDelay(1000);
	DVL_RawCaptureStop();
  DVL_RawCaptureDump();
  return pdPASS;
}

BaseType_t DVL_WaitReady(TickType_t timeout_ticks)
{
  uint32_t flags;

  if (dvlEventFlags == NULL)
  {
    return pdFAIL;
  }

  flags = osEventFlagsWait(dvlEventFlags,
                           DVL_EVENT_READY,
                           osFlagsWaitAny,
                           timeout_ticks);
  if ((flags & osFlagsError) != 0U)
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
  if (huart->Instance == DVL_UART_INSTANCE)
  {
    BaseType_t localTaskWoken = pdFALSE;
    BaseType_t *taskWoken = (higher_priority_task_woken != NULL) ?
                            higher_priority_task_woken :
                            &localTaskWoken;

    if (dvlRxStream != NULL)
    {
      if (xStreamBufferSendFromISR(dvlRxStream,
                                   &dvlRxByte,
                                   1U,
                                   taskWoken) != 1U)
      {
        if (dvlRawCaptureActive != 0U)
        {
          dvlRawCaptureDropped++;
        }
      }
    }
    else if (dvlRawCaptureActive != 0U)
    {
      dvlRawCaptureDropped++;
    }

    DVL_RestartReceiveFromIsr();
  }
}

void DVL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == DVL_UART_INSTANCE)
  {
    DVL_RestartReceiveFromIsr();
  }
}

static void DVL_Task(void *argument)
{
  uint8_t rxChunk[DVL_RX_TASK_CHUNK_SIZE];

  (void)argument;

  while (HAL_UART_Receive_IT(&DVL_UART_HANDLE, &dvlRxByte, 1U) != HAL_OK)
  {
    osDelay(10U);
  }
  (void)osEventFlagsSet(dvlEventFlags, DVL_EVENT_RX_STARTED);

  for (;;)
  {
    size_t receivedLength = xStreamBufferReceive(dvlRxStream,
                                                 rxChunk,
                                                 sizeof(rxChunk),
                                                 portMAX_DELAY);
    size_t i;

    for (i = 0U; i < receivedLength; i++)
    {
      DVL_RawCaptureOnByte(rxChunk[i]);
      DVL_ParseByte(rxChunk[i]);
    }
  }
}

static void DVL_ParseByte(uint8_t byte)
{
  static char line[DVL_LINE_BUFFER_SIZE];
  static uint8_t index;

  if ((byte == (uint8_t)':') &&
      ((index == 0U) || (DVL_RawCaptureIsActive() == 0U)))
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

  if ((index == 0U) && (DVL_RawCaptureIsActive() == 0U))
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
  uint8_t validFrame;
  uint8_t haveDvlSnapshot = 0U;
  DVL_Data_t dvlSnapshot;
  uint32_t now;

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

  validFrame = ((raw_status == (uint8_t)'A') &&
                (raw_vx != 88888.0f) &&
                (raw_vy != 88888.0f)) ? 1U : 0U;

  if ((dvlDataMutex != NULL) &&
      (xSemaphoreTake(dvlDataMutex, portMAX_DELAY) == pdTRUE))
  {
    now = HAL_GetTick();
    dvlData.raw_vx = raw_vx;
    dvlData.raw_vy = raw_vy;
    dvlData.raw_vz = raw_vz;
    dvlData.raw_ve = raw_ve;
    dvlData.status = raw_status;
    dvlData.frame_count++;
    dvlData.timestamp = now;
    dvlSnapshot = dvlData;
    haveDvlSnapshot = 1U;
    xSemaphoreGive(dvlDataMutex);
  }

  if (haveDvlSnapshot != 0U)
  {
    (void)DVL_Fusion_SubmitDvlData(&dvlSnapshot);
  }

  if (validFrame != 0U)
  {
    if (dvlConsecutiveValidFrames < DVL_STARTUP_VALID_FRAME_COUNT)
    {
      dvlConsecutiveValidFrames++;
    }

    if (dvlConsecutiveValidFrames >= DVL_STARTUP_VALID_FRAME_COUNT)
    {
      (void)osEventFlagsSet(dvlEventFlags, DVL_EVENT_READY);
    }
  }
  else
  {
    dvlConsecutiveValidFrames = 0U;
  }
}

static void DVL_RawCaptureStart()
{
  dvlRawCaptureLength = 0U;
  dvlRawCaptureDropped = 0U;
  dvlRawCaptureActive = 1U;
}

static void DVL_RawCaptureStop(void)
{
  dvlRawCaptureActive = 0U;
}

static uint8_t DVL_RawCaptureIsActive(void)
{
  return dvlRawCaptureActive;
}

static void DVL_RawCaptureOnByte(uint8_t byte)
{
  uint32_t index;

  if (dvlRawCaptureActive == 0U)
  {
    return;
  }

  index = dvlRawCaptureLength;
  if (index < DVL_RAW_CAPTURE_BUFFER_SIZE)
  {
    dvlRawCaptureBuffer[index] = byte;
    dvlRawCaptureLength = index + 1U;
  }
  else
  {
    dvlRawCaptureDropped++;
  }
}

static void DVL_RawCaptureDump(void)
{
  uint32_t length = dvlRawCaptureLength;
  uint32_t dropped = dvlRawCaptureDropped;
  uint32_t offset;

  Log_printf("[DVLRAW] len=%lu dropped=%lu\r\n",
             (unsigned long)length,
             (unsigned long)dropped);
  osDelay(DVL_RAW_CAPTURE_LOG_DELAY_MS);

  for (offset = 0U; offset < length; offset += DVL_RAW_CAPTURE_FORWARD_CHUNK_SIZE)
  {
    uint32_t count = length - offset;
    uint32_t i;
    char chunk[DVL_RAW_CAPTURE_FORWARD_CHUNK_SIZE];

    if (count > DVL_RAW_CAPTURE_FORWARD_CHUNK_SIZE)
    {
      count = DVL_RAW_CAPTURE_FORWARD_CHUNK_SIZE;
    }

    for (i = 0U; i < count; i++)
    {
      uint8_t byte = dvlRawCaptureBuffer[offset + i];
      chunk[i] = (byte == 0U) ? '!' : (char)byte;
    }

    (void)Log_write(chunk, count);
    osDelay(DVL_RAW_CAPTURE_LOG_DELAY_MS);
  }

  Log_printf("\r\n[DVLRAW_END]\r\n");
  osDelay(DVL_RAW_CAPTURE_LOG_DELAY_MS);
}

static BaseType_t DVL_SendStartupCommand(const uint8_t *command,
                                         const char *label)
{
  if ((command == NULL) || (label == NULL))
  {
    return pdFAIL;
  }
  Log_printf("[DVLConfig]Send %s\r\n", label);
  
  if (HAL_UART_Transmit(&DVL_UART_HANDLE,
                        (uint8_t *)command,
                        sizeof(command)-1,
                        DVL_STARTUP_TX_TIMEOUT_MS) != HAL_OK)
  {
    DVL_RawCaptureStop();
    DVL_RawCaptureDump();
    Log_printf("[DVLConfig]%s fail\r\n", label);
    return pdFAIL;
  }

  Log_printf("[DVLConfig]%s success\r\n", label);

  return pdPASS;
}

static void DVL_RestartReceiveFromIsr(void)
{
  (void)HAL_UART_Receive_IT(&DVL_UART_HANDLE, &dvlRxByte, 1U);
}
