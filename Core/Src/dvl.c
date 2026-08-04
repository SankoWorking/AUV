#include "dvl.h"

#include "dvl_fusion.h"
#include "log_task.h"
#include "main.h"
#include "semphr.h"
#include "stream_buffer.h"
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart5;
#define DVL_UART_HANDLE   huart5
#define DVL_UART_INSTANCE UART5

#define DVL_EVENT_RX_STARTED    (1U << 0)
#define DVL_EVENT_READY         (1U << 1)
#define DVL_EVENT_STARTUP_ACK   (1U << 2)

#define DVL_RX_STREAM_SIZE      256U
#define DVL_RX_TASK_CHUNK_SIZE  32U

#define DVL_LINE_BUFFER_SIZE    96U
#define DVL_STARTUP_COMMAND     "ML 800,SV 0\r\n"
#define DVL_STARTUP_ACK_CHECK_ENABLE 0U
#define DVL_STARTUP_TX_TIMEOUT_MS 100U
#define DVL_STARTUP_FORWARD_WINDOW_MS 10000U
#define DVL_RX_START_TIMEOUT_MS 2000U
#define DVL_STARTUP_ACK_PREFIX  "ML "
#define DVL_STARTUP_ACK_VALUE   600.0f
#define DVL_STARTUP_ACK_EPSILON 0.1f



static void DVL_Task(void *argument);
static void DVL_ParseByte(uint8_t byte);
static void DVL_ParseLine(char *line);
#if (DVL_STARTUP_ACK_CHECK_ENABLE != 0U)
static uint8_t DVL_ParseStartupAck(char *line);
#endif
static void DVL_RestartReceiveFromIsr(void);

static DVL_Data_t dvlData;
static SemaphoreHandle_t dvlDataMutex;
static StreamBufferHandle_t dvlRxStream;
static osThreadId_t dvlTaskHandle;
static osEventFlagsId_t dvlEventFlags;
static uint8_t dvlRxByte;
static uint8_t dvlConsecutiveValidFrames;
static volatile uint8_t dvlWaitingStartupAck;
static volatile uint8_t dvlForwardStartupLines;

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
  const uint8_t startupCommand[] = DVL_STARTUP_COMMAND;

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

  dvlConsecutiveValidFrames = 0U;
#if (DVL_STARTUP_ACK_CHECK_ENABLE != 0U)
  (void)osEventFlagsClear(dvlEventFlags, DVL_EVENT_READY | DVL_EVENT_STARTUP_ACK);
  dvlWaitingStartupAck = 1U;
  dvlForwardStartupLines = 1U;
#else
  (void)osEventFlagsClear(dvlEventFlags, DVL_EVENT_READY);
  dvlWaitingStartupAck = 0U;
  dvlForwardStartupLines = 0U;
#endif

  if (HAL_UART_Transmit(&DVL_UART_HANDLE,
                        (uint8_t *)startupCommand,
                        (uint16_t)(sizeof(startupCommand) - 1U),
                        DVL_STARTUP_TX_TIMEOUT_MS) != HAL_OK)
  {
    dvlForwardStartupLines = 0U;
    dvlWaitingStartupAck = 0U;
    return pdFAIL;
  }

#if (DVL_STARTUP_ACK_CHECK_ENABLE != 0U)
  osDelay(DVL_STARTUP_FORWARD_WINDOW_MS);
  dvlForwardStartupLines = 0U;
  dvlWaitingStartupAck = 0U;

  flags = osEventFlagsGet(dvlEventFlags);
  if ((flags & DVL_EVENT_STARTUP_ACK) == 0U)
  {
    Log_printf("[DVL] startup_ack result=timeout expected=ML %.1f\r\n",
               DVL_STARTUP_ACK_VALUE);
    return pdFAIL;
  }

  Log_printf("[DVL] startup_ack result=ok\r\n");
#else
  Log_printf("[DVL] startup_ack check=disabled\r\n");
#endif

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
  uint8_t validFrame;
  uint8_t haveDvlSnapshot = 0U;
  DVL_Data_t dvlSnapshot;
  uint32_t now;

  if ((line == NULL) || (line[0] == '\0'))
  {
    return;
  }

  if (dvlForwardStartupLines != 0U)
  {
    Log_printf("[DVL CONFIG]%s\r\n", line);
  }

#if (DVL_STARTUP_ACK_CHECK_ENABLE != 0U)
  if ((dvlWaitingStartupAck != 0U) && (DVL_ParseStartupAck(line) != 0U))
  {
    return;
  }
#endif

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
#if (DVL_STARTUP_ACK_CHECK_ENABLE != 0U)
static uint8_t DVL_ParseStartupAck(char *line)
{
  char *p;
  float ackValue;

  while (*line == ':')
  {
    line++;
  }

  if (strncmp(line, DVL_STARTUP_ACK_PREFIX, strlen(DVL_STARTUP_ACK_PREFIX)) != 0)
  {
    return 0U;
  }

  p = line + strlen(DVL_STARTUP_ACK_PREFIX);
  ackValue = strtof(p, &p);

  if ((ackValue > (DVL_STARTUP_ACK_VALUE - DVL_STARTUP_ACK_EPSILON)) &&
      (ackValue < (DVL_STARTUP_ACK_VALUE + DVL_STARTUP_ACK_EPSILON)))
  {
    (void)osEventFlagsSet(dvlEventFlags, DVL_EVENT_STARTUP_ACK);
    return 1U;
  }

  return 0U;
}
#endif

static void DVL_RestartReceiveFromIsr(void)
{
  (void)HAL_UART_Receive_IT(&DVL_UART_HANDLE, &dvlRxByte, 1U);
}
