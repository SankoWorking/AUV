#include "dvl.h"

#include "main.h"
#include "semphr.h"
#include "stream_buffer.h"

#define DVL_RX_STREAM_SIZE      256U
#define DVL_RX_TASK_CHUNK_SIZE  32U
#define DVL_LINE_BUFFER_SIZE    96U
#define DVL_FIELD_COUNT         6U

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

static volatile uint32_t dvlRxDropCount;
static volatile uint32_t dvlUartErrorCount;
static volatile uint32_t dvlRxRestartErrorCount;

static const osThreadAttr_t dvlTaskAttributes = {
  .name = "dvl_task",
  .stack_size = DVL_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)DVL_TASK_PRIORITY,
};

static int32_t DVL_ParseSignedMmS(const char *text, uint8_t *ok)
{
  int32_t sign = 1;
  int32_t value = 0;
  uint8_t digitCount = 0U;

  if ((text == NULL) || (ok == NULL))
  {
    return 0;
  }

  *ok = 0U;

  if (*text == '-')
  {
    sign = -1;
    text++;
  }
  else if (*text == '+')
  {
    text++;
  }

  while ((*text >= '0') && (*text <= '9'))
  {
    value = (value * 10) + (int32_t)(*text - '0');
    digitCount++;
    text++;
  }

  if ((*text == '\0') && (digitCount > 0U))
  {
    *ok = 1U;
  }

  return value * sign;
}

static void DVL_CopyCounters(DVL_Data_t *data)
{
  data->rx_drop_count = dvlRxDropCount;
  data->uart_error_count = dvlUartErrorCount + dvlRxRestartErrorCount;
}

static void DVL_IncrementParseError(void)
{
  if ((dvlDataMutex != NULL) &&
      (xSemaphoreTake(dvlDataMutex, portMAX_DELAY) == pdTRUE))
  {
    dvlData.parse_error_count++;
    xSemaphoreGive(dvlDataMutex);
  }
}

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
  DVL_CopyCounters(data);
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

    if ((dvlRxStream == NULL) ||
        (xStreamBufferSendFromISR(dvlRxStream,
                                  &dvlRxByte,
                                  1U,
                                  taskWoken) != 1U))
    {
      dvlRxDropCount++;
    }

    DVL_RestartReceiveFromIsr();
  }
}

void DVL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    dvlUartErrorCount++;
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

  if (byte == '\r')
  {
    return;
  }

  if (byte == '\n')
  {
    line[index] = '\0';
    DVL_ParseLine(line);
    index = 0U;
    return;
  }

  if (index >= (DVL_LINE_BUFFER_SIZE - 1U))
  {
    index = 0U;
    DVL_IncrementParseError();
    return;
  }

  line[index++] = (char)byte;
}

static void DVL_ParseLine(char *line)
{
  char *fields[DVL_FIELD_COUNT];
  char *cursor = line;
  uint8_t fieldIndex = 0U;
  uint8_t ok = 0U;
  DVL_Data_t parsedData = {0};

  if ((line == NULL) || (line[0] == '\0'))
  {
    return;
  }

  while ((fieldIndex < DVL_FIELD_COUNT) && (cursor != NULL))
  {
    char *comma = cursor;

    fields[fieldIndex++] = cursor;

    while ((*comma != ',') && (*comma != '\0'))
    {
      comma++;
    }

    if (*comma == ',')
    {
      *comma = '\0';
      cursor = comma + 1;
    }
    else
    {
      cursor = NULL;
    }
  }

  if ((fieldIndex != DVL_FIELD_COUNT) ||
      (cursor != NULL) ||
      (fields[0][0] != ':') ||
      (fields[0][1] != 'B') ||
      (fields[0][2] != 'I') ||
      (fields[0][3] != '\0') ||
      (fields[5][1] != '\0') ||
      ((fields[5][0] != 'A') && (fields[5][0] != 'V')))
  {
    if ((line[0] == ':') && (line[1] == 'B') && (line[2] == 'I'))
    {
      DVL_IncrementParseError();
    }
    return;
  }

  parsedData.velocity_mm_s[0] = DVL_ParseSignedMmS(fields[1], &ok);
  if (ok == 0U)
  {
    DVL_IncrementParseError();
    return;
  }

  parsedData.velocity_mm_s[1] = DVL_ParseSignedMmS(fields[2], &ok);
  if (ok == 0U)
  {
    DVL_IncrementParseError();
    return;
  }

  parsedData.velocity_mm_s[2] = DVL_ParseSignedMmS(fields[3], &ok);
  if (ok == 0U)
  {
    DVL_IncrementParseError();
    return;
  }

  parsedData.velocity_error_mm_s = DVL_ParseSignedMmS(fields[4], &ok);
  if (ok == 0U)
  {
    DVL_IncrementParseError();
    return;
  }

  parsedData.timestamp_ms = HAL_GetTick();
  parsedData.velocity_valid = (fields[5][0] == 'A') ? 1U : 0U;

  if ((dvlDataMutex != NULL) &&
      (xSemaphoreTake(dvlDataMutex, portMAX_DELAY) == pdTRUE))
  {
    uint32_t parseErrorCount = dvlData.parse_error_count;
    uint32_t invalidVelocityCount = dvlData.invalid_velocity_count;

    dvlData = parsedData;
    dvlData.parse_error_count = parseErrorCount;
    dvlData.invalid_velocity_count = invalidVelocityCount;
    if (dvlData.velocity_valid == 0U)
    {
      dvlData.invalid_velocity_count++;
    }
    dvlData.frame_count++;
    xSemaphoreGive(dvlDataMutex);
  }
}

static void DVL_RestartReceiveFromIsr(void)
{
  if (HAL_UART_Receive_IT(&huart1, &dvlRxByte, 1U) != HAL_OK)
  {
    dvlRxRestartErrorCount++;
  }
}
