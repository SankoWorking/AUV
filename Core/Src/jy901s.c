#include "jy901s.h"

#include "cmsis_os2.h"
#include "dvl.h"
#include "main.h"
#include "semphr.h"
#include "stream_buffer.h"

#define JY901S_FRAME_HEADER        0x55U
#define JY901S_FRAME_LENGTH        11U
#define JY901S_FRAME_TYPE_ACC      0x51U
#define JY901S_FRAME_TYPE_GYRO     0x52U
#define JY901S_FRAME_TYPE_ANGLE    0x53U
#define JY901S_RX_STREAM_SIZE      128U
#define JY901S_RX_TASK_CHUNK_SIZE  32U

extern UART_HandleTypeDef huart7;

static void JY901S_Task(void *argument);
static void JY901S_ParseByte(uint8_t byte);
static void JY901S_ParseFrame(const uint8_t *frame);
static void JY901S_RestartReceiveFromIsr(void);
void DVL_UART_RxCpltCallback(UART_HandleTypeDef *huart,
                             BaseType_t *higher_priority_task_woken);
void DVL_UART_ErrorCallback(UART_HandleTypeDef *huart);

static JY901S_Data_t jy901sData;
static SemaphoreHandle_t jy901sDataMutex;
static StreamBufferHandle_t jy901sRxStream;
static osThreadId_t jy901sTaskHandle;
static uint8_t jy901sRxByte;

static volatile uint32_t jy901sRxDropCount;
static volatile uint32_t jy901sUartErrorCount;
static volatile uint32_t jy901sRxRestartErrorCount;

static const osThreadAttr_t jy901sTaskAttributes = {
  .name = "jy901s_task",
  .stack_size = JY901S_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t)JY901S_TASK_PRIORITY,
};

static int16_t JY901S_FrameToInt16(const uint8_t *frame, uint8_t low_index)
{
  uint16_t value = (uint16_t)frame[low_index] |
                   ((uint16_t)frame[low_index + 1U] << 8U);

  return (int16_t)value;
}

static uint8_t JY901S_IsKnownFrameType(uint8_t frame_type)
{
  return ((frame_type == JY901S_FRAME_TYPE_ACC) ||
          (frame_type == JY901S_FRAME_TYPE_GYRO) ||
          (frame_type == JY901S_FRAME_TYPE_ANGLE));
}

static uint8_t JY901S_ChecksumValid(const uint8_t *frame)
{
  uint8_t checksum = 0U;
  uint8_t i;

  for (i = 0U; i < (JY901S_FRAME_LENGTH - 1U); i++)
  {
    checksum = (uint8_t)(checksum + frame[i]);
  }

  return (checksum == frame[JY901S_FRAME_LENGTH - 1U]);
}

static void JY901S_CopyCounters(JY901S_Data_t *data)
{
  data->rx_drop_count = jy901sRxDropCount;
  data->uart_error_count = jy901sUartErrorCount + jy901sRxRestartErrorCount;
}

BaseType_t JY901S_Init(void)
{
  if (jy901sTaskHandle != NULL)
  {
    return pdPASS;
  }

  jy901sDataMutex = xSemaphoreCreateMutex();
  if (jy901sDataMutex == NULL)
  {
    return pdFAIL;
  }

  jy901sRxStream = xStreamBufferCreate(JY901S_RX_STREAM_SIZE, 1U);
  if (jy901sRxStream == NULL)
  {
    return pdFAIL;
  }

  jy901sTaskHandle = osThreadNew(JY901S_Task, NULL, &jy901sTaskAttributes);
  if (jy901sTaskHandle == NULL)
  {
    return pdFAIL;
  }

  return pdPASS;
}

BaseType_t JY901S_GetData(JY901S_Data_t *data)
{
  return JY901S_GetDataTimeout(data, 0U);
}

BaseType_t JY901S_GetDataTimeout(JY901S_Data_t *data, TickType_t timeout_ticks)
{
  if ((data == NULL) || (jy901sDataMutex == NULL))
  {
    return pdFAIL;
  }

  if (xSemaphoreTake(jy901sDataMutex, timeout_ticks) != pdTRUE)
  {
    return pdFAIL;
  }

  *data = jy901sData;
  JY901S_CopyCounters(data);
  xSemaphoreGive(jy901sDataMutex);

  return pdPASS;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if (huart->Instance == UART7)
  {
    if ((jy901sRxStream == NULL) ||
        (xStreamBufferSendFromISR(jy901sRxStream,
                                  &jy901sRxByte,
                                  1U,
                                  &higherPriorityTaskWoken) != 1U))
    {
      jy901sRxDropCount++;
    }

    JY901S_RestartReceiveFromIsr();
  }

  DVL_UART_RxCpltCallback(huart, &higherPriorityTaskWoken);
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART7)
  {
    jy901sUartErrorCount++;
    JY901S_RestartReceiveFromIsr();
  }

  DVL_UART_ErrorCallback(huart);
}

static void JY901S_Task(void *argument)
{
  uint8_t rxChunk[JY901S_RX_TASK_CHUNK_SIZE];

  (void)argument;

  while (HAL_UART_Receive_IT(&huart7, &jy901sRxByte, 1U) != HAL_OK)
  {
    osDelay(10U);
  }

  for (;;)
  {
    size_t receivedLength = xStreamBufferReceive(jy901sRxStream,
                                                 rxChunk,
                                                 sizeof(rxChunk),
                                                 portMAX_DELAY);
    size_t i;

    for (i = 0U; i < receivedLength; i++)
    {
      JY901S_ParseByte(rxChunk[i]);
    }
  }
}

static void JY901S_ParseByte(uint8_t byte)
{
  static uint8_t frame[JY901S_FRAME_LENGTH];
  static uint8_t index;

  if (index == 0U)
  {
    if (byte != JY901S_FRAME_HEADER)
    {
      return;
    }

    frame[index++] = byte;
    return;
  }

  if ((index == 1U) && (JY901S_IsKnownFrameType(byte) == 0U))
  {
    index = (byte == JY901S_FRAME_HEADER) ? 1U : 0U;
    frame[0] = JY901S_FRAME_HEADER;
    return;
  }

  frame[index++] = byte;

  if (index >= JY901S_FRAME_LENGTH)
  {
    if (JY901S_ChecksumValid(frame) != 0U)
    {
      JY901S_ParseFrame(frame);
    }
    else if (jy901sDataMutex != NULL)
    {
      if (xSemaphoreTake(jy901sDataMutex, portMAX_DELAY) == pdTRUE)
      {
        jy901sData.checksum_error_count++;
        xSemaphoreGive(jy901sDataMutex);
      }
    }

    index = 0U;
  }
}

static void JY901S_ParseFrame(const uint8_t *frame)
{
  int16_t x = JY901S_FrameToInt16(frame, 2U);
  int16_t y = JY901S_FrameToInt16(frame, 4U);
  int16_t z = JY901S_FrameToInt16(frame, 6U);
  int16_t temperature = JY901S_FrameToInt16(frame, 8U);
  uint32_t timestamp = HAL_GetTick();

  if ((jy901sDataMutex == NULL) ||
      (xSemaphoreTake(jy901sDataMutex, portMAX_DELAY) != pdTRUE))
  {
    return;
  }

  switch (frame[1])
  {
    case JY901S_FRAME_TYPE_ACC:
      jy901sData.acc_g[0] = ((float)x / 32768.0f) * 16.0f;
      jy901sData.acc_g[1] = ((float)y / 32768.0f) * 16.0f;
      jy901sData.acc_g[2] = ((float)z / 32768.0f) * 16.0f;
      jy901sData.acc_valid = 1U;
      break;

    case JY901S_FRAME_TYPE_GYRO:
      jy901sData.gyro_dps[0] = ((float)x / 32768.0f) * 2000.0f;
      jy901sData.gyro_dps[1] = ((float)y / 32768.0f) * 2000.0f;
      jy901sData.gyro_dps[2] = ((float)z / 32768.0f) * 2000.0f;
      jy901sData.gyro_valid = 1U;
      break;

    case JY901S_FRAME_TYPE_ANGLE:
      jy901sData.angle_deg[0] = ((float)x / 32768.0f) * 180.0f;
      jy901sData.angle_deg[1] = ((float)y / 32768.0f) * 180.0f;
      jy901sData.angle_deg[2] = ((float)z / 32768.0f) * 180.0f;
      jy901sData.angle_valid = 1U;
      break;

    default:
      break;
  }

  jy901sData.temperature_raw = temperature;
  jy901sData.timestamp_ms = timestamp;
  jy901sData.frame_count++;
  xSemaphoreGive(jy901sDataMutex);
}

static void JY901S_RestartReceiveFromIsr(void)
{
  if (HAL_UART_Receive_IT(&huart7, &jy901sRxByte, 1U) != HAL_OK)
  {
    jy901sRxRestartErrorCount++;
  }
}
