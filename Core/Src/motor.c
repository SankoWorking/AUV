#include "motor.h"

#include "main.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

#define MOTOR_PWM_CHANNEL TIM_CHANNEL_1

typedef struct
{
  TIM_HandleTypeDef *htim;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  GPIO_PinState forward_level;
} Motor_DirectionalDef;

static Motor_DirectionalDef frontRollerMotor = {
  &htim13,
  MOTOR_ROLLER_DIR_GPIO_Port,
  MOTOR_ROLLER_DIR_Pin,
  MOTOR_ROLLER_FORWARD_LEVEL,
};

static Motor_DirectionalDef rightMotor = {
  &htim17,
  MOTOR_RIGHT_DIR_GPIO_Port,
  MOTOR_RIGHT_DIR_Pin,
  MOTOR_RIGHT_FORWARD_LEVEL,
};

static Motor_DirectionalDef leftMotor = {
  &htim16,
  MOTOR_LEFT_DIR_GPIO_Port,
  MOTOR_LEFT_DIR_Pin,
  MOTOR_LEFT_FORWARD_LEVEL,
};

static uint8_t Motor_ClampPercent(uint8_t percent)
{
  return (percent > 100U) ? 100U : percent;
}

static uint8_t Motor_AbsClampPercent(int16_t percent)
{
  if (percent >= 100)
  {
    return 100U;
  }

  if (percent <= -100)
  {
    return 100U;
  }

  if (percent < 0)
  {
    return (uint8_t)(-percent);
  }

  return (uint8_t)percent;
}

static GPIO_PinState Motor_ReverseLevel(GPIO_PinState level)
{
  return (level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static uint32_t Motor_PercentToPulse(TIM_HandleTypeDef *htim, uint8_t percent)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;

  percent = Motor_ClampPercent(percent);

  return (period * (uint32_t)percent) / 100U;
}

static void Motor_SetPwm(TIM_HandleTypeDef *htim, uint8_t speed_percent)
{
  __HAL_TIM_SET_COMPARE(htim,
                        MOTOR_PWM_CHANNEL,
                        Motor_PercentToPulse(htim, speed_percent));
}

static uint16_t Motor_TopSuctionPercentToPulseUs(uint8_t speed_percent)
{
  uint32_t pulseRange = MOTOR_TOP_SUCTION_MAX_PULSE_US - MOTOR_TOP_SUCTION_STOP_PULSE_US;

  speed_percent = Motor_ClampPercent(speed_percent);

  return (uint16_t)(MOTOR_TOP_SUCTION_STOP_PULSE_US +
                    ((pulseRange * (uint32_t)speed_percent) / 100U));
}

static void Motor_SetDirectional(const Motor_DirectionalDef *motor, int16_t speed_percent)
{
  uint8_t absSpeed = Motor_AbsClampPercent(speed_percent);

  if (speed_percent > 0)
  {
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, motor->forward_level);
  }
  else if (speed_percent < 0)
  {
    HAL_GPIO_WritePin(motor->dir_port,
                      motor->dir_pin,
                      Motor_ReverseLevel(motor->forward_level));
  }

  Motor_SetPwm(motor->htim, absSpeed);
}

static HAL_StatusTypeDef Motor_StartPwm(TIM_HandleTypeDef *htim)
{
  HAL_StatusTypeDef status;

  Motor_SetPwm(htim, 0U);
  status = HAL_TIM_PWM_Start(htim, MOTOR_PWM_CHANNEL);

  return status;
}

HAL_StatusTypeDef Motor_Init(void)
{
  if (Motor_StartPwm(&htim13) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (Motor_StartPwm(&htim17) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (Motor_StartPwm(&htim16) != HAL_OK)
  {
    return HAL_ERROR;
  }

  Motor_SetTopSuction(0U);
  if (HAL_TIM_PWM_Start(&htim4, MOTOR_PWM_CHANNEL) != HAL_OK)
  {
    return HAL_ERROR;
  }

  Motor_Stop();
  Motor_SetFrontRoller(0);

  return HAL_OK;
}

void Motor_Stop(void)
{
  Motor_SetPwm(&htim16, 0U);
  Motor_SetPwm(&htim17, 0U);
  Motor_SetPwm(&htim13, 0U);
  Motor_SetTopSuction(0U);
}

void Motor_Forward(uint8_t speed_percent)
{
  uint8_t speed = Motor_ClampPercent(speed_percent);

  Motor_SetDrive((int16_t)speed, (int16_t)speed);
}

void Motor_Backward(uint8_t speed_percent)
{
  int16_t speed = (int16_t)Motor_ClampPercent(speed_percent);

  Motor_SetDrive((int16_t)-speed, (int16_t)-speed);
}

void Motor_SetDrive(int16_t left_percent, int16_t right_percent)
{
  Motor_SetDirectional(&leftMotor, left_percent);
  Motor_SetDirectional(&rightMotor, right_percent);
}

void Motor_SetFrontRoller(int16_t speed_percent)
{
  Motor_SetDirectional(&frontRollerMotor, speed_percent);
}

void Motor_SetTopSuction(uint8_t speed_percent)
{
  Motor_SetTopSuctionPulseUs(Motor_TopSuctionPercentToPulseUs(speed_percent));
}

void Motor_SetTopSuctionPulseUs(uint16_t pulse_us)
{
  uint32_t autoreload = __HAL_TIM_GET_AUTORELOAD(&htim4);

  if (pulse_us < MOTOR_TOP_SUCTION_STOP_PULSE_US)
  {
    pulse_us = MOTOR_TOP_SUCTION_STOP_PULSE_US;
  }
  else if (pulse_us > MOTOR_TOP_SUCTION_MAX_PULSE_US)
  {
    pulse_us = MOTOR_TOP_SUCTION_MAX_PULSE_US;
  }

  if ((uint32_t)pulse_us > autoreload)
  {
    pulse_us = (uint16_t)autoreload;
  }

  __HAL_TIM_SET_COMPARE(&htim4, MOTOR_PWM_CHANNEL, pulse_us);
}
