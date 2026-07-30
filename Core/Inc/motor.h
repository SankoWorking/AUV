#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifndef MOTOR_ROLLER_FORWARD_LEVEL
#define MOTOR_ROLLER_FORWARD_LEVEL GPIO_PIN_SET
#endif

#ifndef MOTOR_RIGHT_FORWARD_LEVEL
#define MOTOR_RIGHT_FORWARD_LEVEL GPIO_PIN_SET
#endif

#ifndef MOTOR_LEFT_FORWARD_LEVEL	
#define MOTOR_LEFT_FORWARD_LEVEL GPIO_PIN_RESET
#endif

#define MOTOR_TOP_SUCTION_STOP_PULSE_US 1000U
#define MOTOR_TOP_SUCTION_MAX_PULSE_US  2000U

HAL_StatusTypeDef Motor_Init(void);

void Motor_Stop(void);
void Motor_Forward(uint8_t speed_percent);
void Motor_Backward(uint8_t speed_percent);
void Motor_SetDrive(int16_t left_percent, int16_t right_percent);

void Motor_SetFrontRoller(int16_t speed_percent);
void Motor_SetTopSuction(uint8_t speed_percent);
void Motor_SetTopSuctionPulseUs(uint16_t pulse_us);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
