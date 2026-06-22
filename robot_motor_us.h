/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Macros --------------------------------*/
#define PWM_PORT_CH1  GPIOA
#define PWM_PIN_CH1   GPIO_PIN_5

#define PWM_PORT_CH2  GPIOB
#define PWM_PIN_CH2   GPIO_PIN_3

#define IZQ_IN1_PORT  GPIOA
#define IZQ_IN1_PIN   GPIO_PIN_0
#define IZQ_IN2_PORT  GPIOA
#define IZQ_IN2_PIN   GPIO_PIN_1

#define DER_IN3_PORT  GPIOA
#define DER_IN3_PIN   GPIO_PIN_2
#define DER_IN4_PORT  GPIOA
#define DER_IN4_PIN   GPIO_PIN_3

// HC-SR04
#define TRG_PORT	GPIOB
#define TRG_PIN		GPIO_PIN_0
#define ECHO_PORT	GPIOB
#define ECHO_PIN	GPIO_PIN_6

#define ON GPIO_PIN_SET
#define OFF GPIO_PIN_RESET

// duration is in microseconds (TIM4 tick = 1us). Round trip: cm = us / 58.
#define _GetDistance(duration_us)	(((float)(duration_us)) / 58.0f) // [cm]

#define IC_TIMER TIM4
#define IC_PRESCALER 16
#define _ICPeriod 0xFFFF // tick = 1us at 16MHz/16

#define BASE_TIMER TIM3
#define BASE_PRESCALER 16000
#define _BasePeriod 1 // Period in [ms] non fractional

#define PWM_TIMER TIM2
#define PWM_PRESCALER 16000
#define _PWMPeriod 100 // Period in [ms] non fractional

#define OBSTACLE_DISTANCE_CM  20
#define STOP_TIME_MS 3000
#define TURN_TIME_MS 1000

#define LEFT   0
#define RIGHT  1


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
