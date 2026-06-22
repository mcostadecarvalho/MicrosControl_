/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

typedef enum{
	Espera_Flanco_Ascend = 0,
	Espera_Flanco_Descend = 1,
	Fin_Captura = 2
}IC_State;

typedef enum{
	Origin = 0,
	Triggering = 1,
	Capturing = 2
}HC_State;

typedef enum
{
    HW_INIT,
    AVANCE,
    PARADA,
    GIRO_IZQ,
    GIRO_DER
} Robot_State;

volatile IC_State ic_state;
volatile uint16_t duration;
volatile uint16_t rval;
volatile uint16_t fval;
volatile uint32_t counter_parada = 0;
volatile uint32_t counter_giro = 0;
volatile uint8_t distance_valid = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

static void Ultrasonic_Update(float *distance_cm, uint8_t enabled);
static void Motor_SetSpeedPercent(uint32_t percent);
static void Motor_Forward(void);
static void Motor_Stop(void);
static void Motor_Turn_Left(void);
static void Motor_Turn_Right(void);
static void Robot_TransitionTo(Robot_State *state, Robot_State next_state);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  Robot_State robot_state = HW_INIT;
  float distance_cm = 0;
  uint32_t dist_cm_int = 0;

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

  Motor_Stop();
  ic_state = Fin_Captura;


  while (1)
  {
    Ultrasonic_Update(&distance_cm, ((robot_state == AVANCE) || (robot_state == HW_INIT)) ? 1U : 0U);

    switch(robot_state){
      case HW_INIT:

        Motor_Stop();

        if (distance_valid)
        {
          Robot_TransitionTo(&robot_state, AVANCE);
        }
        break;

      case AVANCE:
        
        Motor_Forward();

        if(distance_cm <= OBSTACLE_DISTANCE_CM){
          Robot_TransitionTo(&robot_state, PARADA);
        }
        break;
      
      case PARADA:
        
        Motor_Stop();

        if (counter_parada == 0U)
        {
          dist_cm_int = (uint32_t)(distance_cm + 0.5f);
          if (dist_cm_int % 2u == 0u) Robot_TransitionTo(&robot_state, GIRO_DER); 
          else Robot_TransitionTo(&robot_state, GIRO_IZQ);
        }
        break;
      
      case GIRO_IZQ:
        
        Motor_Turn_Left();

        if (counter_giro == 0U)
        {
          Robot_TransitionTo(&robot_state, AVANCE);
        }
        break;

      
      case GIRO_DER:
        Motor_Turn_Right();

        if (counter_giro == 0U)
        {
          Robot_TransitionTo(&robot_state, AVANCE);
        }
        break;
      
      default:
        Robot_TransitionTo(&robot_state, AVANCE);
        break;
    

    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = PWM_PRESCALER - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = _PWMPeriod - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = BASE_PRESCALER - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = _BasePeriod - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = IC_PRESCALER - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = _ICPeriod; // tick = 1us at 16MHz/16
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Estado inicial */
  HAL_GPIO_WritePin(GPIOA,
                    GPIO_PIN_0 |
                    GPIO_PIN_1 |
                    GPIO_PIN_2 |
                    GPIO_PIN_3,
                    GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_0,
                    GPIO_PIN_RESET);

  /* IN1 IN2 IN3 IN4 -> salidas */
  GPIO_InitStruct.Pin =
          GPIO_PIN_0 |
          GPIO_PIN_1 |
          GPIO_PIN_2 |
          GPIO_PIN_3;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TRIG -> salida */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // salida PWM para el control de velocidad
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // entrada de captura
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN 4 */
static void Robot_TransitionTo(Robot_State *state, Robot_State next_state)
{
  if (*state == next_state)
  {
    return;
  }

  *state = next_state;

  if (next_state == PARADA)
  {
    counter_parada = STOP_TIME_MS;
  }
  else if ((next_state == GIRO_IZQ) || (next_state == GIRO_DER))
  {
    counter_giro = TURN_TIME_MS;
  }
}

static void Ultrasonic_Update(float *distance_cm, uint8_t enabled)
{
  static HC_State hc_state = Origin;
  static uint16_t delay_us = 0;

  if (enabled == 0U)
  {
    HAL_GPIO_WritePin(TRG_PORT, TRG_PIN, GPIO_PIN_RESET);
    ic_state = Fin_Captura;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    hc_state = Origin; 
    return;
  }

  switch (hc_state)
  {
    case Origin:
      HAL_GPIO_WritePin(TRG_PORT, TRG_PIN, GPIO_PIN_SET);
      delay_us = IC_TIMER->CNT;
      hc_state = Triggering;
      break;

    case Triggering:
      if ((uint16_t)(IC_TIMER->CNT - delay_us) >= 10U)
      {
        HAL_GPIO_WritePin(TRG_PORT, TRG_PIN, GPIO_PIN_RESET);
        ic_state = Espera_Flanco_Ascend;
        delay_us = IC_TIMER->CNT;
        hc_state = Capturing;
      }
      break;

    case Capturing:
      if (ic_state == Fin_Captura)
      {
        *distance_cm = _GetDistance(duration);
        if (*distance_cm > 40.0f)
        {
          *distance_cm = 40.0f;
        }
        else
        {
          distance_valid = 1U;
        }
        hc_state = Origin;
      }
      else if ((uint16_t)(IC_TIMER->CNT - delay_us) >= 38000U)
      {
        *distance_cm = 40.0f;
        ic_state = Fin_Captura;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        hc_state = Origin;
      }
      break;

    default:
      hc_state = Origin;
      break;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    if (counter_parada > 0U)
    {
      counter_parada--;
    }

    if (counter_giro > 0U)
    {
      counter_giro--;
    }
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    switch (ic_state)
    {
      case Espera_Flanco_Ascend:
        rval = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        ic_state = Espera_Flanco_Descend;
        break;

      case Espera_Flanco_Descend:
        fval = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
        if (fval >= rval)
        {
          duration = fval - rval;
        }
        else
        {
          duration = fval + ((uint16_t)IC_TIMER->ARR + 1U - rval);
        }
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        ic_state = Fin_Captura;
        break;

      default:
        break;
    }
  }
}

static void Motor_SetSpeedPercent(uint32_t percent)
{
  uint32_t arr;
  uint32_t pulse;

  if (percent > 100U)
  {
    percent = 100U;
  }

  arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
  pulse = ((arr + 1U) * percent) / 100U;
  if (pulse > arr)
  {
    pulse = arr;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
}

static void Motor_Forward(void)
{
  HAL_GPIO_WritePin(IZQ_IN1_PORT, IZQ_IN1_PIN, ON);
  HAL_GPIO_WritePin(IZQ_IN2_PORT, IZQ_IN2_PIN, OFF);
  HAL_GPIO_WritePin(DER_IN3_PORT, DER_IN3_PIN, ON);
  HAL_GPIO_WritePin(DER_IN4_PORT, DER_IN4_PIN, OFF);
  Motor_SetSpeedPercent(65U);
}

static void Motor_Stop(void)
{
  HAL_GPIO_WritePin(IZQ_IN1_PORT, IZQ_IN1_PIN, OFF);
  HAL_GPIO_WritePin(IZQ_IN2_PORT, IZQ_IN2_PIN, OFF);
  HAL_GPIO_WritePin(DER_IN3_PORT, DER_IN3_PIN, OFF);
  HAL_GPIO_WritePin(DER_IN4_PORT, DER_IN4_PIN, OFF);
  Motor_SetSpeedPercent(0U);
}

static void Motor_Turn_Left(void)
{
  HAL_GPIO_WritePin(IZQ_IN1_PORT, IZQ_IN1_PIN, OFF);
  HAL_GPIO_WritePin(IZQ_IN2_PORT, IZQ_IN2_PIN, OFF);
  HAL_GPIO_WritePin(DER_IN3_PORT, DER_IN3_PIN, ON);
  HAL_GPIO_WritePin(DER_IN4_PORT, DER_IN4_PIN, OFF);
  Motor_SetSpeedPercent(55U);
}

static void Motor_Turn_Right(void)
{
  HAL_GPIO_WritePin(IZQ_IN1_PORT, IZQ_IN1_PIN, ON);
  HAL_GPIO_WritePin(IZQ_IN2_PORT, IZQ_IN2_PIN, OFF);
  HAL_GPIO_WritePin(DER_IN3_PORT, DER_IN3_PIN, OFF);
  HAL_GPIO_WritePin(DER_IN4_PORT, DER_IN4_PIN, OFF);
  Motor_SetSpeedPercent(55U);
}
/* USER CODE END 4 */


