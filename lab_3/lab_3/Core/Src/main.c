/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd16x2.h"
#include "pid.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
int _16_BIT_FLAG = 0xFFFF;
double MID_POT_VOLTAGE = 1.5;
double VBAT_MAX = 12.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void select_ADC_CH10(void);
void select_ADC_CH11(void);
void select_ADC_CH12(void);

double get_pot_voltage(int pot);
double get_battery_voltage(void);

double get_rot_speed_set_l(double pot1Voltage, double pot2Voltage);
double get_rot_speed_set_r(double pot1Voltage, double pot2Voltage);
void set_PWM_pulse( double pulse_l, double pulse_r );

void display_lcd(void);
void set_battery_indicator_leds( uint8_t LED_state );

void switch_1();
void switch_2();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double rrpm = 0;
double lrpm = 0;
bool mode = 1;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  double battery_voltage;
  double pot1Voltage;
  double pot2Voltage;
  double rot_speed_l_set;
  double rot_speed_r_set;
  int pulse_r;
  int pulse_l;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  // begin transmitting 0% duty cycle PWM and timers
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim4);

  // initialize LCD
  lcd16x2_init_4bits(GPIOB, LCD_RS_Pin, LCD_EN_Pin,
		  GPIOA, LCD_OUT4_Pin, LCD_OUT5_Pin, LCD_OUT6_Pin, LCD_OUT7_Pin);
  lcd16x2_cursorShow(false);

  // initialize PID structs to control DC motors
  struct PID *pid_motor_r, pid1;
  pid_motor_r = &pid1;
  struct PID *pid_motor_l, pid2;
  pid_motor_l = &pid2;

  double K_p = 350;
  double K_i = 300;
  double K_d = 75;
  int max_pulse = 1000;
  int min_pulse = -1000;

  init_pid(pid_motor_r, K_p, K_i, K_d, max_pulse, min_pulse);
  init_pid(pid_motor_l, K_p, K_i, K_d, max_pulse, min_pulse);

  // detect mode and set LEDs before loop
  if ( HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_10 ) ) {
	  mode = 1;
	  HAL_GPIO_WritePin(GPIOC, LED_OUT5_Pin, GPIO_PIN_SET);
	  HAL_TIM_OC_DelayElapsedCallback(&htim3);
  } else {
	  mode = 0;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
  	  if (mode == 1) {
  		  pot1Voltage = get_pot_voltage(1);
  		  pot2Voltage = get_pot_voltage(2);

  		  rot_speed_l_set = get_rot_speed_set_l( pot1Voltage, pot2Voltage );
  		  rot_speed_r_set = get_rot_speed_set_r( pot1Voltage, pot2Voltage );

  		  pid_motor_l->u = rot_speed_l_set;
  		  pid_motor_r->u = rot_speed_r_set;

  		  pulse_l = set_pulse_PID(pid_motor_l, rot_speed_l_set, lrpm);
  		  pulse_r = set_pulse_PID(pid_motor_r, rot_speed_r_set, rrpm);

  		  display_lcd();

  	  } else {

  		  // if low RPM turn off PWM signals
  		  if (fabs(lrpm) < 30.0) {
  			  pulse_l = 0;
  		  } else {
  	  		  pulse_l = set_pulse_PID(pid_motor_l, 0, lrpm);
  		  }

  		  if (fabs(rrpm) < 30.0) {
  			  pulse_r = 0;
  		  } else {
  	  		  pulse_r = set_pulse_PID(pid_motor_r, 0, rrpm);
  		  }

  		  lcd16x2_clear();
  	  }

	  set_PWM_pulse( pulse_l, pulse_r );



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 159;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 159;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 122;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 159;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 159;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_OUT5_GPIO_Port, LED_OUT5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_OUT4_Pin|LCD_OUT5_Pin|LCD_OUT6_Pin|LCD_OUT7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_EN_Pin|LCD_RS_Pin|LCD_RW_Pin|MUX_SELECT2_Pin
                          |MUX_SELECT1_Pin|LED_OUT4_Pin|LED_OUT3_Pin|LED_OUT2_Pin
                          |LED_OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_OUT5_Pin */
  GPIO_InitStruct.Pin = LED_OUT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_OUT5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_OUT4_Pin LCD_OUT5_Pin LCD_OUT6_Pin LCD_OUT7_Pin */
  GPIO_InitStruct.Pin = LCD_OUT4_Pin|LCD_OUT5_Pin|LCD_OUT6_Pin|LCD_OUT7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_EN_Pin LCD_RS_Pin LCD_RW_Pin MUX_SELECT2_Pin
                           MUX_SELECT1_Pin LED_OUT4_Pin LED_OUT3_Pin LED_OUT2_Pin
                           LED_OUT1_Pin */
  GPIO_InitStruct.Pin = LCD_EN_Pin|LCD_RS_Pin|LCD_RW_Pin|MUX_SELECT2_Pin
                          |MUX_SELECT1_Pin|LED_OUT4_Pin|LED_OUT3_Pin|LED_OUT2_Pin
                          |LED_OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_MOTOR_1_Q1_Pin DC_MOTOR_2_Q1_Pin */
  GPIO_InitStruct.Pin = DC_MOTOR_1_Q1_Pin|DC_MOTOR_2_Q1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_MOTOR_1_Q2_Pin DC_MOTOR_2_Q2_Pin */
  GPIO_InitStruct.Pin = DC_MOTOR_1_Q2_Pin|DC_MOTOR_2_Q2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @GPIO EXTI interrupt handler
  * @GPIO_Pin pin which calls the interrupt
  */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin ) {
	// Q1 pin of left motor
	if ( GPIO_Pin == DC_MOTOR_1_Q1_Pin ) {
		int cycle_count = __HAL_TIM_GET_COUNTER(&htim1);
		lrpm = (double) 60.0/(1.0e-5*24.0*cycle_count);

		if ( HAL_GPIO_ReadPin( GPIOC, DC_MOTOR_1_Q2_Pin ) ) {
			lrpm *= -1.0;
		}

		__HAL_TIM_SET_COUNTER(&htim1, 0);

	// Q1 pin of right motor
	} else if ( GPIO_Pin == DC_MOTOR_2_Q1_Pin ) {
		int cycle_count = __HAL_TIM_GET_COUNTER(&htim4);
		rrpm = (double)60.0/(1.0e-5*24.0*cycle_count);

		if ( HAL_GPIO_ReadPin( GPIOC, DC_MOTOR_2_Q2_Pin ) ) {
			rrpm *= -1.0;
		}

		__HAL_TIM_SET_COUNTER(&htim4, 0);

	// digital switch
	} else if ( GPIO_Pin == GPIO_PIN_10 ) {
		mode ^= 1;
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		__HAL_TIM_SET_COUNTER(&htim4, 0);

		// Set LEDs and LCD based on new mode
		if (mode) {
			rrpm = 0.0;
			lrpm = 0.0;
			HAL_GPIO_WritePin(GPIOC, LED_OUT5_Pin, GPIO_PIN_SET);
			HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
			HAL_TIM_OC_DelayElapsedCallback(&htim3);

		// turn off LEDs and reset counter
		} else {
			HAL_GPIO_WritePin(GPIOB, LED_OUT1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_OUT2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_OUT3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_OUT4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, LED_OUT5_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_1);
		}
	}
}

/**
  * @Timer interrupt handler
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3) {
	  if (mode) {
		  double battery_voltage = get_battery_voltage();
		  uint8_t LED_state;

		  if (battery_voltage >= 0.9*VBAT_MAX) {
			  LED_state = 0b1000;
		  } else if (battery_voltage >= 0.8*VBAT_MAX) {
			  LED_state = 0b0100;
		  } else if (battery_voltage >= 0.6*VBAT_MAX) {
			  LED_state = 0b0010;
		  } else {
			  LED_state = 0b0001;
		  }

		  set_battery_indicator_leds(LED_state);
	  }
  }
}
/**
  * @Select channel 10 of the ADC
  * @Param none
  */
void select_ADC_CH10(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	Error_Handler();
  }
}

/**
  * @Select channel 11 of the ADC
  * @Param none
  */
void select_ADC_CH11(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	Error_Handler();
  }
}

/**
  * @Select channel 12 of the ADC
  * @Param none
  */
void select_ADC_CH12(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	Error_Handler();
  }
}

/**
  * @Return rounded value of potentiometer voltage
  * @pot = 1 for speed potentiometer, 2 for steering potentiometer
  */
double get_pot_voltage( int pot ) {
	if (pot == 1) {
		select_ADC_CH11();
	} else if (pot == 2) {
		select_ADC_CH12();
	}

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint8_t sample = HAL_ADC_GetValue(&hadc1);

	double ret = (double) sample*(3.3/255.0);
	ret = roundf(ret*10)/10.0;

	return ret;
}

/**
  * @Measure the voltage of the battery sensor circuit
  * @Param none
  */
double get_battery_voltage(void) {
	select_ADC_CH10();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint8_t sample = HAL_ADC_GetValue(&hadc1);
	double ret = (double) sample*(12.0/255.0);

	return ret;
}

/**
  * @Calculate the rotation speed of the left DC motor
  * 		based off the potentiometer voltages
  * @Param:
  * pot1Voltage - voltage of speed potentiometer
  * pot2Voltage - voltage of steering potentiometer
  */
double get_rot_speed_set_l( double pot1Voltage, double pot2Voltage ) {
	double speed_setting = (pot1Voltage - 1.5)*10.0/1.5;
	double steer_setting = (pot2Voltage - 1.5)*10.0/1.5;

	double rot_speed;
	double speed_factor = 4*speed_setting;

	// turning right
	if (steer_setting >= 0.0) {
		rot_speed = speed_factor*10;
	// turning left
	} else {
		rot_speed = speed_factor*(10 + steer_setting);
	}

	return rot_speed;
}

/**
  * @Calculate the rotation speed of the right DC motor
  * 		based off the potentiometer voltages
  * @Param:
  * pot1Voltage - voltage of speed potentiometer
  * pot2Voltage - voltage of steering potentiometer
  */
double get_rot_speed_set_r( double pot1Voltage, double pot2Voltage ) {
	double speed_setting = (pot1Voltage - 1.5)*10.0/1.5;
	double steer_setting = (pot2Voltage - 1.5)*10.0/1.5;

	double rot_speed;
	double speed_factor = 4*speed_setting;

	// turning right
	if (steer_setting >= 0.0) {
		rot_speed = speed_factor*(10 - steer_setting);
	// turning left
	} else {
		rot_speed = speed_factor*10;
	}

	return rot_speed;
}

/**
  * @Set the duty cycle of the PWM signals fed to the DC motors
  * 	and set the MUX select based on positive/negative pulse values
  * @Param:
  * pulse_l - duty cycle of left DC motor PWM signal
  * pulse_r - duty cycle of right DC motor PWM signal
  */
void set_PWM_pulse( double pulse_l, double pulse_r ) {
	if ( pulse_l <= 0.0 ){
		HAL_GPIO_WritePin(GPIOB, MUX_SELECT1_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOB, MUX_SELECT1_Pin, GPIO_PIN_RESET);
	}

	if ( pulse_r <= 0.0 ){
		HAL_GPIO_WritePin(GPIOB, MUX_SELECT2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOB, MUX_SELECT2_Pin, GPIO_PIN_RESET);
	}


	int pulse_li = (int) pulse_l;
	int pulse_ri = (int) pulse_r;

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, abs(pulse_li));
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, abs(pulse_ri));
}

/**
  * @Display text on the LCD in Run Mode
  * @Param none
  */
void display_lcd() {
	if (!mode) {
		return;
	}

	lcd16x2_1stLine();
	lcd16x2_printf("LRPM|  ON  |RRPM");
	lcd16x2_2ndLine();
	int wheel_lrpm = round(lrpm/6);
	int wheel_rrpm = round(rrpm/6);

	int n_spaces = 16;
	if (abs(wheel_lrpm/10) > 0) {
		n_spaces -= 2;
	} else {
		n_spaces -= 1;
	}
	if (abs(wheel_lrpm/10) > 0) {
		n_spaces -= 2;
	} else {
		n_spaces -= 1;
	}
	if (wheel_lrpm < 0) {
		n_spaces -= 1;
	}
	if (wheel_rrpm < 0) {
		n_spaces -= 1;
	}

	lcd16x2_printf("%d", wheel_lrpm);
	for (int i = 0; i < n_spaces; i++) {
		lcd16x2_printf(" ");
	}
	lcd16x2_printf("%d     ", wheel_rrpm);
}

/**
  * @Set the state of all LEDs based on the mode and battery voltage
  * @Param battery_voltage sensed from battery sensor circuit
  */
void set_battery_indicator_leds(uint8_t LED_state) {
	if (LED_state == 0) {
		HAL_GPIO_WritePin(GPIOB, LED_OUT1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT4_Pin, GPIO_PIN_RESET);
	} else if (LED_state&0b1000 && ( (LED_state^0b1000) == 0)) {
		HAL_GPIO_WritePin(GPIOB, LED_OUT1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT4_Pin, GPIO_PIN_RESET);
	} else if (LED_state&0b0100 && ( (LED_state^0b0100) == 0)) {
		HAL_GPIO_WritePin(GPIOB, LED_OUT1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT4_Pin, GPIO_PIN_RESET);
	} else if (LED_state&0b0010 && ( (LED_state^0b0010) == 0)) {
		HAL_GPIO_WritePin(GPIOB, LED_OUT1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT4_Pin, GPIO_PIN_RESET);
	} else if (LED_state&0x0001 && ( (LED_state^0b0001) == 0)) {
		HAL_GPIO_WritePin(GPIOB, LED_OUT1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LED_OUT3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(GPIOB, LED_OUT4_Pin);
	}
}

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
