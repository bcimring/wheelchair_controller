/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BAT_IN_Pin GPIO_PIN_0
#define BAT_IN_GPIO_Port GPIOC
#define POT_IN1_Pin GPIO_PIN_1
#define POT_IN1_GPIO_Port GPIOC
#define POT_IN2_Pin GPIO_PIN_2
#define POT_IN2_GPIO_Port GPIOC
#define PWM_CH1_Pin GPIO_PIN_0
#define PWM_CH1_GPIO_Port GPIOA
#define PWM_CH2_Pin GPIO_PIN_1
#define PWM_CH2_GPIO_Port GPIOA
#define LCD_OUT4_Pin GPIO_PIN_4
#define LCD_OUT4_GPIO_Port GPIOA
#define LCD_OUT5_Pin GPIO_PIN_5
#define LCD_OUT5_GPIO_Port GPIOA
#define LCD_OUT6_Pin GPIO_PIN_6
#define LCD_OUT6_GPIO_Port GPIOA
#define LCD_OUT7_Pin GPIO_PIN_7
#define LCD_OUT7_GPIO_Port GPIOA
#define LCD_EN_Pin GPIO_PIN_0
#define LCD_EN_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_1
#define LCD_RS_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_2
#define LCD_RW_GPIO_Port GPIOB
#define MUX_SELECT_Pin GPIO_PIN_15
#define MUX_SELECT_GPIO_Port GPIOB
#define DC_MOTOR_1_Q1_Pin GPIO_PIN_6
#define DC_MOTOR_1_Q1_GPIO_Port GPIOC
#define DC_MOTOR_1_Q1_EXTI_IRQn EXTI9_5_IRQn
#define DC_MOTOR_1_Q2_Pin GPIO_PIN_7
#define DC_MOTOR_1_Q2_GPIO_Port GPIOC
#define DC_MOTOR_2_Q1_Pin GPIO_PIN_8
#define DC_MOTOR_2_Q1_GPIO_Port GPIOC
#define DC_MOTOR_2_Q1_EXTI_IRQn EXTI9_5_IRQn
#define DC_MOTOR_2_Q2_Pin GPIO_PIN_9
#define DC_MOTOR_2_Q2_GPIO_Port GPIOC
#define SWITCH_1_Pin GPIO_PIN_10
#define SWITCH_1_GPIO_Port GPIOA
#define SWITCH_2_Pin GPIO_PIN_11
#define SWITCH_2_GPIO_Port GPIOA
#define LED_OUT4_Pin GPIO_PIN_3
#define LED_OUT4_GPIO_Port GPIOB
#define LED_OUT3_Pin GPIO_PIN_4
#define LED_OUT3_GPIO_Port GPIOB
#define LED_OUT2_Pin GPIO_PIN_5
#define LED_OUT2_GPIO_Port GPIOB
#define LED_OUT1_Pin GPIO_PIN_6
#define LED_OUT1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
