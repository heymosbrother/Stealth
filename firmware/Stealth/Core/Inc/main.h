/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "DCmotor.h"
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define EncoderStretch_EXTI_Pin GPIO_PIN_0
#define EncoderStretch_EXTI_GPIO_Port GPIOC
#define EncoderStretch_EXTI_EXTI_IRQn EXTI0_IRQn
#define EncoderUp_EXTI_Pin GPIO_PIN_1
#define EncoderUp_EXTI_GPIO_Port GPIOC
#define EncoderUp_EXTI_EXTI_IRQn EXTI1_IRQn
#define EncoderLeft_EXTI_Pin GPIO_PIN_2
#define EncoderLeft_EXTI_GPIO_Port GPIOC
#define EncoderLeft_EXTI_EXTI_IRQn EXTI2_IRQn
#define EncoderRight_EXTI_Pin GPIO_PIN_3
#define EncoderRight_EXTI_GPIO_Port GPIOC
#define EncoderRight_EXTI_EXTI_IRQn EXTI3_IRQn
#define EncoderStretch_Pin GPIO_PIN_0
#define EncoderStretch_GPIO_Port GPIOA
#define EncoderUp_Pin GPIO_PIN_1
#define EncoderUp_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define MotorStretch_ENA_Pin GPIO_PIN_4
#define MotorStretch_ENA_GPIO_Port GPIOA
#define MotorStretch_ENB_Pin GPIO_PIN_5
#define MotorStretch_ENB_GPIO_Port GPIOA
#define MotorUp_ENA_Pin GPIO_PIN_6
#define MotorUp_ENA_GPIO_Port GPIOA
#define MotorUp_ENB_Pin GPIO_PIN_7
#define MotorUp_ENB_GPIO_Port GPIOA
#define MotorLeft_ENA_Pin GPIO_PIN_4
#define MotorLeft_ENA_GPIO_Port GPIOC
#define MotorLeft_ENB_Pin GPIO_PIN_5
#define MotorLeft_ENB_GPIO_Port GPIOC
#define MotorRight_ENA_Pin GPIO_PIN_0
#define MotorRight_ENA_GPIO_Port GPIOB
#define MotorRight_ENB_Pin GPIO_PIN_1
#define MotorRight_ENB_GPIO_Port GPIOB
#define EncoderLeft_Pin GPIO_PIN_2
#define EncoderLeft_GPIO_Port GPIOB
#define EncoderRight_Pin GPIO_PIN_10
#define EncoderRight_GPIO_Port GPIOB
#define MotorRight_PWM_Pin GPIO_PIN_8
#define MotorRight_PWM_GPIO_Port GPIOA
#define MotorLeft_PWM_Pin GPIO_PIN_9
#define MotorLeft_PWM_GPIO_Port GPIOA
#define MotorUp_PWM_Pin GPIO_PIN_10
#define MotorUp_PWM_GPIO_Port GPIOA
#define MotorStretch_PWM_Pin GPIO_PIN_11
#define MotorStretch_PWM_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
