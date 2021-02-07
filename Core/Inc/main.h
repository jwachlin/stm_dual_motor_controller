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
#include "adc_interface.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ARM_MATH_CM4
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void enable_motor_task_from_isr(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define configUSE_QUEUE_SETS 1
#define USE_HAL_CAN_REGISTER_CALLBACKS 1
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_2
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_3
#define IN2_GPIO_Port GPIOA
#define SNS1_Pin GPIO_PIN_4
#define SNS1_GPIO_Port GPIOA
#define SNS2_Pin GPIO_PIN_5
#define SNS2_GPIO_Port GPIOA
#define IN3_Pin GPIO_PIN_6
#define IN3_GPIO_Port GPIOA
#define IN4_Pin GPIO_PIN_7
#define IN4_GPIO_Port GPIOA
#define FL1_Pin GPIO_PIN_11
#define FL1_GPIO_Port GPIOE
#define FRAM_CS_Pin GPIO_PIN_12
#define FRAM_CS_GPIO_Port GPIOB
#define SCK_Pin GPIO_PIN_13
#define SCK_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_14
#define MISO_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_15
#define MOSI_GPIO_Port GPIOB
#define INDEX3_Pin GPIO_PIN_14
#define INDEX3_GPIO_Port GPIOD
#define M1IN1_Pin GPIO_PIN_6
#define M1IN1_GPIO_Port GPIOC
#define M1IN2_Pin GPIO_PIN_7
#define M1IN2_GPIO_Port GPIOC
#define INDEX2_Pin GPIO_PIN_8
#define INDEX2_GPIO_Port GPIOC
#define INDEX1_Pin GPIO_PIN_9
#define INDEX1_GPIO_Port GPIOC
#define M2IN1_Pin GPIO_PIN_8
#define M2IN1_GPIO_Port GPIOA
#define M2IN2_Pin GPIO_PIN_9
#define M2IN2_GPIO_Port GPIOA
#define CAN_RX_Pin GPIO_PIN_0
#define CAN_RX_GPIO_Port GPIOD
#define CAN_TX_Pin GPIO_PIN_1
#define CAN_TX_GPIO_Port GPIOD
#define CAN_RS_Pin GPIO_PIN_2
#define CAN_RS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
