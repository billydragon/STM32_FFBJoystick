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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define JBUTTON2_Pin GPIO_PIN_2
#define JBUTTON2_GPIO_Port GPIOE
#define JBUTTON2_EXTI_IRQn EXTI2_IRQn
#define JBUTTON4_Pin GPIO_PIN_4
#define JBUTTON4_GPIO_Port GPIOE
#define JBUTTON4_EXTI_IRQn EXTI4_IRQn
#define JBUTTON5_Pin GPIO_PIN_5
#define JBUTTON5_GPIO_Port GPIOE
#define JBUTTON5_EXTI_IRQn EXTI9_5_IRQn
#define JBUTTON6_Pin GPIO_PIN_6
#define JBUTTON6_GPIO_Port GPIOE
#define JBUTTON6_EXTI_IRQn EXTI9_5_IRQn
#define USER_BUTTON_Pin GPIO_PIN_0
#define USER_BUTTON_GPIO_Port GPIOA
#define JBUTTON1_Pin GPIO_PIN_1
#define JBUTTON1_GPIO_Port GPIOA
#define JBUTTON1_EXTI_IRQn EXTI1_IRQn
#define SPI1_CS1_Pin GPIO_PIN_4
#define SPI1_CS1_GPIO_Port GPIOA
#define SPI1_CS2_Pin GPIO_PIN_4
#define SPI1_CS2_GPIO_Port GPIOC
#define MOTOR_Y_DIR_Pin GPIO_PIN_5
#define MOTOR_Y_DIR_GPIO_Port GPIOC
#define MOTOR_X_EN_Pin GPIO_PIN_0
#define MOTOR_X_EN_GPIO_Port GPIOB
#define MOTOR_Y_EN_Pin GPIO_PIN_1
#define MOTOR_Y_EN_GPIO_Port GPIOB
#define MOTOR_X_DIR_Pin GPIO_PIN_2
#define MOTOR_X_DIR_GPIO_Port GPIOB
#define TIM1_PWM1_Pin GPIO_PIN_9
#define TIM1_PWM1_GPIO_Port GPIOE
#define TIM1_PWM3_Pin GPIO_PIN_13
#define TIM1_PWM3_GPIO_Port GPIOE
#define TIM1_PWM4_Pin GPIO_PIN_14
#define TIM1_PWM4_GPIO_Port GPIOE
#define DAC856x_CLR_Pin GPIO_PIN_10
#define DAC856x_CLR_GPIO_Port GPIOB
#define DAC856x_LDAC_Pin GPIO_PIN_11
#define DAC856x_LDAC_GPIO_Port GPIOB
#define X_LIMIT_MIN_Pin GPIO_PIN_12
#define X_LIMIT_MIN_GPIO_Port GPIOB
#define X_LIMIT_MIN_EXTI_IRQn EXTI15_10_IRQn
#define Y_LIMIT_MIN_Pin GPIO_PIN_13
#define Y_LIMIT_MIN_GPIO_Port GPIOB
#define Y_LIMIT_MIN_EXTI_IRQn EXTI15_10_IRQn
#define X_LIMIT_MAX_Pin GPIO_PIN_14
#define X_LIMIT_MAX_GPIO_Port GPIOB
#define X_LIMIT_MAX_EXTI_IRQn EXTI15_10_IRQn
#define Y_LIMIT_MAX_Pin GPIO_PIN_15
#define Y_LIMIT_MAX_GPIO_Port GPIOB
#define Y_LIMIT_MAX_EXTI_IRQn EXTI15_10_IRQn
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOD
#define JBUTTON7_Pin GPIO_PIN_7
#define JBUTTON7_GPIO_Port GPIOC
#define JBUTTON7_EXTI_IRQn EXTI9_5_IRQn
#define JBUTTON8_Pin GPIO_PIN_8
#define JBUTTON8_GPIO_Port GPIOC
#define JBUTTON8_EXTI_IRQn EXTI9_5_IRQn
#define JBUTTON9_Pin GPIO_PIN_9
#define JBUTTON9_GPIO_Port GPIOC
#define JBUTTON9_EXTI_IRQn EXTI9_5_IRQn
#define JBUTTON10_Pin GPIO_PIN_10
#define JBUTTON10_GPIO_Port GPIOC
#define JBUTTON10_EXTI_IRQn EXTI15_10_IRQn
#define JBUTTON3_Pin GPIO_PIN_11
#define JBUTTON3_GPIO_Port GPIOC
#define JBUTTON3_EXTI_IRQn EXTI15_10_IRQn
#define JBUTTON0_Pin GPIO_PIN_0
#define JBUTTON0_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
