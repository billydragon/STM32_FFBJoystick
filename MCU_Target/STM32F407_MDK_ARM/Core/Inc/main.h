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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define JBUTTON2_Pin GPIO_PIN_2
#define JBUTTON2_GPIO_Port GPIOE
#define JBUTTON3_Pin GPIO_PIN_3
#define JBUTTON3_GPIO_Port GPIOE
#define JBUTTON4_Pin GPIO_PIN_4
#define JBUTTON4_GPIO_Port GPIOE
#define JBUTTON5_Pin GPIO_PIN_5
#define JBUTTON5_GPIO_Port GPIOE
#define JBUTTON6_Pin GPIO_PIN_6
#define JBUTTON6_GPIO_Port GPIOE
#define USR_BUTTON_Pin GPIO_PIN_0
#define USR_BUTTON_GPIO_Port GPIOA
#define USR_BUTTON_EXTI_IRQn EXTI0_IRQn
#define JBUTTON1_Pin GPIO_PIN_1
#define JBUTTON1_GPIO_Port GPIOA
#define JBUTTON1_EXTI_IRQn EXTI1_IRQn
#define SPI1_CS1_Pin GPIO_PIN_4
#define SPI1_CS1_GPIO_Port GPIOA
#define SPI1_CS2_Pin GPIO_PIN_4
#define SPI1_CS2_GPIO_Port GPIOC
#define MOTOR_X_EN_Pin GPIO_PIN_5
#define MOTOR_X_EN_GPIO_Port GPIOC
#define MOTOR_Y_EN_Pin GPIO_PIN_0
#define MOTOR_Y_EN_GPIO_Port GPIOB
#define MOTOR_X_DIR_Pin GPIO_PIN_1
#define MOTOR_X_DIR_GPIO_Port GPIOB
#define TIM1_PWM1_Pin GPIO_PIN_9
#define TIM1_PWM1_GPIO_Port GPIOE
#define TIM1_PWM2_Pin GPIO_PIN_11
#define TIM1_PWM2_GPIO_Port GPIOE
#define TIM1_PWM3_Pin GPIO_PIN_13
#define TIM1_PWM3_GPIO_Port GPIOE
#define TIM1_PWM4_Pin GPIO_PIN_14
#define TIM1_PWM4_GPIO_Port GPIOE
#define DAC856x_CLR_Pin GPIO_PIN_10
#define DAC856x_CLR_GPIO_Port GPIOB
#define DAC856x_LDAC_Pin GPIO_PIN_11
#define DAC856x_LDAC_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
