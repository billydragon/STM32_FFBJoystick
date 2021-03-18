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
#ifndef __CPPMAIN_H
#define __CPPMAIN_H

//standard constant define here

#pragma once
#ifdef __cplusplus

//C++ define here


extern "C"
{
#endif

  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "types.h"
#include "usb_device.h"
#include "usbd_def.h"


	extern uint8_t * JoystickHIDReportDescr;
	extern uint16_t  JoystickHIDReportDescr_Size;
	void cppmain ();
	long map (long x, long in_min, long in_max, long out_min, long out_max);
	uint32_t micros (); // Returns microsecond scaled time
	void delay_us (uint16_t us);

#ifdef __cplusplus
}


#endif



#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
