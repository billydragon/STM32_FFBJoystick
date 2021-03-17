/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_joystick_hid_if.h
 * @version        : v1.0_Cube
 * @brief          : Header for usbd_joystick_hid_if.c file.
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
#ifndef __USBD_JOYSTICK_HID_IF_H__
#define __USBD_JOYSTICK_HID_IF_H__

#ifdef __cplusplus
extern "C"
{
#endif

  /* Includes ------------------------------------------------------------------*/
#include <usbd_joystickhid.h>

  /* USER CODE BEGIN INCLUDE */

  /* USER CODE END INCLUDE */

  /** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
   * @brief For Usb device.
   * @{
   */

  /** @defgroup USBD_JOYSTICK_HID USBD_JOYSTICK_HID
   * @brief Usb custom human interface device module.
   * @{
   */

  /** @defgroup USBD_JOYSTICK_HID_Exported_Defines USBD_JOYSTICK_HID_Exported_Defines
   * @brief Defines.
   * @{
   */

  /* USER CODE BEGIN EXPORTED_DEFINES */

  /* USER CODE END EXPORTED_DEFINES */

  /**
   * @}
   */

  /** @defgroup USBD_JOYSTICK_HID_Exported_Types USBD_JOYSTICK_HID_Exported_Types
   * @brief Types.
   * @{
   */

  /* USER CODE BEGIN EXPORTED_TYPES */

  /* USER CODE END EXPORTED_TYPES */

  /**
   * @}
   */

  /** @defgroup USBD_JOYSTICK_HID_Exported_Macros USBD_JOYSTICK_HID_Exported_Macros
   * @brief Aliases.
   * @{
   */

  /* USER CODE BEGIN EXPORTED_MACRO */

  /* USER CODE END EXPORTED_MACRO */

  /**
   * @}
   */

  /** @defgroup USBD_JOYSTICK_HID_Exported_Variables USBD_JOYSTICK_HID_Exported_Variables
   * @brief Public variables.
   * @{
   */

  /** JOYSTICKHID Interface callback. */
  extern USBD_COMPOSITE_HID_ItfTypeDef USBD_JoystickHID_fops_FS;

  /* USER CODE BEGIN EXPORTED_VARIABLES */
  extern uint16_t JoystickHIDReportDescr_Size;
  //void Creat_HID_Report_Desc (uint8_t *pHIDDesc, uint16_t len);

  int8_t USBD_JOYSTICK_HID_SendReport_FS (uint8_t *report, uint16_t len);

  int8_t USBD_CUSTOM_HID_SendReport_FS (uint8_t *report, uint16_t len);
/* USER CODE END EXPORTED_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_JOYSTICK_HID_Exported_FunctionsPrototype USBD_JOYSTICK_HID_Exported_FunctionsPrototype
 * @brief Public functions declaration.
 * @{
 */

/* USER CODE BEGIN EXPORTED_FUNCTIONS */

/* USER CODE END EXPORTED_FUNCTIONS */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_JOYSTICK_HID_IF_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
