/**
 ******************************************************************************
 * @file    usbd_customhid.h
 * @author  MCD Application Team
 * @brief   header file for the usbd_customhid.c file.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                      www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_JOYSTICKHID_H
#define __USB_JOYSTICKHID_H

#ifdef __cplusplus
extern "C"
{
#endif

  /* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

  /** @addtogroup STM32_USB_DEVICE_LIBRARY
   * @{
   */

  /** @defgroup USBD_JOYSTICK_HID
   * @brief This file is the Header file for USBD_customhid.c
   * @{
   */

  /** @defgroup USBD_JOYSTICK_HID_Exported_Defines
   * @{
   */
#ifndef USBD_MAX_NUM_INTERFACES
#define USBD_MAX_NUM_INTERFACES     					1U
#endif

#define HID_JOYSTICK_INTERFACE           				0x00U
#define HID_CUSTOM_INTERFACE        					0x01U

#define JOYSTICK_HID_EPIN_ADDR                         	0x81U
#define JOYSTICK_HID_EPIN_SIZE                         	0x40U

#define CUSTOM_HID_EPIN_ADDR                         	0x82U
#define CUSTOM_HID_EPIN_SIZE                         	0x40U

#define JOYSTICK_HID_EPOUT_ADDR                        	0x01U
#define JOYSTICK_HID_EPOUT_SIZE                        	0x40U

#define CUSTOM_HID_EPOUT_ADDR                        	0x02U
#define CUSTOM_HID_EPOUT_SIZE                        	0x40U

#define USB_COMPOSITE_HID_CONFIG_DESC_SIZ               73U
#define USB_JOYSTICK_HID_DESC_SIZ                      	9U
#define USB_CUSTOM_HID_DESC_SIZ                      	9U

#ifndef JOYSTICK_HID_HS_BINTERVAL
#define JOYSTICK_HID_HS_BINTERVAL                      	0x01U
#endif /* JOYSTICK_HID_HS_BINTERVAL */

#ifndef JOYSTICK_HID_FS_BINTERVAL
#define JOYSTICK_HID_FS_BINTERVAL                      	0x01U
#endif /* JOYSTICK_HID_FS_BINTERVAL */

#ifndef CUSTOM_HID_HS_BINTERVAL
#define CUSTOM_HID_HS_BINTERVAL                      	0x01U
#endif /* CUSTOM_HID_HS_BINTERVAL */

#ifndef CUSTOM_HID_FS_BINTERVAL
#define CUSTOM_HID_FS_BINTERVAL                      	0x01U
#endif /* CUSTOM_HID_FS_BINTERVAL */

#ifndef USBD_JOYSTICKHID_OUTREPORT_BUF_SIZE
#define USBD_JOYSTICKHID_OUTREPORT_BUF_SIZE            	0x40U
#endif /* USBD_JOYSTICKHID_OUTREPORT_BUF_SIZE */

#ifndef USBD_JOYSTICKHID_INREPORT_BUF_SIZE
#define USBD_JOYSTICKHID_INREPORT_BUF_SIZE             	0x40U
#endif /* USBD_JOYSTICKHID_OUTREPORT_BUF_SIZE */

#ifndef USBD_CUSTOMHID_OUTREPORT_BUF_SIZE
#define USBD_CUSTOMHID_OUTREPORT_BUF_SIZE            	0x20U
#endif /* USBD_CUSTOMHID_OUTREPORT_BUF_SIZE */

#ifndef USBD_CUSTOMHID_INREPORT_BUF_SIZE
#define USBD_CUSTOMHID_INREPORT_BUF_SIZE             	0x20U
#endif /* USBD_CUSTOMHID_OUTREPORT_BUF_SIZE */

#ifndef USBD_JOYSTICK_HID_REPORT_DESC_SIZE
#define USBD_JOYSTICK_HID_REPORT_DESC_SIZE             	2U
#endif /* USBD_JOYSTICK_HID_REPORT_DESC_SIZE */

#ifndef USBD_CUSTOM_HID_REPORT_DESC_SIZE
#define USBD_CUSTOM_HID_REPORT_DESC_SIZE             	36U
#endif /* USBD_CUSTOM_HID_REPORT_DESC_SIZE */

#define HID_DESCRIPTOR_TYPE                   	0x21U
#define HID_REPORT_DESC                       	0x22U

#define HID_REQ_SET_PROTOCOL                  	0x0BU
#define HID_REQ_GET_PROTOCOL                  	0x03U

#define HID_REQ_SET_IDLE                      	0x0AU
#define HID_REQ_GET_IDLE                      	0x02U

#define HID_REQ_SET_REPORT                    	0x09U
#define HID_REQ_GET_REPORT                    	0x01U

#define HID_INPUT_LENGTH 	USBD_CUSTOMHID_INREPORT_BUF_SIZE
#define HID_OUTPUT_LENGTH 	USBD_CUSTOMHID_OUTREPORT_BUF_SIZE
#define HID_FEATURE_LENGTH  144
#define LEW(x) (x) & 0xFF, (x) >> 8

  /**
   * @}
   */

  /** @defgroup USBD_CORE_Exported_TypesDefinitions
   * @{
   */
  typedef enum
  {
    HID_IDLE = 0U, HID_BUSY,
  } HID_StateTypeDef;

  typedef struct _USBD_COMPOSITE_HID_Itf
  {
    uint8_t *pReport;
	int8_t (*Init)(void);
	int8_t (*DeInit)(void);
	int8_t (*OutEvent)(uint16_t, uint8_t*);
	int8_t (*SetFeature)(uint16_t, uint8_t*, uint16_t, uint16_t);
	int8_t (*GetFeature)(uint16_t, uint16_t, uint8_t*, uint16_t*);

  } USBD_COMPOSITE_HID_ItfTypeDef;

  typedef struct
  {
    uint8_t Joyctick_buf[USBD_JOYSTICKHID_OUTREPORT_BUF_SIZE];
    uint8_t CustomHid_buf[HID_FEATURE_LENGTH];
    uint32_t Protocol;
    uint32_t IdleState;
    uint32_t AltSetting;
    uint32_t IsReportAvailable;
    HID_StateTypeDef JoystickState; 				//JoystickState;
    HID_StateTypeDef CustomHidState;
  } USBD_HID_HandleTypeDef;
  /**
   * @}
   */

  /** @defgroup USBD_CORE_Exported_Macros
   * @{
   */

  /**
   * @}
   */

  /** @defgroup USBD_CORE_Exported_Variables
   * @{
   */

extern USBD_ClassTypeDef USBD_COMPOSITE_HID;
#define USBD_COMPOSITE_HID_CLASS &USBD_COMPOSITE_HID

extern uint16_t JoystickHIDReportDescr_Size;

  /**
   * @}
   */

  /** @defgroup USB_CORE_Exported_Functions
   * @{
   */
uint8_t USBD_JOYSTICK_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);

uint8_t USBD_CUSTOM_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);

uint8_t USBD_JOYSTICK_HID_ReceivePacket(USBD_HandleTypeDef *pdev);

uint8_t USBD_CUSTOM_HID_ReceivePacket(USBD_HandleTypeDef *pdev);

uint8_t USBD_JOYSTICK_HID_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_COMPOSITE_HID_ItfTypeDef *fops);

//uint8_t USBD_CUSTOM_HID_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_COMPOSITE_HID_ItfTypeDef *fops);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_JOYSTICKHID_H */
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
