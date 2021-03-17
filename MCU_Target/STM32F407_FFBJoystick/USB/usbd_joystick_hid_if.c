/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_joystick_hid_if.c
 * @version        : v1.0_Cube
 * @brief          : USB Device Custom HID interface file.
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

/* Includes ------------------------------------------------------------------*/
#include <usbd_joystick_hid_if.h>
#include "USB_Handler.h"


/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @brief Usb device.
 * @{
 */

/** @addtogroup USBD_JOYSTICK_HID
 * @{
 */

/** @defgroup USBD_JOYSTICK_HID_Private_TypesDefinitions USBD_JOYSTICK_HID_Private_TypesDefinitions
 * @brief Private types.
 * @{
 */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
 * @}
 */

/** @defgroup USBD_JOYSTICK_HID_Private_Defines USBD_JOYSTICK_HID_Private_Defines
 * @brief Private defines.
 * @{
 */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
 * @}
 */

/** @defgroup USBD_JOYSTICK_HID_Private_Macros USBD_JOYSTICK_HID_Private_Macros
 * @brief Private macros.
 * @{
 */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
 * @}
 */

/** @defgroup USBD_JOYSTICK_HID_Private_Variables USBD_JOYSTICK_HID_Private_Variables
 * @brief Private variables.
 * @{
 */

/** Usb HID report descriptor. */

__ALIGN_BEGIN static uint8_t JOYSTICK_HID_ReportDesc_FS[USBD_JOYSTICK_HID_REPORT_DESC_SIZE] __ALIGN_END =
      {
		 0,
		 0 };

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_JOYSTICK_HID_Exported_Variables USBD_JOYSTICK_HID_Exported_Variables
 * @brief Public variables.
 * @{
 */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
 * @}
 */

/** @defgroup USBD_JOYSTICK_HID_Private_FunctionPrototypes USBD_JOYSTICK_HID_Private_FunctionPrototypes
 * @brief Private functions declaration.
 * @{
 */

static int8_t JOYSTICK_HID_Init_FS (void);
static int8_t JOYSTICK_HID_DeInit_FS (void);
static int8_t HID_OutEvent_FS (uint16_t epnum, uint8_t *buffer);
static int8_t USB_HID_SetFeature (uint16_t event_idx, uint8_t *buffer, uint16_t wLenght, uint16_t wIndex);
static int8_t USB_HID_GetFeature (uint16_t event_idx, uint16_t wIndex, uint8_t *buffer, uint16_t *length);

/**
 * @}
 */
//uint8_t *JOYSTICK_HID_ReportDesc;

USBD_COMPOSITE_HID_ItfTypeDef USBD_JoystickHID_fops_FS =
  {
	JOYSTICK_HID_ReportDesc_FS,
	JOYSTICK_HID_Init_FS,
	JOYSTICK_HID_DeInit_FS,
    HID_OutEvent_FS,
	USB_HID_SetFeature,
	USB_HID_GetFeature,
  };

/*
void Creat_HID_Report_Desc (uint8_t *pHIDDesc, uint16_t len)
{

  JoystickHIDReportDescr_Size = len;
  JOYSTICK_HID_ReportDesc = USBD_malloc (len);
  memcpy (JOYSTICK_HID_ReportDesc, pHIDDesc, len);
  USBD_JoystickHID_fops_FS.pReport = JOYSTICK_HID_ReportDesc;

}
*/
/** @defgroup USBD_JOYSTICK_HID_Private_Functions USBD_JOYSTICK_HID_Private_Functions
 * @brief Private functions.
 * @{
 */

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initializes the CUSTOM HID media low layer
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t JOYSTICK_HID_Init_FS (void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
 * @brief  DeInitializes the CUSTOM HID media low layer
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t JOYSTICK_HID_DeInit_FS (void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
 * @brief  Manage the CUSTOM HID class events
 * @param  event_idx: Event index
 * @param  state: Event state
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t HID_OutEvent_FS (uint16_t epnum, uint8_t *buffer)
{
  /* USER CODE BEGIN 6 */
  //UNUSED(buffer);
  uint8_t rpID = buffer[0];
  if (epnum == (JOYSTICK_HID_EPOUT_ADDR & 0x7F))
    {
      switch (rpID)
	{
	case 2:
	  USBD_JOYSTICK_HID_SendReport_FS (
	      GetPIDStatus (), sizeof(USB_FFBReport_PIDStatus_Input_Data_t));
	  break;
	default:
	  RecvfromUsb (buffer);
	  break;
	}
      /* Start next USB packet transfer once data processing is completed */
      USBD_JOYSTICK_HID_ReceivePacket (&hUsbDeviceFS);

    }
  else if (epnum == (CUSTOM_HID_EPOUT_ADDR & 0x7F))
    {
      //ConfigDataFormUSB(buffer);
      /* Start next USB packet transfer once data processing is completed */
      USBD_CUSTOM_HID_ReceivePacket (&hUsbDeviceFS);
    }

  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */

static int8_t USB_HID_SetFeature (uint16_t event_idx, uint8_t *buffer, uint16_t wLenght,
		    uint16_t wIndex)
{

  uint8_t report_id = LOBYTE(event_idx);		//setup.wValueL;
  uint8_t report_type = HIBYTE(event_idx);	//setup.wValueH;

  if (report_type == DYNAMIC_HID_REPORT_TYPE_FEATURE)
    {

      if (wIndex == 0)
	{
	  if (report_id == 5)
	    {

	      USB_FFBReport_CreateNewEffect_Feature_Data_t ans;
	      memcpy (&ans, (USB_FFBReport_CreateNewEffect_Feature_Data_t*) buffer,
		      sizeof(USB_FFBReport_CreateNewEffect_Feature_Data_t));
	      CreatNewEffect (&ans);

	      return (USBD_OK);
	    }

	}

      if (wIndex == 1)
	{
	  if (report_id == 0)
	    {

	      HostToDevSetFeature (buffer, wLenght);
	      return (USBD_OK);

	    }

	}
    }
  if (report_type == DYNAMIC_HID_REPORT_TYPE_OUTPUT)
    {

      return (USBD_OK);
    }

  if (report_type == DYNAMIC_HID_REPORT_TYPE_INPUT)
    {
      if (wIndex == 0)
	{

	  return (USBD_OK);
	}
      if (wIndex == 1)
	{

	  return (USBD_OK);
	}

    }
  return (USBD_FAIL);

}

static int8_t USB_HID_GetFeature (uint16_t event_idx, uint16_t wIndex, uint8_t *buffer,
		    uint16_t *length)
{
  uint8_t report_id = LOBYTE(event_idx);		//	setup.wValueL;
  uint8_t report_type = HIBYTE(event_idx);	// setup.wValueH;
//  uint16_t GetFeatureDataSize;
//  uint8_t *pGetFeatureData;

  if (report_type == DYNAMIC_HID_REPORT_TYPE_INPUT)
    {
      if (wIndex == 0)
	{
	  if (report_id == 2)
	    {

	      uint8_t *pGetFeatureData = GetPIDStatus ();
	      uint16_t GetFeatureDataSize = sizeof(USB_FFBReport_PIDStatus_Input_Data_t);
	      memcpy (buffer, pGetFeatureData, GetFeatureDataSize);
	      *length = GetFeatureDataSize;

	      return (USBD_OK);

	    }

	}

      if (wIndex == 1)
	{
	  return (USBD_OK);
	}

    }

  if (report_type == DYNAMIC_HID_REPORT_TYPE_OUTPUT)
    {

    }

  if (report_type == DYNAMIC_HID_REPORT_TYPE_FEATURE)
    {
      if (wIndex == 0)
	{

	  if ((report_id == 6) || (report_id == 0x12))// && (gNewEffectBlockLoad.reportId==6))
	    {

	    uint16_t GetFeatureDataSize = sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t);
	    uint8_t *pGetFeatureData = GetPIDBlockLoad ();
	      memcpy (buffer, pGetFeatureData, GetFeatureDataSize);
	      *length = GetFeatureDataSize;

	      return (USBD_OK);
	    }

	  if ((report_id == 7) || (report_id == 0x13))
	    {

	      USB_FFBReport_PIDPool_Feature_Data_t ans;
	      ans.reportId = report_id;
	      ans.ramPoolSize = 0xffff;
	      ans.maxSimultaneousEffects = MAX_EFFECTS;
	      ans.memoryManagement = 3;
	      uint16_t GetFeatureDataSize = sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t);
	      *length = GetFeatureDataSize;
	      memcpy (buffer, &ans, GetFeatureDataSize);

	      return (USBD_OK);
	    }

	}
      if (wIndex == 1)
      {

		 uint8_t *pGetFeatureData = Get_SysConfig();
		 uint16_t GetFeatureDataSize = sizeof(SYS_CONFIG_t);
		  //buffer[0] = 0;
		  uint16_t offset = 0;
		  while (offset < GetFeatureDataSize)
			{
			  uint16_t bufferLength =
			  (GetFeatureDataSize - offset) > HID_FEATURE_LENGTH ? HID_FEATURE_LENGTH : (GetFeatureDataSize - offset);
			  memcpy (&buffer[0], &pGetFeatureData[offset], bufferLength);
			  offset += bufferLength == HID_FEATURE_LENGTH ? bufferLength - 1 : bufferLength;
			  *length = bufferLength ;
			  return (USBD_OK);
	    }
	}

    }
  return (USBD_FAIL);

}

/**
 * @brief  Send the report to the Host
 * @param  report: The report to be sent
 * @param  len: The report length
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */

int8_t
USBD_JOYSTICK_HID_SendReport_FS (uint8_t *report, uint16_t len)
{
  return USBD_JOYSTICK_HID_SendReport (&hUsbDeviceFS, report, len);
}

int8_t
USBD_CUSTOM_HID_SendReport_FS (uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport (&hUsbDeviceFS, report, len);
}

/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

