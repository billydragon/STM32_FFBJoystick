/**
 ******************************************************************************
 * @file    usbd_customhid.c
 * @author  MCD Application Team
 * @brief   This file provides the JOYSTICK_HID core functions.
 *
 * @verbatim
 *
 *          ===================================================================
 *                                JOYSTICK_HID Class  Description
 *          ===================================================================
 *           This module manages the JOYSTICK_HID class V1.11 following the "Device Class Definition
 *           for Human Interface Devices (JOYSTICK_HID) Version 1.11 Jun 27, 2001".
 *           This driver implements the following aspects of the specification:
 *             - The Boot Interface Subclass
 *             - Usage Page : Generic Desktop
 *             - Usage : Vendor
 *             - Collection : Application
 *
 * @note     In HS mode and when the DMA is used, all variables and data structures
 *           dealing with the DMA during the transaction process should be 32-bit aligned.
 *
 *
 *  @endverbatim
 *
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

/* BSPDependencies
 - "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
 - "stm32xxxxx_{eval}{discovery}_io.c"
 EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_joystickhid.h"
#include "usbd_ctlreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_JOYSTICK_HID
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_JOYSTICK_HID_Private_TypesDefinitions
 * @{
 */

/**
 * @}
 */

/** @defgroup USBD_JOYSTICK_HID_Private_Defines
 * @{
 */

/**
 * @}
 */

/** @defgroup USBD_JOYSTICK_HID_Private_Macros
 * @{
 */
/**
 * @}
 */
/** @defgroup USBD_JOYSTICK_HID_Private_FunctionPrototypes
 * @{
 */

static uint8_t USBD_COMPOSITE_HID_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_COMPOSITE_HID_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_COMPOSITE_HID_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_JOYSTICK_HID_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_CUSTOM_HID_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static uint8_t USBD_JOYSTICK_HID_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_JOYSTICK_HID_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_JOYSTICK_HID_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t* USBD_HID_GetFSCfgDesc (uint16_t *length);
static uint8_t* USBD_HID_GetHSCfgDesc (uint16_t *length);
static uint8_t* USBD_HID_GetOtherSpeedCfgDesc (uint16_t *length);
static uint8_t* USBD_HID_GetDeviceQualifierDesc (uint16_t *length);

/**
 * @}
 */

/** @defgroup USBD_JOYSTICK_HID_Private_Variables
 * @{
 */

USBD_ClassTypeDef USBD_COMPOSITE_HID =
  {
      USBD_COMPOSITE_HID_Init,
	  USBD_COMPOSITE_HID_DeInit,
      USBD_COMPOSITE_HID_Setup,
      NULL, /*EP0_TxSent*/
      USBD_JOYSTICK_HID_EP0_RxReady, /*EP0_RxReady*//* STATUS STAGE IN */
      USBD_JOYSTICK_HID_DataIn, /*DataIn*/
      USBD_JOYSTICK_HID_DataOut,
      NULL, /*SOF */
      NULL,
      NULL, USBD_HID_GetHSCfgDesc,
	  USBD_HID_GetFSCfgDesc,
      USBD_HID_GetOtherSpeedCfgDesc,
	  USBD_HID_GetDeviceQualifierDesc, };

uint16_t JoystickHIDReportDescr_Size;

/** Usb CUSTOM HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END
=
  { 0x06, 0x00, 0xff, 		// USAGE_PAGE (Vendor Defined Page 1)
      0x09, 0x01, 				// USAGE (Vendor Usage 1)
      0xa1, 0x01, 				// COLLECTION (Application)
      0x15, 0x00, 					// LOGICAL_MINIMUM (0)
      0x26, 0xff, 0x00, 				// LOGICAL_MAXIMUM (255)
      0x75, 0x08, 					// REPORT_SIZE (8)
      0x96, LEW(USBD_CUSTOMHID_INREPORT_BUF_SIZE), 	// REPORT_COUNT (64)
      0x09, 0x01, 				// USAGE (Vendor Usage 1)
      0x81, 0x02, 					// INPUT (Data,Var,Abs)
      0x96, LEW(USBD_CUSTOMHID_OUTREPORT_BUF_SIZE), // REPORT_COUNT (64)
      0x09, 0x02, 				// USAGE (Vendor Usage 2)
      0x91, 0x02, 					// OUTPUT (Data,Var,Abs)
      0x96, LEW(HID_FEATURE_LENGTH), 		// REPORT_COUNT (1024)
      0x09, 0x03, 				// USAGE (Vendor Usage 3)
      0xB1, 0x02, 				// FEATURE (Data,Var,Abs)
      0xc0 					// END_COLLECTION
    /*  36  */
    };

/* USB JOYSTICK_HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgFSDesc[USB_COMPOSITE_HID_CONFIG_DESC_SIZ] __ALIGN_END
=
  { 0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  LOBYTE(USB_COMPOSITE_HID_CONFIG_DESC_SIZ), /* wTotalLength: Bytes returned */
  HIBYTE(USB_COMPOSITE_HID_CONFIG_DESC_SIZ),
  USBD_MAX_NUM_INTERFACES, /* bNumInterfaces: 1 interface */
  0x01, /* bConfigurationValue: Configuration value */
  0x00, /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0, /* bmAttributes: bus powered */
  0x32, /* MaxPower 100 mA: this current is used for detecting Vbus */

  /************** Descriptor of CUSTOM HID interface ****************/
  /* 09 */
  0x09, /* bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
  HID_JOYSTICK_INTERFACE, /* bInterfaceNumber: Number of Interface */
  0x00, /* bAlternateSetting: Alternate setting */
  0x02, /* bNumEndpoints*/
  0x03, /* bInterfaceClass: JOYSTICK_HID */
  0x00, /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x00, /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0x00, /* iInterface: Index of string descriptor */
  /******************** Descriptor of JOYSTICK_HID *************************/
  /* 18 */
  0x09, /* bLength: JOYSTICK_HID Descriptor size */
  HID_DESCRIPTOR_TYPE, /* bDescriptorType: JOYSTICK_HID */
  0x11, /* bJOYSTICK_HIDUSTOM_HID: JOYSTICK_HID Class Spec release number */
  0x01, 0x00, /* bCountryCode: Hardware target country */
  0x01, /* bNumDescriptors: Number of JOYSTICK_HID class descriptors to follow */
  0x22, /* bDescriptorType */
  LOBYTE(USBD_JOYSTICK_HID_REPORT_DESC_SIZE), /* wItemLength: Total length of Report descriptor */
  HIBYTE(USBD_JOYSTICK_HID_REPORT_DESC_SIZE),
  /******************** Descriptor of Custom HID endpoints ********************/
  /* 27 */
  0x07, /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */

  JOYSTICK_HID_EPIN_ADDR, /* bEndpointAddress: Endpoint Address (IN) */
  0x03, /* bmAttributes: Interrupt endpoint */
  JOYSTICK_HID_EPIN_SIZE, /* wMaxPacketSize: 2 Byte max */
  0x00,
  JOYSTICK_HID_FS_BINTERVAL, /* bInterval: Polling Interval */
  /* 34 */

  0x07, /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  JOYSTICK_HID_EPOUT_ADDR, /* bEndpointAddress: Endpoint Address (OUT) */
  0x03, /* bmAttributes: Interrupt endpoint */
  JOYSTICK_HID_EPOUT_SIZE, /* wMaxPacketSize: 2 Bytes max  */
  0x00,
  JOYSTICK_HID_FS_BINTERVAL, /* bInterval: Polling Interval */
  /* 41 */

  /************** Descriptor of CUSTOM HID interface ****************/

  0x09, /* bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
  0x01, /* bInterfaceNumber: Number of Interface */
  0x00, /* bAlternateSetting: Alternate setting */
  0x02, /* bNumEndpoints*/
  0x03, /*  bInterfaceClass: CUSTOM_HID */
  0x00, /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x00, /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0x00, /* iInterface: Index of string descriptor */
  /******************** Descriptor of CUSTOM_HID *************************/
  /* 50 */
  0x09, /* bLength: JOYSTICK_HID Descriptor size */
  HID_DESCRIPTOR_TYPE, /* bDescriptorType: JOYSTICK_HID */
  0x11, /* bJOYSTICK_HIDUSTOM_HID: JOYSTICK_HID Class Spec release number */
  0x01, 0x00, /* bCountryCode: Hardware target country */
  0x01, /* bNumDescriptors: Number of JOYSTICK_HID class descriptors to follow */
  0x22, /* bDescriptorType */
  LOBYTE(USBD_CUSTOM_HID_REPORT_DESC_SIZE), /* wItemLength: Total length of Report descriptor */
  HIBYTE(USBD_CUSTOM_HID_REPORT_DESC_SIZE),

  /******************** Descriptor of Custom HID endpoints ********************/
  /* 59 */
  0x07, /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */

  CUSTOM_HID_EPIN_ADDR, /* bEndpointAddress: Endpoint Address (IN) */
  0x03, /* bmAttributes: Interrupt endpoint */
  CUSTOM_HID_EPIN_SIZE, /* wMaxPacketSize: 2 Byte max */
  0x00,
  CUSTOM_HID_FS_BINTERVAL, /* bInterval: Polling Interval */
  /* 66 */

  0x07, /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  CUSTOM_HID_EPOUT_ADDR, /* bEndpointAddress: Endpoint Address (OUT) */
  0x03, /* bmAttributes: Interrupt endpoint */
  LOBYTE(CUSTOM_HID_EPOUT_SIZE), /* wMaxPacketSize: 2 Bytes max  */
  HIBYTE(CUSTOM_HID_EPOUT_SIZE),
  CUSTOM_HID_FS_BINTERVAL, /* bInterval: Polling Interval */
  /* 73 */

  };

/* USB JOYSTICK_HID device HS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgHSDesc[USB_COMPOSITE_HID_CONFIG_DESC_SIZ] __ALIGN_END
=
  { 0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  LOBYTE(USB_COMPOSITE_HID_CONFIG_DESC_SIZ), /* wTotalLength: Bytes returned */
  HIBYTE(USB_COMPOSITE_HID_CONFIG_DESC_SIZ),
  USBD_MAX_NUM_INTERFACES, /* bNumInterfaces: 1 interface */
  0x01, /* bConfigurationValue: Configuration value */
  0x00, /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0, /* bmAttributes: bus powered */
  0x32, /* MaxPower 100 mA: this current is used for detecting Vbus */

  /************** Descriptor of CUSTOM HID interface ****************/
  /* 09 */
  0x09, /* bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
  HID_JOYSTICK_INTERFACE, /* bInterfaceNumber: Number of Interface */
  0x00, /* bAlternateSetting: Alternate setting */
  0x02, /* bNumEndpoints*/
  0x03, /* bInterfaceClass: JOYSTICK_HID */
  0x00, /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x00, /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0x00, /* iInterface: Index of string descriptor */
  /******************** Descriptor of JOYSTICK_HID *************************/
  /* 18 */
  0x09, /* bLength: JOYSTICK_HID Descriptor size */
  HID_DESCRIPTOR_TYPE, /* bDescriptorType: JOYSTICK_HID */
  0x11, /* bJOYSTICK_HIDUSTOM_HID: JOYSTICK_HID Class Spec release number */
  0x01, 0x00, /* bCountryCode: Hardware target country */
  0x01, /* bNumDescriptors: Number of JOYSTICK_HID class descriptors to follow */
  0x22, /* bDescriptorType */
  LOBYTE(USBD_JOYSTICK_HID_REPORT_DESC_SIZE), /* wItemLength: Total length of Report descriptor */
  HIBYTE(USBD_JOYSTICK_HID_REPORT_DESC_SIZE),
  /******************** Descriptor of Custom HID endpoints ********************/
  /* 27 */
  0x07, /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */

  JOYSTICK_HID_EPIN_ADDR, /* bEndpointAddress: Endpoint Address (IN) */
  0x03, /* bmAttributes: Interrupt endpoint */
  JOYSTICK_HID_EPIN_SIZE, /* wMaxPacketSize: 2 Byte max */
  0x00,
  JOYSTICK_HID_FS_BINTERVAL, /* bInterval: Polling Interval */
  /* 34 */

  0x07, /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  JOYSTICK_HID_EPOUT_ADDR, /* bEndpointAddress: Endpoint Address (OUT) */
  0x03, /* bmAttributes: Interrupt endpoint */
  JOYSTICK_HID_EPOUT_SIZE, /* wMaxPacketSize: 2 Bytes max  */
  0x00,
  JOYSTICK_HID_FS_BINTERVAL, /* bInterval: Polling Interval */
  /* 41 */

  /************** Descriptor of CUSTOM HID interface ****************/

  0x09, /* bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
  0x01, /* bInterfaceNumber: Number of Interface */
  0x00, /* bAlternateSetting: Alternate setting */
  0x02, /* bNumEndpoints*/
  0x03, /* bInterfaceClass: CUSTOM_HID */
  0x00, /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x00, /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0x00, /* iInterface: Index of string descriptor */
  /******************** Descriptor of CUSTOM_HID *************************/
  /* 50 */
  0x09, /* bLength: JOYSTICK_HID Descriptor size */
  HID_DESCRIPTOR_TYPE, /* bDescriptorType: JOYSTICK_HID */
  0x11, /* bJOYSTICK_HIDUSTOM_HID: JOYSTICK_HID Class Spec release number */
  0x01, 0x00, /* bCountryCode: Hardware target country */
  0x01, /* bNumDescriptors: Number of JOYSTICK_HID class descriptors to follow */
  0x22, /* bDescriptorType */
  LOBYTE(USBD_CUSTOM_HID_REPORT_DESC_SIZE), /* wItemLength: Total length of Report descriptor */
  HIBYTE(USBD_CUSTOM_HID_REPORT_DESC_SIZE),

  /******************** Descriptor of Custom HID endpoints ********************/
  /* 59 */
  0x07, /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */

  CUSTOM_HID_EPIN_ADDR, /* bEndpointAddress: Endpoint Address (IN) */
  0x03, /* bmAttributes: Interrupt endpoint */
  CUSTOM_HID_EPIN_SIZE, /* wMaxPacketSize: 2 Byte max */
  0x00,
  CUSTOM_HID_FS_BINTERVAL, /* bInterval: Polling Interval */
  /* 66 */

  0x07, /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  CUSTOM_HID_EPOUT_ADDR, /* bEndpointAddress: Endpoint Address (OUT) */
  0x03, /* bmAttributes: Interrupt endpoint */
  LOBYTE(CUSTOM_HID_EPOUT_SIZE), /* wMaxPacketSize: 2 Bytes max  */
  HIBYTE(CUSTOM_HID_EPOUT_SIZE),
  CUSTOM_HID_FS_BINTERVAL, /* bInterval: Polling Interval */
  /* 73 */
  };

/* USB JOYSTICK_HID device Other Speed Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_OtherSpeedCfgDesc[USB_COMPOSITE_HID_CONFIG_DESC_SIZ] __ALIGN_END
=
  { 0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  LOBYTE(USB_COMPOSITE_HID_CONFIG_DESC_SIZ), /* wTotalLength: Bytes returned */
  HIBYTE(USB_COMPOSITE_HID_CONFIG_DESC_SIZ),
  USBD_MAX_NUM_INTERFACES, /* bNumInterfaces: 1 interface */
  0x01, /* bConfigurationValue: Configuration value */
  0x00, /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0, /* bmAttributes: bus powered */
  0x32, /* MaxPower 100 mA: this current is used for detecting Vbus */

  /************** Descriptor of CUSTOM HID interface ****************/
  /* 09 */
  0x09, /* bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
  HID_JOYSTICK_INTERFACE, /* bInterfaceNumber: Number of Interface */
  0x00, /* bAlternateSetting: Alternate setting */
  0x02, /* bNumEndpoints*/
  0x03, /* bInterfaceClass: JOYSTICK_HID */
  0x00, /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x00, /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0x00, /* iInterface: Index of string descriptor */
  /******************** Descriptor of JOYSTICK_HID *************************/
  /* 18 */
  0x09, /* bLength: JOYSTICK_HID Descriptor size */
  HID_DESCRIPTOR_TYPE, /* bDescriptorType: JOYSTICK_HID */
  0x11, /* bJOYSTICK_HIDUSTOM_HID: JOYSTICK_HID Class Spec release number */
  0x01, 0x00, /* bCountryCode: Hardware target country */
  0x01, /* bNumDescriptors: Number of JOYSTICK_HID class descriptors to follow */
  0x22, /* bDescriptorType */
  LOBYTE(USBD_JOYSTICK_HID_REPORT_DESC_SIZE), /* wItemLength: Total length of Report descriptor */
  HIBYTE(USBD_JOYSTICK_HID_REPORT_DESC_SIZE),
  /******************** Descriptor of Custom HID endpoints ********************/
  /* 27 */
  0x07, /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */

  JOYSTICK_HID_EPIN_ADDR, /* bEndpointAddress: Endpoint Address (IN) */
  0x03, /* bmAttributes: Interrupt endpoint */
  JOYSTICK_HID_EPIN_SIZE, /* wMaxPacketSize: 2 Byte max */
  0x00,
  JOYSTICK_HID_FS_BINTERVAL, /* bInterval: Polling Interval */
  /* 34 */

  0x07, /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  JOYSTICK_HID_EPOUT_ADDR, /* bEndpointAddress: Endpoint Address (OUT) */
  0x03, /* bmAttributes: Interrupt endpoint */
  JOYSTICK_HID_EPOUT_SIZE, /* wMaxPacketSize: 2 Bytes max  */
  0x00,
  JOYSTICK_HID_FS_BINTERVAL, /* bInterval: Polling Interval */
  /* 41 */

  /************** Descriptor of CUSTOM HID interface ****************/

  0x09, /* bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
  0x01, /* bInterfaceNumber: Number of Interface */
  0x00, /* bAlternateSetting: Alternate setting */
  0x02, /* bNumEndpoints*/
  0x03, /* bInterfaceClass: CUSTOM_HID */
  0x00, /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x00, /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0x00, /* iInterface: Index of string descriptor */
  /******************** Descriptor of CUSTOM_HID *************************/
  /* 50 */
  0x09, /* bLength: JOYSTICK_HID Descriptor size */
  HID_DESCRIPTOR_TYPE, /* bDescriptorType: JOYSTICK_HID */
  0x11, /* bJOYSTICK_HIDUSTOM_HID: JOYSTICK_HID Class Spec release number */
  0x01, 0x00, /* bCountryCode: Hardware target country */
  0x01, /* bNumDescriptors: Number of JOYSTICK_HID class descriptors to follow */
  0x22, /* bDescriptorType */
  LOBYTE(USBD_CUSTOM_HID_REPORT_DESC_SIZE), /* wItemLength: Total length of Report descriptor */
  HIBYTE(USBD_CUSTOM_HID_REPORT_DESC_SIZE),

  /******************** Descriptor of Custom HID endpoints ********************/
  /* 59 */
  0x07, /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */

  CUSTOM_HID_EPIN_ADDR, /* bEndpointAddress: Endpoint Address (IN) */
  0x03, /* bmAttributes: Interrupt endpoint */
  CUSTOM_HID_EPIN_SIZE, /* wMaxPacketSize: 2 Byte max */
  0x00,
  CUSTOM_HID_FS_BINTERVAL, /* bInterval: Polling Interval */
  /* 66 */

  0x07, /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  CUSTOM_HID_EPOUT_ADDR, /* bEndpointAddress: Endpoint Address (OUT) */
  0x03, /* bmAttributes: Interrupt endpoint */
  LOBYTE(CUSTOM_HID_EPOUT_SIZE), /* wMaxPacketSize: 2 Bytes max  */
  HIBYTE(CUSTOM_HID_EPOUT_SIZE),
  CUSTOM_HID_FS_BINTERVAL, /* bInterval: Polling Interval */
  /* 73 */
  };

//#define D_HIDREPORT(length)		{ 9, JOYSTICK_HID_DESCRIPTOR_TYPE, 0x11, 0x01, 0, 1, 0x22, LOBYTE(length), HIBYTE(length)}

/* USB JOYSTICK_HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_JOYSTICK_HID_Desc[USB_JOYSTICK_HID_DESC_SIZ] __ALIGN_END
=
  {
  /* 18 */
  0x09, /* bLength: JOYSTICK_HID Descriptor size */
  HID_DESCRIPTOR_TYPE, /* bDescriptorType: JOYSTICK_HID */
  0x11, /* bJOYSTICK_HIDUSTOM_HID: JOYSTICK_HID Class Spec release number */
  0x01, 0x00, /* bCountryCode: Hardware target country */
  0x01, /* bNumDescriptors: Number of JOYSTICK_HID class descriptors to follow */
  0x22, /* bDescriptorType */
  LOBYTE(USBD_JOYSTICK_HID_REPORT_DESC_SIZE), /* wItemLength: Total length of Report descriptor */
  HIBYTE(USBD_JOYSTICK_HID_REPORT_DESC_SIZE), };

/* USB CUSTOM_HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CUSTOM_HID_Desc[USB_CUSTOM_HID_DESC_SIZ] __ALIGN_END
=
  {
  /* 18 */
  0x09, /* bLength: JOYSTICK_HID Descriptor size */
  HID_DESCRIPTOR_TYPE, /* bDescriptorType: JOYSTICK_HID */
  0x11, /* bJOYSTICK_HIDUSTOM_HID: JOYSTICK_HID Class Spec release number */
  0x01, 0x00, /* bCountryCode: Hardware target country */
  0x01, /* bNumDescriptors: Number of JOYSTICK_HID class descriptors to follow */
  0x22, /* bDescriptorType */
  LOBYTE(USBD_CUSTOM_HID_REPORT_DESC_SIZE), /* wItemLength: Total length of Report descriptor */
  HIBYTE(USBD_CUSTOM_HID_REPORT_DESC_SIZE), };

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END
    =
      {
      USB_LEN_DEV_QUALIFIER_DESC,
      USB_DESC_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x01,
	  0x00, };

/**
 * @}
 */

/** @defgroup USBD_JOYSTICK_HID_Private_Functions
 * @{
 */

/**
 * @brief  USBD_COMPOSITE_HID_Init
 *         Initialize the JOYSTICK_HID interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t
USBD_COMPOSITE_HID_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_HID_HandleTypeDef *hhid;

  hhid = USBD_malloc (sizeof(USBD_HID_HandleTypeDef));

  if (hhid == NULL)
    {
      pdev->pClassData = NULL;
      return (uint8_t) USBD_EMEM;
    }

  pdev->pClassData = (void*) hhid;

  if (pdev->dev_speed == USBD_SPEED_HIGH)
    {
		pdev->ep_in[JOYSTICK_HID_EPIN_ADDR & 0xFU].bInterval = JOYSTICK_HID_HS_BINTERVAL;
		pdev->ep_out[JOYSTICK_HID_EPOUT_ADDR & 0xFU].bInterval = JOYSTICK_HID_HS_BINTERVAL;
		pdev->ep_in[CUSTOM_HID_EPIN_ADDR & 0xFU].bInterval = CUSTOM_HID_HS_BINTERVAL;
		pdev->ep_out[CUSTOM_HID_EPOUT_ADDR & 0xFU].bInterval = CUSTOM_HID_HS_BINTERVAL;
    }
  else /* LOW and FULL-speed endpoints */
    {
		pdev->ep_in[JOYSTICK_HID_EPIN_ADDR & 0xFU].bInterval = JOYSTICK_HID_FS_BINTERVAL;
		pdev->ep_out[JOYSTICK_HID_EPOUT_ADDR & 0xFU].bInterval = JOYSTICK_HID_FS_BINTERVAL;
		pdev->ep_in[CUSTOM_HID_EPIN_ADDR & 0xFU].bInterval = CUSTOM_HID_FS_BINTERVAL;
		pdev->ep_out[CUSTOM_HID_EPOUT_ADDR & 0xFU].bInterval = CUSTOM_HID_FS_BINTERVAL;
    }

  /* Open EP IN */
	(void) USBD_LL_OpenEP(pdev, JOYSTICK_HID_EPIN_ADDR, USBD_EP_TYPE_INTR, JOYSTICK_HID_EPIN_SIZE);
  pdev->ep_in[JOYSTICK_HID_EPIN_ADDR & 0xFU].is_used = 1U;

	(void) USBD_LL_OpenEP(pdev, CUSTOM_HID_EPIN_ADDR, USBD_EP_TYPE_INTR, CUSTOM_HID_EPIN_SIZE);
  pdev->ep_in[CUSTOM_HID_EPIN_ADDR & 0xFU].is_used = 1U;

  /* Open EP OUT */
	(void) USBD_LL_OpenEP(pdev, JOYSTICK_HID_EPOUT_ADDR, USBD_EP_TYPE_INTR, JOYSTICK_HID_EPOUT_SIZE);
  pdev->ep_out[JOYSTICK_HID_EPOUT_ADDR & 0xFU].is_used = 1U;

	(void) USBD_LL_OpenEP(pdev, CUSTOM_HID_EPOUT_ADDR, USBD_EP_TYPE_INTR, CUSTOM_HID_EPOUT_SIZE);
  pdev->ep_out[CUSTOM_HID_EPOUT_ADDR & 0xFU].is_used = 1U;

  hhid->JoystickState = HID_IDLE;
  hhid->CustomHidState = HID_IDLE;

  ((USBD_COMPOSITE_HID_ItfTypeDef*) pdev->pUserData)->Init ();

  /* Prepare Out endpoint to receive 1st packet */
  (void) USBD_LL_PrepareReceive (pdev, JOYSTICK_HID_EPOUT_ADDR,
				 hhid->Joyctick_buf,
				 USBD_JOYSTICKHID_OUTREPORT_BUF_SIZE);
  (void) USBD_LL_PrepareReceive (pdev, CUSTOM_HID_EPOUT_ADDR,
				 hhid->CustomHid_buf, HID_FEATURE_LENGTH);

  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_COMPOSITE_HID_Init
 *         DeInitialize the JOYSTICK_HID layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t
USBD_COMPOSITE_HID_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  /* Close JOYSTICK_HID EP IN */
  (void) USBD_LL_CloseEP (pdev, JOYSTICK_HID_EPIN_ADDR);
  pdev->ep_in[JOYSTICK_HID_EPIN_ADDR & 0xFU].is_used = 0U;
  pdev->ep_in[JOYSTICK_HID_EPIN_ADDR & 0xFU].bInterval = 0U;

  (void) USBD_LL_CloseEP (pdev, CUSTOM_HID_EPIN_ADDR);
  pdev->ep_in[CUSTOM_HID_EPIN_ADDR & 0xFU].is_used = 0U;
  pdev->ep_in[CUSTOM_HID_EPIN_ADDR & 0xFU].bInterval = 0U;

  /* Close JOYSTICK_HID EP OUT */
  (void) USBD_LL_CloseEP (pdev, JOYSTICK_HID_EPOUT_ADDR);
  pdev->ep_out[JOYSTICK_HID_EPOUT_ADDR & 0xFU].is_used = 0U;
  pdev->ep_out[JOYSTICK_HID_EPOUT_ADDR & 0xFU].bInterval = 0U;

  (void) USBD_LL_CloseEP (pdev, CUSTOM_HID_EPOUT_ADDR);
  pdev->ep_out[CUSTOM_HID_EPOUT_ADDR & 0xFU].is_used = 0U;
  pdev->ep_out[CUSTOM_HID_EPOUT_ADDR & 0xFU].bInterval = 0U;

  /* FRee allocated memory */
  if (pdev->pClassData != NULL)
    {
      ((USBD_COMPOSITE_HID_ItfTypeDef*) pdev->pUserData)->DeInit ();
      USBD_free (pdev->pClassData);
      pdev->pClassData = NULL;
    }

  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_JOYSTICK_HID_Setup
 *         Handle the JOYSTICK_HID specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t
USBD_JOYSTICK_HID_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef*) pdev->pClassData;
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;
  uint8_t buffer[USBD_JOYSTICKHID_INREPORT_BUF_SIZE];
  int8_t state;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
	{
	case HID_REQ_SET_PROTOCOL:
	  hhid->Protocol = (uint8_t) (req->wValue);
	  break;

	case HID_REQ_GET_PROTOCOL:
	  (void) USBD_CtlSendData (pdev, (uint8_t*) &hhid->Protocol, 1U);
	  break;

	case HID_REQ_SET_IDLE:
	  hhid->IdleState = (uint8_t) (req->wValue >> 8);
	  break;

	case HID_REQ_GET_IDLE:
	  (void) USBD_CtlSendData (pdev, (uint8_t*) &hhid->IdleState, 1U);
	  break;

	case HID_REQ_SET_REPORT:
	  hhid->IsReportAvailable = 1U;
	  (void) USBD_CtlPrepareRx (pdev, hhid->Joyctick_buf, req->wLength);
	  break;
	case HID_REQ_GET_REPORT:
	  state =
	      ((USBD_COMPOSITE_HID_ItfTypeDef*) pdev->pUserData)->GetFeature (
		  req->wValue, req->wIndex, buffer, &len);
	  if (state == USBD_OK)
	    {
	      USBD_CtlSendData (pdev, buffer, len);
	    }
	  else
	    {
	      USBD_CtlError (pdev, req);
	      return USBD_FAIL;
	    }
	  break;
	default:
	  USBD_CtlError (pdev, req);
	  ret = USBD_FAIL;
	  break;
	}
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
	{
	case USB_REQ_GET_STATUS:
	  if (pdev->dev_state == USBD_STATE_CONFIGURED)
	    {
	      (void) USBD_CtlSendData (pdev, (uint8_t*) &status_info, 2U);
	    }
	  else
	    {
	      USBD_CtlError (pdev, req);
	      ret = USBD_FAIL;
	    }
	  break;

	case USB_REQ_GET_DESCRIPTOR:
	  if ((req->wValue >> 8) == HID_REPORT_DESC)
	    {
	      //len = MIN(USBD_JOYSTICK_HID_REPORT_DESC_SIZE, req->wLength);
	      len = MIN(JoystickHIDReportDescr_Size, req->wLength);
	      pbuf =
		  ((USBD_COMPOSITE_HID_ItfTypeDef*) pdev->pUserData)->pReport;
	    }
	  else
	    {
	      if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE)
		{
		  pbuf = USBD_JOYSTICK_HID_Desc;
		  len = MIN(USB_JOYSTICK_HID_DESC_SIZ, req->wLength);
		}
	      else
		{
		  USBD_CtlError (pdev, req);
		  ret = USBD_FAIL;
		  break;
		}
	    }

	  (void) USBD_CtlSendData (pdev, pbuf, len);
	  break;

	case USB_REQ_GET_INTERFACE:
	  if (pdev->dev_state == USBD_STATE_CONFIGURED)
	    {
	      (void) USBD_CtlSendData (pdev, (uint8_t*) &hhid->AltSetting, 1U);
	    }
	  else
	    {
	      USBD_CtlError (pdev, req);
	      ret = USBD_FAIL;
	    }
	  break;

	case USB_REQ_SET_INTERFACE:
	  if (pdev->dev_state == USBD_STATE_CONFIGURED)
	    {
	      hhid->AltSetting = (uint8_t) (req->wValue);
	    }
	  else
	    {
	      USBD_CtlError (pdev, req);
	      ret = USBD_FAIL;
	    }
	  break;

	case USB_REQ_CLEAR_FEATURE:
	  break;

	default:
	  USBD_CtlError (pdev, req);
	  ret = USBD_FAIL;
	  break;
	}
      break;

    default:
      USBD_CtlError (pdev, req);
      ret = USBD_FAIL;
      break;
    }
  return (uint8_t) ret;
}

static uint8_t
USBD_CUSTOM_HID_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef*) pdev->pClassData;
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  uint8_t buffer[HID_FEATURE_LENGTH];
  int8_t state;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
	{
	case HID_REQ_SET_PROTOCOL:
	  hhid->Protocol = (uint8_t) (req->wValue);
	  break;

	case HID_REQ_GET_PROTOCOL:
	  (void) USBD_CtlSendData (pdev, (uint8_t*) &hhid->Protocol, 1U);
	  break;

	case HID_REQ_SET_IDLE:
	  hhid->IdleState = (uint8_t) (req->wValue >> 8);
	  break;

	case HID_REQ_GET_IDLE:
	  (void) USBD_CtlSendData (pdev, (uint8_t*) &hhid->IdleState, 1U);
	  break;

	case HID_REQ_SET_REPORT:
	  hhid->IsReportAvailable = 1U;
	  (void) USBD_CtlPrepareRx (pdev, hhid->CustomHid_buf, req->wLength);
	  break;
	case HID_REQ_GET_REPORT:
	  state =
	      ((USBD_COMPOSITE_HID_ItfTypeDef*) pdev->pUserData)->GetFeature (
		  req->wValue, req->wIndex, buffer, &len);
	  if (state == USBD_OK)
	    {
	      USBD_CtlSendData (pdev, buffer, len);
	    }
	  else
	    {
	      USBD_CtlError (pdev, req);
	      return USBD_FAIL;
	    }
	  break;

	default:
	  USBD_CtlError (pdev, req);
	  ret = USBD_FAIL;
	  break;
	}
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
	{
	case USB_REQ_GET_STATUS:
	  if (pdev->dev_state == USBD_STATE_CONFIGURED)
	    {
	      (void) USBD_CtlSendData (pdev, (uint8_t*) &status_info, 2U);
	    }
	  else
	    {
	      USBD_CtlError (pdev, req);
	      ret = USBD_FAIL;
	    }
	  break;

	case USB_REQ_GET_DESCRIPTOR:
	  if ((req->wValue >> 8) == HID_REPORT_DESC)
	    {
	      len = MIN(USBD_CUSTOM_HID_REPORT_DESC_SIZE, req->wLength);
	      //len = MIN(JoystickHIDReportDescr_Size, req->wLength);
	      pbuf = CUSTOM_HID_ReportDesc_FS; //((USBD_COMPOSITE_HID_ItfTypeDef *)pdev->pUserData)->pReport;
	    }
	  else
	    {
	      if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE)
		{
		  pbuf = USBD_CUSTOM_HID_Desc;
		  len = MIN(USB_CUSTOM_HID_DESC_SIZ, req->wLength);
		}
	    }

	  (void) USBD_CtlSendData (pdev, pbuf, len);
	  break;

	case USB_REQ_GET_INTERFACE:
	  if (pdev->dev_state == USBD_STATE_CONFIGURED)
	    {
	      (void) USBD_CtlSendData (pdev, (uint8_t*) &hhid->AltSetting, 1U);
	    }
	  else
	    {
	      USBD_CtlError (pdev, req);
	      ret = USBD_FAIL;
	    }
	  break;

	case USB_REQ_SET_INTERFACE:
	  if (pdev->dev_state == USBD_STATE_CONFIGURED)
	    {
	      hhid->AltSetting = (uint8_t) (req->wValue);
	    }
	  else
	    {
	      USBD_CtlError (pdev, req);
	      ret = USBD_FAIL;
	    }
	  break;

	case USB_REQ_CLEAR_FEATURE:
	  break;

	default:
	  USBD_CtlError (pdev, req);
	  ret = USBD_FAIL;
	  break;
	}
      break;

    default:
      USBD_CtlError (pdev, req);
      ret = USBD_FAIL;
      break;
    }
  return (uint8_t) ret;
}

static uint8_t
USBD_COMPOSITE_HID_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  /* Check which interface is targetted by this request */
  if ((req->wIndex & 0x00FF) == HID_JOYSTICK_INTERFACE)
    {
      return USBD_JOYSTICK_HID_Setup (pdev, req);
    }
  else
    {
      return USBD_CUSTOM_HID_Setup (pdev, req);
    }
}

/**
 * @brief  USBD_JOYSTICK_HID_SendReport
 *         Send JOYSTICK_HID Report
 * @param  pdev: device instance
 * @param  buff: pointer to report
 * @retval status
 */
uint8_t
USBD_JOYSTICK_HID_SendReport (USBD_HandleTypeDef *pdev, uint8_t *report,
			      uint16_t len)
{
  USBD_HID_HandleTypeDef *hhid;

  if (pdev->pClassData == NULL)
    {
      return (uint8_t) USBD_FAIL;
    }

  hhid = (USBD_HID_HandleTypeDef*) pdev->pClassData;

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
    {
      if (hhid->JoystickState == HID_IDLE)
	{
	  hhid->JoystickState = HID_BUSY;
	  (void) USBD_LL_Transmit (pdev, JOYSTICK_HID_EPIN_ADDR, report, len);
	}
      else
	{
	  return (uint8_t) USBD_BUSY;
	}
    }
  return (uint8_t) USBD_OK;
}

uint8_t
USBD_CUSTOM_HID_SendReport (USBD_HandleTypeDef *pdev, uint8_t *report,
			    uint16_t len)
{
  USBD_HID_HandleTypeDef *hhid;

  if (pdev->pClassData == NULL)
    {
      return (uint8_t) USBD_FAIL;
    }

  hhid = (USBD_HID_HandleTypeDef*) pdev->pClassData;

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
    {
      if (hhid->CustomHidState == HID_IDLE)
	{
	  hhid->CustomHidState = HID_BUSY;
	  (void) USBD_LL_Transmit (pdev, CUSTOM_HID_EPIN_ADDR, report, len);
	}
      else
	{
	  return (uint8_t) USBD_BUSY;
	}
    }
  return (uint8_t) USBD_OK;
}

static void Update_HID_Descriptor_Size ()
{
  USBD_HID_CfgFSDesc[25] = LOBYTE(JoystickHIDReportDescr_Size);
  USBD_HID_CfgFSDesc[26] = HIBYTE(JoystickHIDReportDescr_Size);

  USBD_JOYSTICK_HID_Desc[7] = LOBYTE(JoystickHIDReportDescr_Size);
  USBD_JOYSTICK_HID_Desc[8] = HIBYTE(JoystickHIDReportDescr_Size);

}
/**
 * @brief  USBD_HID_GetFSCfgDesc
 *         return FS configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t* USBD_HID_GetFSCfgDesc (uint16_t *length)
{
		Update_HID_Descriptor_Size ();
		*length = (uint16_t) sizeof(USBD_HID_CfgFSDesc);

		return USBD_HID_CfgFSDesc;
}

/**
 * @brief  USBD_HID_GetHSCfgDesc
 *         return HS configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t*
USBD_HID_GetHSCfgDesc (uint16_t *length)
{
		Update_HID_Descriptor_Size ();
		*length = (uint16_t) sizeof(USBD_HID_CfgHSDesc);

  return USBD_HID_CfgHSDesc;
}

/**
 * @brief  USBD_HID_GetOtherSpeedCfgDesc
 *         return other speed configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t*
USBD_HID_GetOtherSpeedCfgDesc (uint16_t *length)
{
		Update_HID_Descriptor_Size ();
		*length = (uint16_t) sizeof(USBD_HID_OtherSpeedCfgDesc);

  return USBD_HID_OtherSpeedCfgDesc;
}

/**
 * @brief  USBD_HID_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t
USBD_JOYSTICK_HID_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(epnum);

  /* Ensure that the FIFO is empty before a new transfer, this condition could
   be caused by  a new transfer before the end of the previous transfer */

  if (epnum == (JOYSTICK_HID_EPIN_ADDR & 0x7F))
    {
      ((USBD_HID_HandleTypeDef*) pdev->pClassData)->JoystickState = HID_IDLE;
    }
  else if (epnum == (CUSTOM_HID_EPIN_ADDR & 0x7F))
    {
      ((USBD_HID_HandleTypeDef*) pdev->pClassData)->CustomHidState = HID_IDLE;
    }
  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_JOYSTICK_HID_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t
USBD_JOYSTICK_HID_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(epnum);
  USBD_HID_HandleTypeDef *hhid;

  if (pdev->pClassData == NULL)
    {
      return (uint8_t) USBD_FAIL;
    }
  hhid = (USBD_HID_HandleTypeDef*) pdev->pClassData;

  /* USB data will be immediately processed, this allow next USB traffic being
   NAKed till the end of the application processing */

  if (epnum == (JOYSTICK_HID_EPOUT_ADDR & 0x7F))
    {
      ((USBD_COMPOSITE_HID_ItfTypeDef*) pdev->pUserData)->OutEvent (
	  epnum, hhid->Joyctick_buf);

    }
  else if (epnum == (CUSTOM_HID_EPOUT_ADDR & 0x7F))
    {
      ((USBD_COMPOSITE_HID_ItfTypeDef*) pdev->pUserData)->OutEvent (
	  epnum, hhid->CustomHid_buf);

    }

  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_JOYSTICK_HID_ReceivePacket
 *         prepare OUT Endpoint for reception
 * @param  pdev: device instance
 * @retval status
 */
uint8_t
USBD_JOYSTICK_HID_ReceivePacket (USBD_HandleTypeDef *pdev)
{
  USBD_HID_HandleTypeDef *hhid;

  if (pdev->pClassData == NULL)
    {
      return (uint8_t) USBD_FAIL;
    }

  hhid = (USBD_HID_HandleTypeDef*) pdev->pClassData;

  /* Resume USB Out process */
  (void) USBD_LL_PrepareReceive (pdev, JOYSTICK_HID_EPOUT_ADDR,
				 hhid->Joyctick_buf,
				 USBD_JOYSTICKHID_OUTREPORT_BUF_SIZE);

  return (uint8_t) USBD_OK;
}

uint8_t
USBD_CUSTOM_HID_ReceivePacket (USBD_HandleTypeDef *pdev)
{
  USBD_HID_HandleTypeDef *hhid;

  if (pdev->pClassData == NULL)
    {
      return (uint8_t) USBD_FAIL;
    }

  hhid = (USBD_HID_HandleTypeDef*) pdev->pClassData;

  /* Resume USB Out process */
  (void) USBD_LL_PrepareReceive (pdev, CUSTOM_HID_EPOUT_ADDR,
				 hhid->CustomHid_buf,
				 USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);

  return (uint8_t) USBD_OK;
}

/**
 * @brief  USBD_JOYSTICK_HID_EP0_RxReady
 *         Handles control request data.
 * @param  pdev: device instance
 * @retval status
 */
static uint8_t
USBD_JOYSTICK_HID_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef*) pdev->pClassData;
  USBD_SetupReqTypedef *req = (USBD_SetupReqTypedef *) &pdev->request;

  //memset(hhid->Joyctick_buf,0,USBD_JOYSTICKHID_OUTREPORT_BUF_SIZE);
  if (hhid->IsReportAvailable == 1U)
    {

      switch (req->wIndex)
	{
	case 0:

	  ((USBD_COMPOSITE_HID_ItfTypeDef*) pdev->pUserData)->SetFeature (
	      req->wValue, hhid->Joyctick_buf, req->wLength, req->wIndex);
	  break;
	case 1:
	  ((USBD_COMPOSITE_HID_ItfTypeDef*) pdev->pUserData)->SetFeature (
	      req->wValue, hhid->CustomHid_buf, req->wLength, req->wIndex);
	  break;
	default:
	  break;

	}

      hhid->IsReportAvailable = 0U;
    }

  return (uint8_t) USBD_OK;
}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t*
USBD_HID_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = (uint16_t) sizeof(USBD_HID_DeviceQualifierDesc);

  return USBD_HID_DeviceQualifierDesc;
}

/**
 * @brief  USBD_JOYSTICK_HID_RegisterInterface
 * @param  pdev: device instance
 * @param  fops: JOYSTICKHID Interface callback
 * @retval status
 */
uint8_t
USBD_JOYSTICK_HID_RegisterInterface (USBD_HandleTypeDef *pdev,
				     USBD_COMPOSITE_HID_ItfTypeDef *fops)
{
  if (fops == NULL)
    {
      return (uint8_t) USBD_FAIL;
    }

  pdev->pUserData = fops;

  return (uint8_t) USBD_OK;
}
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
