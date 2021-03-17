/*
 * USB_Handler.c
 *
 *  Created on: Jan 29, 2021
 *      Author: billy
 */
#include <usbd_joystick_hid_if.h>
#include "USB_Handler.h"
#include "PIDReportHandler.h"
#include "PIDReportType.h"
#include "FFBConfig.h"

extern USBD_COMPOSITE_HID_ItfTypeDef USBD_JoystickHID_fops_FS;
extern PIDReportHandler pidReportHandler;
extern FFBConfig config;

uint8_t USB_Recv_Buff[64];

void RecvfromUsb (uint8_t *buff)
{

  pidReportHandler.UppackUsbData (buff, 64);

}

void CreatNewEffect (USB_FFBReport_CreateNewEffect_Feature_Data_t *buffer)
{

  pidReportHandler.CreateNewEffect (buffer);

}

uint8_t* GetPIDBlockLoad ()
{
  return pidReportHandler.getPIDBlockLoad ();

}

uint8_t* GetPIDStatus ()
{
  return pidReportHandler.getPIDStatus ();
}

uint8_t* Get_SysConfig ()
{

  return config.GetSysConfig();

}

void HostToDevSetFeature (uint8_t *buff, uint16_t len)
{
  uint8_t *SetFeatureData = new uint8_t[len];
  memcpy (SetFeatureData, buff, len);
  config.Host_To_Dev_SetFeature (SetFeatureData);
}

void SetPIDBlockLoadReportID ()
{
  pidReportHandler.pidBlockLoad.reportId = 0;
}

