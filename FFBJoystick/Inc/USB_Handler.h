/*
 * USB_Handler.h
 *
 *  Created on: Jan 29, 2021
 *      Author: billy
 */

#ifndef INC_USB_HANDLER_H_
#define INC_USB_HANDLER_H_
#include "main.h"
//#include "PIDReportType.h"
#include "ffb.h"
#include "FFBConfig_DataType.h"

// HID Request Type HID1.11 Page 51 7.2.1 Get_Report Request
#define DYNAMIC_HID_REPORT_TYPE_INPUT   1
#define DYNAMIC_HID_REPORT_TYPE_OUTPUT  2
#define DYNAMIC_HID_REPORT_TYPE_FEATURE 3

#ifdef __cplusplus
extern "C"
{
#endif

  extern uint8_t USB_Recv_Buff[64];

  void RecvfromUsb (uint8_t *buff);

  void CreatNewEffect (USB_FFBReport_CreateNewEffect_Feature_Data_t *buffer);
  uint8_t* GetPIDBlockLoad (void);
  void SetPIDBlockLoadReportID (void);
  uint8_t* GetPIDStatus (void);
  uint8_t* Get_SysConfig (void);
  uint8_t * GetPIDPool(void);

  void HostToDevSetFeature (uint8_t *buff, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif /* INC_USB_HANDLER_H_ */
