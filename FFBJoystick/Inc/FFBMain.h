/*
 * FFBMain.h
 *
 *  Created on: Jan 26, 2021
 *      Author: duongnt
 */

#ifndef INC_FFBMAIN_H_
#define INC_FFBMAIN_H_
#pragma once
#ifdef __cplusplus
extern "C"
{
#endif
#include "cppmain.h"

#ifdef __cplusplus
}
#endif
  struct __attribute__((packed)) TDF_AXIS
  {
    volatile uint32_t cPR;
    volatile uint16_t maxAngle;
    volatile int32_t maxValue;
    volatile int32_t minValue;
    volatile bool inverted;
    volatile uint32_t lastEncoderTime;
    volatile int32_t currentPosition;
    volatile int32_t lastPosition;
    volatile int32_t correctPosition;
    volatile int32_t currentVelocity;
    volatile int32_t lastVelocity;
    volatile int32_t maxVelocity;
    volatile int32_t currentAcceleration;
    volatile int32_t maxAcceleration;
    volatile int32_t positionChange;
    volatile int32_t maxPositionChange;
  };

  struct __attribute__((packed)) TDF_BUTTON
  {
    GPIO_TypeDef *Port;
    volatile uint16_t pinNumber;
    volatile int CurrentState;
    volatile int LastState;
    volatile uint32_t millis_time;
  };

#define X_AXIS					0
#define Y_AXIS					1
#define RX_AXIS					0
#define RY_AXIS					1

#define X_LIMIT_MAX				0
#define X_LIMIT_MIN				1
#define Y_LIMIT_MAX				2
#define Y_LIMIT_MIN				3

#define NUM_OF_ADC_CHANNELS		6
#define NUM_OF_ENC_AXIS			2
#define NUM_OF_ANALOG_AXIS		2

#define NUM_OF_LIMITSWITCH		4
#define NUM_OF_BUTTONS			32
#define NUM_OF_HATSWITCH		0
#define ENCODER_MAX			 	1000000 		//-32767
#define ENCODER_MIN			  	-1000000		//32767

#define DIGITAL_AXIS_MIN		-10000
#define DIGITAL_AXIS_MAX		 10000

#define ADC_AXIS_MIN			 -2047
#define ADC_AXIS_MAX			 2047
#define XY_FORCE_MAX			32767
#define XY_FORCE_MIN			-32767

#define DEBOUNCE_TIME		   15

#define REPORT_FORCES_DATA		0x01
#define REPORT_JOYSTICK_DATA	0x02
#define AXIS_BACKWARD   		100

#define USBLOG_INTERVAL		10000

typedef struct //PID state
{
	int32_t xy_forces[2];
	int32_t current_pos[2];
	int32_t axis_min[2];
	int32_t axis_max[2];
} USB_LoggerReport_t;

  extern TDF_AXIS analog_axis[NUM_OF_ANALOG_AXIS];
  extern TDF_BUTTON Buttons[NUM_OF_BUTTONS];
  extern uint16_t adc_buff[NUM_OF_ADC_CHANNELS];
  extern TDF_BUTTON Limit_Switch[NUM_OF_LIMITSWITCH];


  void init_Joystick();
  void start_joystick(void);
  void Set_Gains(void);
  void SetEffects(void);
  void CalculateMaxSpeedAndMaxAcceleration();
  void Set_PID_Turnings();
  void AutoCalibration(uint8_t idx);
  void gotoPosition(int axis_num, int32_t targetPosition);
  void findCenter_Manual(int axis_num);
  void Set_RunFirstTime_state(bool state);
  void Send_Debug_Report();
  void findCenter_Auto();
  void LimitSwitch_trig(uint8_t lms_type);
#endif /* INC_FFBMAIN_H_ */
