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

#include "stdbool.h"

  struct __attribute__((packed)) TDF_AXIS
  {
    //volatile uint32_t cPR;
    //volatile uint16_t maxAngle;
    volatile int32_t maxValue;
    volatile int32_t minValue;
    volatile bool inverted;
    volatile uint32_t last_EncoderTime;
    volatile int32_t current_Position;
    volatile int32_t last_Position;
    volatile int32_t correct_Position;
    volatile float current_Speed;
    volatile float last_Speed;
    volatile float max_Speed;
    volatile float current_Acceleration;
    volatile float max_Acceleration;
    volatile float position_Changed;
    volatile float max_Position_Changed;
  };

  struct __attribute__((packed)) TDF_BUTTON
  {
    GPIO_TypeDef *Port;
    volatile uint16_t pinNumber;
    volatile int CurrentState;
    volatile int LastState;
    volatile uint32_t millis_time;
  };

#define X_AXIS						0
#define Y_AXIS						1
#define RX_AXIS					0
#define RY_AXIS					1

#define X_LIMIT_MAX				0
#define X_LIMIT_MIN				1
#define Y_LIMIT_MAX				2
#define Y_LIMIT_MIN				3

#define NUM_OF_ADC_CHANNELS	0
#define NUM_OF_ANALOG_AXIS		2

#define NUM_OF_ENC_AXIS			2
#define NUM_OF_LIMITSWITCH		4
#define NUM_OF_BUTTONS			16
#define NUM_OF_EXTI_BUTTONS	12
#define NUM_OF_HATSWITCH		1
#define HAT_START_PIN			5
#define HAT_END_PIN				8
#define JOYSTICK_PUSHBUTTON_A	9
#define JOYSTICK_PUSHBUTTON_B	4
#define JENC_PINA					10
#define JENC_PINB					11


#define ENCODER_MAX			 	1000000 		//-32767
#define ENCODER_MIN			  	-1000000		//32767

#define DIGITAL_AXIS_MIN		-10000
#define DIGITAL_AXIS_MAX		 10000

#define ADC_AXIS_MIN			 -2047
#define ADC_AXIS_MAX			 2047
#define XY_FORCE_MAX			32767
#define XY_FORCE_MIN			-32767

#define DEBOUNCE_TIME		   10

#define REPORT_FORCES_DATA		0x01
#define REPORT_JOYSTICK_DATA	0x02
#define AXIS_BACKWARD_X   		1000
#define AXIS_BACKWARD_Y   		600
#define ENDSTOP_ZONE				5

#define MIN_DAC_OUT_VOLT		0
#define MAX_DAC_OUT_VOLT		10000

#define USBLOG_INTERVAL		10

#define EFFECT_CALC_BY_TIME	1


typedef struct //PID state
{
	int32_t xy_forces[2];
	int32_t current_pos[2];
	int32_t axis_min[2];
	int32_t axis_max[2];
} USB_LoggerReport_t;


  extern TDF_BUTTON Buttons[NUM_OF_BUTTONS];
  extern TDF_BUTTON HatButtons[4];
  extern TDF_BUTTON Limit_Switch[NUM_OF_LIMITSWITCH];

  extern int32_t Encoder_TIM2_Counter;
  extern TDF_BUTTON Estop_Sw;

#if(NUM_OF_ADC_CHANNELS)
  extern uint16_t adc_buff[NUM_OF_ADC_CHANNELS];
  extern TDF_AXIS analog_axis[NUM_OF_ANALOG_AXIS];

#endif

  void init_Joystick();
  void start_joystick(void);
  void Set_Gains(void);
  void SetEffects(void);
  void Set_PID_Turnings();
  void AutoCalibration(uint8_t idx);
  void gotoPosition(int axis_num, int32_t targetPosition);
  void findCenter_Manual(int axis_num);
  void Set_RunFirstTime_state(bool state);
  void Send_Debug_Report();
  void findCenter_Auto();

  //void Correct_Joystick_Positions(int axis_num, int32_t targetPosition);
  float AutoCenter_spring(uint8_t ax);

#endif /* INC_FFBMAIN_H_ */
