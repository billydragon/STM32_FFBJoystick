/*
 * FFBConfig_DataType.h
 *
 *  Created on: Feb 27, 2021
 *      Author: billy
 */

#ifndef INC_FFBCONFIG_DATATYPE_H_
#define INC_FFBCONFIG_DATATYPE_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"
#include "stdint.h"

	enum PWM_MODE
	{
		  RC_PWM=0,
		  CENTERED_PWM=1,
		  PWM_DIR=2,
		  PWM_DUAL=3
	};

	enum PWM_SPEED
	{
		LOW=0,
		MID=1,
		HIGH=2,
		VERYHIGH=3
	};

	enum MOTOR_TYPE
	{
		PWM_OUTPUT=0,
		DAC856x_OUTPUT,
		DAC_INT_OUTPUT,
	};

  typedef struct
  {
    uint8_t totalGain;
    uint8_t constantGain;
    uint8_t rampGain;
    uint8_t squareGain;
    uint8_t sineGain;
    uint8_t triangleGain;
    uint8_t sawtoothdownGain;
    uint8_t sawtoothupGain;
    uint8_t springGain;
    uint8_t damperGain;
    uint8_t inertiaGain;
    uint8_t frictionGain;
    uint8_t customGain;
  } __attribute__((packed)) GAIN_PARAM;

  typedef struct
  {
    float MaxOutput;
    float SampleTime;
    float Kp;
    float Ki;
    float Kd;
  } __attribute__((packed)) PID_PARAM; //pid;

  typedef struct
   {
     float Cutoff_Freq;
     float SamplingTime;
     float Order;
   } __attribute__((packed)) FILTER_PARAM; //pid;

  typedef struct
  {
    uint8_t Motor_Inv_X;
    uint8_t Motor_Inv_Y;
    uint8_t Swap_xy_forces;
    uint8_t Auto_Calibration;
    uint8_t Axis_Inv_X;
    uint8_t Axis_Inv_Y;
    uint8_t Motor_Driver;
    uint8_t Reserve3;
    int32_t Home_Speed;
  } __attribute__((packed)) APP_PARAM;

  typedef struct
  {
    uint8_t HW_Version;
    uint8_t SW_Version;
  } __attribute__((packed)) FW_VERSION;

  typedef struct
  {
    int32_t Motor_Min_Speed;
    int32_t Motor_Max_Speed;
    int32_t Motor_Min_Torque;
    int32_t Motor_Max_Torque;
    int32_t Dead_Zone;
  } __attribute__((packed)) ACSERVO_PARAM;

  typedef struct
   {
     uint8_t PWM_Mode;
     uint8_t PWM_Speed;

   } __attribute__((packed)) PWM_PARAM;

  typedef struct
  {
    FW_VERSION FW_Version;
    APP_PARAM AppConfig;
    GAIN_PARAM Gain[2];
    PID_PARAM Pid[2];
    ACSERVO_PARAM AC_MotorSettings[2];
    PWM_PARAM PWM_Settings;
    FILTER_PARAM Filter_Parameter;
    uint16_t Checksum;
  }  SYS_CONFIG_t;





#ifdef __cplusplus
}
#endif
#endif /* INC_FFBCONFIG_DATATYPE_H_ */
