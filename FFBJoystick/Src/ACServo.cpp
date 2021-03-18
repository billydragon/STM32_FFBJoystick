/*
 * ACServo.cpp
 *
 *  Created on: Mar 9, 2021
 *      Author: billy
 */

#include "ACServo.h"
#include "cppmain.h"
#include "DAC856x.h"
#include "FFBMain.h"
#include "FFBConfig.h"

#define DAC_STOP	map(0, -32767, 32767, DAC_MIN, DAC_MAX)
extern FFBConfig config;

ACServo::ACServo()
	{
		//printf("Init ACServo Driver\n");

	}

void ACServo::set_motor_dac(int32_t * _xy_forces)
	{
		int32_t x_force = _xy_forces[0];
		int32_t y_force = _xy_forces[1];
		int32_t x_speed = 0;
		int32_t y_speed = 0;

		//AC Servo set gain 500rpm/1Volt
		int32_t x_speed_min = config.SysConfig.AC_MotorSettings[0].Motor_Min_Speed * 327;
		int32_t x_speed_max = config.SysConfig.AC_MotorSettings[0].Motor_Max_Speed * 327;
		//printf("x_speed_min: %ld, x_speed_max: %ld\n",x_speed_min, x_speed_max);
		int32_t y_speed_min = config.SysConfig.AC_MotorSettings[1].Motor_Min_Speed * 327;
		int32_t y_speed_max = config.SysConfig.AC_MotorSettings[1].Motor_Max_Speed * 327;
		//printf("y_speed_min: %d, y_speed_max: %d\n",y_speed_min, y_speed_max);
		if(active == false)
			{
				DAC856x_Init ();
				active = true;
			}

		//torque X

		 if(x_force > 0)
		 {
			 x_speed = constrain(x_force,x_speed_min,x_speed_max);
		 }
		 else if (x_force < 0)
		 {
			 x_speed = constrain(x_force,-x_speed_max,-x_speed_min);
		 }
		 else
			 x_speed =0;

		 //printf("x_speed: %ld\n",x_speed);
		x_force = map(x_force, -32767, 32767, DAC_MIN, DAC_MAX);
		DAC856x_Set_Data(CS1, DAC_CH1,x_force);

		x_speed = map(x_speed, -32767, 32767, DAC_MIN, DAC_MAX);

		DAC856x_Set_Data(CS1, DAC_CH2,x_speed);

		//torque Y

		 if(y_force > 0)
		 		 {
		 			 y_speed = constrain(y_force,y_speed_min,y_speed_max);
		 		 }
		 		 else if (y_force < 0)
		 		 {
		 			 y_speed = constrain(y_force,-y_speed_max,-y_speed_min);
		 		 }
		 		 else
		 			 y_speed =0;

		y_force = map(y_force, -32767, 32767, DAC_MIN, DAC_MAX);
		DAC856x_Set_Data(CS2, DAC_CH1,y_force);
		y_speed = map(y_speed, -32767, 32767, DAC_MIN, DAC_MAX);
		DAC856x_Set_Data(CS2, DAC_CH2,y_speed);
	}

void ACServo::start(){
		HAL_Delay(500);
		DAC856x_Init ();
		active = true;
		//printf("Start ACServo Driver\n");
}

void ACServo::stop()
	{

		DAC856x_Set_Data(CS1, DAC_CH1,DAC_STOP);
		DAC856x_Set_Data(CS1, DAC_CH2,DAC_STOP);
		DAC856x_Set_Data(CS2, DAC_CH1,DAC_STOP);
		DAC856x_Set_Data(CS2, DAC_CH2,DAC_STOP);
		active = false;
		//printf("Stop ACServo Driver\n");
 	 }

bool ACServo::GetState()
{

	return active;
}
ACServo::~ACServo()
	{
		stop();
	}


int32_t ACServo::Voltage_Convert(float voltage)
{
  uint32_t _D_;

  voltage = voltage / 6  + 2.5;   //based on the manual provided by texas instruments

  _D_ = (uint16_t)(65536 * voltage / 5);

  if(_D_ < 32768)
  {
    _D_ -= 100;     //fix the errors
  }

  return _D_;
};
