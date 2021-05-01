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

#define DEAD_ZONE	0
#define DAC_STOP	map(0, -32767, 32767, DAC_MIN, DAC_MAX)



extern FFBConfig config;

uint32_t active_time =0;

ACServo::ACServo()
	{
		//printf("Init ACServo Driver\n");

	}

void ACServo::set_motor_dac(int32_t * _xy_forces)
	{

		if((HAL_GetTick() - active_time) > 25000)
		{
			active = false;
		}

		if(active == false)
		{
			DAC856x_Init ();
			active = true;
			active_time = HAL_GetTick();

		}


		int32_t x_force = 0;
		int32_t y_force = 0;
		int32_t x_speed = 0;
		int32_t y_speed = 0;


		int32_t x_force_dead_zone = 4 * config.SysConfig.AC_MotorSettings[X_AXIS].Dead_Zone;
		int32_t y_force_dead_zone = 4 * config.SysConfig.AC_MotorSettings[Y_AXIS].Dead_Zone;


		int32_t x_speed_min = map(config.SysConfig.AC_MotorSettings[X_AXIS].Motor_Min_Speed,MIN_DAC_OUT_VOLT, MAX_DAC_OUT_VOLT,0,32767);
		int32_t x_speed_max = map(config.SysConfig.AC_MotorSettings[X_AXIS].Motor_Max_Speed,MIN_DAC_OUT_VOLT, MAX_DAC_OUT_VOLT,0,32767);
		int32_t y_speed_min = map(config.SysConfig.AC_MotorSettings[Y_AXIS].Motor_Min_Speed,MIN_DAC_OUT_VOLT, MAX_DAC_OUT_VOLT,0,32767);
		int32_t y_speed_max = map(config.SysConfig.AC_MotorSettings[Y_AXIS].Motor_Max_Speed,MIN_DAC_OUT_VOLT, MAX_DAC_OUT_VOLT,0,32767);
		int32_t x_torque_min = map(config.SysConfig.AC_MotorSettings[X_AXIS].Motor_Min_Torque,MIN_DAC_OUT_VOLT, MAX_DAC_OUT_VOLT,0,32767);
		int32_t x_torque_max = map(config.SysConfig.AC_MotorSettings[X_AXIS].Motor_Max_Torque,MIN_DAC_OUT_VOLT, MAX_DAC_OUT_VOLT,0,32767);
		int32_t y_torque_min = map(config.SysConfig.AC_MotorSettings[Y_AXIS].Motor_Min_Torque,MIN_DAC_OUT_VOLT, MAX_DAC_OUT_VOLT,0,32767);
		int32_t y_torque_max = map(config.SysConfig.AC_MotorSettings[Y_AXIS].Motor_Max_Torque,MIN_DAC_OUT_VOLT, MAX_DAC_OUT_VOLT,0,32767);


		float SPEED_SCALE_X = 0.65f;//config.SysConfig.AC_MotorSettings[X_AXIS].Motor_Max_Speed/config.SysConfig.AC_MotorSettings[X_AXIS].Motor_Max_Torque;
		float SPEED_SCALE_Y = 0.65f;//config.SysConfig.AC_MotorSettings[Y_AXIS].Motor_Max_Speed/config.SysConfig.AC_MotorSettings[Y_AXIS].Motor_Max_Torque;
		//Speed X . Torque X
		 if(_xy_forces[X_AXIS] - x_force_dead_zone > 0)
		//if(_xy_forces[X_AXIS] > 0)
		 {
			 x_speed = constrain(_xy_forces[X_AXIS], x_speed_min, x_speed_max) * SPEED_SCALE_X;
			 x_force = constrain(_xy_forces[X_AXIS], x_torque_min, x_torque_max);

		 }
		else if (_xy_forces[X_AXIS] + x_force_dead_zone  < 0)
		//else if (_xy_forces[X_AXIS] < 0)
		 {
			 x_speed = constrain(_xy_forces[X_AXIS], -x_speed_max, -x_speed_min) * SPEED_SCALE_X ;
			 x_force = constrain(_xy_forces[X_AXIS], -x_torque_max, -x_torque_min);

		 }
		 else
		 {
			 x_speed =0;
			 x_force =0;
		 }

			x_force = map(x_force, -32767, 32767, DAC_MIN, DAC_MAX);
			DAC856x_Set_Data(CS1, DAC_CH1,x_force);

			x_speed = map(x_speed, -32767, 32767, DAC_MIN, DAC_MAX);
			DAC856x_Set_Data(CS1, DAC_CH2,x_speed);

		 //Speed Y, Torque Y
		 if(_xy_forces[Y_AXIS] - y_force_dead_zone > 0)
		 //if(_xy_forces[Y_AXIS]  > 0)
		 {
			 y_speed = constrain(_xy_forces[Y_AXIS], y_speed_min, y_speed_max) * SPEED_SCALE_Y;
			 y_force = constrain(_xy_forces[Y_AXIS], y_torque_min, y_torque_max);

		 }
		 else if (_xy_forces[Y_AXIS] + y_force_dead_zone < 0)
		 //else if (_xy_forces[Y_AXIS] < 0)
		 {
			 y_speed = constrain(_xy_forces[Y_AXIS],-y_speed_max,-y_speed_min) * SPEED_SCALE_Y;
			 y_force = constrain(_xy_forces[Y_AXIS], -y_torque_max, -y_torque_min);

		 }
		 else
		 {
			 y_speed =0;
			 y_force =0;
		 }

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
