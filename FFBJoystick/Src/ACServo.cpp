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
		if(active == false)
			{
				DAC856x_Init ();
				active = true;
			}
		//torque X
		uint16_t xforce = map(_xy_forces[0], -32767, 32767, DAC_MIN, DAC_MAX);
		if(config.SysConfig.AppConfig.Motor_Inv_X)
			xforce = ~xforce;
		DAC856x_Set_Data(CS1, DAC_CH1,xforce);
		//AC Servo set gain 500rpm/1Volt

		DAC856x_Set_Data(CS1, DAC_CH2,xforce);

		//torque Y
		uint16_t yforce = map(_xy_forces[1], -32767, 32767, DAC_MIN, DAC_MAX);
		if(config.SysConfig.AppConfig.Motor_Inv_Y)
					yforce = ~yforce;
		DAC856x_Set_Data(CS2, DAC_CH1,yforce);

		DAC856x_Set_Data(CS2, DAC_CH2,yforce);
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

