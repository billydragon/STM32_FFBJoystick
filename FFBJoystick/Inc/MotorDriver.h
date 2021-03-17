/*
 * MotorDriver.h
 *
 *  Created on: Mar 14, 2021
 *      Author: billy
 */

#ifndef INC_MOTORDRIVER_H_
#define INC_MOTORDRIVER_H_
#include "cppmain.h"

enum
{
	MOTOR_X =0,
	MOTOR_Y,
	ALL_MOTORS
};


class MotorDriver
	{
	public:

		void Init();

		void SetMotorOutput(int32_t * xy_forces);
		//void SetMotorDir(uint8_t MotorAxis, uint8_t dir);
		void MotorDriverOn(uint8_t MotorAxis);
		void MotorDriverOff(uint8_t MotorAxis);
		//void ChangeDriverType(uint8_t DriverType);
	private:
		uint8_t CurrentMotorDriver = 0;
		void DAC_Output(int32_t * xy_forces);
		void PWM_Output(int32_t * xy_forces);
		void ChangeMotoDriver();
	};


#endif /* INC_MOTORDRIVER_H_ */
