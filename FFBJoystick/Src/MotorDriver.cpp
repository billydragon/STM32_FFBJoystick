/*
 * MotorDriver.c
 *
 *  Created on: Mar 14, 2021
 *      Author: billy
 */
#include "MotorDriver.h"
#include "FFBMain.h"
#include "cppmain.h"
#include "FFBConfig.h"
#include "ACServo.h"
#include "MotorPWM.h"
#include "FFBConfig_DataType.h"

extern FFBConfig config;
ACServo AcServo;
MotorPWM pwm;

void MotorDriver::Init()
	{

		CurrentMotorDriver = config.SysConfig.AppConfig.Motor_Driver;

		ChangeMotoDriver();
		//printf("MotorDriver: Init done.\n");
	}


void MotorDriver::ChangeMotoDriver()
{
	MotorDriverOff(ALL_MOTORS);
	switch(CurrentMotorDriver)
				{
				case PWM_OUTPUT:
					//printf("Motor Output PWM selected\n");
					if(AcServo.GetState() == true){
						AcServo.stop();
					}
					pwm.Init();
					pwm.start();
					MotorDriverOn(ALL_MOTORS);
					break;
				case DAC856x_OUTPUT:
					//printf("Motor Output ACServo DAC856x selected\n");
					if(pwm.GetState() == true){
							pwm.stop();
						}
						AcServo.start();
						MotorDriverOn(ALL_MOTORS);
					break;
				case DAC_INT_OUTPUT:
					//printf("Motor Output Internal DAC selected. Disabled\n");
					break;
				default:
					break;
				}

}

void MotorDriver::SetMotorOutput(int32_t * xy_forces)
{
	if(CurrentMotorDriver != config.SysConfig.AppConfig.Motor_Driver)
	{
		CurrentMotorDriver = config.SysConfig.AppConfig.Motor_Driver;
		ChangeMotoDriver();
	}

	if(config.SysConfig.AppConfig.Motor_Inv_X)
	{
		xy_forces[0] = ~xy_forces[0];
	}

	if(config.SysConfig.AppConfig.Motor_Inv_Y)
		{
			xy_forces[1] = ~xy_forces[1];
		}

	switch(CurrentMotorDriver)
	{
	case PWM_OUTPUT:

		pwm.set_motor_pwm(xy_forces);
		break;
	case DAC856x_OUTPUT:

		AcServo.set_motor_dac(xy_forces);
		break;
	case DAC_INT_OUTPUT:
		break;
	default:
		break;
	}

}


void MotorDriver::MotorDriverOn(uint8_t MotorAxis){

	switch(MotorAxis)
	{
	case MOTOR_X:
		HAL_GPIO_WritePin(MOTOR_X_EN_GPIO_Port, MOTOR_X_EN_Pin, GPIO_PIN_RESET);
		//printf("Motor X turned On.\n");
		break;
	case MOTOR_Y:
		HAL_GPIO_WritePin(MOTOR_Y_EN_GPIO_Port, MOTOR_Y_EN_Pin, GPIO_PIN_RESET);
		//printf("Motor Y turned On.\n");
			break;
	case ALL_MOTORS:
		HAL_GPIO_WritePin(MOTOR_X_EN_GPIO_Port, MOTOR_X_EN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_Y_EN_GPIO_Port, MOTOR_Y_EN_Pin, GPIO_PIN_RESET);
		//printf("Motor X and Y turned On.\n");
			break;
	default:
		break;
	}
}

void MotorDriver::MotorDriverOff(uint8_t MotorAxis){

	switch(MotorAxis)
	{
	case MOTOR_X:
		HAL_GPIO_WritePin(MOTOR_X_EN_GPIO_Port, MOTOR_X_EN_Pin, GPIO_PIN_SET);
		//printf("Motor X turned Off.\n");
		break;
	case MOTOR_Y:
		HAL_GPIO_WritePin(MOTOR_Y_EN_GPIO_Port, MOTOR_Y_EN_Pin, GPIO_PIN_SET);
		//printf("Motor Y turned Off.\n");
			break;
	case ALL_MOTORS:
		HAL_GPIO_WritePin(MOTOR_X_EN_GPIO_Port, MOTOR_X_EN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_Y_EN_GPIO_Port, MOTOR_Y_EN_Pin, GPIO_PIN_SET);
		//printf("Motor X and Y turned Off.\n");
			break;
	default:
		break;
	}
}

