/*
 * FFBConfig.cpp
 *
 *  Created on: Feb 19, 2021
 *      Author: billy
 */

#include "FFBConfig.h"
#include "cppmain.h"
#include "usbd_joystick_hid_if.h"
#include "stm32f4_Flash.h"

//#include "MotorPWM.h"

#define CURRENT_HW_VERSION		100
#define CURRENT_SW_VERSION		100
#define MOTOR_MIN_SPEED_RPM		50
#define MOTOR_MAX_SPEED_RPM		1000
#define MOTOR_MIN_TORQUE		50
#define MOTOR_MAX_TORQUE		300
#define CHECKSUM				0
#define REQUEST_INTERVAL		10000 //(microseconds)
uint32_t Host_request_time =0;

// @formatter:off
const SYS_CONFIG_t SysConfig_default = {
							FW_VERSION{CURRENT_HW_VERSION, CURRENT_SW_VERSION},
							APP_PARAM{0, 0, 0, 0, 0, 0, DAC856x_OUTPUT, 0},
							GAIN_PARAM{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
							GAIN_PARAM{50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50},
							PID_PARAM{4000, 1, 1.5, 0.07, 0.01},
							PID_PARAM{4000, 1, 1.5, 0.07, 0.01},
							ACSERVO_PARAM{MOTOR_MIN_SPEED_RPM, MOTOR_MAX_SPEED_RPM, MOTOR_MIN_TORQUE,MOTOR_MAX_TORQUE},
							ACSERVO_PARAM{MOTOR_MIN_SPEED_RPM,MOTOR_MAX_SPEED_RPM, MOTOR_MIN_TORQUE,MOTOR_MAX_TORQUE},
							PWM_PARAM{PWM_DIR,HIGH},
							FILTER_PARAM{5.0,0.001,0},
							CHECKSUM};

// @formatter:on

FFBConfig::FFBConfig()
	{
		FW_VERSION Current_FW;

		Flash_Read8BitDatas(ADDR_FW_VERSION, sizeof(FW_VERSION), (int8_t*) &Current_FW);
		//Check Current Config version
		if ((Current_FW.HW_Version != CURRENT_HW_VERSION) || (Current_FW.SW_Version != CURRENT_SW_VERSION))
			{
				Restore_Default_Config();
				//printf("FFBConfig: Init New Default Configs done.\n");
			}
		else
			{
				Read_Config_From_Flash();
				//printf("FFBConfig: Restore Configs from Flash done.\n");
			}
	}

void FFBConfig::Set_Default_config()
	{
		memcpy(&SysConfig, &SysConfig_default, sizeof(SYS_CONFIG_t));

	}

void FFBConfig::begin()
	{
	//printf("FFBConfig: Init done.\n");
	}

void FFBConfig::Restore_Default_Config()
	{

		Set_Default_config();
		Write_Config_To_Flash();


	}



void FFBConfig::Read_Config_From_Flash()
	{

		uint32_t flash_address = FLASH_START_BASE;
		Flash_Read8BitDatas(flash_address, sizeof(SYS_CONFIG_t), (int8_t*) &SysConfig);

	}

void FFBConfig::Write_Config_To_Flash()
	{

		uint32_t flash_address = FLASH_START_BASE;
		Flash_Write8BitDatas(flash_address, sizeof(SYS_CONFIG_t), (int8_t*) &SysConfig);
		delay_us(100);
	}

uint8_t * FFBConfig::GetSysConfig()
	{
	   //printf("FFBConfig: Host Get Configs done.\n");
		while(REQUEST_INTERVAL > (micros() - Host_request_time))
		{
			;
		}
		Host_request_time = micros();
		return (uint8_t *)&SysConfig;
	}


void FFBConfig::Host_To_Dev_SetFeature(uint8_t *buff)
	{

		uint8_t CmdID = buff[0];
		uint8_t CmdLen = buff[1];
		uint8_t HW_Ver = buff[2];

		switch (CmdID)
			{

			case CMD_WRITE_SYSCONFIG:
				if ((CmdLen != sizeof(SYS_CONFIG_t)) && (HW_Ver != SysConfig.FW_Version.HW_Version))
					break;
				memcpy((uint8_t*) &SysConfig, &buff[2], sizeof(SYS_CONFIG_t));
				Write_Config_To_Flash();			//Write to flash
				//printf("FFBConfig: Host Write to Flash done.\n");
				break;
			case CMD_UPDATE_SYSCONFIG:
				if ((CmdLen != sizeof(SYS_CONFIG_t)) && (HW_Ver != SysConfig.FW_Version.HW_Version))
					break;
				memcpy((uint8_t*) &SysConfig, &buff[2], sizeof(SYS_CONFIG_t));
				//printf("FFBConfig: Host Update change done.\n");
				break;
			case CMD_RESET_SYSCONFIG:
				if (CmdLen != 0) break;					//Not Matching
				Restore_Default_Config();
				//printf("FFBConfig: Host Reset Default done.\n");
				break;

			default:
				break;
			}


	}

FFBConfig::~FFBConfig()
	{

	}

