/*
 * FFBConfig.h
 *
 *  Created on: Feb 19, 2021
 *      Author: billy
 */

#ifndef SRC_FFBCONFIG_H_
#define SRC_FFBCONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include "cppmain.h"
#include <stm32_Flash.h>
#include "FFBConfig_DataType.h"

#define CMD_WRITE_SYSCONFIG   0x10
#define CMD_UPDATE_SYSCONFIG  0x11
#define CMD_RESET_SYSCONFIG   0x12
#define CMD_FINDCENTER		  0x13

#define FLASH_START_BASE		FLASH_USER_START_ADDR
#define ADDR_FW_VERSION			FLASH_USER_START_ADDR + 0
#define ADDR_GAIN_PARAM			ADDR_FW_VERSION + 2
#define ADDR_PID_PARAM			ADDR_GAIN_PARAM + 2 * 13		// 2 axis * 13 uint8_t  parameters
#define ADDR_APP_PARAM			ADDR_PID_PARAM  + 2 * 4 * 5		//2 axis * 4 floats size * 5 parameters

  class FFBConfig
  {
  public:

    SYS_CONFIG_t SysConfig;

    void begin ();
    void Restore_Default_Config ();
    void Read_Config_From_Flash ();
    void Write_Config_To_Flash ();
    uint8_t* GetSysConfig ();
    void Host_To_Dev_SetFeature (uint8_t *buff);
    FFBConfig ();
    virtual ~FFBConfig ();
  private:
    void Set_Default_config ();
    bool Check_Recv_CRC(uint8_t *buff);

  };

#ifdef __cplusplus
}
#endif

#endif /* SRC_FFBCONFIG_H_ */
