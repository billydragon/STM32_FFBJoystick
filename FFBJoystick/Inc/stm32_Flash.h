/*
 * stm32f4_Flash.h
 *
 *  Created on: Feb 20, 2021
 *      Author: billy
 */

#ifndef INC_STM32_FLASH_H_
#define INC_STM32_FLASH_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#include "main.h"

//#include "stm32f4xx.h"

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbyte */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbyte */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbyte */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbyte */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbyte */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbyte */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbyte */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbyte */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbyte */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbyte */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbyte */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbyte */

#define FLASH_USER_START_ADDR ADDR_FLASH_SECTOR_1
#define FLASH_USER_END_ADDR ADDR_FLASH_SECTOR_3

  void Flash_EraseSector (uint32_t start_Add, uint32_t end_Add);
  void Flash_User_Data (uint32_t start_Add, uint32_t end_Add);
  void Flash_Write32BitDatas (uint32_t address, uint16_t length, int32_t *data_32);
  void Flash_Read32BitDatas (uint32_t address, uint16_t length, int32_t *data_32);
  void Flash_Write16BitDatas (uint32_t address, uint16_t length, int16_t *data_16);
  void Flash_Read16BitDatas (uint32_t address, uint16_t length, int16_t *data_16);
  void Flash_Write8BitDatas (uint32_t address, uint16_t length, int8_t *data_8);
  void Flash_Read8BitDatas (uint32_t address, uint16_t length, int8_t *data_8);
  void Flash_WriteByte (uint32_t address, int8_t data_8);
  int8_t Flash_ReadByte (uint32_t address);

#ifdef __cplusplus
}
#endif
#endif /* INC_STM32_FLASH_H_ */
