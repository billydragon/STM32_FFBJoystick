#include <stdio.h>
#include <stm32_Flash.h>
#include "main.h"

static uint32_t
Flash_GetSector (uint32_t Address);
/* Global variable used to store variable value in read sequence */
uint16_t DataVar = 0;

extern HAL_StatusTypeDef
HAL_FLASHEx_Erase (FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
/**
 * @brief  Gets the sector of a given address
 * @param  Address: Flash address
 * @retval The sector of a given address
 */

static uint32_t
Flash_GetSector (uint32_t Address)
{
  uint32_t sector = 0;

  if ((Address < FLASH_USER_START_ADDR) || (Address >= FLASH_USER_END_ADDR))
    {
      // Address out of defined user configured addresses range
      /* Error occurred while page erase */
      Error_Handler ();
    }

  if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
    {
      sector = FLASH_SECTOR_0;
    }
  else if ((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
    {
      sector = FLASH_SECTOR_1;
    }
  else if ((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
    {
      sector = FLASH_SECTOR_2;
    }
  else if ((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
    {
      sector = FLASH_SECTOR_3;
    }
  else if ((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
    {
      sector = FLASH_SECTOR_4;
    }
  else if ((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
    {
      sector = FLASH_SECTOR_5;
    }
  else if ((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
    {
      sector = FLASH_SECTOR_6;
    }
  else if ((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
    {
      sector = FLASH_SECTOR_7;
    }
  else if ((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
    {
      //    sector = FLASH_SECTOR_8;
    }
  else if ((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
    {
      //   sector = FLASH_SECTOR_9;
    }
  else if ((Address < ADDR_FLASH_SECTOR_11)
      && (Address >= ADDR_FLASH_SECTOR_10))
    {
      //   sector = FLASH_SECTOR_10;
    }
  else
    {
      //   sector = FLASH_SECTOR_11;
    }

  return sector;
}

void
Flash_User_Data (uint32_t start_Add, uint32_t end_Add)
{

  Flash_EraseSector (start_Add, end_Add);
}

/****************************************************************************
 * Function: Erase specified sector
 * Entry parameter: SectorNum sector number
 * Export parameters: none
 * Description: None
 * Calling method: none
 ****************************************************************************/
void
Flash_EraseSector (uint32_t start_Add, uint32_t end_Add)
{
  uint32_t UserStartSector;
  uint32_t SectorError;

  FLASH_EraseInitTypeDef pEraseInit;

  HAL_FLASH_Unlock ();
  //__HAL_FLASH_CLEAR_FLAG(
  //    FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  //  FLASH_EraseSector(SectorNum, FLASH_VOLTAGE_RANGE_3);

  /* Get the sector where start the user flash area */
  UserStartSector = Flash_GetSector (start_Add);
  pEraseInit.TypeErase = TYPEERASE_SECTORS;
  pEraseInit.Sector = UserStartSector;
  pEraseInit.NbSectors = Flash_GetSector (end_Add) - UserStartSector + 1;
  pEraseInit.VoltageRange = VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase (&pEraseInit, &SectorError) != HAL_OK)
    {
      /* Error occurred while page erase */
      Error_Handler ();
    }

  // printf("\r\Flash_EraseSector@\r\n");
  HAL_FLASH_Lock ();
}

/****************************************************************************
 * Function: Write 32-bit data with length
 * Entry parameter: address: address
 length: data length
 data_32: data pointer to be written
 * Export parameters: none
 * Description: None
 * Calling method: none
 ****************************************************************************/
void
Flash_Write32BitDatas (uint32_t address, uint16_t length, int32_t *data_32)
{

  uint32_t UserStartSector;
  uint32_t SectorError;
  FLASH_EraseInitTypeDef pEraseInit;

  HAL_FLASH_Unlock ();
  //__HAL_FLASH_DATA_CACHE_DISABLE();
  //__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

 // __HAL_FLASH_DATA_CACHE_RESET();
  //__HAL_FLASH_INSTRUCTION_CACHE_RESET();

  //__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  //__HAL_FLASH_DATA_CACHE_ENABLE();
  //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
  //FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  /* Get the sector where start the user flash area */
  UserStartSector = Flash_GetSector (address);
  pEraseInit.TypeErase = TYPEERASE_SECTORS;
  pEraseInit.Sector = UserStartSector;
  pEraseInit.NbSectors = Flash_GetSector (address + 4 * length)
      - UserStartSector + 1;
  pEraseInit.VoltageRange = VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase (&pEraseInit, &SectorError) != HAL_OK)
    {
      /* Error occurred while page erase */
      Error_Handler ();

    }

  for (uint16_t i = 0; i < length; i++)
    {
      if (HAL_FLASH_Program (FLASH_TYPEPROGRAM_WORD, address, data_32[i])
	  == HAL_OK) //Write DATA_32 to the corresponding address.
	{
	  address = address + 4;
	}
      else
	{
	  while (1)
	    ;
	}
    }

  HAL_FLASH_Lock (); 	//Reading FLASH does not require FLASH to be unlocked.
}

/****************************************************************************
 * Function: Read 32-bit data with length
 * Entry parameter: address: address
 length: data length
 data_32 points to the read data
 * Export parameters: none
 * Description: None
 * Calling method: none
 ****************************************************************************/
void
Flash_Read32BitDatas (uint32_t address, uint16_t length, int32_t *data_32)
{
  uint8_t i;
  for (i = 0; i < length; i++)
    {
      data_32[i] = *(__IO int32_t*) address;
      address = address + 4;
    }
}

/****************************************************************************
 * Function: Write 16-bit data with length
 * Entry parameter: address: address
 length: data length
 data_16: data pointer to be written
 * Export parameters: none
 * Description: None
 * Calling method: none
 ****************************************************************************/
void
Flash_Write16BitDatas (uint32_t address, uint16_t length, int16_t *data_16)
{
  uint32_t UserStartSector;
  uint32_t SectorError;
  FLASH_EraseInitTypeDef pEraseInit;

  HAL_FLASH_Unlock ();
  /*
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
  //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
  //FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	*/
  /* Get the sector where start the user flash area */
  UserStartSector = Flash_GetSector (address);
  pEraseInit.TypeErase = TYPEERASE_SECTORS;
  pEraseInit.Sector = UserStartSector;
  pEraseInit.NbSectors = Flash_GetSector (address + 2 * length)
      - UserStartSector + 1;
  pEraseInit.VoltageRange = VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase (&pEraseInit, &SectorError) != HAL_OK)
    {
      /* Error occurred while page erase */
      Error_Handler ();

    }

  for (uint16_t i = 0; i < length; i++)
    {
      if (HAL_FLASH_Program (FLASH_TYPEPROGRAM_HALFWORD, address, data_16[i])
	  == HAL_OK)
	{
	  address = address + 2;
	}
      else
	{
	  while (1)
	    ;
	}
    }
  HAL_FLASH_Lock (); //Reading FLASH does not require FLASH to be unlocked.
}

/****************************************************************************
 * Function: Read 16-bit data with length
 * Entry parameter: address: address
 length: data length
 data_16 points to the read data
 * Export parameters: none
 * Description: None
 * Calling method: none
 ****************************************************************************/
void
Flash_Read16BitDatas (uint32_t address, uint16_t length, int16_t *data_16)
{
  uint8_t i;
  for (i = 0; i < length; i++)
    {
      data_16[i] = *(__IO int16_t*) address;
      address = address + 2;

    }

}

/****************************************************************************
 * Function: Write 8-bit data with length
 * Entry parameter: address: address
 length: data length
 data_8: data pointer to be written
 * Export parameters: none
 * Description: None
 * Calling method: none
 ****************************************************************************/
void Flash_Write8BitDatas (uint32_t address, uint16_t length, int8_t *data_8)
{
  uint32_t UserStartSector;
  uint32_t SectorError;
  FLASH_EraseInitTypeDef pEraseInit;

  HAL_FLASH_Unlock ();

  /*
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
  //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
  //FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	*/
  /* Get the sector where start the user flash area */
  UserStartSector = Flash_GetSector (address);
  pEraseInit.TypeErase = TYPEERASE_SECTORS;
  pEraseInit.Sector = UserStartSector;
  pEraseInit.NbSectors = Flash_GetSector (address + length) - UserStartSector
      + 1;
  pEraseInit.VoltageRange = VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase (&pEraseInit, &SectorError) != HAL_OK)
    {
      /* Error occurred while page erase */
      Error_Handler ();

    }

  for (uint16_t i = 0; i < length; i++)
    {
      if (HAL_FLASH_Program (FLASH_TYPEPROGRAM_BYTE, address, data_8[i])
	  == HAL_OK)
	{
	  address++;
	}
      else
	{
	  while (1)
	    ;
	}
    }
  HAL_FLASH_Lock (); //Reading FLASH does not require FLASH to be unlocked.
}

/****************************************************************************
 * Function: Read 8-bit data with length
 * Entry parameter: address: address
 * length: data length
 * data_8 points to the read data
 * Export parameters: none
 * Description: None
 * Calling method: none
 ****************************************************************************/
void
Flash_Read8BitDatas (uint32_t address, uint16_t length, int8_t *data_8)
{
  uint8_t i;
  for (i = 0; i < length; i++)
    {
      data_8[i] = *(__IO int8_t*) address;
      address++;
    }
}

/****************************************************************************
 * Function: Write single 8-bit data
 * Entry parameter: address: address
 * data_8: data to be written
 * Export parameters: none
 * Description: None
 * Calling method: none
 ****************************************************************************/
void Flash_WriteByte (uint32_t address, int8_t data_8)
{
  uint32_t UserStartSector;
  uint32_t SectorError;
  FLASH_EraseInitTypeDef pEraseInit;

  HAL_FLASH_Unlock ();
  /*
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
  //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
  //FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	*/

  /* Get the sector where start the user flash area */
  UserStartSector = Flash_GetSector (address);
  pEraseInit.TypeErase = TYPEERASE_SECTORS;
  pEraseInit.Sector = UserStartSector;
  pEraseInit.NbSectors = Flash_GetSector (address + 1) - UserStartSector + 1;
  pEraseInit.VoltageRange = VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase (&pEraseInit, &SectorError) != HAL_OK)
    {
      /* Error occurred while page erase */
      Error_Handler ();

    }

  if (HAL_FLASH_Program (FLASH_TYPEPROGRAM_BYTE, address, data_8) != HAL_OK)
    {
      Error_Handler ();
    }

  HAL_FLASH_Lock (); //Reading FLASH does not require FLASH to be unlocked.
}

/****************************************************************************
 * Function: Read single 8-bit data
 * Entry parameter: address: address
 * data_8 points to the read data
 * Export parameters: none
 * Description: None
 * Calling method: none
 * Return 8-bit read date
 ****************************************************************************/
int8_t Flash_ReadByte (uint32_t address)
{
  int8_t data_8 = 0;
  data_8 = *(__IO int8_t*) address;
  return data_8;
}

