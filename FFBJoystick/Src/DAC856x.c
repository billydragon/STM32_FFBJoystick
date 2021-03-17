/*
 * DAC8563.c
 *
 *  Created on: Mar 5, 2021
 *      Author: billy
 */
#include <DAC856x.h>
#include "main.h"
#include "gpio.h"
#include "spi.h"

/**SPI1 GPIO Configuration
 PA5     ------> SPI1_SCK
 PA6     ------> SPI1_MISO
 PA7     ------> SPI1_MOSI
 #define SPI1_CS_Pin 		GPIO_PIN_4
 #define SPI1_CS_GPIO_Port 	GPIOA
 */

#define CS1_GPIO			SPI1_CS1_GPIO_Port
#define CS1_PIN				SPI1_CS1_Pin
#define CS1_1()				HAL_GPIO_WritePin(CS1_GPIO,CS1_PIN,GPIO_PIN_SET)
#define CS1_0()				HAL_GPIO_WritePin(CS1_GPIO,CS1_PIN,GPIO_PIN_RESET)

#define CS2_GPIO			SPI1_CS2_GPIO_Port
#define CS2_PIN				SPI1_CS2_Pin
#define CS2_1()				HAL_GPIO_WritePin(CS2_GPIO,CS2_PIN,GPIO_PIN_SET)
#define CS2_0()				HAL_GPIO_WritePin(CS2_GPIO,CS2_PIN,GPIO_PIN_RESET)



/* CLR */

#define CLR_GPIO			DAC856x_CLR_GPIO_Port
#define CLR_PIN				DAC856x_CLR_Pin
#define CLR_1()				HAL_GPIO_WritePin(CLR_GPIO,CLR_PIN,GPIO_PIN_SET)
#define CLR_0()				HAL_GPIO_WritePin(CLR_GPIO,CLR_PIN,GPIO_PIN_RESET)

/* LDAC */

#define LDAC_GPIO			DAC856x_LDAC_GPIO_Port
#define LDAC_PIN			DAC856x_LDAC_Pin
#define LDAC_1()			HAL_GPIO_WritePin(LDAC_GPIO,LDAC_PIN,GPIO_PIN_SET)
#define LDAC_0()			HAL_GPIO_WritePin(LDAC_GPIO,LDAC_PIN,GPIO_PIN_RESET)

#define CS_DELAY_us					1

#define SPI_BUFFER_SIZE				4

#define DAC856X_INIT_SPI()			DAC856x_InitSPIParam(SPI_BAUDRATEPRESCALER_2, SPI_PHASE_1EDGE, SPI_POLARITY_LOW)

uint32_t g_spiLen;
uint8_t g_spiTxBuf[SPI_BUFFER_SIZE];

#define X1	0
#define Y1  -100000

#define X2	65535
#define Y2  100000
enum
{
  TRANSFER_WAIT, TRANSFER_COMPLETE, TRANSFER_ERROR
};

__IO uint8_t Selectd_CS = CS1;

__IO uint32_t wTransferState = TRANSFER_WAIT;

void DAC856x_Init24 ()
{

  for (uint8_t i = CS1; i < MAX_NUM_DAC856x; i++)
    {
      Selectd_CS = i;
      CLR_0();
      LDAC_0();
      __ASM volatile ("NOP");
      __ASM volatile ("NOP");
      __ASM volatile ("NOP");

      DAC856x_WriteCmd24 ((4 << 19) | (0 << 16) | (3 << 0));

      DAC856x_WriteCmd24 ((6 << 19) | (0 << 16) | (3 << 0));
      /* Enable if DAC8562 */
      //DAC856x_Set_Data(0, 32767);
      //DAC856x_Set_Data(1, 32767);
      DAC856x_WriteCmd24 ((7 << 19) | (0 << 16) | (1 << 0));
    }
}

void DAC856x_Init ()
{

  for (uint8_t i = CS1; i < MAX_NUM_DAC856x; i++)
    {
      Selectd_CS = i;
      CLR_0();
      LDAC_0();
      __ASM volatile ("NOP");
      __ASM volatile ("NOP");
      __ASM volatile ("NOP");

      DAC856x_WriteCmd (CMD_RESET_REG, DATA_RESET_ALL_REG);      	// reset
      DAC856x_WriteCmd (CMD_PWR_UP_A_B, DATA_PWR_UP_A_B);        // power up
      DAC856x_WriteCmd (CMD_INTERNAL_REF_EN, DATA_INTERNAL_REF_EN); // enable internal reference
      DAC856x_WriteCmd (CMD_GAIN, DATA_GAIN_B2_A2);            // set multiplier
      DAC856x_WriteCmd (CMD_LDAC_DIS, DATA_LDAC_DIS_AB);    // update the caches
    }

}

void DAC856x_Set_CS_Pin (uint8_t _level)
{
  if (_level == CS_ENABLE)
    {
      DAC856X_INIT_SPI();

      if (Selectd_CS == CS1)
	{
	  CS1_0();
	}
      else
	{
	  CS2_0();
	}

      __ASM volatile ("NOP");
      __ASM volatile ("NOP");
      __ASM volatile ("NOP");
      __ASM volatile ("NOP");
      __ASM volatile ("NOP");

    }
  else
    {

      __ASM volatile ("NOP");
      __ASM volatile ("NOP");
      __ASM volatile ("NOP");
      __ASM volatile ("NOP");
      __ASM volatile ("NOP");
      if (Selectd_CS == CS1)
	{
	  CS1_1();
	}
      else
	{
	  CS2_1();
	}
    }
}

void DAC856x_Set_Data24 (uint8_t _selected_CS, uint8_t _ch, uint16_t _dac)
{
  Selectd_CS = _selected_CS;
  if (_ch == DAC_CH1)
    {
      /* Write to DAC-A input register and update DAC-A; */
      DAC856x_WriteCmd24 ((3 << 19) | (0 << 16) | (_dac << 0));
    }
  else if (_ch == DAC_CH2)
    {
      /* Write to DAC-B input register and update DAC-A; */
      DAC856x_WriteCmd24 ((3 << 19) | (1 << 16) | (_dac << 0));
    }

}

void DAC856x_Set_Data (uint8_t _selected_CS, uint8_t _ch, uint16_t _dac)
{
  Selectd_CS = _selected_CS;
  if (_ch == DAC_CH1)
    {
      /* Write to DAC-A input register and update DAC-A; */
      DAC856x_WriteCmd (CMD_SET_A_UPDATE_A, _dac);
    }
  else if (_ch == DAC_CH2)
    {
      /* Write to DAC-B input register and update DAC-A; */
      DAC856x_WriteCmd (CMD_SET_B_UPDATE_B, _dac);
    }

}

void DAC856x_WriteCmd24 (uint32_t _cmd)
{
  DAC856x_Set_CS_Pin (CS_ENABLE);
  g_spiLen = 0;
  g_spiTxBuf[g_spiLen++] = (_cmd >> 16);
  g_spiTxBuf[g_spiLen++] = (_cmd >> 8);
  g_spiTxBuf[g_spiLen++] = (_cmd);
  DAC856x_spiTransfer ();
  DAC856x_Set_CS_Pin (CS_DISABLE);

}

void DAC856x_WriteCmd (uint8_t _cmd, uint16_t _data)
{
  DAC856x_Set_CS_Pin (CS_ENABLE);
  g_spiLen = 0;
  g_spiTxBuf[g_spiLen++] = (_cmd);
  g_spiTxBuf[g_spiLen++] = (_data >> 8) & 0xFF;
  g_spiTxBuf[g_spiLen++] = (_data) & 0xFF;
  DAC856x_spiTransfer ();
  DAC856x_Set_CS_Pin (CS_DISABLE);

}

void DAC856x_spiTransfer (void)
{
  if (g_spiLen > SPI_BUFFER_SIZE)
    {
      return;
    }

  if (HAL_SPI_Transmit (&hspi1, (uint8_t*) g_spiTxBuf, g_spiLen, 10000)
      != HAL_OK)
    {
      Error_Handler ();
    }
}

int32_t CaculTwoPoint (int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x)
{
  return y1 + ((int64_t) (y2 - y1) * (x - x1)) / (x2 - x1);
}

int32_t DAC856x_DacToVoltage (uint16_t _dac)
{
  int32_t y;

  /* CaculTwoPoint(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x);*/
  y = CaculTwoPoint (DAC_MIN, Y1, DAC_MAX, Y2, _dac);
  return y;
}

uint32_t DAC856x_VoltageToDac (int32_t _volt)
{
  /* CaculTwoPoint(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x);*/
  return CaculTwoPoint (Y1, DAC_MIN, Y2, DAC_MAX, _volt);
}
