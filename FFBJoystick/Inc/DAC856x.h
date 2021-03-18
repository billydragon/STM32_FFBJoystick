/*
 * DAC8563.h
 *
 *  Created on: Mar 5, 2021
 *      Author: billy
 */

#ifndef INC_DAC856X_H_
#define INC_DAC856X_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

#define DEFAULT_VREF      3.3383
#define DAC_MIN           512
#define DAC_MAX           65024

#define CMD_SET_A                   0x00  // Write to DAC-A input register
#define CMD_SET_B                   0x01  // Write to DAC-B input register
#define CMD_SET_AB                  0x07  // Write to DAC-A and DAC-B input registers
#define CMD_SET_A_UPDATE_AB         0x10  // Write to DAC-A input register and update all DACs
#define CMD_SET_B_UPDATE_AB         0x11  // Write to DAC-B input register and update all DACs
#define CMD_SET_AB_UPDATE_AB        0x17  // Write to DAC-A and DAC-B input register and update all DACs
#define CMD_SET_A_UPDATE_A          0x18  // Write to DAC-A input register and update DAC-A
#define CMD_SET_B_UPDATE_B          0x19  // Write to DAC-B input register and update DAC-B
#define CMD_SET_ALL_UPDATE_ALL      0x1F  // Write to DAC-A and DAC-B input register and update all DACs
#define CMD_UPDATE_A                0x08  // Update DAC-A
#define CMD_UPDATE_B                0x09  // Update DAC-B
#define CMD_UPDATE_ALL_DACS         0x0F  // Update all DACs

#define CMD_GAIN                    0x02  //
#define DATA_GAIN_B2_A2             0x0000  // Gain: DAC-B gain = 2, DAC-A gain = 2 (default with internal VREF)
#define DATA_GAIN_B2_A1             0x0001  // Gain: DAC-B gain = 2, DAC-A gain = 1
#define DATA_GAIN_B1_A2             0x0002  // Gain: DAC-B gain = 1, DAC-A gain = 2
#define DATA_GAIN_B1_A1             0x0003  // Gain: DAC-B gain = 1, DAC-A gain = 1 (power-on default)

#define CMD_PWR_UP_A_B              0x20  //
#define DATA_PWR_UP_A_B             0x0003  // Power up DAC-A and DAC-B

#define CMD_RESET_REG               0x28  //
#define DATA_RESET_AB_REG           0x0000  // Reset DAC-A and DAC-B input register and update all DACs
#define DATA_RESET_ALL_REG          0x0001  // Reset all registers and update all DACs (Power-on-reset update)

#define CMD_LDAC_DIS                0x30  //
#define DATA_LDAC_ENB_AB            0x0000  // LDAC pin active for DAC-B and DAC-A
#define DATA_LDAC_ENB_B_DIS_A       0x0001  // LDAC pin active for DAC-B; inactive for DAC-A
#define DATA_LDAC_ENB_A_DIS_B       0x0002  // LDAC pin inactive for DAC-B; active for DAC-A
#define DATA_LDAC_DIS_AB            0x0003  // LDAC pin inactive for DAC-B and DAC-A

#define CMD_INTERNAL_REF_DIS      0x38  // Disable internal reference and reset DACs to gain = 1
#define DATA_INTERNAL_REF_DIS     0x0000  // Disable internal reference and reset DACs to gain = 1
#define CMD_INTERNAL_REF_EN       0x38  // Enable Internal Reference & reset DACs to gain = 2
#define DATA_INTERNAL_REF_EN      0x0001  // Enable Internal Reference & reset DACs to gain = 2

#define MAX_NUM_DAC856x		 2

  enum
  {
    DAC_CH1 = 0, DAC_CH2,
  };

  enum
  {
    CS_ENABLE = 0, CS_DISABLE,
  };

  enum
  {
    CS1 = 0, CS2,
  };

  void DAC856x_Set_CS_Pin (uint8_t _level);
  void DAC856x_Init24 ();
  void DAC856x_Set_Data24 (uint8_t _selected_CS, uint8_t _ch, uint16_t _dac);
  void DAC856x_WriteCmd24 (uint32_t _cmd);
  void DAC856x_Init ();
  void DAC856x_Set_Data (uint8_t _selected_CS, uint8_t _ch, uint16_t _dac);
  void DAC856x_WriteCmd (uint8_t _cmd, uint16_t _data);

  void DAC856x_spiTransfer (void);

  int32_t CaculTwoPoint (int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x);
  int32_t DAC856x_DacToVoltage (uint16_t _dac);
  uint32_t DAC856x_VoltageToDac (int32_t _volt);


#ifdef __cplusplus
}
#endif

#endif /* INC_DAC856X_H_ */
