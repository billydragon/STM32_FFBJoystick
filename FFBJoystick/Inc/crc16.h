/*
 * crc16.h
 *
 *  Created on: Mar 21, 2021
 *      Author: billy
 */

#ifndef INC_CRC16_H_
#define INC_CRC16_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include "stdio.h"
#include "stdlib.h"

unsigned short crc16(const unsigned char* data_p, unsigned short length);


#ifdef __cplusplus
}
#endif

#endif /* INC_CRC16_H_ */
