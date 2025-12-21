/*
 * pac8575.h
 *
 *  Created on: Jun 26, 2025
 *      Author: junse
 */

#ifndef INC_TCA9555_H_
#define INC_TCA9555_H_

#define TCA9555_ADDRESS (0x26 << 1)    // HAL에서는 8비트 주소 사용(왼쪽 1비트 shift)

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f1xx_hal.h"

void TCA9555_SetDirection(uint16_t);
void TCA9555_WriteOutput(uint16_t);
uint16_t TCA9555_ReadInput(void);


#endif /* INC_TCA9555_H_ */
