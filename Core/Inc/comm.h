/*
 * comm.h
 *
 *  Created on: Aug 7, 2025
 *      Author: junse
 */

#ifndef INC_COMM_H_
#define INC_COMM_H_

#define TRUE 1
#define FALSE 0

//#include "stdio.h"
//#include "stdlib.h"
//#include "string.h"
#include "stm32f1xx_hal.h"

typedef enum {
    CAM_RX_IDLE = 0,
	CAM_RX_READY = 1,
    CAM_RX_ERR = 2  // Generic error.
} UARTStatus_t;

typedef struct {
	uint8_t command;
	uint8_t color;
}PowerLed;


int __io_putchar(int ch);
void delay_us(uint32_t nCount);
void delay_ms(uint32_t nCount);

#endif /* INC_COMM_H_ */
