/*
 * comm.c
 *
 *  Created on: Aug 7, 2025
 *      Author: junse
 */

#include <comm.h>

extern UART_HandleTypeDef huart1;

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
	return ch;
}

void delay_us(uint32_t nCount)
{
  int i;
  for(;nCount != 0;nCount--)
  {
    for(i= 0;i<7;i++)
      asm("NOP");
  }
}

void delay_ms(uint32_t nCount)
{
  for(;nCount != 0;nCount--)
    delay_us(1000);
}
