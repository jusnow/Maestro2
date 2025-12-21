/*
 * pca8575.c
 *
 *  Created on: Jun 26, 2025
 *      Author: junse
 */

#include <tca9555.h>
#include "main.h"



extern I2C_HandleTypeDef hi2c2;



//--------- led operate function --------//
// mode : led on/off sw && uart bidd command
//
/*void PCA8575_AllLow()
{
	PCA8575_Write(0x0000);
}

void PCA8575_Write(uint16_t data)
{
	uint8_t buffer[2];
	buffer[0] = (uint8_t)(data & 0x00FF);        // LSB first
	buffer[1] = (uint8_t)(data & 0xFF00) >> 8;   // MSB later

	HAL_I2C_Master_Transmit(&hi2c2, PCA8575_ADDRESS, buffer, 2, HAL_MAX_DELAY);
}

void PCA8575_SetPin(uint8_t pin, GPIO_PinState state)
{
	static uint16_t outputState = 0xFFFF;    // 초기값: 전부 입력(High_Z)

	if(state == GPIO_PIN_RESET)
		outputState &= ~(1 << pin);    // 해당 pin을 0 -> 출력 Low
	else
		outputState |= (1 << pin);    // 해당 pin을 1 => 입력(High-Z)

	PCA8575_Write(outputState);
} */

void TCA9555_SetDirection(uint16_t dir)
{
  uint8_t data[2];
  data[0] = dir & 0xFF;         // Port 0
  data[1] = (dir >> 8) & 0xFF;  // Port 1
  HAL_I2C_Mem_Write(&hi2c2, TCA9555_ADDRESS, 0x06, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
}

// Output 값 설정
void TCA9555_WriteOutput(uint16_t value)
{
  uint8_t data[2];
  data[0] = value & 0xFF;        // Port 0
  data[1] = (value >> 8) & 0xFF; // Port 1
  HAL_I2C_Mem_Write(&hi2c2, TCA9555_ADDRESS, 0x02, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
}

// Input 읽기
uint16_t TCA9555_ReadInput(void)
{
  uint8_t data[2];
  HAL_I2C_Mem_Read(&hi2c2, TCA9555_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
  return (data[1] << 8) | data[0];
}
