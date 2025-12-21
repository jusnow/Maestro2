/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FPGA_RST_Pin GPIO_PIN_0
#define FPGA_RST_GPIO_Port GPIOC
#define USB_RST_Pin GPIO_PIN_1
#define USB_RST_GPIO_Port GPIOC
#define SDI_RST_Pin GPIO_PIN_2
#define SDI_RST_GPIO_Port GPIOC
#define HDMI_RST_Pin GPIO_PIN_3
#define HDMI_RST_GPIO_Port GPIOC
#define CUR_SENS_Pin GPIO_PIN_0
#define CUR_SENS_GPIO_Port GPIOA
#define VOL_SENS_Pin GPIO_PIN_1
#define VOL_SENS_GPIO_Port GPIOA
#define AD_KEY_Pin GPIO_PIN_4
#define AD_KEY_GPIO_Port GPIOA
#define MIPI_RST_Pin GPIO_PIN_4
#define MIPI_RST_GPIO_Port GPIOC
#define CAM_RST_Pin GPIO_PIN_5
#define CAM_RST_GPIO_Port GPIOC
#define MCU_MODE_Pin GPIO_PIN_0
#define MCU_MODE_GPIO_Port GPIOB
#define MODE_RES1_Pin GPIO_PIN_1
#define MODE_RES1_GPIO_Port GPIOB
#define MODE_RES2_Pin GPIO_PIN_2
#define MODE_RES2_GPIO_Port GPIOB
#define MODE_IN1_Pin GPIO_PIN_12
#define MODE_IN1_GPIO_Port GPIOB
#define MODE_IN2_Pin GPIO_PIN_13
#define MODE_IN2_GPIO_Port GPIOB
#define MODE_FPS1_Pin GPIO_PIN_14
#define MODE_FPS1_GPIO_Port GPIOB
#define MODE_FPS2_Pin GPIO_PIN_15
#define MODE_FPS2_GPIO_Port GPIOB
#define UART_DIR_Pin GPIO_PIN_6
#define UART_DIR_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_7
#define LED_G_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_8
#define LED_B_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_9
#define LED_R_GPIO_Port GPIOC
#define POWER_ON_Pin GPIO_PIN_10
#define POWER_ON_GPIO_Port GPIOC
#define SW_RST_Pin GPIO_PIN_4
#define SW_RST_GPIO_Port GPIOB
#define SW_RST_EXTI_IRQn EXTI4_IRQn
#define SW_POWER_Pin GPIO_PIN_5
#define SW_POWER_GPIO_Port GPIOB
#define SW_POWER_EXTI_IRQn EXTI9_5_IRQn
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define RED  0
#define GREEN 1
#define BLUE 2
#define ORANGE 3
#define PURPLE 4
#define SKYBLUE 5
#define WHITE 6

#define TRUE 1
#define FALSE 0

#define ON 1
#define OFF 0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
