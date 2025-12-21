/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <tca9555.h>
//#include "SIL9136.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include <stdbool.h>
#include <comm.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define UART_PRINT
//#define TEST_POWERLED


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t _flag_scan;       // flag per 100ms
volatile uint8_t _count_scan;      // counter for TIM1
volatile uint8_t _time_reset;      // MCU <-> CAMERA receive data wait
volatile bool _status_power;       // status of cam power
volatile uint8_t _count_sec;
volatile uint8_t _mode_pic;
volatile uint16_t _status_led;
volatile uint8_t _status_mode;
volatile uint8_t error_code;

uint16_t _ADCvol;
uint16_t _ADCcur;
uint16_t _ADCkey;
uint8_t mode_OLED;
//uint8_t flag_camRx;
UARTStatus_t flag_camRX;
uint8_t _count_Rxwait;
uint16_t time_InfoDisplay;

// ADC DMA Variables
uint32_t gADCData[3] = {0,};

PowerLed _pled;

//visca_tx_type txCommand;
uint8_t txCommand[5] = {0x81, 0x09, 0x00, 0x37, 0xFF};

//UART RxD Variables
char str[100] = {0, };
uint8_t rxBuff[10] = {0, };
uint8_t rxBuff_temp[10];
uint8_t buff_count = 0;


bool cam_valid;

// CAMERA INFO
uint16_t _cam_model = 0;
uint16_t _cam_version = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void Operate_PowerLed(PowerLed);
void Operate_Led(uint8_t);
void Scan_ModeSwitch();
void Init_Board();
void Power_On();
void Power_Off();
void Scan_ADC();
void On_FPGA();
void On_USB();
void On_SDI();
void On_HDMI();
void On_MIPI();
void Parse_VISCA();
void Sel_uart(bool);
void Send_Message(uint8_t*);
void Check_Power();
//void ErrorHandler();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



//--------- time polling function --------//
// _flag_scan : mode switch scan(per 1 s)
// _time_scan : time for CAN Rx wait(per 1 ms)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if(++_count_scan >= 100)
		{
			_flag_scan = 1;
			_count_scan = 0;
		}

	}
	if(htim->Instance == TIM3)
	{
		_time_reset++;
		if(_time_reset == 100)
		{
			//ErrorHandler(1);
			flag_camRX = CAM_RX_FAIL;
			mode_OLED = 2;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == SW_POWER_Pin)    // power switch push
	{
		Check_Power();
	}
	else if(GPIO_Pin == SW_RST_Pin)    // reset switch push
	{
		if(_status_power == TRUE)
		{
			Sel_uart(TRUE);    // UART CAM <-> MCU
			if (HAL_UART_Transmit(&huart2, txCommand, sizeof(txCommand) / sizeof(uint8_t), 10 != HAL_OK))
			{
		    // 전송 실패 시 에러 처리
				//printf("Send Failed\r\n");
				Sel_uart(FALSE);
				return;
			}
			flag_camRX = CAM_RX_READY;
			_time_reset = 0;
			HAL_TIM_Base_Start_IT(&htim3);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		if(flag_camRX == CAM_RX_READY)
		{
			if(rxBuff_temp[0] == 0x90)
			{
				buff_count = 0;
				rxBuff[buff_count] = rxBuff_temp[0];
			}
			else
			{
				buff_count++;
				rxBuff[buff_count] = rxBuff_temp[0];
			}

			if(rxBuff_temp[0] == 0xFF)
			{
				Parse_VISCA();
			}
		}
		HAL_UART_Receive_IT(&huart2, rxBuff_temp, 1);
	}
}

/*
 * @brief  카메라로부터의 uart received data 해석 (VISCA protocol)
 *         data format : y0 50 pp pp pp qq qq FF
 *         PPPPPP: Model Code
 *         qqqq : Version
 * @param
 * @param
 * @retval The sum of a and b.
 */

void Parse_VISCA()
{
	if(buff_count == 2 && rxBuff[1] == 0x38)    // CAM <-> MCU 연결시 CAM 전원 인가 후 RX Data
	{
		cam_valid = TRUE;    // Camera 통신 확인 0x90, 0x38, 0xFF

	}
	else if(buff_count == 7 && rxBuff[1] == 0x50)    // 81 09 00 37 FF(CAM_ModlelInq)에 대한 Reply
	{
		_cam_model = rxBuff[2] << 16 | rxBuff[3] << 8 | rxBuff[4];
		_cam_version = rxBuff[5] << 8 | rxBuff[6];
		mode_OLED = 1;
	}
	HAL_TIM_Base_Stop_IT(&htim3);
	flag_camRX = CAM_RX_IDLE;
	buff_count = 0;
#ifdef UART_PRINT
	sprintf(str, "rx time = %d", _time_reset);    // Test 결과 약 8~9 time 걸림
	printf("%s\r\n", str);
#endif
	Sel_uart(FALSE);    // UART 방향 변경: CAM <-> MCU => CAM <-> PC
}

void Send_Message(uint8_t* message)
{
	//HAL_UART_Transmit(&huart2, message, sizeof(message)/sizeof(uint8_t), 100);

}

void Operate_PowerLed(PowerLed pled)
{
	if(pled.command == 0)
	{
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

	}
	else
	{
		switch(pled.color)
		{
			case RED:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
				break;
			case GREEN:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
				break;
			case BLUE:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
				break;
			case ORANGE:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
				break;
			case PURPLE:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
				break;
			case SKYBLUE:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
				break;
			case WHITE:
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
				break;
			default:
				break;
		}
	}
}

void Operate_Led(uint8_t led)
{
	uint8_t i;
	for(i=0; i<8; i++)
	{
		if((led & i) == 0x01)
		{
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
		}
	}
}

void loop() {
    if(HAL_GPIO_ReadPin(SW_RST_GPIO_Port, SW_RST_Pin) == GPIO_PIN_RESET) {

		ssd1306_TestAll();

    }
}

/* Mode Switch Scan Function
*
* mode_pic : 0(LVDS Single), 1(LVDS Dual), 2(BT.1120), 3(MIPI-CSI)
* mode_res : 0(720P), 1(1080P), 2(1440P)
* mode_fps : 0(30), 1(60)
*/

void Scan_ModeSwitch()
{
	uint16_t mode_switch;
	uint8_t mode_pic;
	uint8_t mode_res;
	uint8_t mode_fps;
	//HDMI_VIDEO_MODE hdmi_mode = HDMI_RES_720P_60;

	mode_pic = READ_BIT(GPIOB->IDR, MODE_IN1_Pin|MODE_IN2_Pin) >> 12;
	mode_res = READ_BIT(GPIOB->IDR, MODE_RES1_Pin|MODE_RES2_Pin) >> 1;
	mode_fps = READ_BIT(GPIOB->IDR, MODE_FPS1_Pin|MODE_FPS2_Pin) >> 14;

	mode_switch = mode_pic << 4 | mode_res << 2 | mode_fps ;

	if(mode_switch != _status_mode || _status_mode == 0x80)
	{
		_status_led |= 0x01FF;
		switch(mode_pic)
		{
			case 0:    // MIPI Mode
				//_status_led |= (1 << 3);
				_status_led &= ~(1 << 3);
				break;
			case 1:    // BT.1120 Mode
				//_status_led |= (1 << 2);
				_status_led &= ~(1 << 2);
				break;
			case 2:    // LVDS Dual Mode
				//_status_led |= (1 << 1);
				_status_led &= ~(1 << 1);
				break;
			case 3:    // LVDS Single Mode
				//_status_led |= (1 << 0);
				_status_led &= ~(1 << 0);
				break;
		}
		switch(mode_res)
		{
			case 0:
				//_status_led |= (1 << 6);
				break;
			case 1:    // 1440p
				_status_led &= ~(1 << 6);
				break;
			case 2:    // 1080p
				_status_led &= ~(1 << 5);
				break;
			case 3:    // 720p
				_status_led &= ~(1 << 4);
				break;
		}
		switch(mode_fps)
		{
			case 0:
				//_status_led |= (1 << 7);
				break;
			case 1:
				//_status_led |= (1 << 7);
				break;
			case 2:    // 60fps
				_status_led &= ~(1 << 8);
				break;
			case 3:    // 30fps
				_status_led &= ~(1 << 7);
				break;
		}

/*		switch (mode_switch & 0x000F)
		{
		case 0x0006:
			hdmi_mode = HDMI_RES_1440P_60;
			break;
		case 0x0007:
			hdmi_mode = HDMI_RES_1440P_30;
			break;
		case 0x000A:
			hdmi_mode = HDMI_RES_1080P_60;
			break;
		case 0x000B:
			hdmi_mode = HDMI_RES_1080P_30;
			break;
		case 0x000E:
			hdmi_mode = HDMI_RES_720P_60;
			break;
		case 0x000F:
			hdmi_mode = HDMI_RES_720P_30;
			break;
		} */

		TCA9555_WriteOutput(_status_led);
		//SIL9136_Init(hdmi_mode);

		_status_mode = mode_switch;
#ifdef UART_PRINT
//	sprintf(str, "switch = %x", _status_led);
//	printf("%s\r\n", str);
#endif
	}
}


void Check_Power()
{
	PowerLed pled;

	GPIO_PinState isInput = HAL_GPIO_ReadPin(SW_POWER_GPIO_Port, SW_POWER_Pin);
	if(isInput == GPIO_PIN_SET)
	{
		pled.command = ON;
		pled.color = BLUE;
		Operate_PowerLed(pled);
		HAL_GPIO_WritePin(POWER_ON_GPIO_Port, POWER_ON_Pin, GPIO_PIN_SET);
		_status_power = TRUE;
	}
	else
	{
		pled.command = ON;
		pled.color = GREEN;
		Operate_PowerLed(pled);
		HAL_GPIO_WritePin(POWER_ON_GPIO_Port, POWER_ON_Pin, GPIO_PIN_RESET);
		buff_count = 0;
		cam_valid = FALSE;
		_status_power = FALSE;
	}
}

void On_FPGA()
{
	HAL_GPIO_WritePin(FPGA_RST_GPIO_Port, FPGA_RST_Pin, GPIO_PIN_SET);
}
void On_USB()
{
	HAL_GPIO_WritePin(USB_RST_GPIO_Port, USB_RST_Pin, GPIO_PIN_SET);
}
void On_SDI()
{
	HAL_GPIO_WritePin(SDI_RST_GPIO_Port, SDI_RST_Pin, GPIO_PIN_SET);
}
void On_HDMI()
{
	HAL_GPIO_WritePin(HDMI_RST_GPIO_Port, HDMI_RST_Pin, GPIO_PIN_SET);
}

void On_MIPI()
{
	HAL_GPIO_WritePin(MIPI_RST_GPIO_Port, MIPI_RST_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CAM_RST_GPIO_Port, CAM_RST_Pin, GPIO_PIN_SET);
}

void Sel_uart(bool dir)
{
	if(dir == TRUE)
		HAL_GPIO_WritePin(UART_DIR_GPIO_Port, UART_DIR_Pin, GPIO_PIN_SET);    // CAM <-> PC
	else
		HAL_GPIO_WritePin(UART_DIR_GPIO_Port, UART_DIR_Pin, GPIO_PIN_RESET);  // CAM <-> MCU
}

void Scan_ADC()
{
/*부동소수점 출력
 * C/C++ Build -> Setting -> MCU Setting -> Use float with printf from newlib-nano(-u_printf_float) 체크
 *
 * 	float ADCvol;
	float ADCcur;
	float ADCkey;
	HAL_ADC_Start_DMA(&hadc1, gADCData, 3);

	ADCvol = (float)((gADCData[0]>>2)*3.3)/1024;

	ADCcur = (float)((gADCData[1]>>2)*3.3*6)/1024;

	ADCkey = (float)((gADCData[2]>>2)*3.3)/1024;

#ifdef UART_PRINT
	sprintf(str, "ADC %.2f, %.2f, %.2f", ADCvol, ADCcur, ADCkey);
	printf("%s\r\n", str);
#endif */

	HAL_ADC_Start_DMA(&hadc1, gADCData, 3);
	// AD Value : 4095(0x0FFF) = 3.3V

	_ADCcur = ((gADCData[0]>>2)*33)/100;    // 소비전류 ,0.1 Ohms 사용시 x1, 0.01 Ohms 일 경우 x10

	_ADCvol = ((gADCData[1]>>2)*33*6)/100;    // 인가전압 12V 입력시 AD전압은 2V, Range : 0 ~ 19.8V

	_ADCkey = ((gADCData[2]>>2)*33)/100;    // AD Key

	#ifdef UART_PRINT
	//	sprintf(str, "ADC %d, %d, %d", _ADCvol, _ADCcur, _ADCkey);
	//	printf("%s\r\n", str);
	#endif
}

void Conf_OLED()
{
	if(mode_OLED == 1 && HAL_GPIO_ReadPin(SW_RST_GPIO_Port, SW_RST_Pin) == GPIO_PIN_RESET)
	{
	   	ssd1306_Display(CAM_INFO);
	}
	else if(mode_OLED == 2 && HAL_GPIO_ReadPin(SW_RST_GPIO_Port, SW_RST_Pin) == GPIO_PIN_RESET)
	{
		mode_OLED = 0;
	  	//if(_status_power == TRUE)
	   		//ssd1306_Display(NORMAL);
	   	//else
	  		ssd1306_Display(CAM_RX_FAIL);
	}
	else
	{
	  	mode_OLED = 0;
	  	if(_status_power == TRUE)
	   		ssd1306_Display(NORMAL);
	   	else
	  		ssd1306_Display(READY);
	}
}

void Init_Board()
{
	_status_mode = 0x80;
	_status_led = 0xFFFF;
	HAL_TIM_Base_Start_IT(&htim2);
	TCA9555_SetDirection(0x0000);    // IO Expander All Output

	HAL_Delay(10);

	On_FPGA();
	On_USB();
	On_SDI();
	On_HDMI();
	On_MIPI();

	ssd1306_Init();

	Check_Power();
	Sel_uart(FALSE);    // CAM <-> External uart connect
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  // Timer Initialize
  Init_Board();
  HAL_UART_Receive_IT(&huart2, rxBuff_temp, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  loop();
	  if(_flag_scan == 1)
      {
	    _flag_scan = 0;
	    Scan_ModeSwitch();
	    Scan_ADC();

	    // CAM Info Data 수신 완료 && Reset 스위치 눌려 있는 동안 OLED CAM Info Display

	    Conf_OLED();
	    /*if(mode_OLED == 1 && HAL_GPIO_ReadPin(SW_RST_GPIO_Port, SW_RST_Pin) == GPIO_PIN_RESET)
	    {
	    	ssd1306_Display(CAM_INFO);
	    }
	    else
	    {
	    	mode_OLED = 0;
	    	if(_status_power == TRUE)
	    		ssd1306_Display(NORMAL);
	    	else
	    		ssd1306_Display(READY);

	    }*/

#ifdef TEST_POWERLED
	  		 _pled.color = (++_count_sec)%7;
	  		 _pled.command = ON;
	  		 Operate_PowerLed(_pled);
//	  		PCA8575_SetPin(0, GPIO_PIN_RESET);
#endif
	  	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FPGA_RST_Pin|USB_RST_Pin|SDI_RST_Pin|HDMI_RST_Pin
                          |MIPI_RST_Pin|CAM_RST_Pin|UART_DIR_Pin|LED_G_Pin
                          |LED_B_Pin|LED_R_Pin|POWER_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FPGA_RST_Pin USB_RST_Pin SDI_RST_Pin HDMI_RST_Pin
                           MIPI_RST_Pin CAM_RST_Pin UART_DIR_Pin LED_G_Pin
                           LED_B_Pin LED_R_Pin POWER_ON_Pin */
  GPIO_InitStruct.Pin = FPGA_RST_Pin|USB_RST_Pin|SDI_RST_Pin|HDMI_RST_Pin
                          |MIPI_RST_Pin|CAM_RST_Pin|UART_DIR_Pin|LED_G_Pin
                          |LED_B_Pin|LED_R_Pin|POWER_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_MODE_Pin MODE_RES1_Pin MODE_RES2_Pin MODE_IN1_Pin
                           MODE_IN2_Pin MODE_FPS1_Pin MODE_FPS2_Pin */
  GPIO_InitStruct.Pin = MCU_MODE_Pin|MODE_RES1_Pin|MODE_RES2_Pin|MODE_IN1_Pin
                          |MODE_IN2_Pin|MODE_FPS1_Pin|MODE_FPS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_RST_Pin */
  GPIO_InitStruct.Pin = SW_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_POWER_Pin */
  GPIO_InitStruct.Pin = SW_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_POWER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
